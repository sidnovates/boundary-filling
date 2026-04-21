"""
Boundary-filling benchmark runner for APF and HPF methods.

This script runs selected methods headlessly (without matplotlib animation),
collects per-step and per-run metrics, and exports:
- per-step CSV logs
- run summary CSV
- grouped aggregate CSV
- PNG plots
- markdown report

Default benchmark scope:
- Classic APF baseline (V-gradient)
- Classic APF equal target forces
- Classic APF left-to-right target forces
- APF with Adaptive Bias
- Harmonic PF hybrid
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import random
import statistics
import sys
import time
import traceback
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass(frozen=True)
class MethodSpec:
    method_id: str
    label: str
    family: str
    variant: str
    module_path: Path
    supports_adaptive: bool = False
    supports_harmonic: bool = False


ROOT = Path(__file__).resolve().parent

METHOD_SPECS: List[MethodSpec] = [
    MethodSpec(
        method_id="apf_v_gradient",
        label="Classic APF (V-gradient)",
        family="classic_apf",
        variant="v_gradient",
        module_path=ROOT / "FirstTimeLinedUp_APF" / "multiRobots_APF.py",
    ),
    MethodSpec(
        method_id="apf_equal",
        label="Classic APF (Equal Forces)",
        family="classic_apf",
        variant="equal_forces",
        module_path=ROOT / "FirstTimeLinedUp_APF" / "multiRobots_APF_EqualTargetForces.py",
    ),
    MethodSpec(
        method_id="apf_left_to_right",
        label="Classic APF (Left-to-Right)",
        family="classic_apf",
        variant="left_to_right",
        module_path=ROOT / "FirstTimeLinedUp_APF" / "multiRobots_APF_LeftToRightTargetForces.py",
    ),
    MethodSpec(
        method_id="apf_adaptive_bias",
        label="APF (Adaptive Bias)",
        family="apf_adaptive",
        variant="adaptive_bias",
        module_path=ROOT / "APF_WithBiasPF" / "multiRobots_APF_WithAdaptiveBias.py",
        supports_adaptive=True,
    ),
    MethodSpec(
        method_id="hpf_hybrid",
        label="Harmonic PF (Hybrid)",
        family="harmonic_pf",
        variant="hybrid",
        module_path=ROOT / "Harmonic_PF" / "multiRobots_HPF.py",
        supports_harmonic=True,
    ),
]


PROFILE_OVERRIDES: Dict[str, Dict[str, Dict[str, Any]]] = {
    "fair_paper": {},
    "fast_eval": {
        "hpf_hybrid": {
            "HARMONIC_CELL_SIZE": 2.0,
            "HARMONIC_MAX_ITERS": 800,
            "HARMONIC_TOL": 1e-3,
            "HARMONIC_RECOMPUTE_ON_PARK": False,
            "HARMONIC_GRAD_SAMPLE_STEP": 1.2,
        }
    },
}


def safe_mean(values: Iterable[float]) -> float:
    vals = [float(v) for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return math.nan
    return float(np.mean(vals))


def safe_std(values: Iterable[float]) -> float:
    vals = [float(v) for v in values if not (isinstance(v, float) and math.isnan(v))]
    if len(vals) < 2:
        return 0.0
    return float(np.std(vals, ddof=1))


def percentile(values: List[float], q: float) -> float:
    vals = [float(v) for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return math.nan
    return float(np.percentile(vals, q))


def describe(values: List[float]) -> Dict[str, float]:
    vals = [float(v) for v in values if not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return {
            "mean": math.nan,
            "std": math.nan,
            "median": math.nan,
            "p10": math.nan,
            "p90": math.nan,
            "min": math.nan,
            "max": math.nan,
        }
    return {
        "mean": float(np.mean(vals)),
        "std": float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0,
        "median": float(np.median(vals)),
        "p10": percentile(vals, 10),
        "p90": percentile(vals, 90),
        "min": float(np.min(vals)),
        "max": float(np.max(vals)),
    }


def load_module(module_path: Path, alias: str):
    spec = importlib.util.spec_from_file_location(alias, str(module_path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load module from {module_path}")

    module = importlib.util.module_from_spec(spec)

    parent = str(module_path.parent)
    added_path = False
    if parent not in sys.path:
        sys.path.insert(0, parent)
        added_path = True

    try:
        spec.loader.exec_module(module)
    finally:
        if added_path:
            sys.path.remove(parent)

    return module


@contextmanager
def temporary_module_overrides(module, overrides: Dict[str, Any]):
    previous: Dict[str, Any] = {}
    applied: List[str] = []
    for key, value in overrides.items():
        if hasattr(module, key):
            previous[key] = getattr(module, key)
            setattr(module, key, value)
            applied.append(key)
    try:
        yield
    finally:
        for key in applied:
            setattr(module, key, previous[key])


def target_index_lookup(target_points: List[Tuple[float, float]]) -> Dict[Tuple[float, float], int]:
    return {(float(x), float(y)): i for i, (x, y) in enumerate(target_points)}


def guidance_index(env, robot, lookup: Dict[Tuple[float, float], int]) -> Optional[int]:
    gp = env.get_guidance_point(robot)
    key = (float(gp[0]), float(gp[1]))
    if key in lookup:
        return lookup[key]

    # Fallback: nearest target point in case of tiny floating drift
    if not env.target_points:
        return None
    best_idx = min(
        range(len(env.target_points)),
        key=lambda idx: math.hypot(gp[0] - env.target_points[idx][0], gp[1] - env.target_points[idx][1]),
    )
    return int(best_idx)


def sequential_order_accuracy(
    weights: List[float],
    occupancy_steps: List[Optional[int]],
    max_steps: int,
) -> float:
    if not weights:
        return math.nan

    times = [step if step is not None else (max_steps + 1) for step in occupancy_steps]
    comparable = 0
    correct = 0
    eps = 1e-9

    for i in range(len(weights)):
        for j in range(i + 1, len(weights)):
            wi, wj = weights[i], weights[j]
            if abs(wi - wj) <= eps:
                continue

            comparable += 1
            expected_i_before_j = wi > wj
            actual_i_before_j = times[i] <= times[j]
            if expected_i_before_j == actual_i_before_j:
                correct += 1

    if comparable == 0:
        return math.nan
    return float(correct) / float(comparable)


def compute_ci95(values: List[float]) -> float:
    vals = [float(v) for v in values if not (isinstance(v, float) and math.isnan(v))]
    if len(vals) < 2:
        return 0.0
    return 1.96 * safe_std(vals) / math.sqrt(len(vals))


def run_single_trial(
    spec: MethodSpec,
    module,
    profile_name: str,
    profile_overrides: Dict[str, Any],
    seed: int,
    run_index: int,
    num_robots: int,
    max_steps: int,
    write_raw_steps: bool,
    raw_steps_dir: Path,
    near_collision_scale: float,
    stall_threshold_default: float,
) -> Dict[str, Any]:
    random.seed(seed)
    np.random.seed(seed)

    t0 = time.perf_counter()

    with temporary_module_overrides(module, profile_overrides):
        env = module.Environment(module.ENV_WIDTH, module.ENV_HEIGHT, num_robots=num_robots)

        n_robots = len(env.robots)
        target_points = list(env.target_points)
        n_targets = len(target_points)

        lookup = target_index_lookup(target_points)
        start_positions = [(float(r.position[0]), float(r.position[1])) for r in env.robots]

        path_lengths = [0.0 for _ in env.robots]
        parked_steps: List[Optional[int]] = [None for _ in env.robots]
        prev_guidance_idx: List[Optional[int]] = [None for _ in env.robots]
        guidance_switches = [0 for _ in env.robots]

        prev_adaptive_active = [False for _ in env.robots]
        adaptive_activation_count = 0
        adaptive_active_steps = 0

        speed_all: List[float] = []
        speed_active: List[float] = []

        pairwise_distances: List[float] = []
        min_inter_robot_distance = float("inf")
        near_collision_count = 0

        stall_events = 0
        active_robot_steps = 0

        coverage_history: List[int] = []
        occupancy_steps: List[Optional[int]] = [None for _ in target_points]
        previous_occupancy = list(env.target_point_occupied)

        first_entry_step: Optional[int] = None
        completed = False

        harmonic_rebuild_events: List[Tuple[int, int, float, float]] = []
        if hasattr(env, "harmonic_rebuilds") and hasattr(env, "harmonic_solver"):
            harmonic_rebuild_events.append(
                (
                    0,
                    int(getattr(env, "harmonic_rebuilds", 0)),
                    float(getattr(env.harmonic_solver, "last_solve_iterations", math.nan)),
                    float(getattr(env.harmonic_solver, "last_residual", math.nan)),
                )
            )
        prev_rebuild_count = int(getattr(env, "harmonic_rebuilds", 0)) if hasattr(env, "harmonic_rebuilds") else 0

        near_collision_threshold = float(getattr(module, "ROBOT_RADIUS", 1.5)) * 2.0 * near_collision_scale
        force_stall_threshold = float(getattr(module, "CONFUSION_THRESHOLD", stall_threshold_default))

        raw_writer = None
        raw_file = None
        if write_raw_steps:
            raw_path = raw_steps_dir / f"{spec.method_id}_r{num_robots}_seed{seed}.csv"
            raw_file = raw_path.open("w", newline="", encoding="utf-8")
            raw_writer = csv.DictWriter(
                raw_file,
                fieldnames=[
                    "method_id",
                    "robot_count",
                    "seed",
                    "timestep",
                    "coverage_count",
                    "coverage_ratio",
                    "parked_count",
                    "active_count",
                    "mean_speed_all",
                    "mean_speed_active",
                    "min_pair_distance_active",
                    "stall_events_step",
                    "adaptive_active_step",
                    "harmonic_rebuilds",
                    "harmonic_last_iters",
                    "harmonic_last_residual",
                ],
            )
            raw_writer.writeheader()

        timestep = 0

        try:
            while timestep < max_steps:
                # Track guidance switching from the same pre-step state used by robot updates.
                for i, robot in enumerate(env.robots):
                    if robot.parked:
                        continue
                    gi = guidance_index(env, robot, lookup)
                    if gi is None:
                        continue
                    if prev_guidance_idx[i] is None:
                        prev_guidance_idx[i] = gi
                    elif prev_guidance_idx[i] != gi:
                        guidance_switches[i] += 1
                        prev_guidance_idx[i] = gi

                pre_positions = [(float(r.position[0]), float(r.position[1])) for r in env.robots]

                env.step()
                timestep += 1

                step_speeds_all: List[float] = []
                step_speeds_active: List[float] = []
                stall_events_step = 0
                adaptive_active_step = 0

                for i, robot in enumerate(env.robots):
                    post_x = float(robot.position[0])
                    post_y = float(robot.position[1])
                    dx = post_x - pre_positions[i][0]
                    dy = post_y - pre_positions[i][1]
                    path_lengths[i] += math.hypot(dx, dy)

                    speed = float(np.linalg.norm(robot.velocity))
                    step_speeds_all.append(speed)
                    speed_all.append(speed)

                    if not robot.parked:
                        step_speeds_active.append(speed)
                        speed_active.append(speed)

                    if robot.parked and parked_steps[i] is None:
                        parked_steps[i] = timestep
                        if first_entry_step is None:
                            first_entry_step = timestep

                    comp = getattr(robot, "last_force_components", {})
                    f_total = comp.get("total", np.zeros(2))
                    total_norm = float(np.linalg.norm(f_total))

                    if not robot.parked:
                        active_robot_steps += 1
                        if total_norm < force_stall_threshold:
                            stall_events += 1
                            stall_events_step += 1

                    if "adaptive_bias" in comp:
                        adaptive_norm = float(np.linalg.norm(comp["adaptive_bias"]))
                        is_active = adaptive_norm > 1e-9
                        if is_active:
                            adaptive_active_steps += 1
                            adaptive_active_step += 1
                        if is_active and not prev_adaptive_active[i]:
                            adaptive_activation_count += 1
                        prev_adaptive_active[i] = is_active

                active_positions = [
                    (float(r.position[0]), float(r.position[1]))
                    for r in env.robots
                    if not r.parked
                ]

                step_min_pair = math.nan
                if len(active_positions) >= 2:
                    local_min = float("inf")
                    for i in range(len(active_positions)):
                        xi, yi = active_positions[i]
                        for j in range(i + 1, len(active_positions)):
                            xj, yj = active_positions[j]
                            d = math.hypot(xi - xj, yi - yj)
                            pairwise_distances.append(d)
                            if d < local_min:
                                local_min = d
                            if d < min_inter_robot_distance:
                                min_inter_robot_distance = d
                            if d < near_collision_threshold:
                                near_collision_count += 1
                    step_min_pair = local_min

                coverage_count = int(sum(1 for occ in env.target_point_occupied if occ))
                coverage_ratio = float(coverage_count) / float(max(1, n_targets))
                coverage_history.append(coverage_count)

                for idx, occ in enumerate(env.target_point_occupied):
                    if occ and not previous_occupancy[idx] and occupancy_steps[idx] is None:
                        occupancy_steps[idx] = timestep
                previous_occupancy = list(env.target_point_occupied)

                harmonic_rebuilds = int(getattr(env, "harmonic_rebuilds", 0)) if hasattr(env, "harmonic_rebuilds") else 0
                harmonic_last_iters = math.nan
                harmonic_last_residual = math.nan
                if hasattr(env, "harmonic_solver"):
                    harmonic_last_iters = float(getattr(env.harmonic_solver, "last_solve_iterations", math.nan))
                    harmonic_last_residual = float(getattr(env.harmonic_solver, "last_residual", math.nan))
                if harmonic_rebuilds > prev_rebuild_count and hasattr(env, "harmonic_solver"):
                    harmonic_rebuild_events.append(
                        (timestep, harmonic_rebuilds, harmonic_last_iters, harmonic_last_residual)
                    )
                prev_rebuild_count = harmonic_rebuilds

                if raw_writer is not None:
                    raw_writer.writerow(
                        {
                            "method_id": spec.method_id,
                            "robot_count": num_robots,
                            "seed": seed,
                            "timestep": timestep,
                            "coverage_count": coverage_count,
                            "coverage_ratio": coverage_ratio,
                            "parked_count": int(sum(1 for r in env.robots if r.parked)),
                            "active_count": int(sum(1 for r in env.robots if not r.parked)),
                            "mean_speed_all": safe_mean(step_speeds_all),
                            "mean_speed_active": safe_mean(step_speeds_active),
                            "min_pair_distance_active": step_min_pair,
                            "stall_events_step": stall_events_step,
                            "adaptive_active_step": adaptive_active_step,
                            "harmonic_rebuilds": harmonic_rebuilds,
                            "harmonic_last_iters": harmonic_last_iters,
                            "harmonic_last_residual": harmonic_last_residual,
                        }
                    )

                if env.reached_target_line():
                    completed = True
                    break
        finally:
            if raw_file is not None:
                raw_file.close()

    runtime_seconds = time.perf_counter() - t0

    completion_time = int(timestep if completed else max_steps)
    final_coverage_count = int(sum(1 for occ in env.target_point_occupied if occ))
    final_coverage_ratio = float(final_coverage_count) / float(max(1, n_targets))

    parked_count = sum(1 for s in parked_steps if s is not None)
    convergence_censored = [s if s is not None else max_steps for s in parked_steps]
    convergence_success = [s for s in parked_steps if s is not None]

    total_path_length = float(sum(path_lengths))
    mean_path_length = float(total_path_length / max(1, n_robots))

    straightness_scores: List[float] = []
    for i, robot in enumerate(env.robots):
        sx, sy = start_positions[i]
        ex, ey = float(robot.position[0]), float(robot.position[1])
        straight = math.hypot(ex - sx, ey - sy)
        path = path_lengths[i]
        if path <= 1e-12:
            score = 1.0 if straight <= 1e-12 else 0.0
        else:
            score = min(1.0, straight / path)
        straightness_scores.append(score)

    inter_robot_distance_variance = float(np.var(pairwise_distances)) if pairwise_distances else math.nan
    mean_separation_distance = safe_mean(pairwise_distances)

    stall_frequency = float(stall_events) / float(max(1, active_robot_steps))

    order_accuracy = sequential_order_accuracy(
        list(getattr(env, "target_point_weights", [])),
        occupancy_steps,
        max_steps,
    )

    harmonic_iters = [event[2] for event in harmonic_rebuild_events if not math.isnan(event[2])]
    harmonic_residuals = [event[3] for event in harmonic_rebuild_events if not math.isnan(event[3])]

    coverage_slope = math.nan
    if len(coverage_history) >= 2:
        x = np.arange(1, len(coverage_history) + 1, dtype=float)
        y = np.array(coverage_history, dtype=float)
        x_center = x - np.mean(x)
        denom = float(np.dot(x_center, x_center))
        if denom > 0:
            coverage_slope = float(np.dot(x_center, (y - np.mean(y))) / denom)

    return {
        "method_id": spec.method_id,
        "method_label": spec.label,
        "family": spec.family,
        "variant": spec.variant,
        "profile": profile_name,
        "robot_count": num_robots,
        "seed": seed,
        "run_index": run_index,
        "max_steps": max_steps,
        "completed": int(completed),
        "completion_time": completion_time,
        "time_to_first_entry": float(first_entry_step) if first_entry_step is not None else math.nan,
        "avg_convergence_time": safe_mean(convergence_censored),
        "avg_convergence_time_success_only": safe_mean(convergence_success),
        "parked_count": parked_count,
        "final_coverage_count": final_coverage_count,
        "final_coverage_ratio": final_coverage_ratio,
        "coverage_progress_slope": coverage_slope,
        "total_path_length": total_path_length,
        "mean_path_length": mean_path_length,
        "average_velocity_all": safe_mean(speed_all),
        "average_velocity_active": safe_mean(speed_active),
        "mean_straightness": safe_mean(straightness_scores),
        "inter_robot_distance_variance": inter_robot_distance_variance,
        "mean_separation_distance": mean_separation_distance,
        "min_inter_robot_distance": min_inter_robot_distance if min_inter_robot_distance != float("inf") else math.nan,
        "near_collision_count": near_collision_count,
        "stall_events": stall_events,
        "stall_frequency": stall_frequency,
        "target_switch_total": int(sum(guidance_switches)),
        "target_switch_mean_per_robot": safe_mean(guidance_switches),
        "sequential_order_accuracy": order_accuracy,
        "adaptive_activation_count": int(adaptive_activation_count),
        "adaptive_active_steps": int(adaptive_active_steps),
        "harmonic_rebuild_count": int(getattr(env, "harmonic_rebuilds", 0)) if hasattr(env, "harmonic_rebuilds") else 0,
        "harmonic_mean_solve_iterations": safe_mean(harmonic_iters),
        "harmonic_final_residual": float(getattr(env.harmonic_solver, "last_residual", math.nan))
        if hasattr(env, "harmonic_solver")
        else math.nan,
        "harmonic_mean_residual": safe_mean(harmonic_residuals),
        "runtime_seconds": float(runtime_seconds),
        "runtime_per_step_seconds": float(runtime_seconds) / float(max(1, timestep)),
        "runtime_per_1000_steps_seconds": float(runtime_seconds) * 1000.0 / float(max(1, timestep)),
        "error": "",
    }


def aggregate_run_rows(run_rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[Tuple[str, int], List[Dict[str, Any]]] = {}
    for row in run_rows:
        key = (str(row["method_id"]), int(row["robot_count"]))
        grouped.setdefault(key, []).append(row)

    metrics_for_stats = [
        "completion_time",
        "time_to_first_entry",
        "avg_convergence_time",
        "final_coverage_ratio",
        "total_path_length",
        "average_velocity_active",
        "mean_straightness",
        "inter_robot_distance_variance",
        "mean_separation_distance",
        "stall_frequency",
        "target_switch_mean_per_robot",
        "sequential_order_accuracy",
        "near_collision_count",
        "runtime_seconds",
    ]

    aggregate_rows: List[Dict[str, Any]] = []
    for (method_id, robot_count), rows in sorted(grouped.items(), key=lambda x: (x[0][1], x[0][0])):
        first = rows[0]
        out: Dict[str, Any] = {
            "method_id": method_id,
            "method_label": first["method_label"],
            "family": first["family"],
            "variant": first["variant"],
            "profile": first["profile"],
            "robot_count": robot_count,
            "runs": len(rows),
            "completion_rate": safe_mean([float(r["completed"]) for r in rows]),
            "completion_time_ci95": compute_ci95([float(r["completion_time"]) for r in rows]),
            "stall_frequency_ci95": compute_ci95([float(r["stall_frequency"]) for r in rows]),
        }

        for metric in metrics_for_stats:
            vals = [float(r[metric]) for r in rows]
            stats = describe(vals)
            out[f"{metric}_mean"] = stats["mean"]
            out[f"{metric}_std"] = stats["std"]
            out[f"{metric}_median"] = stats["median"]
            out[f"{metric}_p10"] = stats["p10"]
            out[f"{metric}_p90"] = stats["p90"]
            out[f"{metric}_min"] = stats["min"]
            out[f"{metric}_max"] = stats["max"]

        out["harmonic_rebuild_count_mean"] = safe_mean([float(r["harmonic_rebuild_count"]) for r in rows])
        out["harmonic_mean_solve_iterations_mean"] = safe_mean(
            [float(r["harmonic_mean_solve_iterations"]) for r in rows]
        )
        out["adaptive_activation_count_mean"] = safe_mean([float(r["adaptive_activation_count"]) for r in rows])

        aggregate_rows.append(out)

    return aggregate_rows


def write_csv(rows: List[Dict[str, Any]], path: Path) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return

    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def create_metric_trend_plot(run_rows: List[Dict[str, Any]], metric: str, output_path: Path) -> None:
    robot_counts = sorted({int(r["robot_count"]) for r in run_rows})
    methods = sorted({str(r["method_id"]) for r in run_rows})

    plt.figure(figsize=(9, 5))
    for method in methods:
        y = []
        for rc in robot_counts:
            vals = [
                float(r[metric])
                for r in run_rows
                if str(r["method_id"]) == method and int(r["robot_count"]) == rc
            ]
            y.append(safe_mean(vals))
        plt.plot(robot_counts, y, marker="o", linewidth=2, label=method)

    plt.xlabel("Robot Count")
    plt.ylabel(metric)
    plt.title(f"{metric} vs Robot Count")
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def create_completion_boxplots(run_rows: List[Dict[str, Any]], plots_dir: Path) -> None:
    robot_counts = sorted({int(r["robot_count"]) for r in run_rows})
    methods = sorted({str(r["method_id"]) for r in run_rows})

    for rc in robot_counts:
        data = []
        labels = []
        for method in methods:
            vals = [
                float(r["completion_time"])
                for r in run_rows
                if str(r["method_id"]) == method and int(r["robot_count"]) == rc
            ]
            if vals:
                data.append(vals)
                labels.append(method)

        if not data:
            continue

        plt.figure(figsize=(10, 5))
        plt.boxplot(data, tick_labels=labels, showmeans=True)
        plt.title(f"Completion Time Distribution (robot_count={rc})")
        plt.ylabel("Timesteps")
        plt.xticks(rotation=20, ha="right")
        plt.grid(axis="y", alpha=0.25)
        plt.tight_layout()
        plt.savefig(plots_dir / f"completion_time_boxplot_r{rc}.png", dpi=150)
        plt.close()


def generate_report(
    output_path: Path,
    args: argparse.Namespace,
    run_rows: List[Dict[str, Any]],
    aggregate_rows: List[Dict[str, Any]],
) -> None:
    lines: List[str] = []

    lines.append("# Boundary Filling Benchmark Report")
    lines.append("")
    lines.append(f"Generated: {datetime.now().isoformat(timespec='seconds')}")
    lines.append(f"Profile: {args.profile}")
    lines.append(f"Runs per configuration: {args.runs}")
    lines.append(f"Robot counts: {', '.join(str(x) for x in args.robot_counts)}")
    lines.append(f"Max steps: {args.max_steps}")
    lines.append("")

    total_runs = len(run_rows)
    completed_runs = sum(int(r["completed"]) for r in run_rows)
    lines.append("## Batch Summary")
    lines.append("")
    lines.append(f"- Total runs: {total_runs}")
    lines.append(f"- Completed runs: {completed_runs}")
    lines.append(f"- Overall completion rate: {completed_runs / max(1, total_runs):.3f}")
    lines.append("")

    lines.append("## Aggregated Results by Method and Scale")
    lines.append("")
    lines.append(
        "| Method | Robots | Completion Rate | Completion Time Mean | Stall Frequency Mean | "
        "Tortuosity (Straightness) Mean | Runtime Mean (s) |"
    )
    lines.append("|---|---:|---:|---:|---:|---:|---:|")

    for row in sorted(aggregate_rows, key=lambda r: (int(r["robot_count"]), float(r["completion_time_mean"]))):
        lines.append(
            "| "
            f"{row['method_id']} | {int(row['robot_count'])} | {float(row['completion_rate']):.3f} | "
            f"{float(row['completion_time_mean']):.2f} | {float(row['stall_frequency_mean']):.4f} | "
            f"{float(row['mean_straightness_mean']):.4f} | {float(row['runtime_seconds_mean']):.3f} |"
        )

    lines.append("")
    lines.append("## Notes")
    lines.append("")
    lines.append("- Sequential packing order accuracy uses target-point weight priority for comparison.")
    lines.append("- For equal-force target weights (all equal), sequential order accuracy may be NaN (undefined priority).")
    lines.append("- Stall frequency is measured as fraction of active robot-step events with low net force magnitude.")
    lines.append("- Adaptive-bias diagnostics are populated only for the adaptive method.")
    lines.append("- Harmonic diagnostics are populated only for HPF hybrid.")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark APF/HPF boundary-filling methods.")
    parser.add_argument("--runs", type=int, default=30, help="Number of seeded runs per method/config.")
    parser.add_argument(
        "--robot-counts",
        type=int,
        nargs="+",
        default=[5, 10, 15],
        help="Robot-count sweep values.",
    )
    parser.add_argument("--max-steps", type=int, default=10000, help="Max timesteps per run.")
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_OVERRIDES.keys()),
        default="fast_eval",
        help="Benchmark runtime profile.",
    )
    parser.add_argument(
        "--methods",
        type=str,
        nargs="+",
        default=[m.method_id for m in METHOD_SPECS],
        choices=[m.method_id for m in METHOD_SPECS],
        help="Subset of methods to run.",
    )
    parser.add_argument("--seed-base", type=int, default=0, help="Base seed offset.")
    parser.add_argument("--output-root", type=str, default="benchmark_outputs", help="Output root directory.")
    parser.add_argument("--no-raw-steps", action="store_true", help="Disable per-step raw CSV logs.")
    parser.add_argument("--near-collision-scale", type=float, default=1.05, help="Near-collision threshold scale on 2*radius.")
    parser.add_argument("--stall-threshold", type=float, default=2.0, help="Fallback low-force threshold when method does not define one.")
    parser.add_argument("--smoke", action="store_true", help="Run a quick smoke batch (1 run/method/count, 500 max steps unless set).")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.smoke:
        args.runs = 1
        if "--max-steps" not in sys.argv:
            args.max_steps = min(500, args.max_steps)

    selected_specs = [m for m in METHOD_SPECS if m.method_id in set(args.methods)]
    if not selected_specs:
        raise ValueError("No methods selected.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = ROOT / args.output_root / f"benchmark_{timestamp}_{args.profile}"
    raw_steps_dir = output_dir / "raw_steps"
    runs_summary_dir = output_dir / "runs_summary"
    aggregates_dir = output_dir / "aggregates"
    plots_dir = output_dir / "plots"
    report_dir = output_dir / "report"

    for d in [raw_steps_dir, runs_summary_dir, aggregates_dir, plots_dir, report_dir]:
        d.mkdir(parents=True, exist_ok=True)

    print(f"[benchmark] Output directory: {output_dir}")
    print(f"[benchmark] Loading {len(selected_specs)} method modules...")

    modules: Dict[str, Any] = {}
    for idx, spec in enumerate(selected_specs, start=1):
        alias = f"benchmark_mod_{idx}_{spec.method_id}"
        modules[spec.method_id] = load_module(spec.module_path, alias)

    total_runs = len(selected_specs) * len(args.robot_counts) * int(args.runs)
    print(f"[benchmark] Planned runs: {total_runs}")

    run_rows: List[Dict[str, Any]] = []
    run_counter = 0

    for spec in selected_specs:
        module = modules[spec.method_id]
        method_overrides = PROFILE_OVERRIDES.get(args.profile, {}).get(spec.method_id, {})

        for robot_count in args.robot_counts:
            for run_index in range(args.runs):
                seed = int(args.seed_base) + int(run_index)
                run_counter += 1
                print(
                    f"[run {run_counter}/{total_runs}] method={spec.method_id} robots={robot_count} seed={seed}",
                    flush=True,
                )

                try:
                    row = run_single_trial(
                        spec=spec,
                        module=module,
                        profile_name=args.profile,
                        profile_overrides=method_overrides,
                        seed=seed,
                        run_index=run_index,
                        num_robots=int(robot_count),
                        max_steps=int(args.max_steps),
                        write_raw_steps=not args.no_raw_steps,
                        raw_steps_dir=raw_steps_dir,
                        near_collision_scale=float(args.near_collision_scale),
                        stall_threshold_default=float(args.stall_threshold),
                    )
                except Exception as exc:
                    row = {
                        "method_id": spec.method_id,
                        "method_label": spec.label,
                        "family": spec.family,
                        "variant": spec.variant,
                        "profile": args.profile,
                        "robot_count": int(robot_count),
                        "seed": seed,
                        "run_index": run_index,
                        "max_steps": int(args.max_steps),
                        "completed": 0,
                        "completion_time": int(args.max_steps),
                        "time_to_first_entry": math.nan,
                        "avg_convergence_time": math.nan,
                        "avg_convergence_time_success_only": math.nan,
                        "parked_count": 0,
                        "final_coverage_count": 0,
                        "final_coverage_ratio": math.nan,
                        "coverage_progress_slope": math.nan,
                        "total_path_length": math.nan,
                        "mean_path_length": math.nan,
                        "average_velocity_all": math.nan,
                        "average_velocity_active": math.nan,
                        "mean_straightness": math.nan,
                        "inter_robot_distance_variance": math.nan,
                        "mean_separation_distance": math.nan,
                        "min_inter_robot_distance": math.nan,
                        "near_collision_count": math.nan,
                        "stall_events": math.nan,
                        "stall_frequency": math.nan,
                        "target_switch_total": math.nan,
                        "target_switch_mean_per_robot": math.nan,
                        "sequential_order_accuracy": math.nan,
                        "adaptive_activation_count": math.nan,
                        "adaptive_active_steps": math.nan,
                        "harmonic_rebuild_count": math.nan,
                        "harmonic_mean_solve_iterations": math.nan,
                        "harmonic_final_residual": math.nan,
                        "harmonic_mean_residual": math.nan,
                        "runtime_seconds": math.nan,
                        "runtime_per_step_seconds": math.nan,
                        "runtime_per_1000_steps_seconds": math.nan,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                    print("[benchmark] Run failed:")
                    traceback.print_exc()

                run_rows.append(row)

    runs_csv_path = runs_summary_dir / "run_summary.csv"
    write_csv(run_rows, runs_csv_path)

    aggregate_rows = aggregate_run_rows(run_rows)
    aggregate_csv_path = aggregates_dir / "aggregate_summary.csv"
    write_csv(aggregate_rows, aggregate_csv_path)

    print("[benchmark] Generating plots...")
    create_completion_boxplots(run_rows, plots_dir)
    for metric in [
        "completion_time",
        "stall_frequency",
        "mean_straightness",
        "total_path_length",
        "runtime_seconds",
    ]:
        create_metric_trend_plot(run_rows, metric, plots_dir / f"trend_{metric}.png")

    report_path = report_dir / "benchmark_report.md"
    generate_report(report_path, args, run_rows, aggregate_rows)

    print("[benchmark] Done.")
    print(f"[benchmark] Run summary CSV: {runs_csv_path}")
    print(f"[benchmark] Aggregate CSV: {aggregate_csv_path}")
    print(f"[benchmark] Report: {report_path}")
    print(f"[benchmark] Plots dir: {plots_dir}")


if __name__ == "__main__":
    main()
