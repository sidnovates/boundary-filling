"""
Swarm Robotics: Multi-Robot Target-Line Filling with Pure Harmonic Control

This variant is intentionally harmonic-only for control comparison:
- Motion guidance force uses only -k_h * grad(U)
- No random exploration force
- No APF-style robot repulsion
- No APF-style wall repulsion
- No parked-point repulsion field

The same environment, target-line parking logic, and collision-safe step backoff
are kept so behavior can be compared fairly with the hybrid script.
"""

import math
import random

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

try:
    from .harmonic_field_solver import HarmonicFieldSolver
except ImportError:
    from harmonic_field_solver import HarmonicFieldSolver

# ====== Environment Parameters ======
ENV_WIDTH = 100
ENV_HEIGHT = 100
NUM_ROBOTS = 10

ROBOT_RADIUS = 1.5
ROBOT_DIAMETER = 2.0 * ROBOT_RADIUS
ROBOT_SPEED = 2.0

TARGET_WIDTH_MULTIPLE = 10
TARGET_LINE_WIDTH = TARGET_WIDTH_MULTIPLE * ROBOT_DIAMETER
TARGET_POINT_SPACING = ROBOT_DIAMETER

OBSTACLE_CENTER_X = ENV_WIDTH / 2.0
OBSTACLE_BOTTOM_Y = 40.0
OBSTACLE_TOP_Y = 80.0
OBSTACLE_LEFT_X = OBSTACLE_CENTER_X - TARGET_LINE_WIDTH / 2.0
OBSTACLE_RIGHT_X = OBSTACLE_CENTER_X + TARGET_LINE_WIDTH / 2.0

TARGET_LINE = ((OBSTACLE_LEFT_X, OBSTACLE_BOTTOM_Y), (OBSTACLE_RIGHT_X, OBSTACLE_BOTTOM_Y))

PARK_DISTANCE_THRESHOLD = 1.0
MAX_STEP_BACKOFF_ITERS = 10
SAFETY_EPS = 1e-4

MAX_TIMESTEPS = 10000
ANIMATION_INTERVAL_MS = 30

# ====== Harmonic Parameters ======
HARMONIC_CELL_SIZE = 1.0
HARMONIC_MAX_ITERS = 1500
HARMONIC_TOL = 2e-4
HARMONIC_SOR_OMEGA = 1.85
HARMONIC_WALL_VALUE = 1.0
HARMONIC_GOAL_VALUE = 0.0
HARMONIC_GOAL_RADIUS = 0.9 * ROBOT_RADIUS
HARMONIC_WALL_THICKNESS = ROBOT_RADIUS
HARMONIC_FORCE_GAIN = 180.0
HARMONIC_FORCE_MAX = 12.0
HARMONIC_RECOMPUTE_ON_PARK = True
HARMONIC_FIELD_ALPHA = 0.24


class Robot:
    def __init__(self, x, y, env_width, env_height, radius=ROBOT_RADIUS):
        self.position = [x, y]
        self.env_width = env_width
        self.env_height = env_height
        self.radius = radius

        self.speed = ROBOT_SPEED
        self.angle = random.uniform(0.0, 2.0 * math.pi)

        self.velocity = np.zeros(2)
        self.parked = False
        self.parked_target_index = None

        self.last_force_components = {
            "harmonic": np.zeros(2),
            "total": np.zeros(2),
            "harmonic_grad_norm": 0.0,
            "harmonic_potential": 0.0,
        }

    def _dist_to_segment(self, px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 == 0:
            return math.hypot(px - x1, py - y1), x1, y1
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y), proj_x, proj_y

    def _build_wall_segments(self, inner_walls):
        walls = [
            (0.0, 0.0, self.env_width, 0.0),
            (0.0, self.env_height, self.env_width, self.env_height),
            (0.0, 0.0, 0.0, self.env_height),
            (self.env_width, 0.0, self.env_width, self.env_height),
        ]
        for wall in inner_walls:
            walls.append((wall[0][0], wall[0][1], wall[1][0], wall[1][1]))
        return walls

    def _is_valid_position(self, x, y, other_robots, inner_walls):
        if x < self.radius - SAFETY_EPS or x > self.env_width - self.radius + SAFETY_EPS:
            return False
        if y < self.radius - SAFETY_EPS or y > self.env_height - self.radius + SAFETY_EPS:
            return False

        walls = self._build_wall_segments(inner_walls)
        for x1, y1, x2, y2 in walls:
            dist, _, _ = self._dist_to_segment(x, y, x1, y1, x2, y2)
            if dist < self.radius - SAFETY_EPS:
                return False

        for other in other_robots:
            if other is self:
                continue
            dist = math.hypot(x - other.position[0], y - other.position[1])
            if dist < (self.radius + other.radius) - SAFETY_EPS:
                return False

        return True

    def _safe_displacement(self, displacement, other_robots, inner_walls):
        if np.linalg.norm(displacement) <= 1e-12:
            return np.zeros(2)

        candidate = np.array(displacement, dtype=float)
        for _ in range(MAX_STEP_BACKOFF_ITERS):
            new_x = self.position[0] + candidate[0]
            new_y = self.position[1] + candidate[1]
            if self._is_valid_position(new_x, new_y, other_robots, inner_walls):
                return candidate
            candidate *= 0.5

        return np.zeros(2)

    def update(self, env):
        if self.parked:
            self.velocity = np.zeros(2)
            self.last_force_components = {
                "harmonic": np.zeros(2),
                "total": np.zeros(2),
                "harmonic_grad_norm": 0.0,
                "harmonic_potential": 0.0,
            }
            return

        f_harmonic, harmonic_grad_norm, harmonic_potential = env.sample_harmonic_guidance(
            self.position[0],
            self.position[1],
            HARMONIC_FORCE_GAIN,
            HARMONIC_FORCE_MAX,
        )
        f_total = f_harmonic

        self.last_force_components = {
            "harmonic": f_harmonic,
            "total": f_total,
            "harmonic_grad_norm": harmonic_grad_norm,
            "harmonic_potential": harmonic_potential,
        }

        magnitude = np.linalg.norm(f_total)
        if magnitude > 1e-12:
            desired_velocity = (f_total / magnitude) * min(magnitude, self.speed)
        else:
            desired_velocity = np.zeros(2)

        safe_step = self._safe_displacement(desired_velocity, env.robots, env.inner_walls)
        self.position[0] += safe_step[0]
        self.position[1] += safe_step[1]
        self.velocity = safe_step

        if np.linalg.norm(safe_step) > 1e-12:
            self.angle = math.atan2(safe_step[1], safe_step[0])


class Environment:
    def __init__(self, width, height, num_robots=NUM_ROBOTS):
        self.width = width
        self.height = height

        self.inner_walls = [
            ((OBSTACLE_LEFT_X, OBSTACLE_BOTTOM_Y), (OBSTACLE_LEFT_X, OBSTACLE_TOP_Y)),
            ((OBSTACLE_RIGHT_X, OBSTACLE_BOTTOM_Y), (OBSTACLE_RIGHT_X, OBSTACLE_TOP_Y)),
            ((OBSTACLE_LEFT_X, OBSTACLE_TOP_Y), (OBSTACLE_RIGHT_X, OBSTACLE_TOP_Y)),
        ]

        self.target_line = TARGET_LINE
        self.line_y = self.target_line[0][1]

        self.target_points = []
        self.target_point_occupied = []
        self.target_point_occupants = []
        self._init_target_points()

        self.harmonic_solver = HarmonicFieldSolver(
            self.width,
            self.height,
            cell_size=HARMONIC_CELL_SIZE,
            max_iters=HARMONIC_MAX_ITERS,
            tolerance=HARMONIC_TOL,
            sor_omega=HARMONIC_SOR_OMEGA,
        )
        self.harmonic_goal_count = 0
        self.harmonic_rebuilds = 0

        self.robots = []
        for i in range(num_robots):
            self.robots.append(self._spawn_robot(i, num_robots))

        self._rebuild_harmonic_field()

    def _init_target_points(self):
        left_x = self.target_line[0][0] + ROBOT_RADIUS
        right_x = self.target_line[1][0] - ROBOT_RADIUS
        if right_x < left_x:
            self.target_points = []
            self.target_point_occupied = []
            self.target_point_occupants = []
            return

        span = right_x - left_x
        num_points = int(math.floor(span / TARGET_POINT_SPACING)) + 1
        xs = [left_x + i * TARGET_POINT_SPACING for i in range(num_points)]

        self.target_points = [(x, self.line_y) for x in xs]
        self.target_point_occupied = [False for _ in range(num_points)]
        self.target_point_occupants = [None for _ in range(num_points)]

    def _active_goal_points(self):
        uncovered = [self.target_points[idx] for idx, occupied in enumerate(self.target_point_occupied) if not occupied]
        if uncovered:
            return uncovered
        if self.target_points:
            return [self.target_points[-1]]
        return []

    def _rebuild_harmonic_field(self):
        self.harmonic_solver.reset(initial_value=0.5)
        self.harmonic_solver.set_outer_boundary(HARMONIC_WALL_VALUE)

        for wall in self.inner_walls:
            self.harmonic_solver.add_segment_dirichlet(
                wall[0],
                wall[1],
                HARMONIC_WALL_VALUE,
                thickness=HARMONIC_WALL_THICKNESS,
            )

        goal_points = self._active_goal_points()
        self.harmonic_goal_count = len(goal_points)
        if goal_points:
            self.harmonic_solver.add_points_dirichlet(
                goal_points,
                HARMONIC_GOAL_VALUE,
                radius=HARMONIC_GOAL_RADIUS,
            )

        self.harmonic_solver.solve()
        self.harmonic_rebuilds += 1

    def sample_harmonic_guidance(self, x, y, gain, max_force):
        potential = float(self.harmonic_solver.sample_potential(x, y))
        gradient = self.harmonic_solver.sample_gradient(x, y)
        force = -float(gain) * gradient

        magnitude = float(np.linalg.norm(force))
        if magnitude > max_force and magnitude > 1e-12:
            force = force * (max_force / magnitude)

        return force, float(np.linalg.norm(gradient)), potential

    def _can_park_at_point(self, robot, point_index):
        if point_index is None:
            return False
        if self.target_point_occupied[point_index]:
            return False

        target_x, target_y = self.target_points[point_index]
        dist = math.hypot(robot.position[0] - target_x, robot.position[1] - target_y)
        if dist > PARK_DISTANCE_THRESHOLD:
            return False

        for other in self.robots:
            if other is robot:
                continue
            dist_other = math.hypot(target_x - other.position[0], target_y - other.position[1])
            if dist_other < (robot.radius + other.radius) - SAFETY_EPS:
                return False

        return True

    def _try_park_robots(self):
        changed = False
        candidates = []

        for robot in self.robots:
            if robot.parked:
                continue

            uncovered_indices = [idx for idx, occupied in enumerate(self.target_point_occupied) if not occupied]
            if not uncovered_indices:
                continue

            point_index = min(
                uncovered_indices,
                key=lambda idx: math.hypot(
                    robot.position[0] - self.target_points[idx][0],
                    robot.position[1] - self.target_points[idx][1],
                ),
            )
            target_x, target_y = self.target_points[point_index]
            dist = math.hypot(robot.position[0] - target_x, robot.position[1] - target_y)
            candidates.append((dist, robot, point_index))

        candidates.sort(key=lambda item: item[0])

        for _, robot, point_index in candidates:
            if self._can_park_at_point(robot, point_index):
                robot.position[0], robot.position[1] = self.target_points[point_index]
                robot.velocity = np.zeros(2)
                robot.parked = True
                robot.parked_target_index = point_index
                self.target_point_occupied[point_index] = True
                self.target_point_occupants[point_index] = robot
                changed = True

        return changed

    def _spawn_robot(self, index, total_robots):
        spacing = self.width / (total_robots + 1)
        spawn_x = spacing * (index + 1)
        spawn_y = self.height - 10.0
        return Robot(spawn_x, spawn_y, self.width, self.height, radius=ROBOT_RADIUS)

    def step(self):
        for robot in self.robots:
            robot.update(self)

        occupancy_changed = self._try_park_robots()
        if occupancy_changed and HARMONIC_RECOMPUTE_ON_PARK:
            self._rebuild_harmonic_field()

    def reached_target_line(self):
        return all(self.target_point_occupied) or all(r.parked for r in self.robots)


def main():
    multiple = TARGET_LINE_WIDTH / ROBOT_DIAMETER
    if abs(multiple - round(multiple)) > 1e-9:
        raise ValueError("Target line width must be an integer multiple of robot diameter.")

    env = Environment(ENV_WIDTH, ENV_HEIGHT, NUM_ROBOTS)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, ENV_WIDTH)
    ax.set_ylim(0, ENV_HEIGHT)
    ax.set_aspect("equal")
    ax.set_title("Multi-Robot Pure Harmonic Control - Line Seeking")

    harmonic_field_img = ax.imshow(
        env.harmonic_solver.potential,
        extent=[0, ENV_WIDTH, 0, ENV_HEIGHT],
        origin="lower",
        cmap="viridis",
        alpha=HARMONIC_FIELD_ALPHA,
        zorder=0,
    )
    cbar = fig.colorbar(harmonic_field_img, ax=ax, fraction=0.046, pad=0.02)
    cbar.set_label("Harmonic Potential")

    ax.plot([0, ENV_WIDTH, ENV_WIDTH, 0, 0], [0, 0, ENV_HEIGHT, ENV_HEIGHT, 0], color="black", linewidth=1.5)

    for wall in env.inner_walls:
        x_coords = [wall[0][0], wall[1][0]]
        y_coords = [wall[0][1], wall[1][1]]
        ax.plot(x_coords, y_coords, color="red", linewidth=3)

    ax.plot(
        [env.target_line[0][0], env.target_line[1][0]],
        [env.target_line[0][1], env.target_line[1][1]],
        color="gray",
        linestyle="--",
        linewidth=1.5,
        label="Target Line",
    )

    target_point_scat = ax.scatter(
        [point[0] for point in env.target_points],
        [point[1] for point in env.target_points],
        s=20,
        c=["orange" for _ in env.target_points],
        marker="o",
        alpha=0.85,
        zorder=2,
        label="Target Points",
    )

    robot_circles = []
    for robot in env.robots:
        circle = plt.Circle(
            (robot.position[0], robot.position[1]),
            robot.radius,
            color="blue",
            fill=True,
            linewidth=0,
            alpha=0.85,
            zorder=3,
        )
        ax.add_patch(circle)
        robot_circles.append(circle)

    force_harmonic_line = ax.plot([], [], color="green", linewidth=1.8, zorder=5, label="F_harmonic")[0]
    force_total_line = ax.plot([], [], color="black", linewidth=2.0, zorder=6, label="F_total")[0]

    stats_text = ax.text(
        0.01,
        0.99,
        "Click a robot to inspect pure harmonic guidance",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=8,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
        zorder=7,
    )

    selection_circle = plt.Circle((0, 0), ROBOT_RADIUS * 1.35, fill=False, color="gold", linewidth=2.0, alpha=0.9, zorder=4)
    selection_circle.set_visible(False)
    ax.add_patch(selection_circle)

    selected = {"index": None}
    state = {"timestep": 0}

    def _set_force_line(line, x0, y0, force_vec, max_len=10.0, scale=1.6):
        mag = float(np.linalg.norm(force_vec))
        if mag <= 1e-12:
            line.set_data([], [])
            return
        draw_len = min(max_len, mag * scale)
        unit = force_vec / mag
        x1 = x0 + unit[0] * draw_len
        y1 = y0 + unit[1] * draw_len
        line.set_data([x0, x1], [y0, y1])

    def on_click(event):
        if event.inaxes != ax:
            return
        if event.xdata is None or event.ydata is None:
            return

        click_x, click_y = event.xdata, event.ydata
        best_idx = None
        best_dist = float("inf")
        for i, robot in enumerate(env.robots):
            dist = math.hypot(robot.position[0] - click_x, robot.position[1] - click_y)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        if best_idx is not None and best_dist <= ROBOT_DIAMETER * 1.2:
            selected["index"] = best_idx
        else:
            selected["index"] = None

        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_click)

    def update_frame(_):
        env.step()
        state["timestep"] += 1

        harmonic_field_img.set_data(env.harmonic_solver.potential)

        for circle, robot in zip(robot_circles, env.robots):
            circle.set_center((robot.position[0], robot.position[1]))
            circle.set_facecolor("navy" if robot.parked else "blue")

        point_colors = ["gray" if occ else "orange" for occ in env.target_point_occupied]
        target_point_scat.set_color(point_colors)

        if selected["index"] is None:
            selection_circle.set_visible(False)
            force_harmonic_line.set_data([], [])
            force_total_line.set_data([], [])
            stats_text.set_text("Click a robot to inspect pure harmonic guidance")
        else:
            sel = env.robots[selected["index"]]
            sx, sy = sel.position[0], sel.position[1]
            selection_circle.set_visible(True)
            selection_circle.set_center((sx, sy))

            comp = sel.last_force_components
            _set_force_line(force_harmonic_line, sx, sy, comp["harmonic"])
            _set_force_line(force_total_line, sx, sy, comp["total"])

            stats_text.set_text(
                f"Robot #{selected['index']} | parked={sel.parked}\n"
                f"pos=({sx:.2f}, {sy:.2f}) vel=({sel.velocity[0]:.3f}, {sel.velocity[1]:.3f})\n"
                f"harmonic_U={comp['harmonic_potential']:.3f}, grad|U|={comp['harmonic_grad_norm']:.4f}\n"
                f"|F_harmonic|={np.linalg.norm(comp['harmonic']):.3f}, |F_total|={np.linalg.norm(comp['total']):.3f}\n"
                f"solver_iters={env.harmonic_solver.last_solve_iterations}, residual={env.harmonic_solver.last_residual:.5f}, goals={env.harmonic_goal_count}, rebuilds={env.harmonic_rebuilds}"
            )

        if env.reached_target_line():
            print(f"All robots reached target line at timestep {state['timestep']}.")
            plt.close(fig)
        elif state["timestep"] >= MAX_TIMESTEPS:
            print(f"Stopped at max timesteps ({MAX_TIMESTEPS}) before reaching target line.")
            plt.close(fig)

        return [
            harmonic_field_img,
            target_point_scat,
            selection_circle,
            force_harmonic_line,
            force_total_line,
            stats_text,
            *robot_circles,
        ]

    animation.FuncAnimation(
        fig,
        update_frame,
        frames=MAX_TIMESTEPS,
        interval=ANIMATION_INTERVAL_MS,
        blit=False,
        repeat=False,
    )

    plt.show()


if __name__ == "__main__":
    main()
