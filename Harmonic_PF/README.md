# Harmonic PF Multi-Robot Boundary Filling

This folder contains a harmonic potential field version of the APF line-filling simulation.

## Goal

The mission is unchanged: robots navigate through the U-shaped obstacle, occupy target-line points, and stop when the line is fully claimed.

## Files

- `multiRobots_HPF.py`: main simulation, robot/environment logic, and visualization
- `multiRobots_HPF_pure.py`: pure harmonic-only control variant for direct comparison
- `harmonic_field_solver.py`: grid-based Laplace solver and gradient sampling utilities

## Core Equations Used

### 1. Harmonic potential equation

In free space, the scalar potential is harmonic:

$$
\nabla^2 U(x,y)=0
$$

with boundary conditions set by walls/obstacles and goals.

### 2. Guidance from the harmonic field

Robots are guided by negative potential gradient:

$$
\mathbf{F}_{harmonic} = -k_h\nabla U
$$

and then clamped by `HARMONIC_FORCE_MAX`.

### 3. Discrete Laplace update (solver basis)

Interior grid values satisfy the neighbor average relation:

$$
U_{i,j} = \frac{1}{4}\left(U_{i+1,j}+U_{i-1,j}+U_{i,j+1}+U_{i,j-1}\right)
$$

### 4. SOR iterative update (used for faster convergence)

$$
U_{i,j}^{new} = (1-\omega)U_{i,j}^{old} + \omega\,\overline{U}_{neighbors}
$$

where $\omega$ is `HARMONIC_SOR_OMEGA`.

### 5. Gradient approximation (finite differences)

$$
\frac{\partial U}{\partial x}\approx\frac{U(x+h,y)-U(x-h,y)}{2h},\quad
\frac{\partial U}{\partial y}\approx\frac{U(x,y+h)-U(x,y-h)}{2h}
$$

### 6. Hybrid total force used in this implementation

This is not pure harmonic-only control. The simulation currently uses:

$$
\mathbf{F}_{total}=\mathbf{F}_{random}+\mathbf{F}_{robot}+\mathbf{F}_{wall}+\mathbf{F}_{harmonic}+\mathbf{F}_{parked}
$$

## New Harmonic Parameters (one-line definitions)

The following were introduced for the harmonic method in `multiRobots_HPF.py`:

- `HARMONIC_CELL_SIZE`: world units per grid cell for the harmonic solver.
- `HARMONIC_MAX_ITERS`: maximum solver iterations per harmonic rebuild.
- `HARMONIC_TOL`: convergence threshold for max update change per iteration.
- `HARMONIC_SOR_OMEGA`: SOR relaxation factor controlling convergence speed/stability.
- `HARMONIC_WALL_VALUE`: Dirichlet potential assigned to outer boundaries and obstacle walls.
- `HARMONIC_GOAL_VALUE`: Dirichlet potential assigned to active target goal points.
- `HARMONIC_GOAL_RADIUS`: spatial radius around each goal point fixed to goal potential.
- `HARMONIC_WALL_THICKNESS`: rasterized wall thickness used when stamping obstacle segments onto the grid.
- `HARMONIC_FORCE_GAIN`: gain converting gradient magnitude into guidance force magnitude.
- `HARMONIC_FORCE_MAX`: hard cap on harmonic force magnitude before summation.
- `HARMONIC_RECOMPUTE_ON_PARK`: when true, rebuilds the harmonic field after new parking events.
- `HARMONIC_FIELD_ALPHA`: display transparency for the potential heatmap overlay.

## What Changed vs Normal APF

- Normal APF computes target attraction directly from local formulas each step.
- Harmonic PF computes a global potential map first, then guidance is sampled as $-\nabla U$.
- Local collision/safety terms are still kept (robot repulsion, wall repulsion, step-backoff checks).
- Occupied-point local repulsion is still kept for stable line packing.

## Conceptual FAQ (from this chat)

### Q1. Why do we need precomputation in harmonic PF but not in normal APF?

In normal APF, each force is an explicit local expression, so it can be evaluated directly per step.

In harmonic PF, guidance comes from solving a global PDE over the full map. The potential at one location depends on neighboring values across the domain, so the field is solved first (precomputed), then sampled during robot motion.

### Q2. We still use random forces. Are we really better, or is this just another method?

It is a different method, and this implementation is a hybrid controller. Harmonic guidance now handles global routing, while random/local terms provide robustness and collision handling. Improvement usually comes from smoother global flow and reduced trap risk near concave obstacles.

### Q3. Is harmonic PF always better than normal APF?

Not always. Harmonic PF is often better for static-map global guidance (like this U-obstacle setup), but it has extra solver cost. Normal APF is cheaper and simpler. For multi-robot swarms in practice, a hybrid often works best: harmonic for global direction plus local reactive safety.

### Q4. Could we remove random force entirely?

Yes, but that makes motion more deterministic and can reduce exploration in weak-gradient regions. A common progression is to start hybrid, then gradually reduce random force once harmonic guidance and local safety are tuned.

## Run

From workspace root:

```powershell
& "c:/CHAITYA/SEM 6/Robotics/boundary_filling/venv/Scripts/python.exe" .\Harmonic_PF\multiRobots_HPF.py
```

Pure harmonic-only variant:

```powershell
& "c:/CHAITYA/SEM 6/Robotics/boundary_filling/venv/Scripts/python.exe" .\Harmonic_PF\multiRobots_HPF_pure.py
```

## Hybrid vs Pure Harmonic (this folder)

- `multiRobots_HPF.py` (hybrid):

$$
\mathbf{F}_{total}=\mathbf{F}_{random}+\mathbf{F}_{robot}+\mathbf{F}_{wall}+\mathbf{F}_{harmonic}+\mathbf{F}_{parked}
$$

- `multiRobots_HPF_pure.py` (pure harmonic-only control):

$$
\mathbf{F}_{total}=\mathbf{F}_{harmonic}=-k_h\nabla U
$$

The pure script keeps the same map, goal-point parking logic, and collision-safe step backoff checks, but removes all non-harmonic control forces.

## Suggested Tuning Order

1. Tune `HARMONIC_FORCE_GAIN` for global pull strength.
2. Tune `HARMONIC_FORCE_MAX` to limit overshoot.
3. Tune `HARMONIC_CELL_SIZE`, `HARMONIC_TOL`, and `HARMONIC_SOR_OMEGA` for field quality and runtime.
4. Decide whether to keep `HARMONIC_RECOMPUTE_ON_PARK=True` based on occupancy behavior.
5. Reduce random-force influence only after harmonic behavior is stable.
