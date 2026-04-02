# single_robot_target.py — Complete Technical Math & Parameter Reference

This document is a full technical walkthrough of the current `single_robot_target.py` implementation.
It explains:

1. every configured parameter,
2. every derived quantity,
3. every force term and dependent sub-expression,
4. motion integration and safety constraints,
5. parking/coverage logic,
6. debug telemetry semantics.

The goal is to make the simulation behavior traceable from top to bottom, directly from equations.

---

## 1) System Overview

The simulator models a multi-robot APF system in a 2D environment with:

- bounded world,
- a U-shaped obstacle (closed top, open bottom),
- a target line along the open bottom,
- dotted target points on that line,
- robots that move under combined forces and park at target points.

The per-step update for each unparked robot is:

1. compute force components,
2. convert to desired velocity,
3. smooth and clamp velocity,
4. run predictive collision-safe displacement,
5. update pose and heading,
6. attempt parking.

The run ends when all target points are occupied (or all robots are parked).

---

## 2) Coordinate Frame and Geometry

### 2.1 Workspace

- Width: `ENV_WIDTH = 100`
- Height: `ENV_HEIGHT = 100`

Cartesian frame: `x` rightward, `y` upward.

### 2.2 Robot Geometry

- Radius: `ROBOT_RADIUS = r = 1.5`
- Diameter: `ROBOT_DIAMETER = d = 2r = 3.0`

Hard non-overlap geometric constraint between robots `i, j`:

\[ \|p_i - p_j\| \ge r_i + r_j \]

### 2.3 Target Span and U-Obstacle Geometry

- `TARGET_WIDTH_MULTIPLE = 10`
- `TARGET_LINE_WIDTH = TARGET_WIDTH_MULTIPLE * ROBOT_DIAMETER = 30`

Obstacle center and extents:

- `OBSTACLE_CENTER_X = 50`
- `OBSTACLE_BOTTOM_Y = 40`
- `OBSTACLE_TOP_Y = 80`
- `OBSTACLE_LEFT_X = 50 - 15 = 35`
- `OBSTACLE_RIGHT_X = 50 + 15 = 65`

Target line:

\[
\text{TARGET_LINE} = [(35,40),(65,40)]
\]

The U wall segments are:

1. left leg: `((35,40),(35,80))`
2. right leg: `((65,40),(65,80))`
3. top bar: `((35,80),(65,80))`

---

## 3) Full Parameter Dictionary (Current Defaults)

### 3.1 Environment and Robot Count

- `ENV_WIDTH = 100`
- `ENV_HEIGHT = 100`
- `NUM_ROBOTS = 10`

### 3.2 Robot Body and Kinematics

- `ROBOT_RADIUS = 1.5`
- `ROBOT_DIAMETER = 3.0` (derived)
- `ROBOT_SPEED = 0.5` (base speed cap)
- `VELOCITY_SMOOTHING = 0.65` (blending factor)

### 3.3 Random Exploration

- `TURN_PROBABILITY = 0.24`
- `RANDOM_FORCE_SCALE = 1.0`
- `RANDOM_FORCE_MAX_SCALE = 10`
- `RANDOM_NEAR_LINE_FLOOR = 0.1`

### 3.4 Robot–Robot Interaction

- `R_ROBOT_SENSE = 2.0`
- `K_REP = 18`
- `K_TANGENT_SCALE = 0.9`
- `TANGENT_HYSTERESIS = 0.3`

### 3.5 Wall Interaction

- `R_WALL_SENSE = 2.0`
- `K_WALL = 2`
- `NEAR_TARGET_WALL_SCALE = 0.30`

### 3.6 Target-Point Attraction

- `K_ATT = 10`
- `ATT_FORCE_MAX = 8`
- `ATT_NEAR_BOOST_RADIUS = 1.5`
- `ATT_NEAR_BOOST = 0.8`
- `ATT_TOTAL_MAX = 6`
- `ATT_LEFT_GRADIENT_GAIN = 1.5`

### 3.7 Distance Gating for Target Attraction (Important)

- `TARGET_ATTRACTION_FULL_DISTANCE = 5.0`
- `TARGET_ATTRACTION_ZERO_DISTANCE = 10.0`

Interpretation:

- if robot-line distance `<= 5.0`, target attraction is full strength,
- if distance `>= 10.0`, target attraction is zero,
- between 5 and 10, smoothstep decay is used.

### 3.8 Occupied-Point Repulsion (Coverage Masking)

- `PARKED_POINT_REPULSE_SCALE = 0.9`
- `PARKED_POINT_REPULSE_RANGE = 0.9 * ROBOT_DIAMETER = 2.7`

### 3.9 Parking / Completion

- `PARK_DISTANCE_THRESHOLD = 1.0`
- `TARGET_STOP_DISTANCE = 0.15`
- `TARGET_REACHED_EPS = 1e-6`

### 3.10 Safety and Runtime

- `MAX_STEP_BACKOFF_ITERS = 10`
- `SAFETY_EPS = 1e-4`
- `MAX_TIMESTEPS = 10000`
- `ANIMATION_INTERVAL_MS = 30`

### 3.11 Target Point Sampling

- `TARGET_POINT_SPACING = ROBOT_DIAMETER = 3.0`

### 3.12 Present but Inactive Cue Parameters

These are retained for compatibility but disabled in behavior:

- `CUE_FORCE_MAX = 0`
- `CUE_K_ATT = 0`
- `CUE_LINE_LENGTH = 0`
- `CUE_SENSE_RADIUS = 12.0`
- `CUE_ACTIVATION_RADIUS = 30.0`

---

## 4) Core Geometry Primitive: Point-to-Segment Distance

Used for:

- wall clearance,
- distance to target line,
- projected nearest points.

For point `p=(px,py)` and segment endpoints `a=(x1,y1)`, `b=(x2,y2)`:

1. `d = b-a`, `l2 = ||d||^2`
2. if `l2 == 0`, segment is degenerate, distance is `||p-a||`
3. projection scalar:

\[
t = \text{clip}\left(\frac{(p-a)\cdot d}{\|d\|^2}, 0, 1\right)
\]

4. projection point:

\[
p\_{proj} = a + td
\]

5. distance:

\[
\rho = \|p - p\_{proj}\|
\]

---

## 5) Force Model: Top-Level Composition

For each unparked robot:

\[
F*{total} = F*{random} + F*{robot} + F*{wall} + F\_{target}
\]

where each term is computed below.

---

## 6) Random Exploration Term

Let `center_dist` be robot distance to target line.

First:

\[
\text{approach_ratio} = \min\left(1,\frac{center_dist}{TARGET_APPROACH_RADIUS}\right)
\]

With `TARGET_APPROACH_RADIUS = 7.0`.

Then random ratio:

\[
\text{random_ratio} = RANDOM_NEAR_LINE_FLOOR + (1-RANDOM_NEAR_LINE_FLOOR)\cdot\text{approach_ratio}
\]

So near line, randomness shrinks toward floor (`0.1`), far from line it approaches `1.0`.

Heading update probability:

\[
P\_{turn} = \min(0.95, TURN_PROBABILITY\cdot\text{random_ratio})
\]

If random event fires, heading angle is resampled uniformly on `[0,2\pi)`.

Random force scale:

\[
S\_{rand} = \min(RANDOM_FORCE_MAX_SCALE, RANDOM_FORCE_SCALE\cdot\text{random_ratio})
\]

Random vector:

\[
F*{random} = [\cos\theta,\sin\theta]\cdot ROBOT_SPEED\cdot S*{rand}
\]

---

## 7) Robot–Robot Repulsion + Tangential Sliding

For each other robot `j`:

1. center distance:

\[
d\_{ij}=\|p_i-p_j\|
\]

2. effective clearance:

\[
\rho*{ij,eff}=\max(d*{ij}-(r_i+r_j),10^{-3})
\]

In this code, both radii are equal to `r`, so `r_i+r_j = 2r`.

3. active if inside sensing bubble:

\[
\rho*{ij,eff}<R*{bubble},\quad R\_{bubble}=other.R_robot_sense
\]

4. repulsion magnitude:

\[
m*{rep}=K\_{REP}\left(\frac{1}{\rho*{ij,eff}}-\frac{1}{R*{bubble}}\right)\frac{1}{\rho*{ij,eff}^{3/2}}
\]

5. outward unit direction (if non-degenerate):

\[
\hat g = \frac{p_i-p_j}{\|p_i-p_j\|}
\]

Repulsion increment:

\[
\Delta F*{rep}=m*{rep}\hat g
\]

### Tangential sliding term

Two orthogonal tangents to `\hat g`:

\[
t_1=[-\hat g_y,\hat g_x],\quad t_2=[\hat g_y,-\hat g_x]
\]

Guidance vector toward current guidance point:

\[
v*t = p*{guide} - p_i
\]

Compute alignment scores: `dot1 = t1·v_t`, `dot2 = t2·v_t`.

- if `|dot1-dot2| < TANGENT_HYSTERESIS`, reuse previous tangent sign,
- else choose tangent with larger dot score.

Tangential increment:

\[
\Delta F*{tan}=m*{rep}\cdot K\_{TANGENT_SCALE}\cdot t\_{chosen}
\]

Total robot interaction is sum of all `\Delta F_rep + \Delta F_tan`.

---

## 8) Wall Repulsion

Walls include:

1. world bounds (4 segments),
2. U-obstacle inner walls (3 segments).

For each wall segment:

1. distance from robot center to segment: `\rho`
2. effective clearance from robot body:

\[
\rho\_{eff}=\max(\rho-r,10^{-3})
\]

3. active if:

\[
\rho\_{eff}<R\_{WALL_SENSE}
\]

4. magnitude:

\[
m*{wall}=K\_{WALL}\left(\frac{1}{\rho*{eff}}-\frac{1}{R\_{WALL_SENSE}}\right)\frac{1}{\rho\_{eff}^{3/2}}
\]

5. direction is normalized `(p - p_proj)` (outward from wall).

Summed across all active walls gives `F_wall_raw`.

### Near-target wall scaling

After summation, wall force is scaled by:

\[
wall_scale = NEAR_TARGET_WALL_SCALE + (1-NEAR_TARGET_WALL_SCALE)\cdot\text{approach_ratio}
\]

Then:

\[
F*{wall}=wall_scale\cdot F*{wall_raw}
\]

Behavior:

- near line (`approach_ratio` small): weaker wall push (less jitter around target region),
- far from line (`approach_ratio≈1`): near full wall strength.

---

## 9) Target-Point Attraction (with Occupancy Masking)

### 9.1 Dotted Target Points

Generated on line `y = 40` with spacing `TARGET_POINT_SPACING = d`:

- first x: `left_x + r`
- last x: `right_x - r`

Let span:

\[
span = (right_x-r) - (left_x+r)
\]

Point count:

\[
N = \left\lfloor\frac{span}{TARGET_POINT_SPACING}\right\rfloor + 1
\]

Point coordinates:

\[
x_i = (left_x+r) + i\cdot TARGET_POINT_SPACING,\quad i\in[0,N-1]
\]

\[
p_i = (x_i, line_y)
\]

### 9.2 Endpoint-Heavy Weight Profile

For `N>1`:

\[
w_i = \left|2\cdot\frac{i}{N-1} - 1\right|
\]

So both endpoints get weight near `1`, center gets near `0`.

### 9.3 Distance-Based Target Attenuation (line-level gate)

Let `d_line = center_dist` (distance from robot center to target line segment).

Parameters:

- `d_full = TARGET_ATTRACTION_FULL_DISTANCE = 5`
- `d_zero = TARGET_ATTRACTION_ZERO_DISTANCE = 10`

Piecewise attenuation:

\[
A(d*{line})=
\begin{cases}
1, & d*{line} \le d*{full}\\
0, & d*{line} \ge d\_{zero}\\
1-s(t), & \text{otherwise}
\end{cases}
\]

where

\[
t = \frac{d*{line}-d*{full}}{d*{zero}-d*{full}},\quad s(t)=3t^2-2t^3
\]

`s(t)` is smoothstep (C1-smooth at boundaries).

Interpretation:

- below 5 units: full target influence,
- above 10 units: zero target influence,
- 5..10: smooth fade-out.

### 9.4 Per-point force before attenuation

For each target point `p_i`:

Distance and unit direction to robot:

\[
d_i=\|p_i-p\|,
\quad \hat u_i = \frac{p_i-p}{d_i}
\]

Base magnitude:

\[
M\_{base,i}=ATT_FORCE_MAX\cdot\tanh(K\_{ATT}\cdot d_i)
\]

Near boost:

\[
near_ratio_i=\max\left(0,1-\frac{d_i}{ATT_NEAR_BOOST_RADIUS}\right)
\]

\[
M\_{near,i}=ATT_NEAR_BOOST\cdot near_ratio_i
\]

Gradient multiplier:

\[
G_i = 1 + ATT_LEFT_GRADIENT_GAIN\cdot w_i
\]

Clamped point magnitude:

\[
M*i = \min\left(ATT_TOTAL_MAX, (M*{base,i}+M\_{near,i})\cdot G_i\right)
\]

### 9.5 Occupied-point sign handling

If point `i` is unoccupied:

\[
sign_i = +1
\]

If occupied by this same robot: skipped.

If occupied by another robot:

- only active when `d_i <= PARKED_POINT_REPULSE_RANGE`,
- sign becomes repulsive:

\[
sign_i = -PARKED_POINT_REPULSE_SCALE
\]

Raw summed target field:

\[
F\_{target,raw} = \sum_i sign_i\cdot \hat u_i\cdot M_i
\]

Final target field with line-level attenuation:

\[
F*{target}=A(d*{line})\cdot F\_{target,raw}
\]

This is the key mechanism ensuring far-from-line robots are primarily exploration + repulsion driven.

---

## 10) Guidance Point (Used by Tangential Robot Avoidance)

For each robot, candidate uncovered index set:

\[
U = \{i \mid point_occupied_i = False\}
\]

Selected guidance index:

\[
i^\* = \arg\min\_{i\in U} \frac{\|p-p_i\|}{\max(0.1,1+w_i)}
\]

This is not a hard assignment; it only supplies a direction reference for local tangent decisions.

---

## 11) Velocity and Motion Update

After computing `F_total`:

\[
\|F\| = \|F\_{total}\|
\]

Approach speed scale:

\[
S\_{approach}=\max\left(0.25,\min\left(1,\frac{center_dist}{TARGET_APPROACH_RADIUS}\right)\right)
\]

Max speed this step:

\[
v*{max}=ROBOT_SPEED\cdot S*{approach}
\]

Desired velocity:

\[
v*{des}=
\begin{cases}
\frac{F*{total}}{\|F*{total}\|}\cdot\min(\|F*{total}\|,v*{max}), & \|F*{total}\|>0\\
0, & \text{otherwise}
\end{cases}
\]

Smoothing:

\[
v*{blend}=VELOCITY_SMOOTHING\cdot v*{prev} + (1-VELOCITY_SMOOTHING)\cdot v\_{des}
\]

If `||v_blend|| > v_max`, normalize to `v_max`.

Candidate displacement is `v_blend` for this discrete step.

---

## 12) Predictive Safety Filter (No Overlap / No Wall Penetration)

Before applying displacement, `_safe_displacement` tests candidate step against `_is_valid_position`.

### 12.1 Validity checks for candidate `(x,y)`

1. World bounds for full body:

\[
r-\epsilon \le x \le W-r+\epsilon,
\quad r-\epsilon \le y \le H-r+\epsilon
\]

2. For every wall segment, body clearance:

\[
dist((x,y),segment) \ge r-\epsilon
\]

3. For every other robot `j`:

\[
\| (x,y)-p_j \| \ge (r+r_j)-\epsilon
\]

### 12.2 Backoff loop

If candidate invalid, repeatedly halve displacement up to `MAX_STEP_BACKOFF_ITERS`.

- first valid halved candidate is applied,
- if none valid, zero displacement is applied.

This is the hard geometric safety layer independent of APF quality.

---

## 13) Parking Logic and Coverage State

After all robots update motion in a step, environment attempts parking.

For each unparked robot:

1. choose nearest uncovered weighted point using same score as guidance,
2. compute distance to that point,
3. add candidate `(distance, robot, point_index)`.

Candidates are sorted by distance (closest first).

A candidate parks if `_can_park_at_point` is true:

1. point exists and is still unoccupied,
2. robot-point distance `<= PARK_DISTANCE_THRESHOLD`,
3. snapping to point would still satisfy body non-overlap with all other robots.

When parked:

- position snaps to point center,
- velocity becomes zero,
- `parked = True`,
- `parked_target_index` stored,
- robot sensing bubble shrinks: `R_robot_sense = 0.15`,
- occupancy arrays updated.

---

## 14) Completion Condition

Simulation ends when:

\[
\text{all target points occupied} \;\lor\; \text{all robots parked}
\]

or when timestep reaches `MAX_TIMESTEPS`.

---

## 15) Debug Overlay Semantics

Clicking a robot displays:

- selected index and parked state,
- position and velocity,
- sensing radii and near-boost radius,
- `line_dist` (`center_dist` to target line),
- `target_attn` (`A(d_line)` from section 9.3),
- magnitudes of `F_random`, `F_robot`, `F_wall`, `F_target`, `F_total`,
- `turn_p` and `rand_scale`.

Vector lines drawn from robot center correspond exactly to last computed force component vectors.

This telemetry is intended to diagnose force-balance pathologies (for example, high wall vs target cancellation or far-field target leakage).

---

## 16) Dependency Graph of Key Expressions

### 16.1 High-level flow

`constants` → `derived geometry` → `distance primitives` → `component forces` → `F_total` → `velocity` → `safe displacement` → `position update` → `parking` → `occupancy`

### 16.2 Critical dependency chains

1. `center_dist`
   - drives `approach_ratio`
   - drives `random_ratio` → `turn_probability`, `random_scale`
   - drives `wall_scale`
   - drives `target_attenuation`
   - drives `approach_speed_scale`

2. `point_weights`
   - generated from index profile
   - used in guidance score denominator
   - used in target force gradient multiplier

3. `point_occupied` + `point_occupants`
   - change sign/range behavior in target field,
   - define completion condition.

---

## 17) Practical Interpretation of the New Distance Gate

Because the target field sums across many points, raw aggregate attraction can be large even when each individual point is bounded.
The attenuation function fixes this by multiplying the entire target field with a line-distance gate:

- above 10 units from target line: `F_target = 0`,
- between 5 and 10: smooth reduction,
- within 5: full attraction.

So far-from-line behavior is dominated by exploration and local collision avoidance, while near-line behavior remains goal-directed.

---

## 18) Run Instructions

From workspace folder:

```bash
python single_robot_target.py
```

The window closes when completion condition or timestep limit is reached.

---

## 19) Notes on Model Scope

- This is a local APF controller with geometric safety backoff.
- It is stochastic because of heading randomization.
- Convergence patterns can vary by seed and parameter interactions.
- Safety guarantees are geometric (candidate validation), not Lyapunov guarantees.
