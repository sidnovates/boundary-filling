# Swarm Robotics Simulation Parameters

This document details all the absolute dimensions, coordinates, and behavior properties configured in the current random walk simulation (`random_walk.py`).

## 1. Global Environment
- **Total Area Width:** `100` units
- **Total Area Height:** `100` units
- **Coordinate Space:** `X` goes from `0` to `100`, `Y` goes from `0` to `100`
- **Outer Bound Walls:** Solid continuous lines located at $x=0$, $x=100$, $y=0$, and $y=100$.

## 2. Inner "U-Shaped" Obstacle
The obstacle is defined by three solid walls and an explicit open side on the bottom, allowing robots to enter and exit.

**Wall coordinates (`x_start, y_start` to `x_end, y_end`):**
- **Left Wall (Vertical):** `(35, 40)` to `(35, 80)`
- **Right Wall (Vertical):** `(65, 40)` to `(65, 80)`
- **Top Wall (Horizontal):** `(35, 80)` to `(65, 80)`
- **Bottom Open Side (Dashed):** `(35, 40)` to `(65, 40)`

**Obstacle Dimensions:**
- **Width:** `30` units ($65 - 35$)
- **Height:** `40` units ($80 - 40$)
- **Location:** Centered horizontally, situated slightly towards the top half of the area.

## 3. Robot Properties
- **Total Number of Robots:** `15`
- **Spawn Behavior:** Robots spawn randomly anywhere in the environment except the 5 closest units to the outer walls.
- **Physical Radius:** `1.5` units (Used for collision calculations against walls and each other)
- **Visual Display Area:** `50` units (Used for plotting circle sizes on the screen)
- **Movement Speed:** `0.5` units traveled per timestep.
- **Initial Direction Angle:** Random heading chosen uniformly between `0` and `2π` ($360^\circ$).

## 4. Simulation Behaviors
- **Total Timesteps:** `1000`
- **Random Walk Turn Probability:** `10%` (`0.1`) chance every timestep that a robot will instantly pick a completely new random direction.
- **Wall Collisions:** Handled dynamically via Artificial Potential Fields (APF).
- **Robot Collisions:** Handled dynamically via Artificial Potential Fields (APF).

## 5. View Configuration (Playback)
- **Animation Speed Interval:** `30 ms` duration per drawn frame. Wait time for matplotlib before updating locations.

## 6. Collision Algorithm (Artificial Potential Fields)
The simulation uses continuous APF physics to achieve smooth collision avoidance without hard bouncing.
- **Robot Sensing Radius ($R_{robot}$):** `8.0` units (Distance from robot edge to another robot edge where repulsion starts).
- **Wall Sensing Radius ($R_{wall}$):** `8.0` units (Distance from robot edge to wall where repulsion starts).
- **Robot Repulsion Gain ($k_{rep}$):** `50.0` (Multiplier for force pushing robots apart).
- **Wall Repulsion Gain ($k_{wall}$):** `50.0` (Multiplier for force pushing robots away from walls).

## 7. Compute Target Attraction
The target attraction force primarily considered Oussama Khatib's exact 1986 mathematical formulation (a purely parabolic well resulting in a linear spring-like attractive force $F_{att} = -k_{att}(x - x_d)$). 

However, this exact textbook implementation was replaced with a bounded attractive field utilizing the `math.tanh` function.

**Reasoning:** Because the pure Khatib formulation scales linearly with distance, placing a target very far away resulted in an overwhelmingly strong attractive force. This massive pull overpowered the local obstacle repulsive fields, causing the robot to continuously crash directly into the closest wall of the U-shaped obstacle instead of smoothly navigating around the border. The `tanh` function introduces a "speed limit" on the attractive force, capping its maximum magnitude so the robot is gently pulled toward the target while still fully respecting the obstacle boundaries.
