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

## 8. Tangential Sliding Force (Local Minima Evasion)
To prevent robots from getting trapped in "local minima" (such as being perfectly balanced pushing directly into a corner, or directly underneath another robot blocking a doorway), a **Tangential Force** modifier is implemented for both Wall Repulsion and Robot Repulsion.
- **Mechanism:** When a repulsive normal vector ($n$) is calculated pointing strictly away from an obstacle, two possible perpendicular tangential vectors exist in 2D space ($t_1$ and $t_2$). The algorithm evaluates which of these two tangents points most strongly toward the center of the target destination using a mathematical dot product. 
- **Effect:** A sliding force equal to the repulsion magnitude scaled by `K_TANGENT_SCALE` (currently `2.0`) is added in that direction. This smoothly forces the robots to peacefully "skirt" around the edges of walls and softly orbit around other stationary robots, breaking gridlocks and preventing them from pushing dead-center against invisible walls.

## 9. Dynamic Repulsion Shrinkage ("Swarm Parking" Logic)
When multiple robots approach the target line simultaneously, the standard robot-robot repulsion field (`R_ROBOT_SENSE` = 2.0 units) creates a massive invisible barrier blocking the entrance. When 2 robots stop at the finish line, their bubbles merge and "clog the doorway" for latecomers.
- **Mechanism:** To solve this without completely disabling physical collision detection, a robot's sensing radius (`R_robot_sense`) is dynamically reduced from `2.0` down to a tiny `0.1` the moment it successfully touches the target line and transitions into a "parked" state. The standard collision algorithm was modified to base its calculation on the *receiving* robot's sensing radius rather than its own, so approaching active robots do not calculate a massive repulsion force from parked ones.
- **Effect:** The parked robot physically occupies its hard-fixed spot on the line (so trailing robots will still experience an infinite repulsion spike if they try to squeeze directly into its physical coordinate). However, its outer invisible "force field bubble" instantly collapses. This allows trailing robots to gracefully slide past the parked robots and find their own empty slot on the remainder of the line without being violently pushed backward.
