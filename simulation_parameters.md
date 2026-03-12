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
- **Wall Collisions:** Robots execute a ray-reflection style deterministic bounce when their physical radius overlaps a wall.
- **Robot Collisions:** If the distance between two robot centers becomes less than twice their radius ($d \le 2R$, physically overlapping), they instantly bounce away by flipping their angle $180^\circ$ ($+π$). 

## 5. View Configuration (Playback)
- **Animation Speed Interval:** `30 ms` duration per drawn frame. Wait time for matplotlib before updating locations.
