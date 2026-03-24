"""
Swarm Robotics: Single Robot Targeting with APF
This simulation keeps the original 100x100 environment and U-shaped obstacle,
but uses a single robot with an attractive field to the first point of the
target line. The run stops when the robot reaches that target point.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import math

# ====== Configurable Parameters ======
ENV_WIDTH = 100  # Total environment width (units)
ENV_HEIGHT = 100  # Total environment height (units)

NUM_ROBOTS = 10  # Spawn 4 robots

ROBOT_RADIUS = 1.5  # Physical robot radius (units)
ROBOT_DIAMETER = 2 * ROBOT_RADIUS  # Physical robot diameter (units)
ROBOT_SPEED = 0.5  # Max robot speed per timestep
TURN_PROBABILITY = 0.24  # Base chance each step to randomize heading

R_ROBOT_SENSE = 2.0  # Robot-robot repulsion sensing radius
R_WALL_SENSE = 2.0  # Wall repulsion sensing radius
K_REP = 18  # Robot-robot repulsion gain
K_WALL = 2 # Wall repulsion gain
K_TANGENT_SCALE = 0.9  # Multiplier for the tangential "sliding" force to skirt around walls
TANGENT_HYSTERESIS = 0.3  # Keep prior tangent direction when options are nearly equal
K_ATT = 10  # Attractive gain shaping how quickly attraction ramps with distance
ATT_FORCE_MAX = 8  # Upper bound on attractive force magnitude for more natural motion
ATT_NEAR_BOOST_RADIUS = 1.5  # Radius around target where point attraction gets a stronger boost
ATT_NEAR_BOOST = 0.8  # Extra near-target attraction to prevent circling around the point
ATT_TOTAL_MAX = 6  # Hard cap on total point-attraction magnitude (base + near boost)
ATT_LEFT_GRADIENT_GAIN = 1.5  # Attraction gain for point-weight profile (used with double gradient)
PARKED_POINT_REPULSE_SCALE = 0.9  # Occupied target points emit equal-strength opposite-sign field
PARKED_POINT_REPULSE_RANGE = 0.9 * ROBOT_DIAMETER  # Occupied-point repulsion stays within local point neighborhood

TARGET_WIDTH_MULTIPLE = 10  # Target/obstacle span as integer multiple of robot diameter
TARGET_LINE_WIDTH = TARGET_WIDTH_MULTIPLE * ROBOT_DIAMETER  # Resulting target line width (units)

OBSTACLE_CENTER_X = ENV_WIDTH / 2.0  # Horizontal center of U-shaped obstacle
OBSTACLE_BOTTOM_Y = 40.0  # Bottom y-coordinate of obstacle legs
OBSTACLE_TOP_Y = 80.0  # Top y-coordinate of obstacle
OBSTACLE_LEFT_X = OBSTACLE_CENTER_X - TARGET_LINE_WIDTH / 2.0  # Left x-coordinate of obstacle
OBSTACLE_RIGHT_X = OBSTACLE_CENTER_X + TARGET_LINE_WIDTH / 2.0  # Right x-coordinate of obstacle

TARGET_LINE = ((OBSTACLE_LEFT_X, OBSTACLE_BOTTOM_Y), (OBSTACLE_RIGHT_X, OBSTACLE_BOTTOM_Y))  # Open bottom side
TARGET_POINT_SPACING = ROBOT_DIAMETER  # Spacing between dotted target points on the target line

CUE_LINE_LENGTH = 0  # Length of perpendicular guide line leading into the target point
CUE_LINE = ((TARGET_LINE[0][0], TARGET_LINE[0][1] - CUE_LINE_LENGTH), TARGET_LINE[0])  # Kept for compatibility (unused)
CUE_FORCE_MAX = 0  # Lesser attraction strength for cue guidance (weaker than point field)
CUE_K_ATT = 0  # Cue-field gain shaping pull toward the guide line
CUE_SENSE_RADIUS = 12.0  # Only apply cue attraction when robot is close to the cue line
CUE_ACTIVATION_RADIUS = 30.0  # Activate cue field only when robot is reasonably near the target

SPAWN_BEHIND_U_Y_OFFSET = 10.0  # Spawn offset above the U's closed horizontal top wall
SPAWN_POINT = (OBSTACLE_CENTER_X, OBSTACLE_TOP_Y + SPAWN_BEHIND_U_Y_OFFSET)  # Spawn behind closed side of U

TARGET_APPROACH_RADIUS = 7.0  # Distance at which near-target smoothing starts
TARGET_STOP_DISTANCE = 0.15  # Distance threshold to snap to target and stop
NEAR_TARGET_WALL_SCALE = 0.30  # Minimum wall-repulsion scale near target to reduce jitter

PARK_DISTANCE_THRESHOLD = 1.0  # Distance threshold to claim a free target point and park
VELOCITY_SMOOTHING = 0.65  # Fraction of previous velocity kept each step
RANDOM_FORCE_SCALE = 1.0  # Base random-force scale relative to robot speed
RANDOM_FORCE_MAX_SCALE = 10  # Hard cap on random-force scaling multiplier
RANDOM_NEAR_LINE_FLOOR = 0.1  # Keep only a small fraction of exploration near the target line
TARGET_ATTRACTION_FULL_DISTANCE = 5.0  # Full target attraction when line distance is within this range
TARGET_ATTRACTION_ZERO_DISTANCE = 10.0  # Target attraction smoothly decays to zero by this line distance
MAX_STEP_BACKOFF_ITERS = 10  # Max halving iterations for collision-safe step backoff
SAFETY_EPS = 1e-4  # Numerical safety epsilon for geometric checks

MAX_TIMESTEPS = 10000  # Safety cap to avoid infinite run
ANIMATION_INTERVAL_MS = 30  # Milliseconds between frames
TARGET_REACHED_EPS = 1e-6  # Numerical tolerance for target-reached check


class Robot:
    """
    Represents a single swarm robot.
    """
    def __init__(self, x, y, env_width, env_height, radius=ROBOT_RADIUS):
        self.position = [x, y]
        self.env_width = env_width
        self.env_height = env_height
        self.radius = radius

        self.speed = ROBOT_SPEED
        self.angle = random.uniform(0, 2 * math.pi)
        self.turn_probability = TURN_PROBABILITY

        self.R_robot_sense = R_ROBOT_SENSE
        self.R_wall_sense = R_WALL_SENSE
        self.k_rep = K_REP
        self.k_wall = K_WALL
        self.k_tangent_scale = K_TANGENT_SCALE
        self.tangent_hysteresis = TANGENT_HYSTERESIS
        self.k_att = K_ATT
        self.att_force_max = ATT_FORCE_MAX
        self.att_near_boost_radius = ATT_NEAR_BOOST_RADIUS
        self.att_near_boost = ATT_NEAR_BOOST
        self.att_total_max = ATT_TOTAL_MAX
        self.att_left_gradient_gain = ATT_LEFT_GRADIENT_GAIN
        self.cue_force_max = CUE_FORCE_MAX
        self.cue_k_att = CUE_K_ATT
        self.cue_sense_radius = CUE_SENSE_RADIUS
        self.cue_activation_radius = CUE_ACTIVATION_RADIUS

        self.target_approach_radius = TARGET_APPROACH_RADIUS
        self.target_stop_distance = TARGET_STOP_DISTANCE
        self.near_target_wall_scale = NEAR_TARGET_WALL_SCALE
        self.velocity_smoothing = VELOCITY_SMOOTHING
        self.random_force_scale = RANDOM_FORCE_SCALE
        self.random_force_max_scale = RANDOM_FORCE_MAX_SCALE
        self.random_near_line_floor = RANDOM_NEAR_LINE_FLOOR
        self.target_attraction_full_distance = TARGET_ATTRACTION_FULL_DISTANCE
        self.target_attraction_zero_distance = TARGET_ATTRACTION_ZERO_DISTANCE

        self.velocity = np.zeros(2)
        self.parked = False
        self.parked_target_index = None
        self.last_tangent_sign = 1
        self.last_force_components = {
            "random": np.zeros(2),
            "robot_repulsion": np.zeros(2),
            "wall_repulsion": np.zeros(2),
            "target_points": np.zeros(2),
            "total": np.zeros(2),
            "turn_probability": 0.0,
            "random_scale": 0.0,
            "approach_ratio": 0.0,
            "target_attenuation": 0.0,
            "line_distance": 0.0,
        }

    def _dist_to_segment(self, px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx * dx + dy * dy
        if l2 == 0:
            return math.hypot(px - x1, py - y1), x1, y1
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y), proj_x, proj_y

    def compute_wall_repulsion(self, inner_walls, target_line):
        f_wall = np.zeros(2)

        walls = [
            (0, 0, self.env_width, 0),
            (0, self.env_height, self.env_width, self.env_height),
            (0, 0, 0, self.env_height),
            (self.env_width, 0, self.env_width, self.env_height),
        ]
        for wall in inner_walls:
            walls.append((wall[0][0], wall[0][1], wall[1][0], wall[1][1]))

        for x1, y1, x2, y2 in walls:
            dist, proj_x, proj_y = self._dist_to_segment(self.position[0], self.position[1], x1, y1, x2, y2)

            effective_dist = dist - self.radius
            if effective_dist < 0.001:
                effective_dist = 0.001

            if effective_dist < self.R_wall_sense:
                magnitude = self.k_wall * (1.0 / effective_dist - 1.0 / self.R_wall_sense) * (1.0 / (effective_dist ** (3/2)))

                if dist > 0.001:
                    grad_rho = np.array([
                        (self.position[0] - proj_x) / dist,
                        (self.position[1] - proj_y) / dist
                    ])
                else:
                    grad_rho = np.array([random.uniform(-1, 1), random.uniform(-1, 1)])

                f_wall += magnitude * grad_rho

        return f_wall

    def compute_robot_repulsion(self, other_robots, guidance_point):
        f_rep = np.zeros(2)
        
        tx = guidance_point[0] - self.position[0]
        ty = guidance_point[1] - self.position[1]

        for other in other_robots:
            if other is self:
                continue

            dist = math.hypot(self.position[0] - other.position[0], self.position[1] - other.position[1])
            effective_dist = dist - (2 * self.radius)

            if effective_dist < 0.001:
                effective_dist = 0.001

            bubble_radius = other.R_robot_sense

            if effective_dist < bubble_radius:
                magnitude = self.k_rep * (1.0 / effective_dist - 1.0 / bubble_radius) * (1.0 / (effective_dist ** (3/2)))

                if dist > 0.001:
                    grad_rho = np.array([
                        (self.position[0] - other.position[0]) / dist,
                        (self.position[1] - other.position[1]) / dist
                    ])
                else:
                    grad_rho = np.array([random.uniform(-1, 1), random.uniform(-1, 1)])

                f_rep += magnitude * grad_rho
                
                # Add Tangential Force (Sliding around other robots)
                t1 = np.array([-grad_rho[1], grad_rho[0]])
                t2 = np.array([grad_rho[1], -grad_rho[0]])
                
                target_vector = np.array([tx, ty])
                dot1 = np.dot(t1, target_vector)
                dot2 = np.dot(t2, target_vector)
                if abs(dot1 - dot2) < self.tangent_hysteresis:
                    tangent = t1 if self.last_tangent_sign >= 0 else t2
                else:
                    if dot1 > dot2:
                        tangent = t1
                        self.last_tangent_sign = 1
                    else:
                        tangent = t2
                        self.last_tangent_sign = -1
                    
                f_rep += magnitude * self.k_tangent_scale * tangent

        return f_rep

    def _compute_target_attenuation(self, line_distance):
        full_d = max(0.0, self.target_attraction_full_distance)
        zero_d = max(full_d + 1e-9, self.target_attraction_zero_distance)

        if line_distance <= full_d:
            return 1.0
        if line_distance >= zero_d:
            return 0.0

        t = (line_distance - full_d) / (zero_d - full_d)
        smoothstep = t * t * (3.0 - 2.0 * t)
        return 1.0 - smoothstep

    def compute_target_points_field(self, target_points, point_weights, point_occupied, point_occupants, target_attenuation):
        if target_attenuation <= 0.0:
            return np.zeros(2)

        field = np.zeros(2)
        for idx, point in enumerate(target_points):
            dx = point[0] - self.position[0]
            dy = point[1] - self.position[1]
            dist = math.hypot(dx, dy)
            if dist <= 0.001:
                continue

            ux = dx / dist
            uy = dy / dist
            base_magnitude = self.att_force_max * math.tanh(self.k_att * dist)
            near_ratio = max(0.0, 1.0 - dist / self.att_near_boost_radius)
            near_magnitude = self.att_near_boost * near_ratio
            gradient_boost = 1.0 + self.att_left_gradient_gain * point_weights[idx]
            magnitude = min(self.att_total_max, (base_magnitude + near_magnitude) * gradient_boost)

            if point_occupied[idx]:
                if point_occupants[idx] is self:
                    continue
                if dist > PARKED_POINT_REPULSE_RANGE:
                    continue
                sign = -PARKED_POINT_REPULSE_SCALE
            else:
                sign = 1.0

            field += sign * np.array([ux, uy]) * magnitude

        return field * target_attenuation

    def _build_wall_segments(self, inner_walls):
        walls = [
            (0, 0, self.env_width, 0),
            (0, self.env_height, self.env_width, self.env_height),
            (0, 0, 0, self.env_height),
            (self.env_width, 0, self.env_width, self.env_height),
        ]
        for wall in inner_walls:
            walls.append((wall[0][0], wall[0][1], wall[1][0], wall[1][1]))
        return walls

    def _min_wall_clearance(self, inner_walls):
        walls = self._build_wall_segments(inner_walls)
        min_clearance = float("inf")
        for x1, y1, x2, y2 in walls:
            dist, _, _ = self._dist_to_segment(self.position[0], self.position[1], x1, y1, x2, y2)
            min_clearance = min(min_clearance, dist - self.radius)
        return max(0.0, min_clearance)

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

    def compute_force_components(self, other_robots, inner_walls, target_line, guidance_point, target_points, point_weights, point_occupied, point_occupants):
        center_dist, _, _ = self._dist_to_segment(
            self.position[0], self.position[1],
            target_line[0][0], target_line[0][1],
            target_line[1][0], target_line[1][1]
        )
        approach_ratio = min(1.0, center_dist / self.target_approach_radius)
        random_ratio = self.random_near_line_floor + (1.0 - self.random_near_line_floor) * approach_ratio

        turn_probability = min(0.95, self.turn_probability * random_ratio)
        if random.random() < turn_probability:
            self.angle = random.uniform(0, 2 * math.pi)
        random_scale = min(self.random_force_max_scale, self.random_force_scale * random_ratio)
        f_random = np.array([math.cos(self.angle), math.sin(self.angle)]) * self.speed * random_scale

        f_robot = self.compute_robot_repulsion(other_robots, guidance_point)
        f_wall = self.compute_wall_repulsion(inner_walls, target_line)
        wall_scale = self.near_target_wall_scale + (1.0 - self.near_target_wall_scale) * approach_ratio
        f_wall *= wall_scale
        target_attenuation = self._compute_target_attenuation(center_dist)
        f_target_points = self.compute_target_points_field(
            target_points,
            point_weights,
            point_occupied,
            point_occupants,
            target_attenuation,
        )
        f_total = f_random + f_robot + f_wall + f_target_points

        return {
            "random": f_random,
            "robot_repulsion": f_robot,
            "wall_repulsion": f_wall,
            "target_points": f_target_points,
            "total": f_total,
            "turn_probability": turn_probability,
            "random_scale": random_scale,
            "approach_ratio": approach_ratio,
            "target_attenuation": target_attenuation,
            "line_distance": center_dist,
        }

    def update(self, env):
        if self.parked:
            self.velocity = np.zeros(2)
            self.last_force_components = {
                "random": np.zeros(2),
                "robot_repulsion": np.zeros(2),
                "wall_repulsion": np.zeros(2),
                "target_points": np.zeros(2),
                "total": np.zeros(2),
                "turn_probability": 0.0,
                "random_scale": 0.0,
                "approach_ratio": 0.0,
                "target_attenuation": 0.0,
                "line_distance": 0.0,
            }
            return

        guidance_point = env.get_guidance_point(self)

        center_dist, _, _ = self._dist_to_segment(
            self.position[0], self.position[1],
            env.target_line[0][0], env.target_line[0][1],
            env.target_line[1][0], env.target_line[1][1],
        )

        force_components = self.compute_force_components(
            env.robots,
            env.inner_walls,
            env.target_line,
            guidance_point,
            env.target_points,
            env.target_point_weights,
            env.target_point_occupied,
            env.target_point_occupants,
        )
        self.last_force_components = force_components
        f_total = force_components["total"]

        magnitude = np.linalg.norm(f_total)
        approach_speed_scale = max(0.25, min(1.0, center_dist / self.target_approach_radius))
        max_speed = self.speed * approach_speed_scale

        if magnitude > 0:
            desired_velocity = (f_total / magnitude) * min(magnitude, max_speed)
        else:
            desired_velocity = np.zeros(2)

        blended_velocity = self.velocity_smoothing * self.velocity + (1.0 - self.velocity_smoothing) * desired_velocity
        speed = np.linalg.norm(blended_velocity)
        if speed > max_speed and speed > 0:
            blended_velocity = (blended_velocity / speed) * max_speed

        safe_step = self._safe_displacement(blended_velocity, env.robots, env.inner_walls)
        self.position[0] += safe_step[0]
        self.position[1] += safe_step[1]
        self.velocity = safe_step

        if np.linalg.norm(safe_step) > 0.001:
            self.angle = math.atan2(safe_step[1], safe_step[0])

    def has_reached_line(self, target_line):
        dist_to_line, _, _ = self._dist_to_segment(
            self.position[0], self.position[1],
            target_line[0][0], target_line[0][1],
            target_line[1][0], target_line[1][1]
        )
        return dist_to_line <= self.target_stop_distance + TARGET_REACHED_EPS


class Environment:
    """
    Represents the 100x100 2D environment with one robot and U-shaped obstacle.
    """
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
        self.target_point_weights = []
        self.target_point_occupied = []
        self.target_point_occupants = []
        self._init_target_points()

        self.robots = []
        for i in range(num_robots):
            self.robots.append(self._spawn_robot(i, num_robots))

    def _init_target_points(self):
        left_x = self.target_line[0][0] + ROBOT_RADIUS
        right_x = self.target_line[1][0] - ROBOT_RADIUS
        if right_x < left_x:
            self.target_points = []
            self.target_point_weights = []
            self.target_point_occupied = []
            self.target_point_occupants = []
            return

        span = right_x - left_x
        num_points = int(math.floor(span / TARGET_POINT_SPACING)) + 1
        xs = [left_x + i * TARGET_POINT_SPACING for i in range(num_points)]
        self.target_points = [(x, self.line_y) for x in xs]
        if num_points <= 1:
            self.target_point_weights = [1.0 for _ in range(num_points)]
        else:
            self.target_point_weights = [abs(2.0 * (i / (num_points - 1)) - 1.0) for i in range(num_points)]
        self.target_point_occupied = [False for _ in range(num_points)]
        self.target_point_occupants = [None for _ in range(num_points)]

    def get_guidance_point(self, robot):
        uncovered_indices = [idx for idx, occupied in enumerate(self.target_point_occupied) if not occupied]
        if not uncovered_indices:
            return self.target_points[-1]
        best_idx = min(
            uncovered_indices,
            key=lambda idx: (
                math.hypot(robot.position[0] - self.target_points[idx][0], robot.position[1] - self.target_points[idx][1])
                / max(0.1, (1.0 + self.target_point_weights[idx]))
            ),
        )
        return self.target_points[best_idx]

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
        candidates = []
        for robot in self.robots:
            if robot.parked:
                continue
            uncovered_indices = [idx for idx, occupied in enumerate(self.target_point_occupied) if not occupied]
            if not uncovered_indices:
                continue
            point_index = min(
                uncovered_indices,
                key=lambda idx: (
                    math.hypot(robot.position[0] - self.target_points[idx][0], robot.position[1] - self.target_points[idx][1])
                    / max(0.1, (1.0 + self.target_point_weights[idx]))
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
                robot.R_robot_sense = 0.15
                self.target_point_occupied[point_index] = True
                self.target_point_occupants[point_index] = robot

    def _spawn_robot(self, index, total_robots):
        spacing = self.width / (total_robots + 1)
        spawn_x = spacing * (index + 1)
        spawn_y = self.height - 10.0  # Top of the grid
        return Robot(spawn_x, spawn_y, self.width, self.height, radius=ROBOT_RADIUS)

    def step(self):
        for robot in self.robots:
            robot.update(self)
        self._try_park_robots()

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
    ax.set_aspect('equal')
    ax.set_title("Multi-Robot APF - Line Seeking")

    ax.plot([0, ENV_WIDTH, ENV_WIDTH, 0, 0], [0, 0, ENV_HEIGHT, ENV_HEIGHT, 0], color='black', linewidth=1.5)

    for wall in env.inner_walls:
        x_coords = [wall[0][0], wall[1][0]]
        y_coords = [wall[0][1], wall[1][1]]
        ax.plot(x_coords, y_coords, color='red', linewidth=3)

    ax.plot(
        [env.target_line[0][0], env.target_line[1][0]],
        [env.target_line[0][1], env.target_line[1][1]],
        color='gray',
        linestyle='--',
        linewidth=1.5,
        label='Target Line',
    )

    target_point_scat = ax.scatter(
        [point[0] for point in env.target_points],
        [point[1] for point in env.target_points],
        s=20,
        c=['orange' for _ in env.target_points],
        marker='o',
        alpha=0.85,
        zorder=2,
        label='Target Points',
    )

    robot_circles = []
    for r in env.robots:
        circle = plt.Circle(
            (r.position[0], r.position[1]),
            r.radius,
            color='blue',
            fill=True,
            linewidth=0,
            alpha=0.85,
            zorder=3,
        )
        ax.add_patch(circle)
        robot_circles.append(circle)

    parked_proxy = plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', alpha=0.85, markersize=8, linewidth=0)
    ax.legend(handles=[parked_proxy], labels=['Robot'], loc='upper right', fontsize='small')

    state = {'timestep': 0}
    selected = {'index': None}

    selection_circle = plt.Circle((0, 0), ROBOT_RADIUS * 1.35, fill=False, color='gold', linewidth=2.0, alpha=0.9, zorder=4)
    selection_circle.set_visible(False)
    ax.add_patch(selection_circle)

    rr_circle = plt.Circle((0, 0), R_ROBOT_SENSE, fill=False, linestyle='--', color='cyan', linewidth=1.2, alpha=0.8, zorder=1)
    rr_circle.set_visible(False)
    ax.add_patch(rr_circle)

    rw_circle = plt.Circle((0, 0), R_WALL_SENSE, fill=False, linestyle='--', color='magenta', linewidth=1.2, alpha=0.7, zorder=1)
    rw_circle.set_visible(False)
    ax.add_patch(rw_circle)

    near_boost_circle = plt.Circle((0, 0), ATT_NEAR_BOOST_RADIUS, fill=False, linestyle=':', color='limegreen', linewidth=1.2, alpha=0.7, zorder=1)
    near_boost_circle.set_visible(False)
    ax.add_patch(near_boost_circle)

    repulse_range_circle = plt.Circle((0, 0), PARKED_POINT_REPULSE_RANGE, fill=False, linestyle=':', color='red', linewidth=1.2, alpha=0.7, zorder=1)
    repulse_range_circle.set_visible(False)
    ax.add_patch(repulse_range_circle)

    force_lines = {
        "random": ax.plot([], [], color='orange', linewidth=1.5, zorder=5, label='F_random')[0],
        "robot_repulsion": ax.plot([], [], color='cyan', linewidth=1.5, zorder=5, label='F_robot')[0],
        "wall_repulsion": ax.plot([], [], color='magenta', linewidth=1.5, zorder=5, label='F_wall')[0],
        "target_points": ax.plot([], [], color='green', linewidth=1.8, zorder=5, label='F_target')[0],
        "total": ax.plot([], [], color='black', linewidth=2.0, zorder=6, label='F_total')[0],
    }

    stats_text = ax.text(
        0.01,
        0.99,
        "Click a robot to inspect forces/radii",
        transform=ax.transAxes,
        va='top',
        ha='left',
        fontsize=8,
        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8),
        zorder=7,
    )

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
        best_dist = float('inf')
        for i, robot in enumerate(env.robots):
            dist = math.hypot(robot.position[0] - click_x, robot.position[1] - click_y)
            if dist < best_dist:
                best_dist = dist
                best_idx = i

        if best_idx is not None and best_dist <= ROBOT_DIAMETER * 1.2:
            selected['index'] = best_idx
        else:
            selected['index'] = None

        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('button_press_event', on_click)

    def update_frame(_):
        env.step()
        state['timestep'] += 1

        for circle, r in zip(robot_circles, env.robots):
            circle.set_center((r.position[0], r.position[1]))
            if r.parked:
                circle.set_facecolor('navy')
            else:
                circle.set_facecolor('blue')

        point_colors = ['gray' if occ else 'orange' for occ in env.target_point_occupied]
        target_point_scat.set_color(point_colors)

        if selected['index'] is None:
            selection_circle.set_visible(False)
            rr_circle.set_visible(False)
            rw_circle.set_visible(False)
            near_boost_circle.set_visible(False)
            repulse_range_circle.set_visible(False)
            for line in force_lines.values():
                line.set_data([], [])
            stats_text.set_text("Click a robot to inspect forces/radii")
        else:
            sel = env.robots[selected['index']]
            sx, sy = sel.position[0], sel.position[1]

            selection_circle.set_visible(True)
            selection_circle.set_center((sx, sy))

            rr_circle.set_visible(True)
            rr_circle.set_center((sx, sy))
            rr_circle.set_radius(sel.R_robot_sense)

            rw_circle.set_visible(True)
            rw_circle.set_center((sx, sy))
            rw_circle.set_radius(sel.R_wall_sense)

            near_boost_circle.set_visible(True)
            near_boost_circle.set_center((sx, sy))
            near_boost_circle.set_radius(sel.att_near_boost_radius)

            repulse_range_circle.set_visible(True)
            repulse_range_circle.set_center((sx, sy))
            repulse_range_circle.set_radius(PARKED_POINT_REPULSE_RANGE)

            comp = sel.last_force_components
            _set_force_line(force_lines["random"], sx, sy, comp["random"])
            _set_force_line(force_lines["robot_repulsion"], sx, sy, comp["robot_repulsion"])
            _set_force_line(force_lines["wall_repulsion"], sx, sy, comp["wall_repulsion"])
            _set_force_line(force_lines["target_points"], sx, sy, comp["target_points"])
            _set_force_line(force_lines["total"], sx, sy, comp["total"])

            stats_text.set_text(
                f"Robot #{selected['index']} | parked={sel.parked}\n"
                f"pos=({sx:.2f}, {sy:.2f}) vel=({sel.velocity[0]:.3f}, {sel.velocity[1]:.3f})\n"
                f"R_robot_sense={sel.R_robot_sense:.2f}, R_wall_sense={sel.R_wall_sense:.2f}\n"
                f"ATT_NEAR_BOOST_RADIUS={sel.att_near_boost_radius:.2f}, PARKED_REPULSE_RANGE={PARKED_POINT_REPULSE_RANGE:.2f}\n"
                f"line_dist={comp['line_distance']:.2f}, target_attn={comp['target_attenuation']:.3f}\n"
                f"|F_random|={np.linalg.norm(comp['random']):.3f}, |F_robot|={np.linalg.norm(comp['robot_repulsion']):.3f}\n"
                f"|F_wall|={np.linalg.norm(comp['wall_repulsion']):.3f}, |F_target|={np.linalg.norm(comp['target_points']):.3f}\n"
                f"|F_total|={np.linalg.norm(comp['total']):.3f}, turn_p={comp['turn_probability']:.3f}, rand_scale={comp['random_scale']:.3f}"
            )

        if env.reached_target_line():
            print(f"All robots reached target line at timestep {state['timestep']}.")
            plt.close(fig)
        elif state['timestep'] >= MAX_TIMESTEPS:
            print(f"Stopped at max timesteps ({MAX_TIMESTEPS}) before reaching target line.")
            plt.close(fig)

        return [
            target_point_scat,
            selection_circle,
            rr_circle,
            rw_circle,
            near_boost_circle,
            repulse_range_circle,
            *force_lines.values(),
            stats_text,
            *robot_circles,
        ]

    ani = animation.FuncAnimation(
        fig,
        update_frame,
        frames=MAX_TIMESTEPS,
        interval=ANIMATION_INTERVAL_MS,
        blit=False,
        repeat=False,
    )

    plt.show()


if __name__ == '__main__':
    main()
