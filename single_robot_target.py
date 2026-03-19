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
ROBOT_SPEED = 0.7  # Max robot speed per timestep
TURN_PROBABILITY = 0.1  # Chance each step to randomize heading

R_ROBOT_SENSE = 2.0  # Robot-robot repulsion sensing radius
R_WALL_SENSE = 10.0  # Wall repulsion sensing radius
K_REP = 20  # Robot-robot repulsion gain
K_WALL = 5  # Wall repulsion gain
K_TANGENT_SCALE = 2.0  # Multiplier for the tangential "sliding" force to skirt around walls
K_ATT = 20  # Attractive gain shaping how quickly attraction ramps with distance
ATT_FORCE_MAX = 0.4  # Upper bound on attractive force magnitude for more natural motion
ATT_NEAR_BOOST_RADIUS = 2.5  # Radius around target where point attraction gets a stronger boost
ATT_NEAR_BOOST = 0.45  # Extra near-target attraction to prevent circling around the point
ATT_TOTAL_MAX = 1.0  # Hard cap on total point-attraction magnitude (base + near boost)

TARGET_WIDTH_MULTIPLE = 10  # Target/obstacle span as integer multiple of robot diameter
TARGET_LINE_WIDTH = TARGET_WIDTH_MULTIPLE * ROBOT_DIAMETER  # Resulting target line width (units)

OBSTACLE_CENTER_X = ENV_WIDTH / 2.0  # Horizontal center of U-shaped obstacle
OBSTACLE_BOTTOM_Y = 40.0  # Bottom y-coordinate of obstacle legs
OBSTACLE_TOP_Y = 80.0  # Top y-coordinate of obstacle
OBSTACLE_LEFT_X = OBSTACLE_CENTER_X - TARGET_LINE_WIDTH / 2.0  # Left x-coordinate of obstacle
OBSTACLE_RIGHT_X = OBSTACLE_CENTER_X + TARGET_LINE_WIDTH / 2.0  # Right x-coordinate of obstacle

TARGET_LINE = ((OBSTACLE_LEFT_X, OBSTACLE_BOTTOM_Y), (OBSTACLE_RIGHT_X, OBSTACLE_BOTTOM_Y))  # Open bottom side
TARGET_POINT_X_OFFSET = 10.0  # Shift target point +X from left tip so it lies on the line, not on obstacle tip
TARGET_POINT = (TARGET_LINE[0][0] + TARGET_POINT_X_OFFSET, TARGET_LINE[0][1])  # Endpoint-following target

CUE_LINE_LENGTH = 0  # Length of perpendicular guide line leading into the target point
CUE_LINE = ((TARGET_POINT[0], TARGET_POINT[1] - CUE_LINE_LENGTH), TARGET_POINT)  # Perpendicular cue (vertical)
CUE_FORCE_MAX = 0  # Lesser attraction strength for cue guidance (weaker than point field)
CUE_K_ATT = 0  # Cue-field gain shaping pull toward the guide line
CUE_SENSE_RADIUS = 12.0  # Only apply cue attraction when robot is close to the cue line
CUE_ACTIVATION_RADIUS = 30.0  # Activate cue field only when robot is reasonably near the target

SPAWN_BEHIND_U_Y_OFFSET = 8.0  # Spawn offset above the U's closed horizontal top wall
SPAWN_POINT = (OBSTACLE_CENTER_X, OBSTACLE_TOP_Y + SPAWN_BEHIND_U_Y_OFFSET)  # Spawn behind closed side of U

TARGET_APPROACH_RADIUS = 10.0  # Distance at which near-target smoothing starts
TARGET_STOP_DISTANCE = 0.15  # Distance threshold to snap to target and stop
NEAR_TARGET_WALL_SCALE = 0.25  # Minimum wall-repulsion scale near target to reduce jitter

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
        self.k_att = K_ATT
        self.att_force_max = ATT_FORCE_MAX
        self.att_near_boost_radius = ATT_NEAR_BOOST_RADIUS
        self.att_near_boost = ATT_NEAR_BOOST
        self.att_total_max = ATT_TOTAL_MAX
        self.cue_force_max = CUE_FORCE_MAX
        self.cue_k_att = CUE_K_ATT
        self.cue_sense_radius = CUE_SENSE_RADIUS
        self.cue_activation_radius = CUE_ACTIVATION_RADIUS

        self.target_approach_radius = TARGET_APPROACH_RADIUS
        self.target_stop_distance = TARGET_STOP_DISTANCE
        self.near_target_wall_scale = NEAR_TARGET_WALL_SCALE

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

    def compute_robot_repulsion(self, other_robots, target_line):
        f_rep = np.zeros(2)
        
        tx = ((target_line[0][0] + target_line[1][0]) / 2.0) - self.position[0]
        ty = ((target_line[0][1] + target_line[1][1]) / 2.0) - self.position[1]

        for other in other_robots:
            if other is self:
                continue

            dist = math.hypot(self.position[0] - other.position[0], self.position[1] - other.position[1])
            effective_dist = dist - (2 * self.radius)

            if effective_dist < 0.001:
                effective_dist = 0.001

            # Determine bubble size based on the OTHER robot's current sensing radius
            # This allows finished robots to have a drastically collapsed force field
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
                if np.dot(t1, target_vector) > np.dot(t2, target_vector):
                    tangent = t1
                else:
                    tangent = t2
                    
                f_rep += magnitude * self.k_tangent_scale * tangent

        return f_rep

    def compute_target_line_attraction(self, target_line):
        dist_to_line, proj_x, proj_y = self._dist_to_segment(
            self.position[0], self.position[1],
            target_line[0][0], target_line[0][1],
            target_line[1][0], target_line[1][1]
        )

        if dist_to_line <= 0.001:
            return np.zeros(2)

        ux = (proj_x - self.position[0]) / dist_to_line
        uy = (proj_y - self.position[1]) / dist_to_line
        base_magnitude = self.att_force_max * math.tanh(self.k_att * dist_to_line)
        near_ratio = max(0.0, 1.0 - dist_to_line / self.att_near_boost_radius)
        near_magnitude = self.att_near_boost * near_ratio
        magnitude = min(self.att_total_max, base_magnitude + near_magnitude)
        return np.array([ux, uy]) * magnitude

    def compute_cue_line_attraction(self, cue_line, target_line):
        target_center_dist, _, _ = self._dist_to_segment(
            self.position[0], self.position[1],
            target_line[0][0], target_line[0][1],
            target_line[1][0], target_line[1][1]
        )
        if target_center_dist > self.cue_activation_radius:
            return np.zeros(2)

        dist_to_line, proj_x, proj_y = self._dist_to_segment(
            self.position[0],
            self.position[1],
            cue_line[0][0],
            cue_line[0][1],
            cue_line[1][0],
            cue_line[1][1],
        )

        if dist_to_line > self.cue_sense_radius:
            return np.zeros(2)

        if dist_to_line <= 0.001:
            return np.zeros(2)

        ux = (proj_x - self.position[0]) / dist_to_line
        uy = (proj_y - self.position[1]) / dist_to_line
        magnitude = self.cue_force_max * math.tanh(self.cue_k_att * dist_to_line)
        return np.array([ux, uy]) * magnitude

    def compute_total_force(self, other_robots, inner_walls, target_line, cue_line):
        center_dist, _, _ = self._dist_to_segment(
            self.position[0], self.position[1],
            target_line[0][0], target_line[0][1],
            target_line[1][0], target_line[1][1]
        )
        approach_ratio = min(1.0, center_dist / self.target_approach_radius)

        if random.random() < self.turn_probability * approach_ratio:
            self.angle = random.uniform(0, 2 * math.pi)
        f_random = np.array([math.cos(self.angle), math.sin(self.angle)]) * self.speed * approach_ratio

        f_robot = self.compute_robot_repulsion(other_robots, target_line)
        f_wall = self.compute_wall_repulsion(inner_walls, target_line)
        wall_scale = self.near_target_wall_scale + (1.0 - self.near_target_wall_scale) * approach_ratio
        f_wall *= wall_scale
        f_target = self.compute_target_line_attraction(target_line)
        f_cue = self.compute_cue_line_attraction(cue_line, target_line)

        return f_random + f_robot + f_wall + f_target + f_cue

    def update(self, other_robots, inner_walls, target_line, cue_line):
        center_dist, _, _ = self._dist_to_segment(
            self.position[0], self.position[1],
            target_line[0][0], target_line[0][1],
            target_line[1][0], target_line[1][1]
        )

        # Shrink the repulsive bubble if the robot has arrived at the target line
        if self.has_reached_line(target_line):
            self.R_robot_sense = 0.1

        f_total = self.compute_total_force(other_robots, inner_walls, target_line, cue_line)

        magnitude = np.linalg.norm(f_total)
        approach_speed_scale = min(1.0, center_dist / self.target_approach_radius)
        clamped_speed = min(magnitude, self.speed * approach_speed_scale)

        if magnitude > 0:
            velocity = (f_total / magnitude) * clamped_speed
        else:
            velocity = np.zeros(2)

        if magnitude > 0.001:
            self.angle = math.atan2(velocity[1], velocity[0])

        new_x = self.position[0] + velocity[0]
        new_y = self.position[1] + velocity[1]

        new_x = max(self.radius, min(self.env_width - self.radius, new_x))
        new_y = max(self.radius, min(self.env_height - self.radius, new_y))

        self.position[0] = new_x
        self.position[1] = new_y

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
        self.target_point = TARGET_POINT
        self.cue_line = CUE_LINE

        self.robots = []
        for i in range(num_robots):
            self.robots.append(self._spawn_robot(i, num_robots))

    def _spawn_robot(self, index, total_robots):
        spacing = self.width / (total_robots + 1)
        spawn_x = spacing * (index + 1)
        spawn_y = self.height - 10.0  # Top of the grid
        return Robot(spawn_x, spawn_y, self.width, self.height, radius=ROBOT_RADIUS)

    def step(self):
        for robot in self.robots:
            robot.update(self.robots, self.inner_walls, self.target_line, self.cue_line)

    def reached_target_line(self):
        return all(r.has_reached_line(self.target_line) for r in self.robots)


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

    ax.plot(
        [env.cue_line[0][0], env.cue_line[1][0]],
        [env.cue_line[0][1], env.cue_line[1][1]],
        color='green',
        linewidth=7,
        alpha=0.12,
        zorder=1,
        label='Cue Shadow',
    )

    # Target point removed since entire line is now attractive

    robot_area = 20
    scat = ax.scatter(
        [r.position[0] for r in env.robots],
        [r.position[1] for r in env.robots],
        s=robot_area,
        c='blue',
        edgecolors='None',
        label='Robot',
        zorder=3,
    )

    radius_circles = []
    for r in env.robots:
        circle = plt.Circle(
            (r.position[0], r.position[1]),
            r.radius,
            color='blue',
            fill=False,
            linestyle='-',
            linewidth=1,
            alpha=0.5,
            zorder=2,
        )
        ax.add_patch(circle)
        radius_circles.append(circle)

    ax.legend(loc='upper right', fontsize='small')

    state = {'timestep': 0}

    def update_frame(_):
        env.step()
        state['timestep'] += 1

        new_positions = np.c_[[r.position[0] for r in env.robots], [r.position[1] for r in env.robots]]
        scat.set_offsets(new_positions)
        for circle, r in zip(radius_circles, env.robots):
            circle.set_center((r.position[0], r.position[1]))

        if env.reached_target_line():
            print(f"All robots reached target line at timestep {state['timestep']}.")
            plt.close(fig)
        elif state['timestep'] >= MAX_TIMESTEPS:
            print(f"Stopped at max timesteps ({MAX_TIMESTEPS}) before reaching target line.")
            plt.close(fig)

        return [scat] + radius_circles

    ani = animation.FuncAnimation(
        fig,
        update_frame,
        frames=MAX_TIMESTEPS,
        interval=ANIMATION_INTERVAL_MS,
        blit=True,
        repeat=False,
    )

    plt.show()


if __name__ == '__main__':
    main()
