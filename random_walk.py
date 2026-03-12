"""
Swarm Robotics: Phase 1 - Random Walk Exploration
This simulation models N robots in a 100x100 2D continuous environment.
Inside this environment, there is a U-shaped obstacle (3 walls, 1 open side).
Robots can move anywhere inside the 100x100 area, bouncing off outer boundaries,
the inner obstacle walls, and each other.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
import math

class Robot:
    """
    Represents a single swarm robot.
    """
    def __init__(self, x, y, env_width, env_height, radius=1.5):
        # Position [x, y]
        self.position = [x, y]
        self.env_width = env_width
        self.env_height = env_height
        self.radius = radius
        
        # Physics properties
        self.speed = 0.7
        # Initialize with a random direction angle (in radians)
        self.angle = random.uniform(0, 2 * math.pi)

        # APF (Artificial Potential Field) Parameters
        self.R_robot_sense = 2.0   # Sensing radius for other robots
        self.R_wall_sense = 10.0    # Sensing radius for walls
        self.k_rep = 50.0          # Repulsion gain for robots
        self.k_wall = 50.0         # Repulsion gain for walls

    def _dist_to_segment(self, px, py, x1, y1, x2, y2):
        """Helper to find shortest distance from point to line segment"""
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx*dx + dy*dy
        if l2 == 0:
            return math.hypot(px - x1, py - y1), x1, y1
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / l2))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y), proj_x, proj_y

    def compute_wall_repulsion(self, inner_walls):
        f_wall = np.zeros(2)
        
        # Combine outer boundaries and inner walls
        walls = [
            (0, 0, self.env_width, 0),                           # Bottom
            (0, self.env_height, self.env_width, self.env_height), # Top
            (0, 0, 0, self.env_height),                          # Left
            (self.env_width, 0, self.env_width, self.env_height)   # Right
        ]
        for w in inner_walls:
            walls.append((w[0][0], w[0][1], w[1][0], w[1][1]))
            
        for x1, y1, x2, y2 in walls:
            dist, px, py = self._dist_to_segment(self.position[0], self.position[1], x1, y1, x2, y2)
            
            # Effectively measure distance from the robot's physical shell
            effective_dist = dist - self.radius
            if effective_dist < 0.001:
                effective_dist = 0.001
                
            if effective_dist < self.R_wall_sense:
                # F_wall = k_wall * (1/d - 1/R_wall) * (1/d^2) * direction_away
                magnitude = self.k_wall * (1.0/effective_dist - 1.0/self.R_wall_sense) * (1.0/(effective_dist**2))
                
                # Direction away from the wall projection point
                dx = self.position[0] - px
                dy = self.position[1] - py
                if dist > 0.001:
                    dx /= dist
                    dy /= dist
                else:
                    dx, dy = random.uniform(-1, 1), random.uniform(-1, 1)
                    
                f_wall += np.array([dx, dy]) * magnitude
                
        return f_wall

    def compute_robot_repulsion(self, other_robots):
        f_rep = np.zeros(2)
        for other in other_robots:
            if other is self:
                continue
            
            # Distance between centers
            dist = math.hypot(self.position[0] - other.position[0], self.position[1] - other.position[1])
            # Effective distance between robot shells
            effective_dist = dist - (2 * self.radius)
            
            if effective_dist < 0.001:
                effective_dist = 0.001
                
            if effective_dist < self.R_robot_sense:
                # F_repulsion = k_rep * (1/d - 1/R_robot) * (1/d^2) * direction_away
                magnitude = self.k_rep * (1.0/effective_dist - 1.0/self.R_robot_sense) * (1.0/(effective_dist**2))
                
                # Direction away from the other robot
                dx = (self.position[0] - other.position[0]) / dist
                dy = (self.position[1] - other.position[1]) / dist
                
                f_rep += np.array([dx, dy]) * magnitude
                
        return f_rep

    def compute_total_force(self, other_robots, inner_walls):
        # 1. Random Exploration Force (Maintain random walk behavior)
        if random.random() < 0.1:
            self.angle = random.uniform(0, 2 * math.pi)
        f_random = np.array([math.cos(self.angle), math.sin(self.angle)]) * self.speed
        
        # 2. Robot Repulsion Force
        f_robot = self.compute_robot_repulsion(other_robots)
        
        # 3. Wall Repulsion Force
        f_wall = self.compute_wall_repulsion(inner_walls)
        
        return f_random + f_robot + f_wall

    def update(self, other_robots, inner_walls):
        """
        Updates the robot's position using APF (Artificial Potential Field) algorithm.
        """
        # Calculate total combined force vector
        f_total = self.compute_total_force(other_robots, inner_walls)
        
        # Normalize and clamp speed to maximum velocity (self.speed)
        magnitude = np.linalg.norm(f_total)
        clamped_speed = min(magnitude, self.speed)
        
        if magnitude > 0:
            velocity = (f_total / magnitude) * clamped_speed
        else:
            velocity = np.zeros(2)
            
        # Update angle for visual consistency and logic
        if magnitude > 0.001:
            self.angle = math.atan2(velocity[1], velocity[0])
            
        # Move the robot
        new_x = self.position[0] + velocity[0]
        new_y = self.position[1] + velocity[1]
        
        # Fallback coordinate clamping just in case forces fail at corners
        new_x = max(self.radius, min(self.env_width - self.radius, new_x))
        new_y = max(self.radius, min(self.env_height - self.radius, new_y))
        
        self.position[0] = new_x
        self.position[1] = new_y

class Environment:
    """
    Represents the 100x100 2D continuous environment with a U-shaped inner obstacle.
    """
    def __init__(self, width, height, num_robots):
        self.width = width
        self.height = height
        
        # Define inner U-shaped rectangular obstacle walls
        # The structure is open at the bottom.
        # Format: ((x_start, y_start), (x_end, y_end))
        self.inner_walls = [
            ((35, 40), (35, 80)),   # Left vertical wall
            ((65, 40), (65, 80)),   # Right vertical wall
            ((35, 80), (65, 80))    # Top horizontal wall
        ]
        
        # Initialize N robots at random non-overlapping positions
        self.robots = []
        for _ in range(num_robots):
            # Spawn slightly away from the edges to avoid immediate collisions
            # Keep generating positions until we find one that doesn't overlap existing robots
            robot_radius = 1.5
            while True:
                x = random.uniform(5, width - 5)
                y = random.uniform(5, height - 5)
                
                # Check for overlap with already placed robots (using 2*radius as min distance)
                overlap = False
                for r in self.robots:
                    if math.hypot(x - r.position[0], y - r.position[1]) < 2 * robot_radius:
                        overlap = True
                        break
                        
                # Ensure they don't spawn inside the inner red obstacle either
                if 35 - robot_radius <= x <= 65 + robot_radius and 40 - robot_radius <= y <= 80 + robot_radius:
                    overlap = True
                    
                if not overlap:
                    break
                    
            self.robots.append(Robot(x, y, width, height, radius=robot_radius))
        
    def step(self):
        """
        Advances the simulation by one timestep.
        """
        for robot in self.robots:
            robot.update(self.robots, self.inner_walls)

def main():
    # ====== Simulation Parameters ======
    ENV_WIDTH = 100
    ENV_HEIGHT = 100
    NUM_ROBOTS = 15
    NUM_TIMESTEPS = 1000

    env = Environment(ENV_WIDTH, ENV_HEIGHT, NUM_ROBOTS)

    # ====== Visualization ======
    fig, ax = plt.subplots(figsize=(8, 8))
    
    ax.set_xlim(0, ENV_WIDTH)
    ax.set_ylim(0, ENV_HEIGHT)
    ax.set_aspect('equal')
    ax.set_title("Swarm Robots - Environment with U-Shaped Obstacle")

    # Draw the outer bounding box
    ax.plot([0, ENV_WIDTH, ENV_WIDTH, 0, 0], [0, 0, ENV_HEIGHT, ENV_HEIGHT, 0], color='black', linewidth=1.5)

    # Draw the inner U-shaped rectangle
    for wall in env.inner_walls:
        x_coords = [wall[0][0], wall[1][0]]
        y_coords = [wall[0][1], wall[1][1]]
        ax.plot(x_coords, y_coords, color='red', linewidth=3)
        
    # Draw the open side (bottom) as dashed
    ax.plot([35, 65], [40, 40], color='gray', linestyle='--', linewidth=1.5, label="Target / Open Side")
    
    ax.legend(loc="upper right", fontsize='small')

    # Note: s parameter in scatter is area, so we square the radius approximately
    robot_area = 20  # Make the core dot a bit smaller so the radius line is clearer
    scat = ax.scatter([r.position[0] for r in env.robots], 
                      [r.position[1] for r in env.robots], 
                      s=robot_area, c='blue', edgecolors='None', label='Robots', zorder=3)

    # Draw explicit circles around each robot to visualize their exact physical collision radius
    radius_circles = []
    for r in env.robots:
        circle = plt.Circle((r.position[0], r.position[1]), r.radius, 
                            color='blue', fill=False, linestyle='-', linewidth=1, alpha=0.5, zorder=2)
        ax.add_patch(circle)
        radius_circles.append(circle)

    def update_frame(frame):
        env.step()
        new_positions = np.c_[[r.position[0] for r in env.robots], [r.position[1] for r in env.robots]]
        scat.set_offsets(new_positions)
        
        # Update the explicit radius circles as well
        for i, circle in enumerate(radius_circles):
            circle.set_center((env.robots[i].position[0], env.robots[i].position[1]))
            
        # We need to return both the scatter and the list of circles for blitting to work
        return [scat] + radius_circles

    ani = animation.FuncAnimation(
        fig, 
        update_frame, 
        frames=NUM_TIMESTEPS, 
        interval=30, 
        blit=True, 
        repeat=False
    )

    plt.show()

if __name__ == "__main__":
    main()
