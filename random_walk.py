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
        self.speed = 1
        # Initialize with a random direction angle (in radians)
        self.angle = random.uniform(0, 2 * math.pi)

    def check_wall_collision(self, next_x, next_y, walls):
        """
        Continuous collision detection against line segments considering physical radius.
        """
        for (x1, y1), (x2, y2) in walls:
            # Vertical wall obstacle
            if x1 == x2:
                dist_next = abs(next_x - x1)
                dist_curr = abs(self.position[0] - x1)
                # Check if radius intersects the wall AND it is moving closer to it
                if dist_next <= self.radius and dist_next < dist_curr:
                    # Were we within the y-segment of the wall?
                    y_min = min(y1, y2) - self.radius
                    y_max = max(y1, y2) + self.radius
                    if y_min <= self.position[1] <= y_max or y_min <= next_y <= y_max:
                        return True, 'vertical'
            
            # Horizontal wall obstacle
            elif y1 == y2:
                dist_next = abs(next_y - y1)
                dist_curr = abs(self.position[1] - y1)
                # Check if radius intersects the wall AND it is moving closer to it
                if dist_next <= self.radius and dist_next < dist_curr:
                    # Were we within the x-segment of the wall?
                    x_min = min(x1, x2) - self.radius
                    x_max = max(x1, x2) + self.radius
                    if x_min <= self.position[0] <= x_max or x_min <= next_x <= x_max:
                        return True, 'horizontal'
        return False, None
        
    def update(self, other_robots, inner_walls):
        """
        Updates the robot's position based on its speed and direction.
        Handles random walk, boundary bouncing, obstacle, and robot collisions.
        """
        # Random Walk: With a small probability (e.g. 0.1), pick a new random direction
        if random.random() < 0.1:
            self.angle = random.uniform(0, 2 * math.pi)
            
        # Calculate proposed new position
        new_x = self.position[0] + self.speed * math.cos(self.angle)
        new_y = self.position[1] + self.speed * math.sin(self.angle)
        
        bounced = False

        # 1. Outer Environment Boundary Collision
        if new_x - self.radius < 0:
            new_x = self.radius
            self.angle = math.pi - self.angle
            bounced = True
        elif new_x + self.radius > self.env_width:
            new_x = self.env_width - self.radius
            self.angle = math.pi - self.angle
            bounced = True
            
        if new_y - self.radius < 0:
            new_y = self.radius
            self.angle = -self.angle
            bounced = True
        elif new_y + self.radius > self.env_height:
            new_y = self.env_height - self.radius
            self.angle = -self.angle
            bounced = True

        # 2. Inner Obstacle Collision Detection
        if not bounced:
            collided, wall_type = self.check_wall_collision(new_x, new_y, inner_walls)
            if collided:
                if wall_type == 'vertical':
                    self.angle = math.pi - self.angle
                elif wall_type == 'horizontal':
                    self.angle = -self.angle
                
                # Step back to avoid getting stuck inside the wall
                new_x = self.position[0]
                new_y = self.position[1]
                bounced = True

        # 3. Simple Robot-to-Robot Collision
        if not bounced:
            for other in other_robots:
                if other is not self:
                    # Calculate new distance vs old distance
                    dist_new = math.hypot(new_x - other.position[0], new_y - other.position[1])
                    dist_old = math.hypot(self.position[0] - other.position[0], self.position[1] - other.position[1])
                    
                    if dist_new < 2 * self.radius and dist_new < dist_old:
                        # Collision! Bounce away logically by turning around 180 degrees
                        self.angle += math.pi
                        
                        # Step back to current valid position
                        new_x = self.position[0]
                        new_y = self.position[1]
                        break

        # Normalize angle to be within [0, 2*pi]
        self.angle %= (2 * math.pi)
        
        # Update actual position
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
