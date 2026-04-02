import numpy as np
import matplotlib.pyplot as plt
import random
import math

# ==========================================
# Environment Constants (Matches APF Exactly)
# ==========================================
ENV_WIDTH = 100
ENV_HEIGHT = 100
ROBOT_RADIUS = 1.5
OBSTACLE_CENTER_X = 50.0
OBSTACLE_BOTTOM_Y = 40.0
OBSTACLE_TOP_Y = 80.0
TARGET_LINE_WIDTH = 30.0
OBSTACLE_LEFT_X = OBSTACLE_CENTER_X - TARGET_LINE_WIDTH / 2.0  # 35.0
OBSTACLE_RIGHT_X = OBSTACLE_CENTER_X + TARGET_LINE_WIDTH / 2.0 # 65.0

# U-Shape Wall segments (The 3 walls of the trap)
WALLS = [
    ((OBSTACLE_LEFT_X, OBSTACLE_BOTTOM_Y), (OBSTACLE_LEFT_X, OBSTACLE_TOP_Y)),
    ((OBSTACLE_RIGHT_X, OBSTACLE_BOTTOM_Y), (OBSTACLE_RIGHT_X, OBSTACLE_TOP_Y)),
    ((OBSTACLE_LEFT_X, OBSTACLE_TOP_Y), (OBSTACLE_RIGHT_X, OBSTACLE_TOP_Y)),
]

class Node:
    """Represents a single point in our RRT tree."""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def get_dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def intersect(p1, p2, p3, p4):
    """Checks if line segment (p1-p2) intersects with wall segment (p3-p4)."""
    def ccw(A, B, C):
        return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

class RRT:
    def __init__(self, start, goal, walls, step_size=4.0, goal_sample_rate=0.1):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.walls = walls
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.node_list = [self.start]

    def plan(self, max_iter=10000):
        print(f"--- RRT Starting Selection ---")
        print(f"Goal Location: ({self.goal.x}, {self.goal.y})")
        print(f"Start Location: ({self.start.x}, {self.start.y})")
        print(f"Step Size: {self.step_size} units\n")

        for i in range(max_iter):
            # 1. Random Sampling
            # We occasionally sample the goal directly to speed up convergence
            if random.random() < self.goal_sample_rate:
                rnd = [self.goal.x, self.goal.y]
            else:
                rnd = [random.uniform(0, ENV_WIDTH), random.uniform(0, ENV_HEIGHT)]

            # 2. Find the nearest node in the current tree
            nearest_node = self.node_list[0]
            min_dist = get_dist([nearest_node.x, nearest_node.y], rnd)
            for node in self.node_list:
                d = get_dist([node.x, node.y], rnd)
                if d < min_dist:
                    min_dist = d
                    nearest_node = node

            # 3. 'Steer' towards the sample
            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_node = Node(nearest_node.x + self.step_size * math.cos(theta),
                            nearest_node.y + self.step_size * math.sin(theta))
            new_node.parent = nearest_node

            # 4. Collision Check: Did we just walk through a wall?
            if self.is_collision_free(nearest_node, new_node):
                self.node_list.append(new_node)
                
                # Educational Progress Prints
                if i % 200 == 0:
                    print(f"Iter {i}: Tree expanded to ({new_node.x:.1f}, {new_node.y:.1f}). Tree size: {len(self.node_list)} nodes.")
                
                # Check if we are close enough to the goal
                if get_dist([new_node.x, new_node.y], [self.goal.x, self.goal.y]) <= self.step_size:
                    print(f"\n[SUCCESS] Goal reached at iteration {i}!")
                    return self.extract_path(new_node)
        
        print("\n[FAILED] RRT could not find a path within max iterations.")
        return None

    def is_collision_free(self, n1, n2):
        """Returns True if the line between nodes doesn't cross a wall."""
        # Boundary check
        if n2.x < 0 or n2.x > ENV_WIDTH or n2.y < 0 or n2.y > ENV_HEIGHT:
            return False
        
        p1 = (n1.x, n1.y)
        p2 = (n2.x, n2.y)
        
        for w_start, w_end in self.walls:
            if intersect(p1, p2, w_start, w_end):
                return False
        return True

    def extract_path(self, end_node):
        """Follows parent pointers backwards from goal to start."""
        path = [[self.goal.x, self.goal.y]]
        curr = end_node
        while curr.parent is not None:
            path.append([curr.x, curr.y])
            curr = curr.parent
        path.append([self.start.x, self.start.y])
        return path[::-1]

def main():
    # Start: Behind the U (identical to APF spawn)
    # Goal: A target point on the line
    start_pos = (50.0, 90.0)
    goal_pos = (36.5, 40.0) 
    
    rrt = RRT(start_pos, goal_pos, WALLS)
    final_path = rrt.plan()
    
    # --- Visualization ---
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, ENV_WIDTH)
    ax.set_ylim(0, ENV_HEIGHT)
    ax.set_aspect('equal')
    ax.set_title("Single Robot RRT Path Planning")
    
    # Draw U-obstacle walls
    for w_start, w_end in WALLS:
        ax.plot([w_start[0], w_end[0]], [w_start[1], w_end[1]], 'r-', linewidth=3, label='Wall' if w_start == WALLS[0][0] else "")
    
    # Draw the RRT Tree growth
    for node in rrt.node_list:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-', linewidth=0.5, alpha=0.3)
    
    # Draw start and goal
    ax.scatter([start_pos[0]], [start_pos[1]], c='green', s=100, label='Start', zorder=5)
    ax.scatter([goal_pos[0]], [goal_pos[1]], c='orange', s=100, label='Goal', zorder=5)
    
    if final_path:
        path_np = np.array(final_path)
        ax.plot(path_np[:, 0], path_np[:, 1], 'g-', linewidth=3, label='Final Path', zorder=6)
        print(f"Final Path Length: {len(final_path)} nodes.")
    
    ax.legend(loc='upper right')
    print("\nSimulation Complete. Displaying Plot...")
    plt.show()

if __name__ == "__main__":
    main()
