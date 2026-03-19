import matplotlib.pyplot as plt
import numpy as np

# Setup figure
fig, ax = plt.subplots(figsize=(10, 9))
ax.set_xlim(-4, 12)
ax.set_ylim(-7, 9)

# 1. Draw the Wall (Ending abruptly to create a corner)
wall_x, wall_y_min, wall_y_max = 2.0, 2.0, 8.0
ax.plot([wall_x, wall_x], [wall_y_min, wall_y_max], color='red', linewidth=7, label='Obstacle Wall')
ax.scatter([wall_x], [wall_y_min], color='darkred', s=200, zorder=4, label='Wall Corner Tip')

# 2. Draw the Robot (Stuck directly below the corner)
rx, ry = 2.0, 0.0
robot = plt.Circle((rx, ry), 0.5, color='blue', fill=True, label='Robot')
ax.add_patch(robot)

# 3. Normal Vector (grad_rho)
# Robot is at (2, 0). Nearest point on wall segment ((2,2), (2,8)) is the tip (2, 2).
# Vector pointing from tip to robot is (0, -2). Normalized is (0, -1).
nx, ny = 0.0, -1.0
ax.quiver(rx, ry, nx, ny, color='black', scale=8, width=0.008, label='Normal Repulsion (grad_rho)')

# 4. Two Possible Tangents in 2D Space
t1x, t1y = -ny, nx   # [1, 0] -> pointing RIGHT
t2x, t2y = ny, -nx   # [-1, 0] -> pointing LEFT

ax.quiver(rx, ry, t1x, t1y, color='orange', scale=8, width=0.008, label='Tangent 1 (t1 - Right)')
ax.quiver(rx, ry, t2x, t2y, color='purple', scale=8, width=0.008, label='Tangent 2 (t2 - Left)')

# 5. Target Location (The doorway/opening to the right)
tx, ty = 10.0, 2.0
ax.plot([wall_x, tx], [wall_y_min, ty], color='gray', linestyle='--', linewidth=3, label='Target Line')
ax.scatter([tx/2 + wall_x/2], [ty], color='green', s=300, marker='*', zorder=5, label='Target Destination Center')

# Vector towards target center
tgt_cx, tgt_cy = (tx + wall_x)/2, ty
tgt_vx, tgt_vy = tgt_cx - rx, tgt_cy - ry
tgt_norm = np.hypot(tgt_vx, tgt_vy)
tgt_ux, tgt_uy = tgt_vx / tgt_norm, tgt_vy / tgt_norm

ax.quiver(rx, ry, tgt_ux, tgt_uy, color='green', scale=8, width=0.008, 
          alpha=0.5, label='target_vector (Direction to Goal)')

# 6. Mathematical Evaluation Text
dot_t1 = np.dot([t1x, t1y], [tgt_ux, tgt_uy])
dot_t2 = np.dot([t2x, t2y], [tgt_ux, tgt_uy])

text_str = (
    f"MATH AT THE CORNER (Classic Local Minimum):\n"
    f"--------------------\n"
    f"Without sliding forces, Normal Repulsion violently pushes down.\n"
    f"Target Attraction blindly pulls up-and-right.\n"
    f"The forces cancel, and the Robot gets stuck bouncing up and down!\n\n"
    f"Dot Product (t1 [Right] · target_vector) = {dot_t1:.2f}\n"
    f"Dot Product (t2 [Left]  · target_vector) = {dot_t2:.2f}\n\n"
    f"CONCLUSION:\n"
    f"Because {dot_t1:.2f} > {dot_t2:.2f}, Tangent 1 heavily agrees\n"
    f"with the target direction.\n\n"
    f"The robot immediately sweeps RIGHT, gracefully slipping\n"
    f"underneath the red tip and sliding cleanly onto the target line!"
)

plt.text(-3.5, -6.5, text_str, fontsize=10, family='monospace', 
         bbox=dict(facecolor='white', edgecolor='black', alpha=0.9, boxstyle='round,pad=0.5'))

ax.legend(loc='upper right', fontsize='small')
ax.set_title("Solving the Corner Trap via Tangential Sliding", fontsize=14, fontweight='bold')
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_aspect('equal')

plt.show()
