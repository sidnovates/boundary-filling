import matplotlib.pyplot as plt
import numpy as np

# Setup figure
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-3, 10)
ax.set_ylim(-1, 10)

# 1. Draw the Wall
ax.plot([0, 0], [-1, 10], color='red', linewidth=7, label='Obstacle Wall')

# 2. Draw the Robot
rx, ry = 2.0, 4.0
robot = plt.Circle((rx, ry), 0.5, color='blue', fill=True, label='Robot')
ax.add_patch(robot)

# 3. Normal Vector (grad_rho)
# Robot is at x=2, wall is at x=0. Nearest wall point is (0, 4)
# Vector pointing straight away from wall towards robot is [1, 0]
nx, ny = 1.0, 0.0
ax.quiver(rx, ry, nx, ny, color='black', scale=8, width=0.008, label='Normal Repulsion (grad_rho)')

# 4. Two Possible Tangents in 2D Space
t1x, t1y = -ny, nx   # [0, 1] -> pointing UP
t2x, t2y = ny, -nx   # [0, -1] -> pointing DOWN

ax.quiver(rx, ry, t1x, t1y, color='orange', scale=8, width=0.008, label='Tangent 1 (t1)')
ax.quiver(rx, ry, t2x, t2y, color='purple', scale=8, width=0.008, label='Tangent 2 (t2)')

# 5. Target Location
tx, ty = 8.0, 8.0
ax.scatter([tx], [ty], color='green', s=300, marker='*', zorder=5, label='Target Destination')

# Vector towards the Target
tgt_vx, tgt_vy = tx - rx, ty - ry
tgt_norm = np.hypot(tgt_vx, tgt_vy)
tgt_ux, tgt_uy = tgt_vx / tgt_norm, tgt_vy / tgt_norm

ax.quiver(rx, ry, tgt_ux, tgt_uy, color='green', scale=8, width=0.008, 
          alpha=0.5, label='target_vector (Direction to Goal)')

# 6. Mathematical Evaluation Text
dot_t1 = np.dot([t1x, t1y], [tgt_ux, tgt_uy])
dot_t2 = np.dot([t2x, t2y], [tgt_ux, tgt_uy])

text_str = (
    f"MATH IN YOUR CODE:\n"
    f"--------------------\n"
    f"Dot Product (t1 · target_vector) = {dot_t1:.2f}\n"
    f"Dot Product (t2 · target_vector) = {dot_t2:.2f}\n\n"
    f"CONCLUSION:\n"
    f"Because {dot_t1:.2f} > {dot_t2:.2f}, Tangent 1 agrees\n"
    f"with the target direction much more than Tangent 2.\n\n"
    f"Therefore, the robot pushes UP (sliding along the red wall)\n"
    f"instead of getting stuck pushing left into the wall!"
)

plt.text(3, 0, text_str, fontsize=11, family='monospace', 
         bbox=dict(facecolor='white', edgecolor='black', alpha=0.9, boxstyle='round,pad=0.5'))

ax.legend(loc='upper left', fontsize='small')
ax.set_title("How the Tangential Formula Breaks Local Minima", fontsize=14, fontweight='bold')
ax.grid(True, linestyle='--', alpha=0.6)
ax.set_aspect('equal')

plt.show()
