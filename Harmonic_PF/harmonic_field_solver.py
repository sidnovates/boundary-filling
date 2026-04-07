"""
Harmonic potential field solver utilities.

This module provides a grid-based Laplace solver with Dirichlet boundary
conditions and helper methods for sampling potential and gradients in world
coordinates.
"""

import math
import numpy as np


class HarmonicFieldSolver:
    """2D harmonic field solved on a regular grid."""

    def __init__(
        self,
        width,
        height,
        cell_size=1.0,
        max_iters=1200,
        tolerance=1e-4,
        sor_omega=1.85,
    ):
        self.width = float(width)
        self.height = float(height)
        self.cell_size = float(cell_size)
        self.max_iters = int(max_iters)
        self.tolerance = float(tolerance)
        self.sor_omega = float(sor_omega)

        self.nx = int(round(self.width / self.cell_size)) + 1
        self.ny = int(round(self.height / self.cell_size)) + 1

        self.potential = np.full((self.ny, self.nx), 0.5, dtype=float)
        self.fixed_mask = np.zeros((self.ny, self.nx), dtype=bool)
        self.fixed_values = np.zeros((self.ny, self.nx), dtype=float)

        self.last_solve_iterations = 0
        self.last_residual = float("inf")

    def reset(self, initial_value=0.5):
        self.potential.fill(float(initial_value))
        self.fixed_mask.fill(False)
        self.fixed_values.fill(0.0)
        self.last_solve_iterations = 0
        self.last_residual = float("inf")

    def world_to_grid(self, x, y):
        i = int(round(float(x) / self.cell_size))
        j = int(round(float(y) / self.cell_size))
        i = max(0, min(self.nx - 1, i))
        j = max(0, min(self.ny - 1, j))
        return i, j

    def grid_to_world(self, i, j):
        x = i * self.cell_size
        y = j * self.cell_size
        return x, y

    def _set_dirichlet_cell(self, i, j, value):
        if 0 <= i < self.nx and 0 <= j < self.ny:
            self.fixed_mask[j, i] = True
            self.fixed_values[j, i] = float(value)
            self.potential[j, i] = float(value)

    def set_outer_boundary(self, value):
        v = float(value)
        self.fixed_mask[0, :] = True
        self.fixed_mask[-1, :] = True
        self.fixed_mask[:, 0] = True
        self.fixed_mask[:, -1] = True

        self.fixed_values[0, :] = v
        self.fixed_values[-1, :] = v
        self.fixed_values[:, 0] = v
        self.fixed_values[:, -1] = v

        self.potential[0, :] = v
        self.potential[-1, :] = v
        self.potential[:, 0] = v
        self.potential[:, -1] = v

    def add_segment_dirichlet(self, p1, p2, value, thickness=0.75):
        x1, y1 = float(p1[0]), float(p1[1])
        x2, y2 = float(p2[0]), float(p2[1])
        thickness = max(0.0, float(thickness))

        seg_len = math.hypot(x2 - x1, y2 - y1)
        sample_step = max(1e-6, 0.5 * self.cell_size)
        sample_count = max(2, int(math.ceil(seg_len / sample_step)) + 1)

        radius_cells = int(math.ceil(thickness / self.cell_size))

        for t in np.linspace(0.0, 1.0, sample_count):
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            ic, jc = self.world_to_grid(x, y)

            for dj in range(-radius_cells, radius_cells + 1):
                for di in range(-radius_cells, radius_cells + 1):
                    ii = ic + di
                    jj = jc + dj
                    if not (0 <= ii < self.nx and 0 <= jj < self.ny):
                        continue

                    wx, wy = self.grid_to_world(ii, jj)
                    if math.hypot(wx - x, wy - y) <= thickness + 0.35 * self.cell_size:
                        self._set_dirichlet_cell(ii, jj, value)

    def add_points_dirichlet(self, points, value, radius=0.8):
        radius = max(0.0, float(radius))
        radius_cells = int(math.ceil(radius / self.cell_size))

        for x, y in points:
            ic, jc = self.world_to_grid(x, y)
            for dj in range(-radius_cells, radius_cells + 1):
                for di in range(-radius_cells, radius_cells + 1):
                    ii = ic + di
                    jj = jc + dj
                    if not (0 <= ii < self.nx and 0 <= jj < self.ny):
                        continue

                    wx, wy = self.grid_to_world(ii, jj)
                    if math.hypot(wx - x, wy - y) <= radius + 0.35 * self.cell_size:
                        self._set_dirichlet_cell(ii, jj, value)

    def solve(self):
        if not np.any(self.fixed_mask):
            raise ValueError("No boundary conditions were set before solve().")

        omega = self.sor_omega
        max_delta = float("inf")
        iters_used = self.max_iters

        for iteration in range(1, self.max_iters + 1):
            max_delta = 0.0

            for j in range(1, self.ny - 1):
                for i in range(1, self.nx - 1):
                    if self.fixed_mask[j, i]:
                        continue

                    old_val = self.potential[j, i]
                    avg_nb = 0.25 * (
                        self.potential[j + 1, i]
                        + self.potential[j - 1, i]
                        + self.potential[j, i + 1]
                        + self.potential[j, i - 1]
                    )
                    new_val = (1.0 - omega) * old_val + omega * avg_nb
                    self.potential[j, i] = new_val

                    delta = abs(new_val - old_val)
                    if delta > max_delta:
                        max_delta = delta

            # Keep all Dirichlet constraints exact after each sweep.
            self.potential[self.fixed_mask] = self.fixed_values[self.fixed_mask]

            if max_delta < self.tolerance:
                iters_used = iteration
                break

        self.last_solve_iterations = iters_used
        self.last_residual = max_delta
        return iters_used, max_delta

    def sample_potential(self, x, y):
        x = max(0.0, min(self.width, float(x)))
        y = max(0.0, min(self.height, float(y)))

        gx = x / self.cell_size
        gy = y / self.cell_size

        i0 = int(math.floor(gx))
        j0 = int(math.floor(gy))
        i1 = min(i0 + 1, self.nx - 1)
        j1 = min(j0 + 1, self.ny - 1)

        tx = gx - i0
        ty = gy - j0

        v00 = self.potential[j0, i0]
        v10 = self.potential[j0, i1]
        v01 = self.potential[j1, i0]
        v11 = self.potential[j1, i1]

        v0 = (1.0 - tx) * v00 + tx * v10
        v1 = (1.0 - tx) * v01 + tx * v11
        return (1.0 - ty) * v0 + ty * v1

    def sample_gradient(self, x, y, step=None):
        h = self.cell_size if step is None else max(1e-6, float(step))

        x_plus = min(self.width, float(x) + h)
        x_minus = max(0.0, float(x) - h)
        y_plus = min(self.height, float(y) + h)
        y_minus = max(0.0, float(y) - h)

        vx_plus = self.sample_potential(x_plus, y)
        vx_minus = self.sample_potential(x_minus, y)
        vy_plus = self.sample_potential(x, y_plus)
        vy_minus = self.sample_potential(x, y_minus)

        dx_span = max(1e-9, x_plus - x_minus)
        dy_span = max(1e-9, y_plus - y_minus)

        dUdx = (vx_plus - vx_minus) / dx_span
        dUdy = (vy_plus - vy_minus) / dy_span
        return np.array([dUdx, dUdy], dtype=float)
