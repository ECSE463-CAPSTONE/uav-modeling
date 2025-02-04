import numpy as np
from utilities.rotations import R_x, R_y, R_z, R_x_dot, R_y_dot, R_z_dot
from utilities.logger import log, consolidate_logs

class Dynamics:
    def __init__(self, mass, inertia_matrix):
        self.mass = mass  # Mass of the system
        self.inertia_matrix = inertia_matrix  # Inertia matrix I
        self.tracked_data = {}  # Store logs for each iteration

    def compute_accelerations(self, velocities, dt, forces, moments):
        """Compute the accelerations by solving Ax = B with dynamic dt."""
        u, v, w, p, q, r = velocities

        A = np.array([
            [1, -r * dt, q * dt, 0, w * dt, -v * dt],
            [r * dt, 1, -p * dt, -w * dt, 0, u * dt],
            [-q * dt, p * dt, 1, v * dt, -u * dt, 0],
            [0, 0, 0, self.inertia_matrix[0, 0], 0, -self.inertia_matrix[0, 2]],
            [0, 0, 0, 0, self.inertia_matrix[1, 1], -self.inertia_matrix[0, 2]],
            [0, 0, 0, -self.inertia_matrix[0, 2], 0, self.inertia_matrix[2, 2]]
        ])

        B = np.array([
            forces[0] / self.mass - (q * w - r * v),
            forces[1] / self.mass - (r * u - p * w),
            forces[2] / self.mass - (p * v - q * u),
            moments[0] - ((self.inertia_matrix[1, 1] - self.inertia_matrix[2, 2]) * q * r - self.inertia_matrix[0, 2] * (p * q)),
            moments[1] - ((self.inertia_matrix[2, 2] - self.inertia_matrix[0, 0]) * p * r + self.inertia_matrix[0, 2] * (r**2 - p**2)),
            moments[2] - ((self.inertia_matrix[0, 0] - self.inertia_matrix[1, 1]) * p * q - self.inertia_matrix[0, 2] * p)
        ])
        
        accelerations = np.linalg.solve(A, B)
       
        return accelerations