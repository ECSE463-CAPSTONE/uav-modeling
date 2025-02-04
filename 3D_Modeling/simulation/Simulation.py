import numpy as np
from utilities.logger import log, consolidate_logs, flatten_logs

class Simulation:
    def __init__(self, dynamics, rigid_body, control_forces, tow_force, hull_force, dt, t_span, method='euler'):
        self.dynamics = dynamics
        self.rigid_body = rigid_body
        self.control_forces = control_forces  # Dictionary of control force objects and positions
        self.tow_force = tow_force
        self.hull_force = hull_force
        self.dt = dt
        self.t_span = t_span
        self.method = method.lower()  # Either 'euler' or 'rk4'
        self.simulation_logs = {}
        self.flattened_logs = {}

    def initialize(self):
        """Initialize the simulation by adding forces to the rigid body."""
        for name, data in self.control_forces.items():
            self.rigid_body.add_control_force(data['force'], data['position'])
        self.rigid_body.add_tow_force(self.tow_force)
        self.rigid_body.add_hull_force(self.hull_force)

    def log_iteration(self, t, state):
        """Logs simulation data for each iteration."""
        control_force_logs = {name: cf['force'].tracked_data.copy() for name, cf in self.control_forces.items()}
        iteration_logs = consolidate_logs({
            'timestamp': t,
            'rigid_body': self.rigid_body.tracked_data.copy(),
            'control_forces': control_force_logs,
            'hull_force': self.hull_force.tracked_data.copy(),
            'tow_force': self.tow_force.tracked_data.copy(),
            'dynamics': log([state])
        })
        self.simulation_logs[t] = iteration_logs
        self.flattened_logs[t] = flatten_logs(iteration_logs)
    
    def run_simulation(self, initial_state):
        """Custom solver for time-stepping simulation using Euler or RK4."""
        self.initialize()  # Ensure all forces are added before simulation starts
        timesteps = np.arange(self.t_span[0], self.t_span[1], self.dt)
        state = initial_state.copy()

        # State variables: [x, y, z, roll, pitch, yaw, u, v, w, p, q, r]
        # Positions: x, y, z
        # Attitudes: roll, pitch, yaw
        # Velocities: u, v, w (linear), p, q, r (angular)
        
        for t in timesteps:
            forces, moments = self.rigid_body.compute_forces_and_moments()
            velocities = state[6:12]
            state = self._step(state, velocities, forces, moments)
            self.log_iteration(t, state)
        
        return timesteps, state, self.simulation_logs, self.flattened_logs

    def _step(self, state, velocities, forces, moments):
        """Select integration method (Euler or RK4)."""
        if self.method == 'euler':
            return self._euler_step(state, velocities, forces, moments)
        elif self.method == 'rk4':
            return self._rk4_step(state, velocities, forces, moments)
        else:
            raise ValueError("Invalid integration method. Choose 'euler' or 'rk4'.")

    def _euler_step(self, state, velocities, forces, moments):
        """Euler integration step."""
        accelerations = self.dynamics.compute_accelerations(velocities, self.dt, forces, moments)
        state[6:12] += accelerations * self.dt  # Update velocities
        state[:6] += state[6:12] * self.dt  # Update positions and attitudes
        return state

    def _rk4_step(self, state, velocities, forces, moments):
        """Runge-Kutta 4th order integration step."""
        k1 = self.dynamics.compute_accelerations(velocities, self.dt, forces, moments)
        k2 = self.dynamics.compute_accelerations(velocities + 0.5 * self.dt * k1, self.dt, forces, moments)
        k3 = self.dynamics.compute_accelerations(velocities + 0.5 * self.dt * k2, self.dt, forces, moments)
        k4 = self.dynamics.compute_accelerations(velocities + self.dt * k3, self.dt, forces, moments)
        
        state[6:12] += (k1 + 2*k2 + 2*k3 + k4) * (self.dt / 6.0)  # RK4 update for velocities
        state[:6] += state[6:12] * self.dt  # Update positions and attitudes
        return state
