import numpy as np
from utilities.logger import log, consolidate_logs, flatten_logs
from utilities.rotations import T_velocity

from Dynamics import Dynamics



class Simulation:
    def __init__(self, rigid_body, control_forces, tow_force, hull_force):
        self.rigid_body = rigid_body
        self.control_forces = control_forces  # Dictionary of control force objects and positions
        self.tow_force = tow_force
        self.hull_force = hull_force
        self.simulation_logs = {}
        self.flattened_logs = {}
        self.dynamics : Dynamics 

    def initialize(self):
        """Initialize the simulation by adding forces to the rigid body."""
        for name, data in self.control_forces.items():
            self.rigid_body.add_control_force(data['force'], data['position'])
        self.rigid_body.add_tow_force(self.tow_force)
        self.rigid_body.add_hull_force(self.hull_force)

        mass = self.rigid_body.calculate_inertia_matrix()
        inertia = self.rigid_body.calculate_inertia_matrix()
        self.dynamics = Dynamics(mass, inertia)

        self.rigid_body.calculate_COM()

    def log_iteration(self, iteration, state):
        """Logs simulation data for each iteration."""
        control_force_logs = {name: cf['force'].tracked_data.copy() for name, cf in self.control_forces.items()}
        iteration_logs = consolidate_logs({
            'iteration': iteration,
            'rigid_body': self.rigid_body.tracked_data.copy(),
            'control_forces': control_force_logs,
            'hull_force': self.hull_force.tracked_data.copy(),
            'tow_force': self.tow_force.tracked_data.copy(),
            'bf_accelerations': self.dynamics.tracked_data.copy(),
            'dynamics': log([state])
        })
        self.simulation_logs[iteration] = iteration_logs
        self.flattened_logs[iteration] = flatten_logs(iteration_logs)
    
   
    
    def update_forces(self, state):
        """Update all forces and moments based on the current state."""
        delta = state[0:3]
        roll, pitch, yaw = state[3:6]
        velocities = state[6:12]
        T = T_velocity(roll, pitch, yaw)
        bf_velocities = T.T @ velocities

        # Calculate the tow force
        self.tow_force.calculate_tow_force(delta, roll, pitch, yaw)
        # Update control forces based on velocities
        for cf in self.control_forces.values():
           cf['force'].calculate_force(bf_velocities)
        # Update hull force based on velocities
        self.hull_force.calculate_force(bf_velocities)
        forces, moments = self.rigid_body.compute_forces_and_moments()
        return velocities,forces,moments 
   
    def run_simulation(self, initial_state, dt, num_iterations, method='euler'):
        """Custom solver for time-stepping simulation using Euler or RK4."""
        self.initialize()  # Ensure all forces are added before simulation starts
        state = initial_state.copy()
        method = method.lower()

        # Inertial State variables: [x, y, z, roll, pitch, yaw, V_x, V_y, V_z, p, q, r]
        # Positions: x, y, z
        # Attitudes: roll, pitch, yaw
        # Velocities: V_x, V_y, V_z(linear), p, q, r (angular)
        
        for iteration in range(num_iterations):
            forces, moments = self.update_forces(self, state)
            
            # Update state using the selected integration method
            state = self._step(state, forces, moments, dt, method)
            self.log_iteration(iteration, state)
        
        return state, self.simulation_logs, self.flattened_logs


    def _step(self, state, velocities, forces, moments, dt, method):
        """Select integration method (Euler or RK4)."""
        if method == 'euler':
            return self._euler_step(state, velocities, forces, moments, dt)
        elif method == 'rk4':
            return self._rk4_step(state, velocities, forces, moments, dt)
        else:
            raise ValueError("Invalid integration method. Choose 'euler' or 'rk4'.")

    def _euler_step(self, state, forces, moments, dt):
        """Euler integration step."""
        roll, pitch, yaw = state[3:6]
        velocities = state[6:12]
        T = T_velocity(roll, pitch, yaw)
        bf_velocities = T.T @ velocities

        bf_accelerations = self.dynamics.compute_accelerations(bf_velocities, dt, forces, moments)

        bf_velocities += bf_accelerations * dt # Update bf_velocities
        
        ## DO WE HAVE TO UPDATE THE TRANSFORMATION MATRIX IT DEPENDS ON OLD YAW, PITCH, ROLL
        state[6:12] = T @ bf_velocities
        
        # Update positions and attitudes
        state[:6] += state[6:12] * dt  # Update positions and attitudes
        return state

    def _rk4_step(self, state, velocities, forces, moments, dt):
        """Runge-Kutta 4th order integration step."""
        roll, pitch, yaw = state[3:6]
        velocities = state[6:12]
        T = T_velocity(roll, pitch, yaw)
        bf_velocities = T.T @ velocities

        k1 = self.dynamics.compute_accelerations(bf_velocities, dt, forces, moments)
        k2 = self.dynamics.compute_accelerations(bf_velocities + 0.5 * dt * k1, dt, forces, moments)
        k3 = self.dynamics.compute_accelerations(bf_velocities + 0.5 * dt * k2, dt, forces, moments)
        k4 = self.dynamics.compute_accelerations(bf_velocities + dt * k3, dt, forces, moments)
        
        bf_velocities += (k1 + 2*k2 + 2*k3 + k4) * (dt / 6.0)  # RK4 update for velocities
        state[6:12] = T @ bf_velocities
        state[:6] += state[6:12] * dt  # Update positions and attitudes
        return state
