import numpy as np
from scipy.optimize import minimize
from simulation.Simulation import Simulation
from simulation.Dynamics import Dynamics
from utilities.rotations import T_velocity

class Equilibrium:
    def __init__(self, simulation: Simulation, num_iterations=100, tolerance=1e-5):
        self.simulation = simulation
        self.rigid_body = simulation.rigid_body
        self.tow_force = simulation.tow_force
        self.hull_force = simulation.hull_force
        self.control_forces = simulation.control_forces
        self.num_iterations = num_iterations
        self.tolerance = tolerance
        self.equilibrium_logs = {}

        self.dynamics : Dynamics

        self.state = np.zeros(12)  # Assuming 12 state variables [x, y, z, roll, pitch, yaw, u, v, w, p, q, r]
    
    def initialize_system(self):
        """Initialize the simulation system before solving for equilibrium."""
        self.simulation.initialize()
        self.dynamics = self.simulation.dynamics
    
    def update_forces_and_moments(self, state):
        """Update all forces and moments based on the current state."""

        roll, pitch, yaw = state[3:6]
        velocities = state[6:12]
        T = T_velocity(roll, pitch, yaw)
        bf_velocities = T.T @ velocities

        for cf in self.control_forces.values():
            cf['force'].calculate_force(bf_velocities)
        self.hull_force.calculate_force(bf_velocities)
        
        attidude_states = np.array([roll, pitch, yaw])
        return self.rigid_body.sum_forces_moments(attidude_states)
    
    def objective_function(self, variables):
        """
        Objective function to minimize accelerations for equilibrium.
        Variables: [tow_force_magnitude, pitch_angle]
        """
        tow_force_magnitude, pitch_angle = variables
        
        # Apply new values
        self.tow_force.set_magnitude(tow_force_magnitude, pitch_angle)
        
        # Initialize state
        self.state[4] = pitch_angle   
            
        forces, moments = self.update_forces_and_moments(self.state)
        
        # Check for equilibrium convergence (forces & moments close to zero)
        if np.linalg.norm(forces) < self.tolerance and np.linalg.norm(moments) < self.tolerance:
            return 0  # Equilibrium found
            
        
        return np.linalg.norm(forces) + np.linalg.norm(moments)  # Minimize total force/moment magnitude
    
    def solve(self, velocity , initial_guess):
        """Solve for equilibrium state using optimization."""
        self.initialize_system()  # Ensure the system is initialized before solving

        self.state[6] = velocity
        initial_tow_force, initial_pitch = initial_guess
        result = minimize(
            self.objective_function, 
            x0=[initial_tow_force, initial_pitch], 
            bounds=[(0, None), (-np.pi/4, np.pi/4)],  # Tow force must be positive, pitch within reasonable range
            method='SLSQP'
        )
        
        if result.success:
            optimal_tow_force, optimal_pitch = result.x
            print(f"Equilibrium found: Tow Force = {optimal_tow_force}, Pitch Angle = {optimal_pitch}")
            return optimal_tow_force, optimal_pitch
        else:
            print("Equilibrium not found.")
            return None
