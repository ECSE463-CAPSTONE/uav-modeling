import numpy as np
from scipy.integrate import solve_ivp

from .Force import Force, ControlForce, TowingForce

#Global variables
g = 9.81
rho = 1000

class RigidBody:
    def __init__(self, mass: float, volume: float, inertia: float, COM: np.array, center_of_buoyancy: np.array, N: int):
        self.mass = mass
        self.volume = volume
        self.Iyy = inertia
        self.COM = COM  # Center of mass [x, z]
        self.center_of_buoyancy = center_of_buoyancy  # Center of buoyancy [x, z]
        self.buoyancy  = rho * g * self.volume

        # Store forces and moments
        self.control_forces = []  # List of forces acting on the body
        self.tow_force : Force
        self.moments = []  # List of moments about the y-axis (pitch)

        # Initial conditions for position, velocity, and pitch
        self.position = np.zeros((N, 2))  # 2D position [x, z]
        self.velocity = np.zeros((N, 2))  # 2D velocity [u, w]
        self.pitch_angle = np.zeros(N)  # Pitch angle (about y-axis)
        self.pitch_rate = np.zeros(N)  # Angular velocity (pitch rate)

        self.N = N
    
    def add_tow_force(self, force):
        self.tow_force = force

    def add_control_force(self, force):
        """Adds a force instance to the force list"""
        self.control_forces.append(force)
    
    def add_moment(self, moment):
        """Adds a moment about the y-axis"""
        self.moments.append(moment)
    
    def add_hull_force(self, force):
        self.hull_force = force

    ## Fix according to write up:
    def sum_forces(self, theta):
        """Sums up all forces acting on the body in x and z directions"""
        #Extract pitch angle for current iteration
        # theta = self.pitch_angle[i]

        total_force_x = -(self.mass * 9.81 - self.buoyancy) *  np.sin(theta) \
                            + self.tow_force.magnitude * np.cos(self.tow_force.delta_t - theta) \
                            - np.sum([f.magnitude[0] * np.cos(f.alpha_i) for f in self.control_forces]) \
                            + np.sum([f.magnitude[1] * np.sin(f.alpha_i) for f in self.control_forces]) \
                            - self.hull_force.magnitude[0] * np.cos(theta) \
                            + self.hull_force.magnitude[1] * np.sin(theta)
        
        total_force_z = (self.mass * 9.81 - self.buoyancy) *  np.cos(theta) \
                            - self.tow_force.magnitude * np.sin(self.tow_force.delta_t - theta) \
                            - np.sum([f.magnitude[0] * np.sin(f.alpha_i) for f in self.control_forces]) \
                            - np.sum([f.magnitude[1] * np.cos(f.alpha_i) for f in self.control_forces]) \
                            - self.hull_force.magnitude[0] * np.sin(theta) \
                            - self.hull_force.magnitude[1] * np.cos(theta)
        
        return np.array([total_force_x, total_force_z])
    
    def sum_moments(self, theta):
        """Sums up all moments acting on the body about the y-axis (pitch)"""
        #Extract pitch angle for current iteration
        # theta = self.pitch_angle[i]

        #Buoyancy moment
        buoyancy_moment = self.buoyancy * (self.center_of_buoyancy[0] * np.cos(theta) + \
                                           self.center_of_buoyancy[1] * np.sin(theta))
        
        #Tow Force Moment
        tow_force_moment = self.tow_force.magnitude * (np.sin(self.tow_force.delta_t - theta) * self.tow_force.location[0] +\
                                                       np.cos(self.tow_force.delta_t - theta) * self.tow_force.location[1])

        #Control Force Moment
        control_force_moment = np.sum([(f.location[0] * (f.magnitude[0] * np.sin(f.alpha_i) + f.magnitude[1] * np.cos(f.alpha_i)) \
                                        +f.location[1] * (-f.magnitude[0] * np.cos(f.alpha_i) + f.magnitude[1] * np.sin(f.alpha_i))) \
                                        for f in self.control_forces])

        #Hull Force Moment
        hull_force_moment = self.hull_force.magnitude[0] *(self.hull_force.location[0] * np.sin(theta) \
                                                           -self.hull_force.location[1] * np.cos(theta)) \
                            +self.hull_force.magnitude[1] *(self.hull_force.location[0] * np.cos(theta) \
                                                           -self.hull_force.location[1] * np.sin(theta))
        
        #Sum
        total_moment_y = buoyancy_moment + tow_force_moment + control_force_moment + hull_force_moment

        return total_moment_y
    
    