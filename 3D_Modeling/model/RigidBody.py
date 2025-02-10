import numpy as np
from scipy.integrate import solve_ivp

from ControlForce import ControlForce
from TowForce import TowForce

#Global variables
g = 9.81
rho = 1000

class RigidBody:
    def __init__(self, mass: float, volume: float, inertia: float, COM: np.array, center_of_buoyancy: np.array, N: int):
        self.mass = mass
        self.volume = volume    
        self.I= inertia                                 # Inertia Matrix
        self.COM = COM                                  # Center of mass [x,
        self.center_of_buoyancy = center_of_buoyancy    # Center of buoyancy [x, z]
        self.buoyancy  = rho * g * self.volume


        # Store forces and moments
        self.control_forces = []  # List of forces acting on the body
        self.tow_force : TowForce
        self.moments = []  # List of moments about the y-axis (pitch)

        # Initial conditions for position, velocity, and pitch
        self.position = np.zeros((N, 2))  # 2D position [x, z]
        self.velocity = np.zeros((N, 2))  # 2D velocity [u, w]
        self.pitch_angle = np.zeros(N)  # Pitch angle (about y-axis)
        self.pitch_rate = np.zeros(N)  # Angular velocity (pitch rate)

        self.N = N
    
    def add_tow_force(self, force):
        self.tow_force = force

    def add_control_force(self, force, global_location):
        """Adds a force instance to the force list"""
        self.control_forces.set_global_location(global_location)
        self.control_forces.append(force)
    
    def add_hull_force(self, force):
        self.hull_force = force

    ## Fix according to write up:
    def sum_forces(self, i):
        """Sums up all forces acting on the body in x and z directions"""
        # print('Summing forces', self.tow_force.magnitude[i, 0] )
        # USE SELF.CONTROL_FORCE.BODY_FORCE TO GET THE FORCE IN THE BODY FRAME
        return 
    
    def sum_moments(self, i):
        
        return 
    
    
    def initialize_system(self, initial_inertial_position, initial_inertial_velocity): 
        """Initializes the system with the initial position and velocity of the body."""
        #FEED COM TO FORCE
        return 




    def calculateCOM(self):
        """Calculates the center of mass of the body."""
        # Assume control forces will have an associated mass and COM wrt wing tip      

        #add up all the masses together
        total_mass = self.hull_force.mass + sum(control_force['mass'] for control_force in self.control_forces)


        positions = np.array(self.hull_force.global_location)  # Shape (N, 3) for 3D or (N, 2) for 2D
        masses = np.array(self.hull_force.mass)  # Shape (N,)
        
        for control_forces in self.control_forces:
            positions.append(control_forces.global_location)
            masses.append(control_forces.mass)

        COM = np.sum(positions.T * masses, axis=1) / total_mass

        # After COM is calculated, the relative location is auto updated
        self.hull_force.calculate_relative_location(COM)
        for control_forces in self.control_forces:
            control_forces.calculate_relative_location(COM)

        return COM, total_mass

    
    def calculate_inertia(self):
        """Calculates the inertia of the body."""
        # Assume inertia is constant and acts at the center of mass
        positions = np.array(self.hull_force.global_location)  # Shape (N, 3) for 3D or (N, 2) for 2D
        masses = np.array(self.hull_force.mass)  # Shape (N,)
        
        for control_forces in self.control_forces:
            positions.append(control_forces.global_location)
            masses.append(control_forces.mass)

        x, y, z = positions.T
        Ixx = np.sum(masses * (y**2 + z**2))
        Iyy = np.sum(masses * (x**2 + z**2))
        Izz = np.sum(masses * (x**2 + y**2))
        Ixy = -np.sum(masses * (x * y))
        Ixz = -np.sum(masses * (x * z))
        Iyz = -np.sum(masses * (y * z))

        inertia_matrix = np.array([
            [Ixx, Ixy, Ixz],
            [Ixy, Iyy, Iyz],
            [Ixz, Iyz, Izz]
        ])
        #Ixx = inertia[0, 0]
        #Iyy = inertia[1, 1]
        #Izz = inertia[2, 2]
        #Ixz = inertia[0, 2]
        return inertia_matrix

    def calculate_mass(self):
        """Calculates the mass of the body."""
        # Assume mass is constant
        mass = self.mass + sum(control_force.mass for control_force in self.control_forces)
        return mass
    
   