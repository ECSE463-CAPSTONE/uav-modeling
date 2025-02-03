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
    
    def add_moment(self, moment):
        """Adds a moment about the y-axis"""
        self.moments.append(moment)
    

    ## Fix according to write up:
    def sum_forces(self, i):
        """Sums up all forces acting on the body in x and z directions"""
        # print('Summing forces', self.tow_force.magnitude[i, 0] )
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
        COM = np.zeros(3)

        # After COM is calculated, the relative location is auto updated
        for control_forces in self.control_forces:
            control_forces.set_relative_location(COM)
        return
    
    def calculate_inertia(self):
        """Calculates the inertia of the body."""
        # Assume inertia is constant and acts at the center of mass
        return

    
   