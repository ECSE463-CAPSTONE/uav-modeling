import numpy as np

# import sys
# sys.path.append('./model')

# from model.Force import *
# from model.RigidBody import *

from model import Force, ControlForce, TowingForce, RigidBody


mass = 0.7  # kg
volume = 0.00075  # m^3
COM = np.array([0, 0])  # Center of mass [x, z]
center_of_buoyancy = np.array([-0.05, -0.0])  # Center of buoyancy [x, z]

Iyy = 1/12*mass*(3 * 0.1**2 + 0.2**2) # kg*m^2

N = 10

# Initialize rigid body
rigid_body = RigidBody(mass, volume, Iyy, COM, center_of_buoyancy, N)

# Initialize forces
control_force_location = np.array([0, -0.15])
towing_force_location = np.array([0.05,0.05])
A, C_L_alpha, C_D0 = [0.1, 0.1, 0.2 ]

control_force = ControlForce(control_force_location, N, A, C_L_alpha, C_D0 )  # Example control force
    
towing_length = 5 # m
initial_towing_magnitude = 15 # N
towing_force = TowingForce(towing_force_location, N, 3, initial_towing_magnitude)  # Example towing force
    
# Add forces to the body
rigid_body.add_control_force(control_force)
rigid_body.add_tow_force(towing_force)


# Initial Conditions:
inertial_position = np.array([0, -2, np.deg2rad(5)]) # [x, y, theta]
initial_inertial_velocity = np.array([2, 0, 0])

# Simulate using the ODE solver
solution = rigid_body.simulate_forward_euler(inertial_position, initial_inertial_velocity, 0.01)

