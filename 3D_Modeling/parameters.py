import numpy as np
from model.RigidBody import RigidBody
from model.ControlForce import ControlForce
from model.TowForce import TowForce
from model.HullForce import HullForce

# Define rigid body parameters
mass = 100.0  # kg
inertia = np.diag([50, 50, 50])  # Inertia matrix (diagonal assumption)
volume = 0.1  # m³
com = np.array([0, 0, 0])  # Center of mass
center_of_buoyancy = np.array([0, 0, -0.1])

# Define control forces (Dictionary of forces and their positions)
control_forces = {
    "horizontal_stabilizer": {
        "force": ControlForce(
            delta_i_h=5,
            delta_i_v=0,
            AR=3,
            area=0.1,
            chord=0.3,
            stall_threshold=15,
            C_L_alpha=6.28,
            C_L_alpha_offset=0.1,
            mass=1.5,
            inertia = np.zeros((3,3)),
            is_vertical=False
        ),
        "position": np.array([1.0, 0.0, -0.2])
    },
    "vertical_stabilizer": {
        "force": ControlForce(
            delta_i_h=3,
            delta_i_v=0,
            AR=4,
            area=0.2,
            chord=0.25,
            stall_threshold=12,
            C_L_alpha=5.5,
            C_L_alpha_offset=0.1,
            mass=1.2,
            inertia = np.zeros((3,3)),
            is_vertical=True
        ),
        "position": np.array([0.5, 0.0, -0.3])
    }
}

# Define tow force
tow_force = TowForce(tow_force_magnitude=50,
                     tow_rope_length=10, 
                     drone_height = 2,
                     probe_depth=2,
                     global_location=np.array([0, 0, -5]))

# Define hull force
hull_force = HullForce(mass = 5,
                       global_location = np.array([0,0,0]),
                       file_path = '3D_Modeling/data/Hull_AeroMap_V1.xlsx')

# Function to initialize rigid body and forces
def initialize_system_components():
    """Initialize and return the system components (rigid body, forces)."""
    rigid_body = RigidBody(mass, volume, inertia, com, center_of_buoyancy)
    return rigid_body, control_forces, tow_force, hull_force
