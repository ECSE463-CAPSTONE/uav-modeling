import numpy as np
import inspect
from utilities.rotations import R_x, R_y, R_z
from utilities.logger import log

class TowForce:
    def __init__(self, tow_rope_length, drone_height, probe_depth, tow_force_magnitude):
        self.tow_rope_length = tow_rope_length  # Length of the tow rope
        self.drone_height = drone_height  # Height of the drone above the water
        self.probe_depth = probe_depth  # Depth of the probe below the water
        self.tow_force_magnitude = tow_force_magnitude  # Constant tow force magnitude
        
        self.tow_global_vector = np.zeros(3)  # Tow vector in global frame
        self.tow_body_vector = np.zeros(3)  # Tow vector in body frame
        self.tow_unit_body_vector = np.zeros(3)  # Normalized body-frame vector
        self.tow_force = np.zeros(3)  # Final force in body frame
        
        self.tracked_data = {}  # Dictionary to store logged values
    
    def calculate_global_vector(self, delta=np.array([0, 0, 0])):
        """Calculate the global frame tow vector given perturbations."""
        r_xy = np.sqrt(self.tow_rope_length**2 - (self.drone_height + self.probe_depth)**2)
        
        self.tow_global_vector = np.array([
            r_xy + delta[0],
            delta[1],
            (self.drone_height + self.probe_depth) + delta[2]
        ])
        
        self.tracked_data.update(log([self.tow_global_vector]))
        return self.tow_global_vector
    
    def calculate_body_frame_vector(self, roll, pitch, yaw):
        """Transform the global tow vector to the probe's body frame."""
        if self.tow_global_vector is None:
            raise ValueError("Global vector must be calculated first.")
        
        T = R_z(yaw) @ R_y(pitch) @ R_x(roll)  # Rotation matrix
        self.tow_body_vector = T.T @ self.tow_global_vector  # Transform to body frame
        self.tow_unit_body_vector = self.tow_body_vector / np.linalg.norm(self.tow_body_vector)
        
        self.tracked_data.update(log([self.tow_body_vector, self.tow_unit_body_vector]))
        return self.tow_body_vector, self.tow_unit_body_vector
    
   
    def calculate_tow_force(self):
        """Calculate the tow force in body-frame coordinates."""
        if self.tow_unit_body_vector is None:
            raise ValueError("Body-frame unit vector must be calculated first.")
        
        self.tow_force = self.tow_force_magnitude * self.tow_unit_body_vector
        
        self.tracked_data.update(log([self.tow_force]))
        return self.tow_force
    
    
    def calculate_tow_force(self, delta, roll, pitch, yaw):
        """Calculate the tow force for a given perturbation and attitude."""
        self.calculate_global_vector(delta)
        self.calculate_body_frame_vector(roll, pitch, yaw)
        tow_force = self.calculate_tow_force()

        return tow_force
        
    
    def calculate_azimuth_elevation(self):
        """Calculate the azimuth and elevation angles in the body frame."""
        if self.tow_body_vector is None:
            raise ValueError("Body-frame vector must be calculated first.")
        
        azimuth = np.arctan2(self.tow_body_vector[1], self.tow_body_vector[0])
        elevation = np.arctan2(self.tow_body_vector[2], np.sqrt(self.tow_body_vector[0]**2 + self.tow_body_vector[1]**2))
        
        self.tracked_data.update(log([azimuth, elevation]))
        return azimuth, elevation
    
   