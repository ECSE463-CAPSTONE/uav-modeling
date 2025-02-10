import numpy as np
import pandas as pd
from scipy.interpolate import griddata
from utilities.angles_of_attack import compute_velocity_alpha_beta
import utilities.rotations as R


#global variable
rho = 999.7


class HullForce:
    def __init__(self, mass, global_location, file_path: str):
        self.file_path = file_path
        self.map = self.load_map()

        #Parameters used for the inertia and COM calculations
        self.mass = mass
        self.global_location = global_location  # Global location initially not set

        self.relative_location = None  # Relative location to COM       

        self.body_forces = np.zeros(3)
        self.body_moments = np.zeros(3)

        self.tracked_data = {}

    def load_map(self):
        map = pd.read_csv(self.file_path)
        return map
  
    # UPDATE THIS FUNCTION TO TAKE THE VELOCITIES AND ROTATIONS
    #   SIMILAR TO CONTROL FORCE OR DONT YOU ALSO NEED THE ATTITUDES
    #   I WILL FIX THE RUN SIMULATION ACCORDINGLY

    def calculate_force(self, velocity_states):
        # The main function to calculate all the forces
        V, alpha, beta = compute_velocity_alpha_beta(velocity_states, self.relative_location)
        coefficients = self.get_coefficients(alpha, beta)

        #[Fx, Fy, Fz, Mx", My, Mz] need to check if they are in the right direction!!
        force_moments = 0.5 * rho * coefficients * V ** 2 #forces and moments at the nose

        rotated_force_moments = self.rotate_force(force_moments, alpha, beta)

        #this line returns a 6x1 column vector of [Fx, Fy, Fz, Mx, My, Mz] about the global center of mass
        translated_rotated_force_moments = self.translate_force(rotated_force_moments)
        return translated_rotated_force_moments
        
    
    def get_coefficients(self, alpha, beta):       
        yaw_pitch_points = self.map[["Yaw", "Pitch"]].to_numpy()
        values = self.map[["CDA", "CSA", "CLA", "CmXA", "CmYA", "CmZA"]].to_numpy()
        coefficients = griddata(yaw_pitch_points, values, (beta, alpha), method='linear')
        return np.array(coefficients) #["CDA", "CSA", "CLA", "CmXA", "CmYA", "CmZA"]

    
    def translate_force(self, rotated_force_moments):
        #Translate forces from nose to COM
        forces = rotated_force_moments[:3, 0]
        moments = rotated_force_moments[3:, 0]

        #add hull com location to relative global com location to get nose to global com distance
        com_global_location = self.relative_location + self.global_location
        
        #com global location comes from the rigid body object
        cross_product = np.cross(forces, com_global_location.flatten())  # Compute cross product
        translated_moments = moments + cross_product.reshape(3, 1)  # Add to v2

        translated_rotated_force_moments = np.vstack((forces.reshape(3, 1), translated_moments))  # Stack vertically

        return translated_rotated_force_moments
    
    def rotate_force(self, force_moments, alpha, beta):
        # transform force from flow to body frame
        T = R.R_z(beta) @ R.R_y(alpha)
        zero_block = np.zeros_like(T)
        rotated_force_moments = np.block([[T, zero_block], [zero_block, T]]) @ force_moments.T
        return rotated_force_moments
    
    def calculate_relative_location(self, COM):
        """Calculate the relative location of the control force with respect to the rigid body's center of mass."""
        if self.global_location is None:
            raise ValueError("Global location is not set.")
        self.relative_location = self.global_location - COM