import numpy as np
import inspect
from utilities.rotations import R_y, R_z
from utilities.logger import log
from utilities.angles_of_attack import compute_velocity_alpha_beta

rho = 999.7  # Density of fluid (kg/m^3)
g = 9.81  # Gravitational acceleration (m/s^2)
mu = 0.001308  # Dynamic viscosity (Pa.s)

class ControlForce:
    def __init__(self, delta_i_h, delta_i_v, AR, area, chord, stall_threshold, C_L_alpha, C_L_alpha_offset, mass, is_vertical=False, e=0.85):
        self.AR = AR  # Aspect ratio
        self.Area = area  # Surface area
        self.chord = chord  # Chord length
        self.COM_rel_to_edge = 1 / 4 * self.chord  # Quarter chord assumption
        self.global_location = None  # Global location initially not set
        self.relative_location = None  # Relative location to COM
        self.mass = mass  # Mass of the control force
        
        self.stall_threshold = stall_threshold
        self.C_L_alpha = C_L_alpha  # Slope of 2D lift curve
        self.C_L_alpha_offset = C_L_alpha_offset  # CL Alpha offset
        self.e = e  # Oswald efficiency factor
        
        self.delta_i_h = delta_i_h  # Fixed horizontal control surface angle of attack
        self.delta_i_v = delta_i_v  # Fixed vertical control surface angle of attack
        
        self.is_vertical = is_vertical  # Determines if the surface is vertical
        
        self.body_forces = np.zeros(3)  # Initialize body forces
        self.tracked_data = {}  # Dictionary to store selected variables
    
    def set_global_location(self, global_location):
        """Assign the global location of the control force relative to nose"""
        self.global_location = np.array(global_location)
        if self.is_vertical:
            self.global_location += np.array([0, self.COM_rel_to_edge, 0])
        else:
            self.global_location += np.array([self.COM_rel_to_edge, 0, 0])
    
    def calculate_relative_location(self, COM):
        """Calculate the relative location of the control force with respect to the rigid body's center of mass."""
        if self.global_location is None:
            raise ValueError("Global location is not set.")
        self.relative_location = self.global_location - COM
    
    def calculate_reynolds_number(self, V):
        """Calculate the Reynolds number based on velocity."""
        return rho * V * self.chord / mu
    
    def calculate_velocity_alpha_beta(self, bf_velocity):
        """Compute the velocity, angle of attack, and sideslip angle using a utility function."""
        V, alpha_i, beta_i = compute_velocity_alpha_beta(bf_velocity, self.relative_location)
        
        # Swap alpha and beta if the control force is vertical
        if self.is_vertical:
            alpha_i, beta_i = beta_i, alpha_i
        
        # Add fixed deflections
        alpha_i += self.delta_i_h
        beta_i += self.delta_i_v
        
        return V, alpha_i, beta_i
    
    def calculate_cl_cd(self, alpha_i, beta_i, Re):
        """Calculate lift and drag coefficients based on angle of attack and Reynolds number."""
        AoA = np.clip(alpha_i, -np.deg2rad(self.stall_threshold), np.deg2rad(self.stall_threshold))
        
        # 3D lift coefficient correction factor
        kappa = self.AR / (self.AR + 2 * (self.AR + 4) / (self.AR + 2))
        Cl = (self.C_L_alpha * kappa * AoA) + self.C_L_alpha_offset
        
        # Induced and skin drag
        Cd_induced = (Cl ** 2) / (np.pi * self.AR * self.e)
        Cd_skin = 0.0576 / (Re ** (1 / 5))
        
        return Cl, Cd_induced, Cd_skin, AoA
    
    def calculate_force(self, velocity_states):
        """Calculate aerodynamic forces in body frame."""
        if self.relative_location is None:
            raise ValueError("Relative location is not set.")
        
        V, alpha_i, beta_i = self.calculate_alpha_beta(velocity_states)
        
        # Reynolds number
        Re = self.calculate_reynolds_number(V)
        
        # Calculate aerodynamic coefficients
        Cl, Cd_induced, Cd_skin, AoA = self.calculate_cl_cd(alpha_i, beta_i, Re)
        
        # Forces in flow frame
        lift = 0.5 * rho * self.Area * Cl * V ** 2
        drag = 0.5 * rho * self.Area * (Cd_induced + Cd_skin) * V ** 2
        force_vector = np.array([-drag, 0, -lift])
        
        # Transform to body frame
        T = R_z(beta_i) @ R_y(alpha_i)
        self.body_forces = T @ force_vector
        
        # Store selected values
        self.tracked_data.update(log([Cl, Cd_induced, Cd_skin, V, lift, drag, alpha_i, beta_i, AoA, self.body_forces]))
        
        return self.body_forces
