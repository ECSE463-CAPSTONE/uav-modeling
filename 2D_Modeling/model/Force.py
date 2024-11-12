import numpy as np

rho = 999.7
g = 9.81
mu = 0.001308

class ControlForce():
    def __init__(self, location, delta_i, AR, area, chord, stall_threshold, C_L_alpha, C_L_alpha_offset , e = 0.85):
        self.AR = AR # aspect ratio
        self.Area = area # surface area
        self.chord = chord
        self.stall_threshold = stall_threshold
        self.C_L_alpha = C_L_alpha #slope of 2D lift curve
        self.C_L_alpha_offset = C_L_alpha_offset #CL Alpha offset
        self.e = e # Oswald efficiency factor
        self.location = location # [r_x, r_z] location relative to COM
        self.delta_i = delta_i #Fixed angle of attack of the control force
        self.magnitude = np.zeros(2) # Array to save temporary values [drag, lift]
        self.alpha_i = [] #temporary value storing
    
    def calculate_alpha_i(self, velocity_states):
        u, w, q = velocity_states
        r_x, r_z = self.location 
        alpha_i = np.arctan((w - q * r_x) / (u + q * r_z))

        if np.isnan(alpha_i):
            alpha_i = 0

        #alpha i is before adding in control surface angle of attack
        return alpha_i

    
    def calculate_cl_cd(self, alpha_i, Re):
        """Calculate the lift and drag coefficients based on the angle of attack"""
        AoA = min(np.deg2rad(self.stall_threshold),abs(alpha_i + self.delta_i)) * np.sign(alpha_i + self.delta_i)

        Cl = -self.C_L_alpha_offset + self.C_L_alpha * self.AR / ( 2 * (self.AR + 4) / (self.AR + 2))  *  AoA # Lift coefficient, eq. 19
        Cd_form = Cl**2 / (np.pi * self.AR * self.e)  # Drag coefficient, eq. 20
        Cd_skin = 0.0576/(Re**(1/5))
        return Cl, Cd_form, Cd_skin

   
    # CALCULATES WITH RESPECT TO V NOT IN BODY FRAME
    def calculate_force(self, velocity_states):
        """Calculate the actual force vector from lift and drag"""
        r_x, r_z = self.location 
        u, w, q = velocity_states

        alpha_i = self.calculate_alpha_i(velocity_states)

        V = np.sqrt((u + q * r_z)**2 + (w - q * r_x)**2)

        Re = rho * V * self.chord / mu
        Cl, Cd_form, Cd_skin = self.calculate_cl_cd(alpha_i, Re)

        lift = 0.5 * rho * Cl * self.Area * V ** 2
        drag_form = 0.5 * rho * Cd_form * self.Area * V ** 2
        drag_skin = 0.5 * rho * Cd_skin * 2*self.Area * V ** 2
        drag = drag_form + drag_skin

        self.magnitude = np.array([drag, lift])  # Return force vector in body frame (x, z)
        self.alpha_i = alpha_i

        return Cl, Cd_form, V, lift, drag, alpha_i #Could add Cd_skin


class HullForce():
    def __init__(self,location, surface_area, frontal_area, chord , Cd, correction):

        self.Cd = Cd * correction #Cylinder Cd Approximation
        self.frontal_area = frontal_area
        self.chord = chord
        self.surface_area = surface_area
        self.magnitude = np.zeros(2) # Array to save temporary values [drag, lift]
        self.location = location
        self.alpha_h = [] #temporary value storing

    def calculate_alpha_h(self, velocity_states):
        u, w, q = velocity_states
        r_x, r_z = self.location 
        alpha_h = np.arctan((w - q * r_x) / (u + q * r_z))

        if np.isnan(alpha_h):
            alpha_h = 0

        return alpha_h

    def calculate_force(self, velocity_states):
        """Calculate the actual force vector from lift and drag"""
        r_x, r_z = self.location 
        u, w, q = velocity_states
        
        self.alpha_h = self.calculate_alpha_h(velocity_states)
        V = np.sqrt((u + q * r_z)**2 + (w - q * r_x)**2)

        lift = 0
        
        #Calcualte drag force
        Re = rho * V * self.chord / mu
        Cd_0 = 0.0576/(Re**(1/5))

        drag_form = 0.5 * rho * self.Cd * self.frontal_area * V ** 2
        drag_skin = 0.5 * rho * Cd_0 * self.surface_area * V ** 2
        drag = drag_form + drag_skin
        self.magnitude = np.array([drag, lift])

        return V, lift, drag # Return force vector in body frame (x, z)


class TowingForce():
    def __init__(self, location, magnitude, drone_height, drone_tow_length, probe_depth):
        self.magnitude = magnitude
        self.drone_height = drone_height
        self.drone_tow_length = drone_tow_length
        self.probe_depth = probe_depth
        self.location = location
        self.delta_t = 0
    
    def calculate_force(self, x, z):
        delta_t = (self.drone_height + self.probe_depth + z) / self.drone_tow_length
        self.delta_t = delta_t

        return delta_t