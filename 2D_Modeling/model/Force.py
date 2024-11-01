import numpy as np

rho = 1000
g = 9.81

class ControlForce():
    def __init__(self, location, delta_i, AR, area, C_L_alpha, C_L_alpha_offset ,C_D0, e = 0.85):
        self.AR = AR # aspect ratio
        self.Area = area # surface area
        self.C_L_alpha = C_L_alpha #slope of 2D lift curve
        self.C_L_alpha_offset = C_L_alpha_offset #CL Alpha offset
        self.C_D0 = C_D0 #parasite drag coefficient
        self.e = e # Oswald efficiency factor
        self.location = location # [r_x, r_z] location relative to COM
        self.delta_i = delta_i #Fixed angle of attack of the control force
        self.magnitude = np.zeros(2) # Array to save temporary values [drag, lift]
        self.alpha_i = [] #temporary value storing
    
    def calculate_alpha_i(self, velocity_states):
        u, w, q = velocity_states
        r_x, r_z = self.location 
        alpha_i = np.arctan((w - q * r_x) / (u + q * r_z + 1e-8))

        if np.isnan(alpha_i):
            alpha_i = 0

        #alpha i is before adding in control surface angle of attack
        return alpha_i

    
    def calculate_cl_cd(self, alpha_i):
        """Calculate the lift and drag coefficients based on the angle of attack"""
        Cl = -self.C_L_alpha_offset + self.C_L_alpha * self.AR / ( 2 * (self.AR + 4) / (self.AR + 2))  * (alpha_i + self.delta_i)  # Lift coefficient, eq. 19
        Cd = self.C_D0 + Cl**2 / (np.pi * self.AR * self.e)  # Drag coefficient, eq. 20
        return Cl, Cd

   
    # CALCULATES WITH RESPECT TO V NOT IN BODY FRAME
    def calculate_force(self, velocity_states):
        """Calculate the actual force vector from lift and drag"""
        r_x, r_z = self.location 
        u, w, q = velocity_states

        alpha_i = self.calculate_alpha_i(velocity_states)
        if alpha_i >= np.deg2rad(15):
            alpha_i = np.deg2rad(15)

        elif alpha_i <= np.deg2rad(-15):
            alpha_i = np.deg2rad(-15)

        Cl, Cd = self.calculate_cl_cd(alpha_i)
        
        V = np.sqrt((u + q * r_z)**2 + (w - q * r_x)**2)

        lift = 0.5 * rho * Cl * self.Area * V ** 2
        drag = 0.5 * rho * Cd * self.Area * V ** 2

        self.magnitude = np.array([drag, lift])  # Return force vector in body frame (x, z)
        self.alpha_i = alpha_i

        return Cl, Cd, V, lift, drag, alpha_i


class HullForce():
    def __init__(self, area, location, Cd = 1.2, correction = 0.75):
        self.Cd = Cd * correction #Cylinder Cd Approximation
        self.area = area
        self.magnitude = np.zeros(2) # Array to save temporary values [drag, lift]
        self.location = location

    def calculate_force(self, velocity_states):
        """Calculate the actual force vector from lift and drag"""
        r_x, r_z = self.location 
        u, w, q = velocity_states
        
        V = np.sqrt((u + q * r_z)**2 + (w - q * r_x)**2)
        lift = 0
        drag = 0.5 * rho * self.Cd * self.area * V ** 2
        self.magnitude = np.array([drag, lift])

        return V, lift, drag # Return force vector in body frame (x, z)


class TowingForce():
    def __init__(self, location, magnitude, delta_t):
        self.magnitude = magnitude
        self.delta_t = delta_t
        self.location = location