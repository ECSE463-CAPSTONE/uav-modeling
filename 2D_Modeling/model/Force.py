import numpy as np

rho = 1000
g = 9.81

class ControlForce():
    def __init__(self, location, delta_i, A, C_L_alpha, C_D0, e = 0.85):
        self.A = A # aspect ratio
        self.C_L_alpha = C_L_alpha #slope of 2D lift curve
        self.C_D0 = C_D0 #parasite drag coefficient
        self.e = e # Oswald efficiency factor
        self.location = location # [r_x, r_z] location relative to COM
        self.delta_i = delta_i #Fixed angle of attack of the control force
        self.magnitude = np.zeros(1,2) # Array to save temporary values [drag, lift]

    
    def calculate_alpha_i(self, velocity_states):
        u, w, q = velocity_states
        r_x, r_z = self.location 
        alpha_i = np.arctan((w - q * r_x) / (u + q * r_z))

        if np.isnan(alpha_i):
            alpha_i = 0

        #alpha i is before adding in control surface angle of attack
        self.alpha_i = alpha_i

    
    def calculate_cl_cd(self):
        """Calculate the lift and drag coefficients based on the angle of attack"""
        Cl = self.C_L_alpha * self.A / ( 2 * (self.A + 4) / (self.A + 2))  * (self.alpha_i + self.delta_i)  # Lift coefficient, eq. 19
        Cd = self.C_D0 + Cl**2 / (np.pi * self.A * self.e)  # Drag coefficient, eq. 20
        return Cl, Cd

   
    # CALCULATES WITH RESPECT TO V NOT IN BODY FRAME
    def calculate_force(self, velocity_states, rel_position):
        """Calculate the actual force vector from lift and drag"""
        r_x, r_z = rel_position 
        u, w, q = velocity_states

        self.calculate_alpha_i(velocity_states, rel_position)
        Cl, Cd = self.calculate_cl_cd(velocity_states)
        
        V = np.sqrt((u + q * r_z)**2 + (w - q * r_x)**2)
        lift = 0.5 * rho * Cl * self.A * V ** 2
        drag = 0.5 * rho * Cd * self.A * V ** 2

        self.magnitude = np.array([drag, lift])  # Return force vector in body frame (x, z)

class HullForce():
    def __init__(self, area, location, Cd = 1.2, correction = 0.75):
        self.Cd = Cd * correction #Cylinder Cd Approximation
        self.area = area
        self.magnitude = np.zeros(1,2) # Array to save temporary values [drag, lift]
        self.location = location

    def calculate_force(self, velocity_states, rel_position):
        """Calculate the actual force vector from lift and drag"""
        r_x, r_z = rel_position 
        u, w, q = velocity_states

        self.calculate_alpha_i(velocity_states, rel_position)
        
        V = np.sqrt((u + q * r_z)**2 + (w - q * r_x)**2)
        lift = 0
        drag = 0.5 * rho * self.Cd * self.area * V ** 2

        return np.array([drag, lift])  # Return force vector in body frame (x, z)


class TowingForce():
    def __init__(self, location, magnitude, delta_t, N):
        self.magnitude = magnitude
        self.delta_t = delta_t
        self.location = location