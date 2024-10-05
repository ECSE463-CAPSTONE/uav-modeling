import numpy as np

rho = 1000
g = 9.81

class Force:
    def __init__(self, location, N):
        self.location = location  # Location wrt body frame [x, z]
        self.magnitude = np.zeros((N,2))  # Force magnitude as a vector
        self.current_magnitude = 0
        self.angle_of_attacks = []
class ControlForce(Force):
    def __init__(self, location, N, A, C_L_alpha, C_D0, e = 0.85):
        super().__init__(location, N)
        self.A = A # aspect ratio
        self.C_L_alpha = C_L_alpha #slope of 2D lift curve
        self.C_D0 = C_D0 #parasite drag coefficient
        self.e = e # Oswald efficiency factor
    
    def calculate_alpha(self, velocity_states, relative_position):
        u, w, q = velocity_states
        r_x, r_z = self.location 
        # print(f'u,v,w : {u}, {w}, {q}')
        # print(f'r_x, r_z: {r_x}  z, {r_z}')
        alpha = np.arctan((w - q * r_x) / (u + q * r_z))

        if np.isnan(alpha):
            alpha = 0

        # print(f'alpha {(alpha)}')
        self.angle_of_attacks.append(alpha)
        self.angle_of_attack = alpha

    
    def calculate_cl_cd(self, velocity_states):
        """Calculate the lift and drag coefficients based on the angle of attack"""
        A = self.A
        Cl = self.C_L_alpha * A / ( 2 * (A + 4) / (A + 2))  * self.angle_of_attack  # Lift coefficient, eq. 19
        Cd = self.C_D0 + Cl**2 / (np.pi * A * self.e)  # Drag coefficient, eq. 20
        return Cl, Cd

    def calculate_force(self, velocity_states, rel_position):
        """Calculate the actual force vector from lift and drag"""
        r_x, r_z = rel_position 
        u, w, q = velocity_states

        self.calculate_alpha(velocity_states, rel_position)
        Cl, Cd = self.calculate_cl_cd(velocity_states)
        
        V = np.sqrt((u + q * r_z)**2 + (w - q * r_x)**2)
        lift = 0.5 * rho * Cl * self.A * V ** 2
        drag = 0.5 * rho * Cd * self.A * V ** 2

        return np.array([-drag, lift])  # Return force vector in body frame (x, z)

class TowingForce(Force):
    def __init__(self, location,  N, tow_length, initial_magntidude, D):
        super().__init__(location, N)
        self.tow_length = tow_length
        self.scalar_magnitude = initial_magntidude
        self.D = D

    def calculate_direction(self, D, tow_depth, pitch_angle):
        # D is distance of drone form see, z is inertia position of uav
        gamma = np.arcsin( (D  - tow_depth) / self.tow_length) - pitch_angle
        if np.isnan(gamma):
            raise ValueError("gamma is NaN, tow-length is too short")
        return gamma
        

    def calculate_force(self, i, D, tow_position, scalar_magnitude, pitch_angle):
        # To implement
        self.scalar_magnitude = scalar_magnitude
        gamma = self.calculate_direction(D, tow_position, pitch_angle)
        self.magnitude[i] = np.array([np.cos(gamma), np.sin(gamma)]) * self.scalar_magnitude

    

        