import numpy as np
from scipy.integrate import solve_ivp

from .Force import Force, ControlForce, TowingForce

#Global variables
g = 9.81
rho = 1000

class RigidBody:
    def __init__(self, mass: float, volume: float, inertia: float, COM: np.array, center_of_buoyancy: np.array, N: int):
        self.mass = mass
        self.volume = volume
        self.Iyy = inertia
        self.COM = COM  # Center of mass [x, z]
        self.center_of_buoyancy = center_of_buoyancy  # Center of buoyancy [x, z]
        self.buoyancy  = rho * g * self.volume


        # Store forces and moments
        self.control_forces = []  # List of forces acting on the body
        self.tow_force : Force
        self.moments = []  # List of moments about the y-axis (pitch)

        # Initial conditions for position, velocity, and pitch
        self.position = np.zeros((N, 2))  # 2D position [x, z]
        self.velocity = np.zeros((N, 2))  # 2D velocity [u, w]
        self.pitch_angle = np.zeros(N)  # Pitch angle (about y-axis)
        self.pitch_rate = np.zeros(N)  # Angular velocity (pitch rate)

        self.N = N
    
    def add_tow_force(self, force):
        self.tow_force = force

    def add_control_force(self, force):
        """Adds a force instance to the force list"""
        self.control_forces.append(force)
    
    def add_moment(self, moment):
        """Adds a moment about the y-axis"""
        self.moments.append(moment)
    
    def sum_forces(self, i):
        """Sums up all forces acting on the body in x and z directions"""
        # print('Summing forces', self.tow_force.magnitude[i, 0] )
        total_force_x = (self.mass * 9.81 - self.buoyancy) * np.sin(self.pitch_angle[i]) + np.sum([f.magnitude[0] for f in self.control_forces]) + self.tow_force.magnitude[i, 0] 
        total_force_z = -(self.mass * 9.81 - self.buoyancy) * np.cos(self.pitch_angle[i]) + np.sum([f.magnitude[1] for f in self.control_forces]) + self.tow_force.magnitude[i, 1] 
        # print('Total force', total_force_x, total_force_z)
        return np.array([total_force_x, total_force_z])
    
    def sum_moments(self, i):
        """Sums up all moments acting on the body about the y-axis (pitch)"""
        x_b = self.center_of_buoyancy[0] - self.COM[0]  #distance of COB from COM
        #print(f'x_b = {self.center_of_buoyancy[0]}, {self.COM[0]}')
        buoyancy_contribution = self.buoyancy * x_b * np.cos(self.pitch_angle[i])
        #print(f'buoyancy_contribution = {buoyancy_contribution}')
        moment_contributions = np.sum(self.moments)
        control_force_contribution = np.sum([np.dot(f.magnitude[i], f.location - self.COM) for f in self.control_forces])
        #print(f'control_force_contribution = {control_force_contribution}')
        tow_force_contribution = np.dot(self.tow_force.magnitude[i], self.tow_force.location - self.COM) 
        #print(f'tow_force_contribution = {tow_force_contribution}')
        total_moment_y = buoyancy_contribution + 0 * moment_contributions + control_force_contribution  + tow_force_contribution
        # print(f'Total moment = {total_moment_y}')
        return total_moment_y
    
    def transformation_matrix(self, pitch_angle):
        """Returns the transformation matrix T to convert body frame to inertial frame."""
        cos_theta = np.cos(pitch_angle)
        sin_theta = np.sin(pitch_angle)

        # Transformation matrix T
        T = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])
        return T
    
    def initialize_system(self, initial_inertial_position, initial_inertial_velocity): 
        self.position[0,:] = initial_inertial_position[:-1]
        self.pitch_angle[0] = initial_inertial_position[-1]
        pitch_angle = self.pitch_angle[0]

        T = self.transformation_matrix(pitch_angle)
        
        self.velocity[0,:] = T.T @ initial_inertial_velocity[:-1]
        self.pitch_rate[0] = initial_inertial_velocity[-1]

    def solve_forces(self, i):
         ## Find towing force
        tow_force = self.tow_force
        r_f_o = tow_force.location - self.COM # Relative Position of towing force relative to COM
        tow_force_inertial_position = self.position[i, :] + r_f_o
        tow_force.calculate_force(i, tow_force.D,tow_force_inertial_position[1], tow_force.scalar_magnitude, self.pitch_angle[i])

        ## Find control forces
        for force in self.control_forces:
            # Extract states
            rel_position = force.location - self.COM
            u, w = self.velocity[i]
            q = self.pitch_rate[i]

            # Calculate velocity at point
            velocity_states = [u, w, q]
           

            # Calculate force
            force.magnitude[i] = force.calculate_force(velocity_states, rel_position)

    def simulate_forward_euler(self, initial_inertial_position, initial_inertial_velocity, dt):
        self.initialize_system(initial_inertial_position, initial_inertial_velocity)
        
        for i in range(1, self.N):
            
            # 1. Calculate forces and moments in the body frame
            self.solve_forces(i - 1)
            total_force = self.sum_forces(i - 1)  # F_body = [Fx, Fz] in body frame
            total_moment = self.sum_moments(i - 1)  # M_body = My
            # print('Total force = ', total_force)
            # 2. Calculate body frame accelerations (q_dot_dot)
            ax_body = total_force[0] / self.mass  
            az_body = total_force[1] / self.mass  
            #print('az', az_body)
            alpha_body = total_moment / self.Iyy  # pitch angular acceleration
            #print('alpha_body = ', alpha_body)

            # 3. Update body frame velocities (q_dot) using Euler integration
            self.velocity[i, 0] = self.velocity[i-1, 0] + ax_body * dt  # u
            self.velocity[i, 1] = self.velocity[i-1, 1] + az_body * dt  # w
            self.pitch_rate[i] = self.pitch_rate[i-1] + alpha_body * dt  # pitch rate (q)

            # 4. Transform q_dot (body velocities) to x_dot (inertial velocities)
            T = self.transformation_matrix(self.pitch_angle[i-1])  # Get the transformation matrix based on pitch angle
            x_dot_inertial = T @ self.velocity[i]  # Transform to [x_dot, z_dot] in inertial frame

            # 5. Update inertial frame position using Euler integration
            self.position[i, :] = self.position[i-1, :] + x_dot_inertial * dt

            # 6. Update pitch angle (integrate angular velocity)
            self.pitch_angle[i] = self.pitch_angle[i-1] + self.pitch_rate[i] * dt

        return

