"Class used to save and run simulation"
import numpy as np
from .Force import HullForce, ControlForce, TowingForce
from .RigidBody import RigidBody

class Simulation_Result():

    def init(self, time, N):
        # Initialize NumPy arrays with shape (N, 2) for 2D vectors and (N,) for 1D arrays.
        # Replace `N` with the desired number of rows, depending on your data needs.

        self.inertial_position = np.zeros((N, 2))    # 2D array for [x, z]
        self.pitch_angle = np.zeros(N)               # 1D array for pitch angle values
        self.inertial_velocity = np.zeros((N, 2))    # 2D array for [x_dot, z_dot]
        self.pitch_rate = np.zeros(N)                # 1D array for pitch rate values
        self.inertial_acceleration = np.zeros((N, 2))# 2D array for acceleration in [x_dot, z_dot]

        self.bf_position = np.zeros((N, 2))          # 2D array for body-frame position
        self.bf_velocity = np.zeros((N, 2))          # 2D array for body-frame velocity
        self.bf_acceleration = np.zeros((N, 2))      # 2D array for body-frame acceleration

        self.control_force_C_D = np.zeros(N)         # 1D array for drag force values
        self.control_force_C_L = np.zeros(N)         # 1D array for lift force values

        self.control_flow_velocity = np.zeros((N, 2))# 2D array for control flow velocity

        # Assuming each element of control_force_magnitude is a list of lists [ [drag, lift] ]
        self.control_force_magnitude = np.zeros((N, 2)) # 2D array for body-frame drag and lift
        self.control_force_alpha_i = np.zeros(N)         # 1D array for angle of attack
        self.control_force_moment = np.zeros(N)          # 1D array for moment

        self.hull_flow_velocity = np.zeros((N, 2))       # 2D array for hull flow velocity
        self.hull_force_magnitude = np.zeros((N, 2))     # 2D array for hull drag and lift
        self.hull_force_moment = np.zeros(N)             # 1D array for hull moment

        






class Simulation():
    def __init__(self, rigidbody : RigidBody, towingForce: TowingForce, hullForce: HullForce, controlForces : list[ControlForce], sim : Simulation_Result):
        self.rigidbody = rigidbody
        self.towingForce = towingForce
        self.hullForce = hullForce
        self.controlForces = controlForces

        self.initialize_rigid_body()
        
        self.sim = sim




    
    def initialize_rigid_body(self):
        self.rigidbody.add_tow_force(self.towingForce)
        self.rigidbody.add_hull_force(self.hullForce)
        for controlForce in self.controlForces:
            self.rigidbody.add_control_force(controlForce)

    # Figure out root solver
    def solve_equalibrium_state(self):
        self.towingForce.magnitude 
        self.towingForce.delta_t
        for controlForce in self.controlForces:    
            controlForce.delta_i
        self.sim.pitch_angle[0]

    
    def initialize_system(self, initial_state): 
        self.sim.inertial_position[0] = initial_state[:2]
        # self.sim.pitch_angle[0] = initial_state[2]

        self.sim.inertial_velocity[0] = initial_state[3:-1] # [[x_dot, z_dot]]
        self.sim.pitch_rate[0] = initial_state[-1]
        


    def solve_forces(self, i):
        ## Find control forces
        for force in self.control_forces:
            # Extract states
            u, w = self.sim.bf_velocity[i]
            q = self.sim.pitch_rate[i]
            # Calculate velocity at point
            velocity_states = [u, w, q]
            # Calculate control lift and drag force [drag, lift]
            Cl, Cd, V, lift, drag = force.calculate_force(velocity_states)
            

            self.sim.control_force_C_D.append(Cd)  #Dont forget to add a parse function
            self.sim.control_force_C_L.append(Cl)
            

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


        # T = self.rigidbody.transformation_matrix(pitch_angle)
        
        # self.velocity[0,:] = T.T @ initial_inertial_velocity[:-1]
        # self.pitch_rate[0] = initial_inertial_velocity[-1]