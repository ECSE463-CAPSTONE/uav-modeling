"Class used to save and run simulation"
import numpy as np
import scipy.optimize as opti
from .Force import HullForce, ControlForce, TowingForce
from .RigidBody import RigidBody


class Simulation_Result():

    def __init__(self, dt = 0.02, N = 10, nb_control_forces = 1):
        # Initialize NumPy arrays with shape (N, 2) for 2D vectors and (N,) for 1D arrays.
        # Replace `N` with the desired number of rows, depending on your data needs.

        self.inertial_position = np.zeros((N, 2))    # 2D array for [x, z]
        self.pitch_angle = np.zeros(N)               # 1D array for pitch angle values
        self.inertial_velocity = np.zeros((N, 2))    # 2D array for [x_dot, z_dot]
        self.pitch_rate = np.zeros(N)                # 1D array for pitch rate values
        self.inertial_acceleration = np.zeros((N, 2))# 2D array for acceleration in [x_dot, z_dot]
        self.angular_acceleration = np.zeros(N)      # 1D array for angular acceleration values

       
        self.bf_velocity = np.zeros((N, 2))          # 2D array for body-frame velocity
        self.bf_acceleration = np.zeros((N, 2))      # 2D array for body-frame acceleration

        self.control_force_C_D = np.zeros((N, nb_control_forces))         # 1D array for drag force values
        self.control_force_C_L = np.zeros((N, nb_control_forces))         # 1D array for lift force values
        self.control_flow_velocity = np.zeros((N, nb_control_forces, 2))# 2D array for control flow velocity

        # Assuming each element of control_force_magnitude is a list of lists [ [drag, lift] ]
        self.control_force_magnitude = np.zeros((N,nb_control_forces, 2)) # 3D array for body-frame drag and lift of different control forces
        self.control_force_alpha_i = np.zeros(N)         # 1D array for angle of attack
        self.control_force_moment = np.zeros(N)          # 1D array for moment

        self.hull_flow_velocity = np.zeros((N, 2))       # 2D array for hull flow velocity
        self.hull_force_magnitude = np.zeros((N, 2))     # 2D array for hull drag and lift
        self.hull_force_moment = np.zeros(N)             # 1D array for hull moment

        self.dt = dt
        self.N = N
        
        

class Simulation():
    def __init__(self, rigidbody : RigidBody, towingForce: TowingForce, hullForce: HullForce, controlForces : list[ControlForce]):
        self.rigidbody = rigidbody
        self.towingForce = towingForce
        self.hullForce = hullForce
        self.controlForces = controlForces

        self.initialize_rigid_body()
        self.sim = Simulation_Result()

    
    def initialize_rigid_body(self):
        self.rigidbody.add_tow_force(self.towingForce)
        self.rigidbody.add_hull_force(self.hullForce)
        for controlForce in self.controlForces:
            self.rigidbody.add_control_force(controlForce)


    def solve_equilibrium_state_fsolve(self, initial_velocity):
        # Initialize system
        initial_state = np.array([0, 0, 0, initial_velocity, 0, 0, 0, 0, 0])
        self.initialize_system(initial_state)

        # Set bounds (not needed for fsolve)
        lb_tow_force, ub_tow_force = np.array([0, 2000])
        lb_delta_t, ub_delta_t = np.rad2deg(np.array([0, 60]))
        lb_delta_i, ub_delta_i = np.rad2deg(np.array([-20, 10]))
        lb_pitch_angle, ub_pitch_angle = np.rad2deg(np.array([-5, 5]))

        # Define the system of equations
        def equations(args):
            pitch_angle, delta_t, towing_force = args
            
            # Override variables being optimized
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            
            # Solve forces
            self.solve_forces(0)

            # Calculate sum of forces/moments
            total_force_x, total_force_z = self.rigidbody.sum_forces(pitch_angle) 
            total_moment_y = self.rigidbody.sum_moments(pitch_angle)

            # Return the equations that should equal zero
            return [total_force_x, total_force_z, total_moment_y]

        # Set initial guess
        x0 = (-5, 40, 5)  # Initial guesses for [pitch_angle, delta_t, towing_force, delta_i]

        # Use fsolve to solve the equations
        results = opti.fsolve(equations, x0)

        # Check results and update the class variables
        if results is not None:
            pitch_angle, delta_t, towing_force = results
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            print("Equilibrium solved:", results)
        else:
            print("Failed to solve for equilibrium")

        return results

    def solve_equilibrium_state_sqrt(self, initial_velocity):
            # Initialize system
            initial_state = np.array([0, 0, 0, initial_velocity, 0, 0, 0, 0, 0])
            self.initialize_system(initial_state)

            # Define objective function for minimization
            def objective(args):
                pitch_angle, delta_t, towing_force, delta_i = args

                # Set the parameters for the system
                self.towingForce.delta_t = delta_t
                self.towingForce.magnitude = towing_force
                self.controlForces[0].delta_i = delta_i
                
                # Solve forces
                self.solve_forces(0)

                # Calculate sum of forces/moments
                total_force_x, total_force_z = self.rigidbody.sum_forces(pitch_angle) 
                total_moment_y = self.rigidbody.sum_moments(pitch_angle)

                # Return the sum of squares of equations to minimize
                return total_force_x**2 + total_force_z**2 + total_moment_y**2

            # Set bounds for the parameters
            bounds = [
                (np.deg2rad(-7), np.deg2rad(7)),     # pitch_angle
                (np.deg2rad(15), np.deg2rad(60)),      # delta_t
                (0, 100),                             # towing_force
                (np.deg2rad(-20), np.deg2rad(10))     # delta_i
            ]

            # Initial guess
            x0 = np.array([-5, 40, 5, -5])

            # Minimize the objective function
            result = opti.minimize(objective, x0, bounds=bounds)

            if result.success:
                print("Optimization Results:")
                print("----------------------")
                print(f"{'Parameter':<15} {'Value':<15} {'Units':<10}")
                print("----------------------")
                print(f"{'Pitch Angle':<15} {np.rad2deg(result.x[0]):<15.2f} {'degrees':<10}")
                print(f"{'Delta_t':<15} {np.rad2deg(result.x[1]):<15.2f} {'degrees':<10}")
                print(f"{'Towing Force':<15} {result.x[2]:<15.2f} {'N':<10}")
                print(f"{'Delta_i':<15} {np.rad2deg(result.x[3]):<15.2f} {'degrees':<10}")
                print("----------------------")
                print(f"Objective Function Value: {result.fun}")
                print("Optimization successful!")
            else:
                print("Optimization failed:", result.message)


            return result.x
    
    def solve_equilibrium_state(self, initial_velocity):
        #Solve for X-Dot = 0
        
        #Initialize system
        initial_state = np.array([0, 0, 0, initial_velocity, 0, 0, 0, 0, 0])
        self.initialize_system(initial_state)

        #Define objective function
        def objective_function_1(args):
            pitch_angle, delta_t, towing_force, delta_i = args
            #Override variables being optimized
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            self.controlForces[0].delta_i = delta_i

            #initialize system

            #Solve forces
            self.solve_forces(0)

            #Calculate sum of forces/moments
            total_force_x,total_force_z = self.rigidbody.sum_forces(pitch_angle) 
            total_moment_y = self.rigidbody.sum_moments(pitch_angle)

            #Set objective
            objective = np.sqrt(total_force_x**2 + total_force_z**2 + total_moment_y**2)
            return objective

        #Define objective function
        def objective_function_2(args):
            pitch_angle, delta_t, towing_force, delta_i = args
            return towing_force
        
        #Set bounds
        lb_tow_force, ub_tow_force = np.array([0, 2000])
        lb_delta_t, ub_delta_t = np.deg2rad(np.array([10, 60]))
        lb_delta_i, ub_delta_i = np.deg2rad(np.array([-20, 10]))
        lb_pitch_angle, ub_pitch_angle = np.deg2rad(np.array([-5, 5]))
        
        bnds = ((lb_pitch_angle, ub_pitch_angle), (lb_delta_t, ub_delta_t), (lb_tow_force, ub_tow_force),(lb_delta_i, ub_delta_i))

        # #Set constraints
        # def constraint_function(pitch_angle, delta_t, towing_force, delta_i):
        #     #Override variables being optimized
        #     self.towingForce.delta_t = delta_t
        #     self.towingForce.magnitude = towing_force
        #     self.controlForces[0].delta_i = delta_i

        #     #Solve forces
        #     self.solve_forces(0)

        #     #Calculate sum of forces/moments
        #     total_force_x,total_force_z = self.rigidbody.sum_forces(pitch_angle) 
        #     total_moment_y = self.rigidbody.sum_moments(pitch_angle)

        #     return total_force_x, total_force_z, total_moment_y


        # cons = ({'type': 'eq', 'fun': constraint_function[0]},
        #         {'type': 'eq', 'fun': constraint_function[1]},
        #         {'type': 'eq', 'fun': constraint_function[2]})

            # Set constraints
        def constraint_function_force_x(args):
            pitch_angle, delta_t, towing_force, delta_i = args
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            self.controlForces[0].delta_i = delta_i
            self.solve_forces(0)
            total_force_x, _ = self.rigidbody.sum_forces(pitch_angle)
            return total_force_x

        def constraint_function_force_z(args):
            pitch_angle, delta_t, towing_force, delta_i = args
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            self.controlForces[0].delta_i = delta_i
            self.solve_forces(0)
            _, total_force_z = self.rigidbody.sum_forces(pitch_angle)
            return total_force_z

        def constraint_function_moment_y(args):
            pitch_angle, delta_t, towing_force, delta_i = args
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            self.controlForces[0].delta_i = delta_i
            self.solve_forces(0)
            total_moment_y = self.rigidbody.sum_moments(pitch_angle)
            return total_moment_y

        cons = (
            {'type': 'eq', 'fun': constraint_function_force_x},
            {'type': 'eq', 'fun': constraint_function_force_z},
            {'type': 'eq', 'fun': constraint_function_moment_y}
        )

        #Set initial guess
        # x0 = (np.mean([lb_pitch_angle, ub_pitch_angle]),  
        #       np.mean([lb_delta_t, ub_delta_t]), 
        #       np.mean([lb_tow_force,ub_tow_force]), 
        #       np.mean([lb_delta_i, ub_delta_i]))

        x0 = (-5, 40, 5, -5)

        results = opti.minimize(objective_function_2, x0, bounds=bnds, constraints=cons, method='L-BFGS-B')

        if not results.success:
            print("failed to optimize for equilibrium")
            print(results.message)

        return results


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
    
    def initialize_system(self, initial_state): 
        self.sim.inertial_position[0] = initial_state[:2]
        self.sim.pitch_angle[0] = initial_state[2]

        self.sim.inertial_velocity[0] = initial_state[3:5] # [[x_dot, z_dot]]
        self.sim.pitch_rate[0] = initial_state[5]
        self.sim.inertial_acceleration[0] = initial_state[6:8]
        self.sim.pitch_rate[0] = initial_state[8]

        T = self.transformation_matrix(self.sim.pitch_angle[0])
        
        self.sim.bf_velocity[0] = T.T @ self.sim.inertial_velocity[0]
        self.sim.bf_acceleration[0,:] = T.T @ self.sim.inertial_acceleration[0]
        
    
    def solve_forces(self, i):
        ## Find control forces
        for f_i, force in enumerate(self.rigidbody.control_forces):
            # Extract states
            u, w = self.sim.bf_velocity[i]
            q = self.sim.pitch_rate[i]
            # Calculate velocity at point
            velocity_states = [u, w, q]
            # Calculate control lift and drag force [drag, lift]
            Cl, Cd, V, lift, drag = force.calculate_force(velocity_states)
        
            # Store values
            self.sim.control_force_C_D[i, f_i] = Cd   
            self.sim.control_force_C_L[i, f_i] = Cl
            self.sim.control_flow_velocity[i, f_i] = V
            self.sim.control_force_magnitude[i, f_i] = np.array([drag, lift])  

            

    def simulate_forward_euler(self, initial_state, dt, N):
        #  Initialize the simulation
        dt = self.sim.dt
        self.sim = Simulation_Result(dt, N, len(self.control_forces))
        self.initialize_system(initial_state)
        
        for i in range(1, self.N):
            theta = self.sim.pitch_angle[ i - 1]
            # 1. Calculate forces and moments in the body frame
            self.solve_forces(i - 1, theta)
            total_force = self.rigidbody.sum_forces(i - 1, theta)  # F_body = [Fx, Fz] in body frame
            total_moment = self.rigidbody.sum_moments(i - 1, theta)  # M_body = My

            # 2. Calculate body frame accelerations (q_dot_dot)
            ax_body = total_force[0] / self.rigidbody.mass  
            az_body = total_force[1] / self.rigidbody.mass  
            self.sim.bf_acceleration[i] = np.array([ax_body, az_body])

            alpha_body = total_moment / self.rigidbody.Iyy  # pitch angular acceleration
            self.sim.angular_acceleration[i] = alpha_body 
            

            # 3. Update body frame velocities (q_dot) using Euler integration
            self.sim.bf_velocity[i, 0] = self.sim.bf_velocity[i-1, 0] + ax_body * dt  # u
            self.sim.bf_velocity[i, 1] = self.sim.bf_velocity[i-1, 1] + az_body * dt  # w
            self.sim.pitch_rate[i] = self.sim.pitch_rate[i-1] + alpha_body * dt  # pitch rate (q)

            # 4. Transform q_dot (body velocities) to x_dot (inertial velocities)
            T = self.transformation_matrix(self.sim.pitch_angle[i-1])  # Get the transformation matrix based on pitch angle
            x_dot_inertial = T @ self.bf_velocity[i]  # Transform to [x_dot, z_dot] in inertial frame
            self.sim.inertial_velocity[i, :] = x_dot_inertial
            
            # 5. Update inertial frame position using Euler integration
            self.sim.inertial_position[i, :] = self.sim.inertial_position[i-1, :] + x_dot_inertial * dt

            # 6. Update pitch angle (integrate angular velocity)
            self.sim.pitch_angle[i] = self.sim.pitch_angle[i-1] + self.sim.pitch_rate[i] * dt

        return self.sim


