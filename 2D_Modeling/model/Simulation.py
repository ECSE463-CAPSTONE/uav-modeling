"Class used to save and run simulation"
import numpy as np
import scipy.optimize as opti
from scipy.optimize import least_squares   
import matplotlib.pyplot as plt
# import plotly.graph.objects as go



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
    
    def plot_simulation_results(self):
        """Plots inertial and pitch states, as well as force magnitudes and moments."""
        time = np.linspace(0, self.N * self.dt, self.N)  # Create time vector
        
        # Plot 1: Inertial position, velocity, and acceleration
        fig, axs = plt.subplots(3, 1, figsize=(10, 10))
        axs[0].plot(time, self.inertial_position[:, 0], label="x-position (m)")
        axs[0].plot(time, self.inertial_position[: , 1], label="z-position (m)")
        axs[0].set_title("Inertial Position")
        axs[0].legend(loc="best")

        axs[1].plot(time, self.inertial_velocity[:  , 0], label="x-velocity (m/s)")
        axs[1].plot(time, self.inertial_velocity[:  , 1], label="z-velocity (m/s)")
        axs[1].set_title("Inertial Velocity")
        axs[1].legend(loc="best")

        axs[2].plot(time, self.inertial_acceleration[: , 0], label="x-acceleration (m/s²)")
        axs[2].plot(time, self.inertial_acceleration[: , 1], label="z-acceleration (m/s²)")
        axs[2].set_title("Inertial Acceleration")
        axs[2].legend(loc="best")

        fig.tight_layout()
        plt.show()

        # Plot 2: Pitch angle, pitch rate, and angular acceleration
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        axs[0].plot(time, np.rad2deg(self.pitch_angle), label="Pitch Angle (deg)")
        axs[0].set_title("Pitch Angle")
        axs[0].legend(loc="best")

        axs[1].plot(time, np.rad2deg(self.pitch_rate), label="Pitch Rate (deg/s)")
        axs[1].set_title("Pitch Rate")
        axs[1].legend(loc="best")

        axs[2].plot(time, np.rad2deg(self.angular_acceleration), label="Angular Acceleration (deg/s²)")
        axs[2].set_title("Angular Acceleration")
        axs[2].legend(loc="best")

        fig.tight_layout()
        plt.show()

        # Plot 3: Control and hull forces (separate lift and drag) and hull moment
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))

        # Control forces: Lift and Drag (aggregated for multiple control forces)
        control_drag = np.sum(self.control_force_magnitude[:, :, 0], axis=1)
        control_lift = np.sum(self.control_force_magnitude[:, :, 1], axis=1)
        axs[0].plot(time, control_drag, label="Control Drag Force (N)")
        axs[0].plot(time, control_lift, label="Control Lift Force (N)")
        axs[0].set_title("Control Forces")
        axs[0].legend(loc="best")

        # Hull forces: Lift and Drag
        axs[1].plot(time, self.hull_force_magnitude[:, 0], label="Hull Drag Force (N)")
        axs[1].plot(time, self.hull_force_magnitude[:, 1], label="Hull Lift Force (N)")
        axs[1].set_title("Hull Forces")
        axs[1].legend(loc="best")

        # Hull moment
        axs[2].plot(time, self.hull_force_moment, label="Hull Force Moment (Nm)")
        axs[2].set_title("Hull Force Moment")
        axs[2].legend(loc="best")

        fig.tight_layout()
        plt.show()
    
        

class Simulation():
    def __init__(self, rigidbody : RigidBody, towingForce: TowingForce, hullForce: HullForce, controlForces : list[ControlForce]):
        self.rigidbody = rigidbody
        self.towingForce = towingForce
        self.hullForce = hullForce
        self.controlForces = controlForces

        self.initialize_rigid_body()
        self.sim = Simulation_Result()

        #Set optimization bounds
        self.lb_tow_force, self.ub_tow_force = np.array([0, 2000])
        self.lb_delta_t, self.ub_delta_t = np.deg2rad(np.array([0, 70]))
        self.lb_delta_i, self.ub_delta_i = np.deg2rad(np.array([-20, 10]))
        self.lb_pitch_angle, self.ub_pitch_angle = np.deg2rad(np.array([-8, -4]))

        self.bounds = [
            (self.lb_pitch_angle, self.ub_pitch_angle),
            (self.lb_delta_t, self.ub_delta_t),
            (self.lb_tow_force, self.ub_tow_force),
            (self.lb_delta_i, self.ub_delta_i)
        ]



    
    def initialize_rigid_body(self):
        self.rigidbody.add_tow_force(self.towingForce)
        self.rigidbody.add_hull_force(self.hullForce)
        for controlForce in self.controlForces:
            self.rigidbody.add_control_force(controlForce)



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
        self.sim.bf_acceleration[0] = T.T @ self.sim.inertial_acceleration[0]
        
    
    def solve_forces(self, i):
        # Extract velocity states
        u, w = self.sim.bf_velocity[i]
        q = self.sim.pitch_rate[i]
        velocity_states = [u, w, q]
        # Solve hull force
        V, lift, drag = self.hullForce.calculate_force(velocity_states)
        self.sim.hull_force_magnitude[i] = np.array([drag, lift])
        self.sim.hull_flow_velocity[i] = V

        ## Find control forces
        for f_i, force in enumerate(self.rigidbody.control_forces):
            # Calculate control lift and drag force [drag, lift]
            Cl, Cd, V, lift, drag = force.calculate_force(velocity_states)
        
            # Store values
            self.sim.control_force_C_D[i, f_i] = Cd   
            self.sim.control_force_C_L[i, f_i] = Cl
            self.sim.control_flow_velocity[i, f_i] = V
            self.sim.control_force_magnitude[i, f_i] = np.array([drag, lift])  

            

    def simulate_forward_euler(self, N, dt, initial_state):
        #  Initialize the simulation
        dt = self.sim.dt
        self.sim = Simulation_Result(dt, N, len(self.controlForces))
       
        self.initialize_system(initial_state)
        
        for i in range(1, N):
            print(i)
            theta = self.sim.pitch_angle[i - 1]
            # 1. Calculate forces and moments in the body frame
            self.solve_forces( i - 1)
            total_force = self.rigidbody.sum_forces(theta)  # F_body = [Fx, Fz] in body frame
            total_moment = self.rigidbody.sum_moments(theta)  # M_body = My

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
            inertial_velocity = T @ self.sim.bf_velocity[i]  # Transform to [x_dot, z_dot] in inertial frame
            self.sim.inertial_velocity[i, :] = inertial_velocity
            
            # 5. Update inertial frame position using Euler integration
            self.sim.inertial_position[i, :] = self.sim.inertial_position[i-1, :] + inertial_velocity * dt

            # 6. Update pitch angle (integrate angular velocity)
            self.sim.pitch_angle[i] = self.sim.pitch_angle[i-1] + self.sim.pitch_rate[i] * dt

        return self.sim

    
    ##################################################################################################
    ######################################## EQUILIBRIUM STATE ########################################
    #################################################################################################

    def solve_equilibrium_state_LS(self, initial_velocity):
        # Initialize system
        initial_state = np.array([0, 0, 0, initial_velocity, 0, 0, 0, 0, 0])
        self.initialize_system(initial_state)

        # Define residual function for least squares optimization
        def residuals(args):
            pitch_angle, delta_t, towing_force, delta_i = args

            # Set parameters for the system
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            self.controlForces[0].delta_i = delta_i

            # Solve forces
            self.solve_forces(0)

            # Calculate sum of forces/moments
            total_force_x, total_force_z = self.rigidbody.sum_forces(pitch_angle)
            total_moment_y = self.rigidbody.sum_moments(pitch_angle)

            # Return the individual residuals to be minimized
            return np.array([total_force_x, total_force_z, total_moment_y])

        # # Set bounds for parameters
        bounds = (
            # pitch angle, delta_t, tow force, delta_i
            [self.lb_pitch_angle, self.lb_delta_t, self.lb_tow_force, self.lb_delta_i],  # Lower bounds ()
            [self.ub_pitch_angle, self.ub_delta_t, self.ub_tow_force, self.ub_delta_i]   # Upper bounds
        )

        # Initial guess
        x0 = np.array([self.lb_pitch_angle, np.deg2rad(40), 5, np.deg2rad(-5)])

        # Perform least squares optimization
        result = least_squares(residuals, x0, bounds = bounds, max_nfev=10000)
        residuals = residuals(result.x)

        print("Optimization Results:")
        print("----------------------")
        print(f"{'Parameter':<15} {'Value':<15} {'Units':<10}")
        print("----------------------")
        print(f"{'Pitch Angle':<15} {np.rad2deg(result.x[0]):<15.2f} {'degrees':<10}")
        print(f"{'Delta_t':<15} {np.rad2deg(result.x[1]):<15.2f} {'degrees':<10}")
        print(f"{'Towing Force':<15} {result.x[2]:<15.2f} {'N':<10}")
        print(f"{'Delta_i':<15} {np.rad2deg(result.x[3]):<15.2f} {'degrees':<10}")
        print("----------------------")
        print(f"{'Fx:':<15} {residuals[0]:<15.2f}{'N':<10}")
        print(f"{'Fz:':<15} {residuals[1]:<15.2f}{'N':<10}")
        print(f"{'My:':<15} {residuals[2]:<15.2f}{'Nm':<10}")
        print(f"Residual Norm: {result.cost:.6f}")

        if result.success:
            print("Optimization successful!")
        else:
            print("Optimization failed:", result.message)

        return result.x
    
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


            # Initial guess
            x0 = np.array([np.deg2rad(-5), np.deg2rad(40), 80, np.deg2rad(-5)])

            # Minimize the objective function
            result = opti.minimize(objective, x0, bounds= self.bounds)
            total_force_x, total_force_z = self.rigidbody.sum_forces(result.x[0]) 
            total_moment_y = self.rigidbody.sum_moments(result.x[0])

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
                print(f"{'Fx constraint':<15} {total_force_x:<15.2f} {'N':<10}")
                print(f"{'Fz constraint':<15} {total_force_z:<15.2f} {'N':<10}")
                print(f"{'My constraint':<15} {total_moment_y:<15.2f} {'Nm':<10}")   

                print(f"Objective Function Value: {result.fun}")
                print("Optimization successful!")
            else:
                print("Optimization failed:", result.message)


            return result
    
    def solve_equilibrium_state_min_FT(self, initial_velocity):
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

        x0 = (np.deg2rad(-5), np.deg2rad(40), 5, np.deg2rad(-5))

        result = opti.minimize(objective_function_2, x0, bounds=self.bounds, constraints=cons, method='SLSQP', options={'ftol': 1e-10, 'disp': True, 'maxiter':10000})

       
        print("Optimization Results:")
        print("----------------------")
        print(f"{'Parameter':<15} {'Value':<15} {'Units':<10}")
        print("----------------------")
        print(f"{'Pitch Angle':<15} {np.rad2deg(result.x[0]):<15.2f} {'degrees':<10}")
        print(f"{'Delta_t':<15} {np.rad2deg(result.x[1]):<15.2f} {'degrees':<10}")
        print(f"{'Towing Force':<15} {result.x[2]:<15.2f} {'N':<10}")
        print(f"{'Delta_i':<15} {np.rad2deg(result.x[3]):<15.2f} {'degrees':<10}")
        print("----------------------")
        print(f"{'Fx constraint':<15} {constraint_function_force_x(result.x):<15.2f} {'N':<10}")
        print(f"{'Fz constraint':<15} {constraint_function_force_z(result.x):<15.2f} {'N':<10}")
        print(f"{'My constraint':<15} {constraint_function_moment_y(result.x):<15.2f} {'Nm':<10}")    

        if result.success:
            print(f"Objective Function Value: {result.fun}")
            print("Optimization successful!")
        else:
            print("Optimization failed:", result.message)

        return result

    # Solve for equilibrium state given a fixed delta_i
    def solve_equilibrium_state_fsolve_fixed_delta_i(self, initial_velocity, delta_i):
        # Initialize system

        initial_state = np.array([0, 0, 0, initial_velocity, 0, 0, 0, 0, 0])
        self.initialize_system(initial_state)
        self.controlForces[0].delta_i = np.deg2rad(delta_i)


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
        x0 = (np.deg2rad(-5), np.deg2rad(40), 5)  # Initial guesses for [pitch_angle, delta_t, towing_force]

        # Use fsolve to solve the equations
        result = opti.fsolve(equations, x0)
        root = equations(result)

        print("Optimization Results:")
        print("----------------------")
        print(f"{'Parameter':<15} {'Value':<15} {'Units':<10}")
        print("----------------------")
        print(f"{'Pitch Angle':<15} {np.rad2deg(result[0]):<15.2f} {'degrees':<10}")
        print(f"{'Delta_t':<15} {np.rad2deg(result[1]):<15.2f} {'degrees':<10}")
        print(f"{'Towing Force':<15} {result[2]:<15.2f} {'N':<10}")
        print("----------------------")
        print(f"{'Total Force X':<15} {root[0]:<15.2f} {'N':<10}")
        print(f"{'Total Force Z':<15} {root[1]:<15.2f} {'N':<10}")
        print(f"{'Total Moment Y':<15} {root[2]:<15.2f} {'Nm':<10}")
        print("Optimization successful!")
      
        return result

    # Solve for equilibrium state given a fixed pitch angle
    def solve_equilibrium_state_fsolve_fixed_pitch(self, pitch_angle, initial_velocity):
        # Initialize system
        pitch_angle = np.deg2rad(pitch_angle)
        initial_state = np.array([0, 0, pitch_angle, initial_velocity, 0, 0, 0, 0, 0])
        self.initialize_system(initial_state)

        # Define the system of equations
        def equations(args):
            delta_i, delta_t, towing_force = args
            
            # Override variables being optimized
            self.controlForces[0].delta_i = delta_i
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
        x0 = (np.deg2rad(-5), np.deg2rad(30), 2)  # Initial guesses for [pitch_angle, delta_t, towing_force, delta_i]

        # Use fsolve to solve the equations
        result = opti.fsolve(equations, x0)

        root = equations(result)

        print("Optimization Results:")
        print("----------------------")
        print(f"{'Parameter':<15} {'Value':<15} {'Units':<10}")
        print("----------------------")
        print(f"{'Delta_i':<15} {np.rad2deg(result[0]):<15.2f} {'degrees':<10}")
        print(f"{'Delta_t':<15} {np.rad2deg(result[1]):<15.2f} {'degrees':<10}")
        print(f"{'Towing Force':<15} {result[2]:<15.2f} {'N':<10}")
        print("----------------------")
        print(f"{'Total Force X':<15} {root[0]:<15.2f} {'N':<10}")
        print(f"{'Total Force Z':<15} {root[1]:<15.2f} {'N':<10}")
        print(f"{'Total Moment Y':<15} {root[2]:<15.2f} {'Nm':<10}")
        print("Optimization successful!")
      

        
        return result
