"Class used to save and run simulation"
import numpy as np
import scipy.optimize as opti
from scipy.optimize import least_squares 
from scipy.integrate import solve_ivp 
import matplotlib.pyplot as plt
# import plotly.graph.objects as go



from .Force import HullForce, ControlForce, TowingForce
from .RigidBody import RigidBody


class Simulation_Result():

    def __init__(self, dt = 0.02, N = 10):
        # Initialize NumPy arrays with shape (N, 2) for 2D vectors and (N,) for 1D arrays.
        # Replace `N` with the desired number of rows, depending on your data needs.
        N += 1
        self.inertial_position = []    # 2D array for [x, z]
        self.pitch_angle = []               # 1D array for pitch angle values
        self.inertial_velocity = []    # 2D array for [x_dot, z_dot]
        self.pitch_rate = []                # 1D array for pitch rate values
        self.inertial_acceleration = [] # 2D array for acceleration in [x_dot, z_dot]
        self.angular_acceleration = []      # 1D array for angular acceleration values

       
        self.bf_velocity = []          # 2D array for body-frame velocity
        self.bf_acceleration = []      # 2D array for body-frame acceleration

        #Tow
        self.tow_force_body = [] # 1D array for body-frame x force of different control forces
        self.tow_force_moment = []          # 1D array for moment

        #Control
        self.control_force_C_D = []                 # 1D array for drag force values
        self.control_force_C_L = []                 # 1D array for lift force values
        self.control_flow_velocity = []         # 2D array for control flow velocity
        self.control_force_inertial = []       # 3D array for body-frame drag and lift of different control forces
        self.control_force_body = []               # 1D array for body-frame x force of different control forces
        self.control_force_alpha_i = []         # 1D array for angle of attack
        self.control_force_moment = []          # 1D array for moment

        #Hull
        self.hull_flow_velocity = []        # 2D array for hull flow velocity
        self.hull_force_inertial = []       # 2D array for hull drag and lift in inertial frame
        self.hull_force_body = []           # Hull force x, z component in body frame
        self.hull_force_moment = []             # 1D array for hull moment

        #Buoyancy
        self.buoyancy_force_body = []     # Hull force x, z component in body frame
        self.buoyancy_moment = []             # 1D array for hull moment

        #Mass
        self.mass_force_body = []     # Hull force x, z component in body frame

        self.i = 0 # iteration number
        self.dt = dt
        self.N = N 
        self.time = []
    
    def plot_simulation_results(self):
        """Plots inertial and pitch states, as well as force magnitudes and moments."""
        x_pos = []
        y_pos = []
        x_vel = []
        y_vel = []
        x_acc = []
        y_acc = []
        ctrl_drag = []
        ctrl_lift = []
        hull_drag = []
        hull_lift = []

        for i, _ in enumerate(self.inertial_position):
            x_pos.append(self.inertial_position[i][0])
            y_pos.append(self.inertial_position[i][1])
            x_vel.append(self.inertial_velocity[i][0])
            y_vel.append(self.inertial_velocity[i][1])
            x_acc.append(self.inertial_acceleration[i][0])
            y_acc.append(self.inertial_acceleration[i][1])
            ctrl_drag.append(self.control_force_inertial[i][0])
            ctrl_lift.append(self.control_force_inertial[i][1])
            hull_drag.append(self.hull_force_inertial[i][0])
            hull_lift.append(self.hull_force_inertial[i][1])

        # Plot 1: Inertial position, velocity, and acceleration
        fig, axs = plt.subplots(3, 1, figsize=(10, 10))
        axs[0].plot(self.time, x_pos , label="x-position (m)")
        axs[0].plot(self.time, y_pos, label="z-position (m)")
        axs[0].set_title("Inertial Position")
        # axs[0].set_ylim([-3, 3])
        axs[0].legend(loc="best")

        axs[1].plot(self.time, x_vel , label="x-velocity (m/s)")
        axs[1].plot(self.time, y_vel, label="z-velocity (m/s)")
        axs[1].set_title("Inertial Velocity")
        # axs[1].set_ylim([1.5, 2.5])
        axs[1].legend(loc="best")

        axs[2].plot(self.time, x_acc, label="x-acceleration (m/s²)")
        axs[2].plot(self.time, y_acc, label="z-acceleration (m/s²)")
        axs[2].set_title("Inertial Acceleration")
        # axs[2].set_ylim([-3, 3])
        axs[2].legend(loc="best")

        fig.tight_layout()
        plt.show()

        # Plot 2: Pitch angle, pitch rate, and angular acceleration
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        axs[0].plot(self.time, np.rad2deg(self.pitch_angle), label="Pitch Angle (deg)")
        axs[0].set_title("Pitch Angle")
        axs[0].legend(loc="best")
        

        axs[1].plot(self.time, np.rad2deg(self.pitch_rate), label="Pitch Rate (deg/s)")
        axs[1].set_title("Pitch Rate")
        axs[1].legend(loc="best")

        axs[2].plot(self.time, np.rad2deg(self.angular_acceleration), label="Angular Acceleration (deg/s²)")
        axs[2].set_title("Angular Acceleration")
        axs[2].legend(loc="best")

        fig.tight_layout()
        plt.show()

        # Plot 3: Control and hull forces (separate lift and drag) and hull moment
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))

        # Control forces: Lift and Drag (aggregated for multiple control forces)
        ####control_drag = np.sum(self.control_force_inertial[:, :, 0], axis=1)
        ####control_lift = np.sum(self.control_force_inertial[:, :, 1], axis=1)
        axs[0].plot(self.time, ctrl_drag, label="Control Drag Force (N)")
        axs[0].plot(self.time, ctrl_lift, label="Control Lift Force (N)")
        axs[0].set_title("Control Forces")
        axs[0].legend(loc="best")

        # Hull forces: Lift and Drag
        axs[1].plot(self.time, hull_drag, label="Hull Drag Force (N)")
        axs[1].plot(self.time, hull_lift, label="Hull Lift Force (N)")
        axs[1].set_title("Hull Forces")
        axs[1].legend(loc="best")

        fig.tight_layout()
        plt.show()

    def save_simulation_results(self, tow_force_x, tow_force_z, tow_force_moment, control_force_C_D, control_force_C_L, control_flow_velocity,
                                control_force_magnitude, control_force_x, control_force_z, control_force_alpha_i, control_force_moment, 
                                hull_flow_velocity, hull_force_magnitude, hull_force_x, hull_force_z, hull_force_moment,
                                buoyancy_force_x, buoyancy_force_z, buoyancy_moment, mass_force_x, mass_force_z, 
                                theta, bf_velocities, bf_accelerations, theta_dot, inertial_position, inertial_velocity, inertial_acceleration, alpha_body):
        
        #Tow
        self.tow_force_body.append([tow_force_x,tow_force_z])
        self.tow_force_moment.append(tow_force_moment)          # 1D array for moment

        #Control
        self.control_force_C_D.append(control_force_C_D)                 # 1D array for drag force values
        self.control_force_C_L.append(control_force_C_L)                 # 1D array for lift force values
        self.control_flow_velocity.append(control_flow_velocity)         # 2D array for control flow velocity
        self.control_force_inertial.append([control_force_magnitude[0][0],control_force_magnitude[0][1]])       # 3D array for body-frame drag and lift of different control forces
        self.control_force_body.append([control_force_x,control_force_z])               # 1D array for body-frame x force of different control forces
        self.control_force_alpha_i.append(control_force_alpha_i)         # 1D array for angle of attack
        self.control_force_moment.append(control_force_moment)          # 1D array for moment

        #Hull
        self.hull_flow_velocity.append(hull_flow_velocity)        # 2D array for hull flow velocity
        self.hull_force_inertial.append([hull_force_magnitude[0],hull_force_magnitude[1]])       # 2D array for hull drag and lift in inertial frame
        self.hull_force_body.append([hull_force_x,hull_force_z])           # Hull force x, z component in body frame
        self.hull_force_moment.append(hull_force_moment)             # 1D array for hull moment

        #Buoyancy
        self.buoyancy_force_body.append([buoyancy_force_x,buoyancy_force_z])     # Hull force x, z component in body frame
        self.buoyancy_moment.append(buoyancy_moment)             # 1D array for hull moment

        #Mass
        self.mass_force_body.append([mass_force_x,mass_force_z])     # Hull force x, z component in body frame
       
        self.bf_velocity.append([bf_velocities[0],bf_velocities[1]])         # 2D array for body-frame velocity
        self.bf_acceleration.append([bf_accelerations[0],bf_accelerations[1]])    # 2D array for body-frame acceleratio
        self.pitch_rate.append(theta_dot)

        self.inertial_acceleration.append([inertial_acceleration[0],inertial_acceleration[1]])
        self.angular_acceleration.append(alpha_body)

        self.inertial_position.append([inertial_position[0],inertial_position[1]]) # 2D array for [x, z]
        self.pitch_angle.append(theta)             # 1D array for pitch angle values
        self.inertial_velocity.append([inertial_velocity[0],inertial_velocity[1]])    # 2D array for [x_dot, z_dot]













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
        self.lb_pitch_angle, self.ub_pitch_angle = np.deg2rad(np.array([-8, -2]))

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
            [cos_theta, sin_theta],
            [-sin_theta, cos_theta]
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
          
    def solve_forces(self, u, w, q):
        # Extract velocity states
        velocity_states = [u, w, q]
        
        # Solve hull force
        V, lift, drag = self.hullForce.calculate_force(velocity_states)
        hull_force_magnitude = np.array([drag, lift])
        hull_flow_velocity = V

        ## Find control forces
        control_force_C_D = []
        control_force_C_L = []
        control_flow_velocity = []
        control_force_magnitude  = []
        control_force_alpha_i = []

        for f_i, force in enumerate(self.rigidbody.control_forces):
            # Calculate control lift and drag force [drag, lift]
            Cl, Cd, V, lift, drag, alpha_i = force.calculate_force(velocity_states)
        
            # Store values
            control_force_C_D.append(Cd)
            control_force_C_L.append(Cl)
            control_flow_velocity.append(V)
            control_force_magnitude.append([drag, lift])
            control_force_alpha_i.append(alpha_i)
        
        return hull_force_magnitude, hull_flow_velocity, control_force_C_D, control_force_C_L, control_flow_velocity, control_force_magnitude, control_force_alpha_i


    ##################################################################################################
    ######################################## FORWARD EULER ###########################################
    ##################################################################################################
    
    def simulate_forward_euler(self, N, dt, initial_state):
        #  Initialize the simulation
        dt = self.sim.dt
        self.sim = Simulation_Result(dt, N, len(self.controlForces))
       
        self.initialize_system(initial_state)
        
        for i in range(1, N):
            # print(i)
            theta = self.sim.pitch_angle[i - 1]
            # 1. Calculate forces and moments in the body frame
            self.solve_forces( i - 1)
            total_force = self.rigidbody.sum_forces(theta)  # F_body = [Fx, Fz] in body frame
            total_moment = self.rigidbody.sum_moments(theta)  # M_body = My

            # 2. Calculate body frame accelerations (q_dot_dot)
            ax_body = total_force[0] / self.rigidbody.mass + self.sim.pitch_rate[i - 1] * self.sim.bf_velocity[i - 1, 1]
            az_body = total_force[1] / self.rigidbody.mass - self.sim.pitch_rate[i - 1] * self.sim.bf_velocity[i - 1, 0] 
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
    ######################################## RK45 SOLVER #############################################
    ##################################################################################################

    def transformation_matrix_dot(self, theta, theta_dot):
        """Returns the time derivative of the transformation matrix."""
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        return np.array([[-sin_theta * theta_dot, cos_theta * theta_dot],
                         [-cos_theta * theta_dot, -sin_theta * theta_dot]])

    def system_dynamics(self, t, y):
        """ODE function for solve_ivp. Calculates derivatives at each timestep."""
        i = self.sim.i  # Determine the current step index

        #0. Unpack the state vector y = [x, z, theta, x_dot, z_dot, theta_dot]
        x, z, theta, x_dot, z_dot, theta_dot = y

        # 1. Calculate body-frame velocities (bf_velocities)
        T = self.transformation_matrix(theta)
        bf_velocities = T.T @ np.array([x_dot, z_dot])  # Body-frame velocities

        # 2. Solve for forces using the body-frame velocities
        hull_force_magnitude, hull_flow_velocity, control_force_C_D, control_force_C_L, control_flow_velocity, control_force_magnitude, control_force_alpha_i = \
            self.solve_forces(bf_velocities[0], bf_velocities[1], theta_dot)

        # 3. Get total forces and moments
        total_force_x, mass_force_x, buoyancy_force_x, tow_force_x, control_force_x, hull_force_x , \
                total_force_z, mass_force_z, buoyancy_force_z, tow_force_z, control_force_z, hull_force_z = self.rigidbody.sum_forces(theta)  # [Fx, Fz] in body frame
        total_moment, buoyancy_moment, tow_force_moment, control_force_moment, hull_force_moment = self.rigidbody.sum_moments(theta)  # Pitch moment

        # 4. Calculate body-frame accelerations (q_dot_dot)
        ax_body = (
            total_force_x / self.rigidbody.mass 
            + theta_dot * bf_velocities[1]
        )
        az_body = (
            total_force_z / self.rigidbody.mass 
            - theta_dot * bf_velocities[0]
        )
        bf_accelerations = np.array([ax_body, az_body])

        # 5. Calculate angular acceleration (alpha_body)
        alpha_body = total_moment / self.rigidbody.Iyy

        # 6. Transform body accelerations to the inertial frame using T_full
        T_dot = self.transformation_matrix_dot(theta, theta_dot)
        T_full = np.block([
            [T, np.zeros((2, 2))],
            [T_dot, T]
        ])

        # Combine accelerations and velocities into vectors for transformation
        body_acc_vector = np.hstack((bf_velocities, bf_accelerations))
        inertial_acc_vector = T_full @ body_acc_vector
        inertial_acceleration = [inertial_acc_vector[2],inertial_acc_vector[3]]  # [x_ddot, z_ddot]         # Extract inertial-frame accelerations from the transformed vector
        inertial_position = [x, z]           #as given by solver
        inertial_velocity = [x_dot, z_dot] #as given by solver

        ## Append results      
        self.sim.save_simulation_results(tow_force_x, tow_force_z, tow_force_moment, control_force_C_D, control_force_C_L, control_flow_velocity,
                                control_force_magnitude, control_force_x, control_force_z, control_force_alpha_i, control_force_moment, 
                                hull_flow_velocity, hull_force_magnitude, hull_force_x, hull_force_z, hull_force_moment,
                                buoyancy_force_x, buoyancy_force_z, buoyancy_moment, mass_force_x, mass_force_z, 
                                theta, bf_velocities, bf_accelerations, theta_dot, inertial_position, inertial_velocity, inertial_acceleration, alpha_body)
        #Iterate
        self.sim.i += 1
        self.sim.time.append(t)

        # 7. Return derivatives [x_dot, z_dot, theta_dot, x_ddot, z_ddot, theta_ddot]
        return [
            x_dot,
            z_dot,
            theta_dot,
            inertial_acceleration[0],  # x_ddot in inertial frame
            inertial_acceleration[1],  # z_ddot in inertial frame
            alpha_body                 # theta_ddot (angular acceleration)
        ]
    

    def simulate_solve_ivp(self, N, dt, initial_state):
        """Simulates the system using solve_ivp."""
        self.sim = Simulation_Result(dt, N)
        #### self.initialize_system(initial_state) #can delete

        # Define the time span and evaluation points
        t_span = (0, N * dt)
        t_eval = np.linspace(0, N * dt, N + 1)
        y0 = initial_state[:6]

        # Call solve_ivp to solve the dynamics
        solution = solve_ivp(
            fun=self.system_dynamics,
            t_span=t_span,
            y0=y0,
            t_eval=t_eval,
            method='RK45'
        )


        return self.sim, solution
    
    ##################################################################################################
    ######################################## CALCULATE JACOBIAN ######################################
    ##################################################################################################

    def calculate_jacobian(self, v, pitch_angle, epsilon):
        # Initialize system
        self.sim = Simulation_Result(dt = 1, N = 2, nb_control_forces= len(self.controlForces))
        full_initial_state = np.zeros(9)
        full_initial_state[2] = pitch_angle
        full_initial_state[3] = v
        self.initialize_system(full_initial_state)

        x = np.zeros(6)
        x[2] = pitch_angle
        x[3] = v
        xdot = self.system_dynamics(0, x)
        # Calculate the Jacobian
        jacobian = np.zeros((6, 6))
        for i in range(6):
            # Perturb the state vector
            perturbed_state = np.copy(x)
            perturbed_state[i] += epsilon
            # Simulate the system with the perturbed state
            xdot_ie = np.array(self.system_dynamics(0, perturbed_state))
            # Calculate the Jacobian element
            jacobian[:, i] = (xdot - xdot_ie) / epsilon
            print('pstate '+ str(perturbed_state))
            print('xdot pertu '+ str(xdot_ie))
        return jacobian



    ##################################################################################################
    ######################################## EQUILIBRIUM STATE ########################################
    #################################################################################################

    def solve_equilibrium_state_LS(self, initial_velocity):
        # Initialize system
        self.eq_sim = Simulation_Result(0, 0)

        # Define residual function for least squares optimization
        def residuals(args):
            pitch_angle, delta_t, towing_force, delta_i = args

            # Set parameters for the system
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            self.controlForces[0].delta_i = delta_i
            
            # Solve forces
            _, _, _, _, _, _, _ = self.solve_forces(initial_velocity, 0, 0)

            # Calculate sum of forces/moments
            total_force_x, _, _, _, _, _, total_force_z, _, _, _, _, _ = self.rigidbody.sum_forces(pitch_angle)
            total_moment_y, _, _, _, _ = self.rigidbody.sum_moments(pitch_angle)

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
        residual = residuals(result.x)

        # Print Results
        print("Optimization Results:")
        print("----------------------")
        print(f"{'Parameter':<15} {'Value':<15} {'Units':<10}")
        print("----------------------")
        print(f"{'Pitch Angle':<15} {np.rad2deg(result.x[0]):<15.2f} {'degrees':<10}")
        print(f"{'Delta_t':<15} {np.rad2deg(result.x[1]):<15.2f} {'degrees':<10}")
        print(f"{'Towing Force':<15} {result.x[2]:<15.2f} {'N':<10}")
        print(f"{'Delta_i':<15} {np.rad2deg(result.x[3]):<15.2f} {'degrees':<10}")
        print("----------------------")
        print(f"{'Fx:':<15} {residual[0]:<15.2f}{'N':<10}")
        print(f"{'Fz:':<15} {residual[1]:<15.2f}{'N':<10}")
        print(f"{'My:':<15} {residual[2]:<15.2f}{'Nm':<10}")
        print(f"Residual Norm: {result.cost:.6f}")

        if result.success:
            print("Optimization successful!")
        else:
            print("Optimization failed:", result.message)

        ## Save equilibrium state
        # Set parameters for the system
        theta = result.x[0]
        self.towingForce.delta_t = result.x[1]
        self.towingForce.magnitude = result.x[2]
        self.controlForces[0].delta_i = result.x[3]

        # Calculate sum of forces/moments
        hull_force_magnitude, hull_flow_velocity, control_force_C_D, control_force_C_L, control_flow_velocity, control_force_magnitude, control_force_alpha_i \
              = self.solve_forces(initial_velocity, 0, 0)    
        
        _, mass_force_x, buoyancy_force_x, tow_force_x, control_force_x, hull_force_x , \
                _, mass_force_z, buoyancy_force_z, tow_force_z, control_force_z, hull_force_z \
                    = self.rigidbody.sum_forces(theta)
        
        _, buoyancy_moment, tow_force_moment, control_force_moment, hull_force_moment \
            = self.rigidbody.sum_moments(theta)  # Pitch moment

        T = self.transformation_matrix(theta)
        theta_dot = 0
        inertial_position = [0,0]
        inertial_velocity = [initial_velocity, 0]
        inertial_acceleration = [0,0]
        alpha_body = 0
        bf_velocities = T.T @ inertial_velocity
        bf_accelerations = T.T @ inertial_acceleration

        # Save equilibrium results
        self.eq_sim.save_simulation_results(tow_force_x, tow_force_z, tow_force_moment, control_force_C_D, control_force_C_L, control_flow_velocity,
                                control_force_magnitude, control_force_x, control_force_z, control_force_alpha_i, control_force_moment, 
                                hull_flow_velocity, hull_force_magnitude, hull_force_x, hull_force_z, hull_force_moment,
                                buoyancy_force_x, buoyancy_force_z, buoyancy_moment, mass_force_x, mass_force_z, 
                                theta, bf_velocities, bf_accelerations, theta_dot, inertial_position, inertial_velocity, inertial_acceleration, alpha_body)
        
        return result.x
    
    def solve_equilibrium_state_sqrt(self, initial_velocity):
            
            # Define objective function for minimization
            def objective(args):
                pitch_angle, delta_t, towing_force, delta_i = args

                # Initialize system
                initial_state = np.array([0, 0, pitch_angle, initial_velocity, 0, 0, 0, 0, 0])
                self.initialize_system(initial_state)

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

        #Define objective function
        def objective_function_1(args):
            pitch_angle, delta_t, towing_force, delta_i = args
            #Override variables being optimized
            self.towingForce.delta_t = delta_t
            self.towingForce.magnitude = towing_force
            self.controlForces[0].delta_i = delta_i

            #Initialize system
            initial_state = np.array([0, 0, pitch_angle, initial_velocity, 0, 0, 0, 0, 0])
            self.initialize_system(initial_state)

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
