import os
import sys
import matplotlib.pyplot as plt
import numpy as np
from tabulate import tabulate


class PlotManager():
    def __init__(self):
        ()

    def plotfbd(self, sim:object, sim_result:object):
    # Define the rotation angle in radians
        theta = -sim_result.pitch_angle[0] # Rotate by pitch (adjust as needed)

        #Hard coding i = 0 for equilibrium purposes only
        i = 0

        # Rotation matrix
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])

        # Define forces with their points of application (relative to origin)
        r_xi = sim.rigidbody.control_forces[0].location[0]
        r_zi = sim.rigidbody.control_forces[0].location[1]
        r_xh = sim.rigidbody.hull_force.location[0]
        r_zh = sim.rigidbody.hull_force.location[1]
        r_xt = sim.rigidbody.tow_force.location[0]
        r_zt = sim.rigidbody.tow_force.location[1]
        r_xb = sim.rigidbody.center_of_buoyancy[0]
        r_zb = sim.rigidbody.center_of_buoyancy[1]
        
        forces = [
            {"point": (r_xi, r_zi), "vector": (0, sim_result.control_force_body[i][1])},            # Control z-force
            {"point": (r_xi, r_zi), "vector": (sim_result.control_force_body[i][0], 0)},            # Control x-force
            {"point": (r_xh, r_zh), "vector": (sim_result.hull_force_body[i][0], sim_result.hull_force_body[i][1])}, # Hull drag
            {"point": (r_xt, r_zt), "vector": (sim_result.tow_force_body[i][0], sim_result.tow_force_body[i][1])},   # Tow force
            {"point": (r_xb, r_zb), "vector": (sim_result.buoyancy_force_body[i][0], sim_result.buoyancy_force_body[i][1])}, # Buoyancy
            {"point": (0, 0), "vector": (sim_result.mass_force_body[i][0], sim_result.mass_force_body[i][1])}       # Weight
        ]

        # Determine the maximum length of the vectors
        max_length = max(np.linalg.norm(force['vector']) for force in forces)

        # Specify the desired maximum length on the plot
        desired_max_length = 0.1

        # Calculate scaling factor
        scaling_factor = desired_max_length / max_length if max_length > 0 else 1

        # Define rectangle properties
        rect_center = (0,0)  # Center of the rectangle
        rect_width, rect_height = 0.15,.08

        # Define rectangle corners (before rotation)
        corners = [
            (rect_center[0] - rect_width / 2, rect_center[1] - rect_height / 2),  # Bottom-left
            (rect_center[0] + rect_width / 2, rect_center[1] - rect_height / 2),  # Bottom-right
            (rect_center[0] + rect_width / 2, rect_center[1] + rect_height / 2),  # Top-right
            (rect_center[0] - rect_width / 2, rect_center[1] + rect_height / 2)   # Top-left
        ]

        # Rotate forces and rectangle corners
        transformed_forces = []
        for force in forces:
            # Transform point and vector
            rotated_point = np.dot(rotation_matrix, force["point"])
            rotated_vector = np.dot(rotation_matrix, force["vector"])

            # Scale the rotated vector
            scaled_vector = rotated_vector * scaling_factor
            
            # Store transformed point and vector
            transformed_forces.append({"point": rotated_point, "vector": scaled_vector})

        # Rotate rectangle corners
        rotated_corners = [np.dot(rotation_matrix, corner) for corner in corners]
        rotated_corners.append(rotated_corners[0])  # Close the rectangle by repeating the first point

        # Calculate limits based on rotated forces and rectangle
        padding = 0.05
        x_min = min(point[0] + vector[0] for force in transformed_forces for point, vector in zip([force['point']], [force['vector']])) - padding
        x_max = max(point[0] + vector[0] for force in transformed_forces for point, vector in zip([force['point']], [force['vector']])) + padding
        y_min = min(point[1] + vector[1] for force in transformed_forces for point, vector in zip([force['point']], [force['vector']])) - padding
        y_max = max(point[1] + vector[1] for force in transformed_forces for point, vector in zip([force['point']], [force['vector']])) + padding

        # Set up plot
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_aspect('equal')

        # Set axis limits
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)

        # Plot transformed forces
        for force in transformed_forces:
            point = force["point"]
            vector = force["vector"]
            ax.quiver(point[0], point[1], vector[0], vector[1], angles='xy', scale_units='xy', scale=1, color='blue')
            ax.plot([0,point[0]], [0,point[1]],'.-')

        # Plot the rotated rectangle
        rotated_x = [corner[0] for corner in rotated_corners]
        rotated_y = [corner[1] for corner in rotated_corners]
        ax.plot(rotated_x, rotated_y, 'g-')  # Green rectangle outline

        # Flip the appearance of the axes without changing vector directions
        ax.invert_xaxis()  # Flip x-axis appearance
        ax.invert_yaxis()  # Flip y-axis appearance

        # Optional: Add grid, labels, and title
        plt.grid(True)
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Free Body Diagram with Rotated Forces and Rectangle')

        plt.show()


    def plot_simulation_results(self,sim_result:object):
        """Plots inertial and pitch states, as well as force magnitudes and moments."""
        #Extract data
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

        for i, _ in enumerate(sim_result.inertial_position):
            x_pos.append(sim_result.inertial_position[i][0])
            y_pos.append(sim_result.inertial_position[i][1])
            x_vel.append(sim_result.inertial_velocity[i][0])
            y_vel.append(sim_result.inertial_velocity[i][1])
            x_acc.append(sim_result.inertial_acceleration[i][0])
            y_acc.append(sim_result.inertial_acceleration[i][1])
            ctrl_drag.append(sim_result.control_force_inertial[i][0])
            ctrl_lift.append(sim_result.control_force_inertial[i][1])
            hull_drag.append(sim_result.hull_force_inertial[i][0])
            hull_lift.append(sim_result.hull_force_inertial[i][1])

        # Plot 1: Inertial position, velocity, and acceleration
        fig, axs = plt.subplots(3, 1, figsize=(10, 10))
        axs[0].plot(sim_result.time2, sim_result.inertial_position[:,0] , label="x-position (m)")
        axs[0].plot(sim_result.time2, sim_result.inertial_position[:,1], label="z-position (m)")
        axs[0].set_title("Inertial Position")
        # axs[0].set_ylim([-3, 3])
        axs[0].legend(loc="best")

        axs[1].plot(sim_result.time2, sim_result.inertial_velocity[:,0] , label="x-velocity (m/s)")
        axs[1].plot(sim_result.time2, sim_result.inertial_velocity[:,1], label="z-velocity (m/s)")
        axs[1].set_title("Inertial Velocity")
        # axs[1].set_ylim([1.5, 2.5])
        axs[1].legend(loc="best")

        axs[2].plot(sim_result.time2, x_acc, label="x-acceleration (m/s²)")
        axs[2].plot(sim_result.time2, y_acc, label="z-acceleration (m/s²)")
        axs[2].set_title("Inertial Acceleration")
        # axs[2].set_ylim([-3, 3])
        axs[2].legend(loc="best")

        fig.tight_layout()
        plt.show()

        # Plot 2: Pitch angle, pitch rate, and angular acceleration
        fig, axs = plt.subplots(3, 1, figsize=(10, 8))
        axs[0].plot(sim_result.time2, np.rad2deg(sim_result.pitch_angle), label="Pitch Angle (deg)")
        axs[0].set_title("Pitch Angle")
        axs[0].legend(loc="best")
        

        axs[1].plot(sim_result.time2, np.rad2deg(sim_result.pitch_rate), label="Pitch Rate (deg/s)")
        axs[1].set_title("Pitch Rate")
        axs[1].legend(loc="best")

        axs[2].plot(sim_result.time, np.rad2deg(sim_result.angular_acceleration), label="Angular Acceleration (deg/s²)")
        axs[2].set_title("Angular Acceleration")
        axs[2].legend(loc="best")

        fig.tight_layout()
        plt.show()

        # Plot 3: Control and hull forces (separate lift and drag) and hull moment
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))

        # Control forces: Lift and Drag (aggregated for multiple control forces)
        ####control_drag = np.sum(sim_result.control_force_inertial[:, :, 0], axis=1)
        ####control_lift = np.sum(sim_result.control_force_inertial[:, :, 1], axis=1)
        axs[0].plot(sim_result.time2, ctrl_drag, label="Control Drag Force (N)")
        axs[0].plot(sim_result.time2, ctrl_lift, label="Control Lift Force (N)")
        axs[0].set_title("Control Forces")
        axs[0].legend(loc="best")

        # Hull forces: Lift and Drag
        axs[1].plot(sim_result.time2, hull_drag, label="Hull Drag Force (N)")
        axs[1].plot(sim_result.time2, hull_lift, label="Hull Lift Force (N)")
        axs[1].set_title("Hull Forces")
        axs[1].legend(loc="best")

        fig.tight_layout()
        plt.show()