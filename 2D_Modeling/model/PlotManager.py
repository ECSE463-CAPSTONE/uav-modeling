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