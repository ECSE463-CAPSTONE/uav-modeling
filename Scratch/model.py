import numpy as np

class Force2D:
    def __init__(self, position, magnitude = 0, direction = 0):
        self.position = np.array(position)  # Position relative to body origin (x, y)
        self.magnitude = magnitude  # Force magnitude
        self.direction = np.deg2rad(direction)  # Force direction in radians

    def get_force_vector(self):
        # Returns the force as a 2D vector (Fx, Fy)
        fx = self.magnitude * np.cos(self.direction)
        fy = self.magnitude * np.sin(self.direction)
        return np.array([fx, fy])

class Vehicle2D:
    def __init__(self, cm, cb):
        self.cm = np.array(cm)  # Center of mass position body frame
        self.cb = np.array(cb)  # Center of buoyancy position body frame
        self.forces = []  # List of forces applied to the vehicle

    def add_force(self, position, magnitude, direction):
        # Adds a new force to the list of forces
        force = Force2D(position, magnitude, direction)
        self.forces.append(force)

    def net_force(self):
        # Calculates the net force on the vehicle
        total_force = np.array([0.0, 0.0])
        for force in self.forces:
            total_force += force.get_force_vector()
        return total_force

    def net_moment(self):
        # Calculates the net moment about the center of mass
        total_moment = 0.0
        for force in self.forces:
            r = force.position - self.cm
            F = force.get_force_vector()
            moment = np.cross(r, F)  # Moment = r x F (2D cross product gives scalar)
            total_moment += moment
        return total_moment

# Example usage
vehicle = Vehicle2D(cm=(0, 0))
vehicle.add_force(position=(1, 0), magnitude=10, direction=90)  # Force of 10N upward at (1,0)
vehicle.add_force(position=(-1, 0), magnitude=5, direction=0)   # Force of 5N rightward at (-1,0)

net_force = vehicle.net_force()
net_moment = vehicle.net_moment()

print(f"Net Force: {net_force}")
print(f"Net Moment: {net_moment}")
