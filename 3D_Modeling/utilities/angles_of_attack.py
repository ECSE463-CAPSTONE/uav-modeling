import numpy as np

def compute_velocity_alpha_beta(velocity_states, relative_location):
    """
    Compute velocity magnitude, angle of attack (alpha), and sideslip angle (beta) for a control force location.
    
    Args:
        velocity_states (array): Array containing u, v, w, p, q, r (body-frame velocities and angular rates).
        relative_location (array): Position of the control force relative to the center of mass.
    
    Returns:
        tuple: (Velocity magnitude, angle of attack, sideslip angle)
    """
    u, v, w, p, q, r = velocity_states
    r_x, r_y, r_z = relative_location
    
    # Compute local velocity at the control force location
    V_x = u + q * r_z - r * r_y
    V_y = v + r * r_x - p * r_z
    V_z = w + p * r_y - q * r_x
    
    # Compute total velocity magnitude
    V = np.sqrt(V_x**2 + V_y**2 + V_z**2)
    
    # Compute angles
    alpha_i = np.arctan2(V_z, V_x)  # Angle of attack
    beta_i = np.arcsin(V_y / V)     # Sideslip angle
    
    return V, alpha_i, beta_i
