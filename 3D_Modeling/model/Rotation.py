import numpy as np



def R_x(theta_x):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta_x), -np.sin(theta_x)],
        [0, np.sin(theta_x), np.cos(theta_x)]
    ])

def R_x_dot(theta_x, theta_x_dot):
    return theta_x_dot * np.array([
        [0, 0, 0],
        [0, -np.sin(theta_x), -np.cos(theta_x)],
        [0, np.cos(theta_x), -np.sin(theta_x)]
    ])

def R_y(theta_y):
    return np.array([
        [np.cos(theta_y), 0, np.sin(theta_y)],
        [0, 1, 0],
        [-np.sin(theta_y), 0, np.cos(theta_y)]
    ])

def R_y_dot(theta_y, theta_y_dot):
    return theta_y_dot * np.array([
        [-np.sin(theta_y), 0, np.cos(theta_y)],
        [0, 0, 0],
        [-np.cos(theta_y), 0, -np.sin(theta_y)]
    ])

def R_z(theta_z):
    return np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z), 0],
        [0, 0, 1]
    ])

def R_z_dot(theta_z, theta_z_dot):
    return theta_z_dot * np.array([
        [-np.sin(theta_z), -np.cos(theta_z), 0],
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [0, 0, 0]
    ])

