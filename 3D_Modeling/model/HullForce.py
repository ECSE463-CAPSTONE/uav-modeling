import numpy as np
import pandas as pd
import math
from scipy.interpolate import griddata

#global variable
rho = 999.7


class HullForce:
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.map = self.load_map()

        #Positional Variables describing the nose position relative to the COM
        self.r_x = []
        self.r_y = []
        self.r_z = []

    def load_map(self):
        map = pd.read_csv(self.file_path)
        return map
  
    def calculate_force(self,u,v,w):
        # The main function to calculate all the forces
        V = self.flow_velocity(u, v, w)
        alpha = self.flow_alpha(u, v, w)
        beta = self.flow_beta(u, v, w, V)
        coefficients = self.get_coefficients(alpha, beta)

        force_moments = 0.5 * rho * coefficients * V ** 2 #forces and moments at the nose
        

    
    def flow_velocity(self, u, v, w, p, q, r):
        V = math.sqrt((u + q * self.r_z - r * self.r_y) ** 2 +
                     (v + r * self.r_x - p * self.r_z) ** 2 +
                     (w + p * self.r_y - q * self.r_x) ** 2)
        return V
    
    def flow_alpha(self, u, v, w, p, q, r):
        alpha = math.atan2((w + p * self.r_y - q * self.r_x), (u + q * self.r_z - r * self.r_y))
        return alpha
    
    def flow_beta(self, u, v, w, p, q, r, V):
        beta = math.asin((v + r * self.r_x - p * self.r_z) / V)
        return beta
    
    def get_coefficients(self, alpha, beta):       
        yaw_pitch_points = self.map[["Yaw", "Pitch"]].to_numpy()
        values = self.map[["CDA", "CSA", "CLA", "CmXA", "CmYA", "CmZA"]].to_numpy()
        interpolated_values = griddata(yaw_pitch_points, values, (beta, alpha), method='linear')
        return interpolated_values #["CDA", "CSA", "CLA", "CmXA", "CmYA", "CmZA"]

    
    def translate_force():
        return
    
    def rotate_force():
        return
    