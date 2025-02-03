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

        #Positional Variables


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
        

    
    def flow_velocity(self, u, v, w):
        V = math.sqrt( (u) ** 2 + (v) ** 2 + (w) ** 2 )
        return V
    
    def flow_alpha(self, u, v, w):
        alpha = math.atan2(w, u)
        return alpha
    
    def flow_beta(self, u, v, w, V):
        beta = math.asin((v) / V)
        return beta
    
    def get_coefficients(self, alpha, beta):       
        yaw_pitch_points = self.map[["Yaw", "Pitch"]].to_numpy()
        values = self.map[["CDA", "CLA", "CmYA", "CmZA", "CmXA", "CSA"]].to_numpy()
        interpolated_values = griddata(yaw_pitch_points, values, (beta, alpha), method='linear')
        return interpolated_values #[CDA, CLA, CmYA, CmZA, CmXA, CSA]

    
    def translate_force():       
        return
    
    def rotate_force():
        return
    