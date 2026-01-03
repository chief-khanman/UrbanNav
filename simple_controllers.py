import shapely
import math
import numpy as np
from controller_template import ControllerTemplate
from utils_data_transform import normalize_zero_one, normalize_minus_one_one
from typing import Dict 



class SimpleController(ControllerTemplate):
    def __init__(self, max_acceleration, max_heading_change):
        super().__init__(max_acceleration, max_heading_change)
        return None
    
    def __call__(self, observation:Dict):
        # for vp design info is being used
        final_position = observation['end']
        current_position = observation['current_position']
        current_heading = observation['current_heading']


        acceleration = np.random.uniform(0,1)
        final_heading = math.atan2(final_position.y - current_position.y, 
                                    final_position.x - current_position.x)
        
        heading_change_rad = final_heading - current_heading
        
        heading_change = normalize_minus_one_one(heading_change_rad, -math.pi, math.pi)

        action = np.array([acceleration, heading_change])
        
        
        return action





# def simple_controller(current_position:shapely.Point, final_position:shapely.Point, current_heading:float):
    
#     acceleration = np.random.uniform(0,1)
#     final_heading = math.atan2(final_position.y - current_position.y, 
#                                 final_position.x - current_position.x)
    
#     heading_change_rad = final_heading - current_heading
    
#     heading_change = normalize_minus_one_one(heading_change_rad, -math.pi, math.pi)

#     action = np.array([acceleration, heading_change])
    
#     return action