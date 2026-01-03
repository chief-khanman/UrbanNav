from typing import List, Dict, Tuple
from controller_template import ControllerTemplate
from shapely import Point
import math

class NonCoopControllerSmooth(ControllerTemplate):

    def __init__(self, max_acceleration, max_heading_change):
        super().__init__(max_acceleration, max_heading_change)
    
    def __call__(self, observation:List[Dict])-> Tuple:
        self_observation = observation[0]
        final_heading = math.atan2(
            self_observation['end'].y - self_observation['current_position'].y, 
            self_observation['end'].x - self_observation['current_position'].x)

        d_heading = final_heading - self_observation['current_heading']  
        
        d_heading = (d_heading + math.pi) % (2*math.pi) - math.pi

        # TODO: add a percentage of heading_change for smooth operation
        heading_change = max(
                            -self.max_heading_change, 
                             min(d_heading, self.max_heading_change)
                            )
        acceleration = self.max_acceleration # replace with constant acceleration 
        
        action = (acceleration, heading_change)
        return action