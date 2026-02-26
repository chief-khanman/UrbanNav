import math
from dynamics_template import Dynamics
from uav import UAV
from uav_template import UAV_template

class PointMass(Dynamics):
    def __init__(self,):
        super().__init__()


    def step(self, action, uav: UAV|UAV_template) -> None:
        #! make sure heading_change is in radians 
        acceleration, heading_change = action 

        # update heading
        #! make sure to perform heading wrap around math
        #! heading should be between -math.pi, math.pi
        uav.current_heading += heading_change 
        # update speed 
        uav.current_speed += acceleration * self.dt
        # update velocity 
        uav.vx = uav.current_speed * math.cos(heading_change)
        uav.vy = uav.current_speed * math.sin(heading_change)
        # uav.vz -> unchanged 

