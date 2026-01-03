import math
from dynamics_template import DynamicsTemplate
from shapely import Point
from uav_v2_template import UAV_v2_template
from uav_v2 import UAV_v2
from auto_uav_v2 import Auto_UAV_v2
import numpy as np

class PointMassDynamics(DynamicsTemplate):

    def __init__(self, dt=0.1):
        super().__init__(dt)

    def update(self,uav:UAV_v2_template, action):
        """Update the UAV's position, speed, and heading based on the action taken."""
        # action[0] -> acceleration
        # action[1] -> heading_change 
        # print(f"action: {action}")

        if isinstance(uav, UAV_v2):
            self.is_learning = False
            # print("===UAV_v2===")
        elif isinstance(uav, Auto_UAV_v2): 
            self.is_learning = True
            # print("===Auto_UAV_v2===")
        else:
            raise ValueError("Unknown UAV type")

        if self.is_learning:
            # print("===is_learning block===")
            # Force action to be a numpy array and flatten to ensure operation in action sampling and prediction
            action = np.array(action)
            action = action.flatten()
            # Normalize action to match UAV's dynamics
            self._acceleration = uav.max_acceleration * action[0] # max_acceleration set to 1
            self._heading_change = uav.max_heading_change * action[1] # max_heading_change set to math.pi
            # print(self._acceleration)
            # print(self._heading_change)
        else:
            # print("===not learning block===")
            # Use the action directly
            self._acceleration = action[0]
            self._heading_change = action[1]
            # print(self._acceleration)
            # print(self._heading_change)

        if uav.mission_complete_status:
            #UAV has reached its destination
            uav.current_position = uav.end
        else:
            #!TODO - need to make sure we enforce max speed limit
            new_speed = uav.current_speed + self._acceleration * self.dt
            # Ensure speed is within bounds
            new_speed = max(0, min(new_speed, uav.max_speed))
            # Bound heading between -π and π
            new_heading = uav.current_heading + self._heading_change
            new_heading = ((new_heading + math.pi) % (2 * math.pi)) - math.pi

            v_x = new_speed * math.cos(new_heading)
            a_x = self._acceleration * math.cos(new_heading)
            
            v_y = new_speed * math.sin(new_heading)
            a_y = self._acceleration * math.sin(new_heading)

            _x = uav.current_position.x + (v_x * self.dt) + (0.5 * a_x * self.dt**2)
            _y = uav.current_position.y + (v_y * self.dt) + (0.5 * a_y * self.dt**2)

            uav.current_position = Point(_x, _y)
            uav.current_speed = new_speed
            uav.current_heading = new_heading
            
            #TODO: this needs to be placed somewhere more appropriate 
            #TODO: WHY only AutoUAV - WHY not other UAVs ??
            if isinstance(uav, Auto_UAV_v2):
                
                updated_final_heading = math.atan2(uav.end.y - uav.current_position.y,
                                               uav.end.x - uav.current_position.x)
                
                uav.final_heading = updated_final_heading
                # print(f'new final heading: {uav.final_heading}')
                