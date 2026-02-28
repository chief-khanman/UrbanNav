import math
import numpy as np
from shapely import Point
from dynamics_template import Dynamics
from uav import UAV
from uav_template import UAV_template

class PointMass(Dynamics):
    ''' Non holonomic model '''
    def __init__(self,):
        super().__init__()


    def step(self, action, uav: UAV|UAV_template) -> None:
        #! make sure heading_change is in radians
        acceleration, yaw_rate_command = action
        acceleration = np.clip(acceleration, -uav.max_acceleration, uav.max_acceleration)
        # update heading
        if uav.current_speed > 0.1:
            dynamic_max_yaw_rate = uav.max_lateral_acceleration/uav.current_speed
            yaw_rate = np.clip(yaw_rate_command, -dynamic_max_yaw_rate, dynamic_max_yaw_rate)
        else:
            yaw_rate = yaw_rate_command

        uav.current_heading += yaw_rate * self.dt
        # Wrap heading to [-π, π].
        # (θ + π) % (2π) − π maps any angle onto [-π, π]:
        #   shift up by π  →  [0, 2π] domain for modulo
        #   modulo 2π      →  discard full rotations
        #   shift down by π →  back to [-π, π]
        uav.current_heading = (uav.current_heading + math.pi) % (2 * math.pi) - math.pi

        # update speed
        uav.current_speed += acceleration * self.dt
        # update velocity
        uav.vx = uav.current_speed * math.cos(uav.current_heading)
        uav.vy = uav.current_speed * math.sin(uav.current_heading)
        # uav.vz -> unchanged
        uav.current_position = Point(uav.current_position.x + uav.vx * self.dt, uav.current_position.y + uav.vy * self.dt)
