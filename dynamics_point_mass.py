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
        uav.current_heading += heading_change
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
