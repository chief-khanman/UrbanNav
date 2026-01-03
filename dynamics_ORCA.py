import math
from dynamics_template import DynamicsTemplate
from shapely import Point
from uav_template import UAV_template
from uav import UAV
from auto_uav import Auto_UAV
import numpy as np


class ORCA_Dynamics(DynamicsTemplate):
    def __init__(self, dt=0.1):
        super().__init__(dt)

    def update(self, uav, action):
        return None
