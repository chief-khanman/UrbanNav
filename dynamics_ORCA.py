import math
from dynamics_template import DynamicsTemplate
from shapely import Point
from uav_v2_template import UAV_v2_template
from uav_v2 import UAV_v2
from auto_uav_v2 import Auto_UAV_v2
import numpy as np


class ORCA_Dynamics(DynamicsTemplate):
    def __init__(self, dt=0.1):
        super().__init__(dt)

    def update(self, uav, action):
        return None
