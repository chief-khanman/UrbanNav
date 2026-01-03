#controller_ORCA.py

import rvo2 
from controller_template import ControllerTemplate

import numpy as np 
from typing import Tuple, List, Dict


class ORCA_controller(ControllerTemplate):
    '''Every UAV that uses ORCA will get its own local RVO2 simulator 
    which will calculate necessary attribute - new velocity.'''

    def __init__(self, max_acceleration, max_heading_change):
        self.max_acceleration = max_acceleration
        self.max_heading_change = max_heading_change




    def __call__(self, observation):
        return (0,0) 
