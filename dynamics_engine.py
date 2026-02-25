from abc import ABC
from typing import Dict, Any, List
from uav import UAV
from component_schema import UAMConfig, VALID_DYNAMICS
from dynamics_point_mass import PointMass
from dynamics_six_dof import SixDOF
from dynamics_two_d_vector import TwoDVector
# in the config file - 
# dynamics mode and solver will be defined for use with DynamicsEngine()
#  
class DynamicsEngine:
    def __init__(self, 
                 config:UAMConfig, 
                 dynamics_uav_map:Dict[str, List[int]],
                 uav_dict:Dict[int, UAV]):
        self.config = config
        self.dt = self.config.simulator.dt
        # comes from atc.dynamics_map
        self.dynamics_uav_map = dynamics_uav_map # dict: dynamics_str -> [id1, id2, ....]
        # comes from atc
        self.uav_dict = uav_dict # dict: 'id1': UAV, 'id2':UAV, ....
        # not defined yet 
        self.dynamics_str_obj_map = {} #dict: 'dyn_str' -> DynamicsObj
        # each uav in UAVs can have unique dynamics
        # 2D:
        #   1. point_mass
        #   2. ...
        # 3D:
        #   1. 6DOF
        #   2. ...
        # External:
        #   1. PyRVO2
        #   2. ...


    def register_uav_dynamics(self) -> None:
        """Register UAV's dynamics instance and 
        store the dynamics_model in a list/dict"""
        #TODO: spin up dynamics modules that are present in simulation config 
        # use the dynamics_uav_map to create instances of Dynamics
        # and save them in dynamics_str_obj_map
        # use VALID_DYNAMICS here 



    def step(self,actions_dict):
        """Update all UAV states using their dynamics"""
        # action_dict: uav_id -> action
        # dynamics_map: dynamics_model -> uav_id  
        # UAV state does not hold information about dynamics_model 
        for uav_id,action in actions_dict.items(): 
            #! temporary 
            if action is None:
                # Default: maintain current state
                # action = [0.0, 0.0]
                raise RuntimeError('Action cannot be None')
            
            dynamics_model = self.dynamics_str_obj_map[uav_id]
            dynamics_model.step(action, self.uav_dict[uav_id])
        


#### FUTURE task - start ####
class Solver(ABC):
    def __init__(self,):
        self.name = 'GenericSolver'

class FirstOrderEuler(Solver):
    def __init__(self):
        super().__init__()
        self.name = 'FirstOrderEuler'
    

class RK45(Solver):
    def __init__(self):
        super().__init__()
        self.name = 'RK45'

#### FUTURE task - end   ####        