from abc import ABC
from typing import Dict, Any, List
from component_schema import UAMConfig
import uav_component_registry as uav_component_registry
# in the config file - 
# dynamics mode and solver will be defined for use with DynamicsEngine()
#  
class DynamicsEngine:
    def __init__(self, config:UAMConfig):
        self.config = config
        self.dt = self.config.simulator.dt
        self.dynamics_map: Dict[str, Any] = {}  # Maps UAV id to dynamics instance
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


    def register_uav_dynamics(self, uav) -> None:
        """Register UAV's dynamics instance and 
        store the dynamics_model in a list/dict"""
        
        if hasattr(uav, 'dynamics') and uav.dynamics is not None:
            self.dynamics_map[uav.id] = uav.dynamics
        else:
            raise ValueError(f"UAV {uav.id} missing dynamics instance")

    # def register_all_uavs(self, uav_list) -> None:
    #     """Register all UAVs' dynamics"""
    #     for uav in uav_list:
    #         self.register_uav_dynamics(uav)

    # def step(self, current_state:SimulatorState, actions:Dict[str, Any], dt:float = None) -> None:
    #     """Update all UAV states using their dynamics"""
    #     dt = dt or self.dt

    #     for uav in current_state.uavs:
    #         if uav.id not in self.dynamics_map:
    #             raise ValueError(f"No dynamics registered for UAV {uav.id}")

    #         dynamics = self.dynamics_map[uav.id]
    #         action = actions.get(uav.id)

    #         if action is None:
    #             # Default: maintain current state
    #             action = [0.0, 0.0]

    #         # Update UAV state (mutates in place)
    #         dynamics.update(uav, action)

    def step(self, uav_state, action:Dict[str, Any], dt:float) -> :
        """Update all UAV states using their dynamics"""
        dt = dt or self.dt

        #UAV state does not hold information about dynamics_model 
        dynamics = self.dynamics_map[uav.id]
        
        action = actions.get(uav.id)

        if action is None:
            # Default: maintain current state
            action = [0.0, 0.0]

        # Update UAV state (mutates in place)
        dynamics.update(uav, action)



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