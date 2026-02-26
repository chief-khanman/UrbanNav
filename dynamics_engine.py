from abc import ABC
from typing import Dict, Any, List
from uav_template import UAV_template
from uav import UAV
from component_schema import UAMConfig, VALID_DYNAMICS
from dynamics_template import Dynamics
from dynamics_point_mass import PointMass
from dynamics_six_dof import SixDOF
from dynamics_two_d_vector import TwoDVector
# in the config file -
# dynamics mode and solver will be defined for use with DynamicsEngine()

# Maps VALID_DYNAMICS string names → Dynamics subclasses.
# Only types with a concrete implementation are listed here.
# To add a new dynamics type: add it to VALID_DYNAMICS in component_schema.py,
# implement the class, import it above, and add its entry to this map.
DYNAMICS_CLASS_MAP: Dict[str, type] = {
    'PointMass':    PointMass,
    'SixDOF':       SixDOF,
    'TwoDVector':   TwoDVector,
    # 'ORCA': ORCA,  # TODO: add once dynamics_ORCA.py is implemented
}


class DynamicsEngine:
    def __init__(self,
                 config: UAMConfig,
                 dynamics_uav_map: Dict[str, List[int]],
                 uav_dict: Dict[int, UAV | UAV_template]):
        self.config = config
        self.dt = self.config.simulator.dt
        # comes from atc.dynamics_map — dict: dynamics_name(str) -> [uav_id(int), ...]
        self.dynamics_uav_map = dynamics_uav_map
        # comes from atc.uav_dict — dict: uav_id(int) -> UAV object
        self.uav_dict = uav_dict
        # populated by register_uav_dynamics()
        # dict: uav_id(int) -> Dynamics instance
        # Typed as Dynamics (abstract base) so IDE can resolve .step() and other
        # shared interface methods on every subclass without losing autocomplete.
        self.dynamics_str_obj_map: Dict[int, Dynamics] = {}
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
        """Spin up one Dynamics instance per unique type used in this simulation
        and map every UAV id to its corresponding Dynamics instance.

        Uses dynamics_uav_map (dict: dyn_name -> [uav_id, ...]) populated by ATC
        when UAVs are created.  Only dynamics types actually referenced in the
        current config's fleet are instantiated — unused types are never created.

        Each instance is created with the simulator's dt so all subclass step()
        calls use the correct timestep without requiring dt to be passed at
        call time.

        Populates dynamics_str_obj_map: { uav_id(int) -> Dynamics instance }
        which is keyed by uav_id so step() can resolve a model in O(1).
        """
        # Instantiate one Dynamics object per unique type present in this simulation.
        # VALID_DYNAMICS guards against unknown names; DYNAMICS_CLASS_MAP guards
        # against names that are valid but not yet implemented.
        type_to_instance: Dict[str, Dynamics] = {}
        for dyn_name in self.dynamics_uav_map:
            if dyn_name not in VALID_DYNAMICS:
                raise ValueError(
                    f"Unknown dynamics type '{dyn_name}'. "
                    f"Valid options: {sorted(VALID_DYNAMICS)}"
                )
            if dyn_name not in DYNAMICS_CLASS_MAP:
                raise NotImplementedError(
                    f"Dynamics type '{dyn_name}' is valid but has no concrete "
                    f"implementation yet. Implemented types: {sorted(DYNAMICS_CLASS_MAP)}"
                )
            # Construct instance then inject the simulator's dt so subclasses
            # that call super().__init__() with the default (0.1) are corrected.
            instance = DYNAMICS_CLASS_MAP[dyn_name]()
            instance.dt = self.dt
            type_to_instance[dyn_name] = instance

        # Fan out: map every uav_id to its shared Dynamics instance.
        # UAVs sharing the same dynamics type share the same object.
        for dyn_name, uav_id_list in self.dynamics_uav_map.items():
            dyn_obj = type_to_instance[dyn_name]
            for uav_id in uav_id_list:       # uav_id is int (set by atc._set_uav)
                self.dynamics_str_obj_map[uav_id] = dyn_obj


    def step(self, actions_dict):
        """Update all UAV states using their dynamics"""
        # action_dict: uav_id(int) -> action
        # dynamics_str_obj_map: uav_id(int) -> Dynamics instance (already constructed
        #   with correct dt in register_uav_dynamics — retrieve, do not re-instantiate)
        for uav_id, action in actions_dict.items():
            #! temporary
            if action is None:
                # Default: maintain current state
                # action = [0.0, 0.0]
                raise RuntimeError('Action cannot be None')

            # Retrieve the pre-built Dynamics instance for this UAV.
            # dynamics_str_obj_map values are instances (not classes), so no
            # call with dt here — dt was set at registration time.
            dynamics_model: Dynamics = self.dynamics_str_obj_map[uav_id]
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
