from typing import Dict, List
from shapely import Point
from plan_template import PlannerTemplate
from typing import Dict, List
from uav_template import UAV_template
from uav import UAV
from component_schema import UAMConfig, VALID_PLANNERS
from plan_point_mass_pid import PointMassPIDPlanner


# Maps VALID_PLANNERS string names → PlannerTemplate subclasses.
# Only types with a concrete implementation are listed here.
# To add a new planner type: add it to VALID_PLANNERS in component_schema.py,
# implement the class, import it above, and add its entry to this map.
PLANNER_CLASS_MAP: Dict[str, type] = {
    'PointMass-PID': PointMassPIDPlanner,
    # 'PointMass-RL': PointMassRLPlanner,   # TODO: add once implemented
    # 'SixDOF-PID':   SixDOFPIDPlanner,     # TODO: add once implemented
}


class PlannerEngine:

    def __init__(self,
                 config: UAMConfig,
                 plan_uav_map: Dict[str, List[int]],
                 uav_dict: Dict[int, UAV | UAV_template]):

        self.config = config
        self.dt = self.config.simulator.dt

        # comes from atc.planner_map — dict: plan_name(str) -> [uav_id(int), ...]
        self.plan_uav_map = plan_uav_map
        # comes from atc.uav_dict — dict: uav_id(int) -> UAV object
        self.uav_dict = uav_dict

        # populated by register_uav_planners()
        # dict: uav_id(int) -> PlannerTemplate instance
        # Unlike DynamicsEngine (one instance per type), each UAV gets its own
        # planner instance because planners are stateful — they track each UAV's
        # waypoint progress independently.
        self.plan_str_obj_map: Dict[int, PlannerTemplate] = {}

        self.plan_dict: Dict[int, List[Point]] = {}

    def register_uav_planners(self) -> None:
        """Spin up one Planner instance per UAV and map every UAV id to its planner.

        Unlike DynamicsEngine where a single Dynamics instance is shared across all
        UAVs of the same type (dynamics are stateless — state lives in the UAV object),
        planners are stateful: each planner tracks which waypoint its UAV is currently
        targeting via current_idx. Therefore each UAV requires its own dedicated
        planner instance.

        Uses plan_uav_map (dict: plan_name -> [uav_id, ...]) populated by ATC when
        UAVs are created. Each planner is initialised with waypoints derived from the
        UAV's assigned start and end vertiports.

        Populates plan_str_obj_map: { uav_id(int) -> PlannerTemplate instance }
        which is keyed by uav_id so get_plans() can resolve a model in O(1).
        """
        for plan_name, uav_id_list in self.plan_uav_map.items():
            if plan_name not in VALID_PLANNERS:
                raise ValueError(
                    f"Unknown planner type '{plan_name}'. "
                    f"Valid options: {sorted(VALID_PLANNERS)}"
                )
            if plan_name not in PLANNER_CLASS_MAP:
                raise NotImplementedError(
                    f"Planner type '{plan_name}' is valid but has no concrete "
                    f"implementation yet. Implemented types: {sorted(PLANNER_CLASS_MAP)}"
                )
            # Each UAV gets its own planner instance (stateful — tracks current_idx).
            for uav_id in uav_id_list:
                uav = self.uav_dict[uav_id]
                waypoints = [uav.start_vertiport.location, uav.end_vertiport.location]
                instance = PLANNER_CLASS_MAP[plan_name](waypoints=waypoints)
                self.plan_str_obj_map[uav_id] = instance

    #TODO: get_plans() will need to be updated - return should not be List[Points], return should be List[Tuple[Point|float]]
    #TODO:  because get_plans will include not just position, it can also include, pitch,roll, yaw, vx,vy,vz, and ddot_pitch, ddot_roll, ddot_yaw
    def get_plans(self) -> Dict[int, List[Point]]:
        """Retrieve the current target waypoint for every UAV with a registered planner.

        Each planner's get_plan() is called with the UAV's current_position so that
        waypoint-advancement logic runs correctly at every step.

        Returns:
            plan_dict: { uav_id(int) -> List[Point] } current plan for each UAV.
        """
        for uav_id, plan_model in self.plan_str_obj_map.items():
            uav = self.uav_dict[uav_id]
            self.plan_dict[uav_id] = plan_model.get_plan(uav.current_position)
        return self.plan_dict

    def set_plans(self, *args, **kwargs):
        pass
