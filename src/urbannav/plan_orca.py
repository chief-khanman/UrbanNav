from urbannav.plan_holonomic import HolonomicPlanner


class ORCAPlanner(HolonomicPlanner):
    """Waypoint-following planner for ORCA UAVs.

    ORCA only needs a target point to derive its preferred velocity, which is the
    same contract HolonomicPlanner already implements (sequential waypoints,
    returned as a one-element List[Point]). This subclass exists so ORCA fleets are
    configured as the usual dynamics/controller/planner trio.

    Works with:
      - dynamics_orca.py (ORCADynamics)
      - controller_orca.py (ORCAController): turns plan[0] into v_pref, then runs ORCA.
    """
