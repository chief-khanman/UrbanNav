from typing import List
from shapely import Point
from plan_template import PlannerTemplate


class HolonomicPlanner(PlannerTemplate):
    """Waypoint-following planner for 2D holonomic UAVs.

    Works with:
      - dynamics_holonomic.py (HolonomicDynamics): updates UAV state using (ax, ay).
      - controller_holonomic.py (HolonomicPIDController): computes (ax, ay)
        from (uav, target_pos) where target_pos is the Point returned by get_plan().

    The planner advances through waypoints sequentially. When the UAV comes within
    `threshold` metres of the current target it advances to the next waypoint.
    Once all waypoints are visited, is_mission_complete becomes True and the planner
    continues to return the final waypoint so the controller can hold position.

    The planner is dynamics-agnostic; it tracks waypoint progress and returns a target 
    Point. The holonomic controller consumes that Point directly in (x, y) space without
    any heading-related logic.
    """

    def __init__(self, waypoints: List[Point]) -> None:
        self.waypoints = waypoints
        self.current_idx = 0
        # Distance (metres) to consider a waypoint 'reached'.
        # Matches uav_template.mission_complete_distance order-of-magnitude.
        self.threshold: float = 2.0

    @property
    def is_mission_complete(self) -> bool:
        """True once the UAV has advanced past the final waypoint."""
        return self.current_idx >= len(self.waypoints)

    def get_plan(self, current_pos: Point) -> List[Point]:
        """Return the current target waypoint list given the UAV's current position.

        Advances current_idx when the UAV is within threshold of the current target.
        Always returns a single-element list containing the immediate target Point so
        the controller can access it as plan[0] (a Shapely Point with .x / .y).

        Args:
            current_pos: UAV's current_position (shapely.Point).

        Returns:
            List[Point] with one element — the current target waypoint.
        """
        # Mission already complete — hold at final waypoint.
        if self.is_mission_complete:
            return [self.waypoints[-1]]

        target = self.waypoints[self.current_idx]
        dist = target.distance(current_pos)

        if dist < self.threshold:
            self.current_idx += 1

        # Clamp in case the increment pushed us past the end.
        next_idx = min(self.current_idx, len(self.waypoints) - 1)
        return [self.waypoints[next_idx]]
