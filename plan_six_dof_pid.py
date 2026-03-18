import math
from typing import List
import numpy as np
from shapely import Point
from plan_template import PlannerTemplate


class SixDOFPIDPlanner(PlannerTemplate):
    """7th-order minimum-snap trajectory planner for SixDOF (3D) UAVs.

    Works with:
      - dynamics_six_dof.py  (SixDOF): updates UAV state using (ax, ay, az, yaw_rate)
      - controller_cascade_PID_six_dof.py (CascadedPIDSixDOFController): computes the
        4-component action from (uav, target_pos) where target_pos is the Point returned
        by get_plan().

    Trajectory generation (Mellinger & Kumar minimum-snap, single segment):
      For each axis i in {x, y, z}:
        p_i(t) = sum_{k=0}^{7} c_k^i * t^k

      Boundary conditions (hover-to-hover: v0=a0=j0=0, vT=aT=jT=0):
        c0 = p0,  c1 = c2 = c3 = 0  (determined directly from start conditions)
        c4 =  35*Δp / T^4
        c5 = -84*Δp / T^5
        c6 =  70*Δp / T^6
        c7 = -20*Δp / T^7
      where Δp = p_goal - p_start.

    Trajectory time T is chosen so that peak velocity ≤ max_speed:
        T = max(1.0,  1.875 * dist_3d / max_speed)

    At each get_plan() call the polynomial is evaluated at the current elapsed time
    to produce a 3D reference position (Shapely Point) for the cascaded PID controller.

    The `dt` attribute must be set by the caller after instantiation (planner_engine.py
    injects self.dt via `if hasattr(instance, 'dt'): instance.dt = self.dt`).
    """

    # Max speed fraction used for trajectory time calculation.
    # Peak velocity of the minimum-snap profile = 1.875 * Δp / T.
    # Setting T = _PEAK_VEL_FACTOR * dist / max_speed guarantees peak ≤ max_speed.
    _PEAK_VEL_FACTOR: float = 1.875

    def __init__(self, waypoints: List[Point]):
        # Nominal dt — overwritten by planner_engine after instantiation.
        self.dt: float = 0.1

        # Polynomial coefficients per axis: [c0, c1, ..., c7]
        self._coeffs_x: np.ndarray = np.zeros(8)
        self._coeffs_y: np.ndarray = np.zeros(8)
        self._coeffs_z: np.ndarray = np.zeros(8)

        # Total trajectory duration (seconds).
        self._T: float = 1.0

        # Elapsed simulation time since trajectory start.
        self._t_elapsed: float = 0.0

        # Initialise waypoints via the property setter so coefficients are computed.
        self._waypoints: List[Point] = []
        self.waypoints = waypoints   # triggers _compute_trajectory()

    # ------------------------------------------------------------------
    # Waypoints property — recomputes trajectory on reassignment
    # ------------------------------------------------------------------

    @property
    def waypoints(self) -> List[Point]:
        return self._waypoints

    @waypoints.setter
    def waypoints(self, new_waypoints: List[Point]) -> None:
        """Reassign waypoints and recompute the minimum-snap trajectory.

        Called at init and whenever planner_engine detects a mission change.
        Resets elapsed time so the new trajectory starts fresh.
        """
        self._waypoints = new_waypoints
        self._t_elapsed = 0.0
        self.current_idx = 0   # kept for compatibility with planner_engine reset logic
        self._compute_trajectory()

    # ------------------------------------------------------------------
    # Trajectory computation
    # ------------------------------------------------------------------

    def _compute_trajectory(self) -> None:
        """Compute 7th-order minimum-snap polynomial coefficients for each axis.

        Uses closed-form solution for the hover-to-hover single-segment case:
            c0 = p0,  c1=c2=c3=0
            c4 =  35*Δp / T^4
            c5 = -84*Δp / T^5
            c6 =  70*Δp / T^6
            c7 = -20*Δp / T^7
        """
        if len(self._waypoints) < 2:
            # Not enough waypoints yet — leave coefficients zeroed.
            return

        start = self._waypoints[0]
        end   = self._waypoints[-1]

        p0x = start.x
        p0y = start.y
        p0z = start.z if start.has_z else 0.0

        pTx = end.x
        pTy = end.y
        pTz = end.z if end.has_z else 0.0

        dist_3d = math.sqrt((pTx - p0x)**2 + (pTy - p0y)**2 + (pTz - p0z)**2)

        # Estimate a reasonable max_speed from UAVTypeConfig defaults.
        # If we ever have access to the UAV object here we could use uav.max_speed,
        # but planners are initialised with only waypoints.  Use a conservative 10 m/s.
        _max_speed_estimate: float = 10.0
        self._T = max(1.0, self._PEAK_VEL_FACTOR * dist_3d / _max_speed_estimate)

        T = self._T

        def _coeffs_for_axis(p0: float, pT: float) -> np.ndarray:
            delta = pT - p0
            c = np.zeros(8)
            c[0] = p0
            # c1=c2=c3=0 (zero initial velocity, acceleration, jerk)
            c[4] =  35.0 * delta / (T**4)
            c[5] = -84.0 * delta / (T**5)
            c[6] =  70.0 * delta / (T**6)
            c[7] = -20.0 * delta / (T**7)
            return c

        self._coeffs_x = _coeffs_for_axis(p0x, pTx)
        self._coeffs_y = _coeffs_for_axis(p0y, pTy)
        self._coeffs_z = _coeffs_for_axis(p0z, pTz)

    # ------------------------------------------------------------------
    # Polynomial evaluation
    # ------------------------------------------------------------------

    @staticmethod
    def _eval_poly(coeffs: np.ndarray, t: float) -> float:
        """Evaluate p(t) = sum_{k=0}^{7} c_k * t^k using Horner's method."""
        result = 0.0
        for k in range(7, -1, -1):
            result = result * t + coeffs[k]
        return result

    # ------------------------------------------------------------------
    # PlannerTemplate interface
    # ------------------------------------------------------------------

    @property
    def is_mission_complete(self) -> bool:
        """True once the trajectory duration has elapsed."""
        return self._t_elapsed >= self._T

    def get_plan(self, current_pos: Point) -> List[Point]:
        """Evaluate the minimum-snap trajectory at the current elapsed time.

        Advances the internal clock by dt each call.  Once the trajectory is
        complete the final waypoint (hover position) is returned so the
        controller can hold position.

        Args:
            current_pos: UAV's current_position (shapely.Point) — not used for
                         polynomial evaluation but kept for interface compatibility.

        Returns:
            List[Point] with one element — the current 3D reference position.
        """
        t_eval = min(self._t_elapsed, self._T)

        rx = self._eval_poly(self._coeffs_x, t_eval)
        ry = self._eval_poly(self._coeffs_y, t_eval)
        rz = self._eval_poly(self._coeffs_z, t_eval)

        self._t_elapsed += self.dt

        return [Point(rx, ry, rz)]
