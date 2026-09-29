import math
from typing import Dict, List, Optional, Tuple

from urbannav.controller_template import Controller
from urbannav.rvo2_python import RVOSimulator, Vector2


class ORCAController(Controller):
    """Optimal Reciprocal Collision Avoidance (ORCA) velocity controller.

    Papers:
      - J. van den Berg, S. J. Guy, M. Lin, D. Manocha, "Reciprocal n-Body Collision
        Avoidance", ISRR 2009 / Springer STAR vol. 70, 2011.
        https://doi.org/10.1007/978-3-642-19457-3_1
        (project page: https://gamma.cs.unc.edu/ORCA/)
      - J. van den Berg, M. Lin, D. Manocha, "Reciprocal Velocity Obstacles for
        Real-Time Multi-Agent Navigation", ICRA 2008.
        https://doi.org/10.1109/ROBOT.2008.4543489
    Implementation: the vendored pure-Python RVO2 port in urbannav.rvo2_python
    (https://github.com/chengji253/RVO2-python).

    Each step, for every ORCA UAV, a preferred velocity is taken straight toward the
    planner's target waypoint, then ORCA returns the velocity closest to it that is
    collision-free (for time_horizon seconds) with respect to nearby UAVs, assuming
    each neighbor takes half the responsibility for avoiding the collision.

    Action: (vx, vy) world-frame velocity [m/s], consumed by dynamics_orca.ORCADynamics.

    Neighbors are read from the live ATC uav_dict (injected by AerBus via bind_fleet()).
    Every in-flight UAV is a neighbor regardless of its own controller. Non-ORCA UAVs
    do not reciprocate, so against them the half-responsibility assumption is
    optimistic. This is the standard limitation of mixing ORCA with other agents.

    Works with:
      - dynamics_orca.py (ORCADynamics): applies (vx, vy) as a single integrator.
      - plan_orca.py (ORCAPlanner): supplies the current target waypoint.
    """

    # ORCA tuning parameters (UAV physical limits come from UAV_TYPE_REGISTRY).
    time_horizon: float = 10.0        # s — look-ahead for UAV-UAV avoidance
    time_horizon_obst: float = 10.0   # s — look-ahead for static obstacles (none added yet)
    max_neighbors: int = 10
    # m — added to every agent's radius inside ORCA. ORCA keeps separation >= r_i + r_j,
    # while PartialSensor flags a collision at <= r_i + r_j, so without a margin a
    # grazing pass lands exactly on the collision threshold.
    radius_margin: float = 1.0
    # rad — v_pref is rotated by this angle whenever neighbors are present. This breaks
    # the perfectly symmetric head-on deadlock, in which both agents just slow down
    # along the line. The RVO2 examples add a tiny random perturbation for the
    # same reason; a fixed rotation keeps runs reproducible (both agents veer left).
    symmetry_break_angle: float = 1e-3

    def __init__(self, dt) -> None:
        super().__init__(dt=dt)
        self.uav_dict: Optional[Dict[int, object]] = None

    def bind_fleet(self, uav_dict: Dict[int, object]) -> None:
        """Give the controller the live {uav_id: UAV} map so it can see neighbors.

        Called once by AerBus.register_uav_controllers(); the dict is ATC's own
        uav_dict, so removals/additions are reflected automatically.
        """
        self.uav_dict = uav_dict

    def get_control_action(self, uav, target_pos) -> Tuple[float, float]:
        """Return the ORCA-safe velocity (vx, vy) for one UAV this step.

        Math: with preferred velocity v_pref, ORCA builds for each neighbor j a
        half-plane ORCA_ij = {v | (v - (v_i + u/2)) . n >= 0}, where u is the
        smallest change to the relative velocity that takes it out of the velocity
        obstacle VO^tau_ij (the truncated cone of relative velocities leading to a
        collision within tau = time_horizon), and n is the outward normal of VO at
        the point closest to the relative velocity. The new velocity is the solution of
        the 2D linear program  argmin_{v in ∩_j ORCA_ij ∩ D(0, v_max)} ||v - v_pref||.
        It is solved incrementally (linearProgram1/2), and linearProgram3 falls back
        to the least-violating velocity when the constraints are infeasible.

        Neighbors are computed against current neighbor states. AerBus gathers all
        actions before DynamicsEngine moves any UAV, so solving each UAV
        independently gives the same result as RVO2's synchronous doStep().

        Args:
            uav:        The ego UAV (current_position, vx, vy, pz, radius, max_speed,
                        detection_radius, uav_in_flight).
            target_pos: Shapely Point — current target waypoint from the planner.

        Returns:
            Tuple (vx, vy) — commanded world-frame velocity [m/s].
        """
        pref_vx, pref_vy = self._preferred_velocity(uav, target_pos)

        if not self._avoidance_active(uav):
            return pref_vx, pref_vy

        neighbors = self._get_neighbors(uav)
        if not neighbors:
            return pref_vx, pref_vy

        sim = RVOSimulator(
            timeStep=self.dt,
            neighborDist=uav.detection_radius,
            maxNeighbors=self.max_neighbors,
            timeHorizon=self.time_horizon,
            timeHorizonObst=self.time_horizon_obst,
            radius=uav.radius + self.radius_margin,
            maxSpeed=uav.max_speed,
        )
        ego_no = sim.addAgent(
            Vector2(float(uav.current_position.x), float(uav.current_position.y)),
            velocity=Vector2(float(uav.vx), float(uav.vy)),
        )
        for other in neighbors:
            sim.addAgent(
                Vector2(float(other.current_position.x), float(other.current_position.y)),
                radius=other.radius + self.radius_margin,
                maxSpeed=other.max_speed,
                velocity=Vector2(float(other.vx), float(other.vy)),
            )
        # rotate v_pref by symmetry_break_angle: [cos -sin; sin cos] @ v_pref
        cos_a, sin_a = math.cos(self.symmetry_break_angle), math.sin(self.symmetry_break_angle)
        sim.setAgentPrefVelocity(ego_no, Vector2(cos_a * pref_vx - sin_a * pref_vy,
                                                 sin_a * pref_vx + cos_a * pref_vy))

        # Solve only the ego agent's linear program (RVOSimulator.doStep() would also
        # solve every neighbor's and integrate positions, neither of which we want).
        sim.kdTree_.buildAgentTree()
        ego = sim.agents_[ego_no]
        ego.computeNeighbors(sim.kdTree_)
        ego.computeNewVelocity(self.dt)
        return ego.newVelocity_.x_, ego.newVelocity_.y_

    def _preferred_velocity(self, uav, target_pos) -> Tuple[float, float]:
        """Velocity straight at the target, at max_speed, or slower to land on it.

        v_pref = d_hat * min(v_max, ||d|| / dt), where d = target - position. Capping
        the speed at ||d||/dt stops the UAV exactly on the target instead of
        overshooting it within a single step.
        """
        dx = target_pos.x - uav.current_position.x
        dy = target_pos.y - uav.current_position.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0
        speed = min(uav.max_speed, dist / self.dt)
        return speed * dx / dist, speed * dy / dist

    @staticmethod
    def _avoidance_active(uav) -> bool:
        """ORCA only runs while airborne and away from vertiports.

        This follows the sensor's shut-off zone (get_sensor_operational()). UAVs
        leaving or approaching a pad fly straight in, as they do for the other controllers.
        """
        return getattr(uav, 'uav_in_flight', True) and uav.get_sensor_operational()

    def _get_neighbors(self, uav) -> List[object]:
        """Return in-flight UAVs that could collide with `uav` given the 3D collision test.

        The sensor declares a collision on 3D distance <= r_i + r_j, and UAVs cruise at
        a fixed altitude (pz). So a UAV whose vertical separation is at least
        r_i + r_j can never collide with the ego and is excluded. Otherwise ORCA,
        which works in 2D, would swerve around traffic at other altitudes. Also
        excluded: grounded UAVs, UAVs outside detection_radius, and exactly
        coincident UAVs (ORCA's geometry is undefined at zero separation).
        """
        if self.uav_dict is None:
            return []
        ego_x, ego_y = uav.current_position.x, uav.current_position.y
        ego_z = getattr(uav, 'pz', 0.0)
        neighbors = []
        for other in self.uav_dict.values():
            if other is uav or not getattr(other, 'uav_in_flight', True):
                continue
            if abs(getattr(other, 'pz', 0.0) - ego_z) >= uav.radius + other.radius:
                continue
            dist = math.hypot(other.current_position.x - ego_x,
                              other.current_position.y - ego_y)
            if 1e-6 < dist <= uav.detection_radius:
                neighbors.append(other)
        return neighbors

    def set_control_action(self) -> None:
        """Not used — actions are returned from get_control_action()."""
        pass

    def reset(self) -> None:
        """ORCA is stateless between steps; nothing to reset."""
        pass
