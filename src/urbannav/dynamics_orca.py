import math

from shapely import Point

from urbannav.dynamics_template import Dynamics
from urbannav.uav import UAV
from urbannav.uav_template import UAV_template


class ORCADynamics(Dynamics):
    """2D velocity-controlled (single-integrator) dynamics for ORCA agents.

    Mirrors RVO2's ``Agent.update()``: the commanded velocity is adopted
    directly (no acceleration limit) and position is integrated one step.
    ORCA's collision-free guarantee assumes exactly this model, so max_acceleration
    is intentionally not applied; only max_speed is enforced as a safety clamp
    (ORCAController already returns velocities within max_speed).

    State: (x, y, vx, vy). Control input: (vx, vy) — world-frame velocity [m/s].
    Altitude (pz) is left unchanged, as in PointMass.

    Works with:
      - controller_orca.py (ORCAController): computes (vx, vy) via ORCA.
      - plan_orca.py (ORCAPlanner): supplies the current target waypoint.
    """

    def __init__(self) -> None:
        super().__init__()

    def update(self, uav_id: str, action) -> None:
        """Not used by DynamicsEngine; step() is the primary interface."""
        return None

    def step(self, action, uav: UAV | UAV_template) -> None:
        """Adopt the commanded velocity (vx, vy) and integrate position by dt.

        Args:
            action: Tuple (vx, vy) — world-frame velocity command [m/s].
            uav:    UAV updated in-place. Writes vx, vy, current_speed,
                    current_heading, current_position, px, py.
        """
        vx, vy = action
        speed = math.hypot(vx, vy)
        if speed > uav.max_speed:
            scale = uav.max_speed / speed
            vx, vy = vx * scale, vy * scale
            speed = uav.max_speed

        uav.vx = vx
        uav.vy = vy
        uav.current_speed = speed
        # Heading is undefined at zero speed — keep the previous value.
        if speed > 1e-6:
            uav.current_heading = math.atan2(vy, vx)

        uav.current_position = Point(
            uav.current_position.x + vx * self.dt,
            uav.current_position.y + vy * self.dt,
        )
        # px/py feed the sensor's spatial hash and the renderer.
        uav.px = uav.current_position.x
        uav.py = uav.current_position.y
