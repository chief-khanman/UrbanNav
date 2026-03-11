import math
import numpy as np
from shapely import Point
from dynamics_template import Dynamics
from uav import UAV
from uav_template import UAV_template


class HolonomicDynamics(Dynamics):
    """2D holonomic dynamics model.

    A holonomic model imposes no heading constraint — the UAV can accelerate
    freely in any world-frame direction regardless of its current heading.
    State: (x, y, vx, vy).  Control input: (ax, ay) — world-frame acceleration.

    Works with:
      - controller_holonomic.py (HolonomicPIDController): computes (ax, ay)
        from (uav, target_pos) where target_pos is the Point returned by get_plan().
      - plan_holonomic.py (HolonomicPlanner): returns the current target waypoint.
    """

    def __init__(self) -> None:
        super().__init__()

    def update(self, uav_id: str, action) -> None:
        """Not used by DynamicsEngine; step() is the primary interface."""
        return None

    def step(self, action, uav: UAV | UAV_template) -> None:
        """Apply world-frame acceleration (ax, ay) to update the UAV's state.

        Update order:
          1. Clip accelerations to ±max_acceleration.
          2. Integrate velocity: v += a * dt.
          3. Enforce speed cap: if ||v|| > max_speed, scale down uniformly.
          4. Derive current_speed and current_heading from the velocity vector.
          5. Integrate position: pos += v * dt.

        Args:
            action: Tuple (ax, ay) — world-frame accelerations [m/s²].
            uav:    UAV object whose state is updated in-place.
                    Reads: max_acceleration, max_speed, vx, vy, current_position.
                    Writes: vx, vy, current_speed, current_heading, current_position.
        """
        ax, ay = action

        # 1. Clip accelerations to physical limits.
        ax = np.clip(ax, -uav.max_acceleration, uav.max_acceleration)
        ay = np.clip(ay, -uav.max_acceleration, uav.max_acceleration)

        # 2. Integrate velocity.
        uav.vx += ax * self.dt
        uav.vy += ay * self.dt

        # 3. Enforce speed cap — scale both components proportionally.
        speed = math.hypot(uav.vx, uav.vy)
        if speed > uav.max_speed:
            scale = uav.max_speed / speed
            uav.vx *= scale
            uav.vy *= scale
            speed = uav.max_speed

        # 4. Derived scalar state.
        uav.current_speed = speed
        # Heading is undefined at zero speed — keep the previous value.
        if speed > 1e-6:
            uav.current_heading = math.atan2(uav.vy, uav.vx)

        # 5. Integrate position.
        uav.current_position = Point(
            uav.current_position.x + uav.vx * self.dt,
            uav.current_position.y + uav.vy * self.dt,
        )
