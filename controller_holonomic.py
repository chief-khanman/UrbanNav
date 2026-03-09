import numpy as np
from controller_template import Controller


class HolonomicPIDController(Controller):
    """PD controller for 2D holonomic UAVs.

    Computes world-frame acceleration commands (ax, ay) to drive a holonomic
    UAV toward a target position.  Because a holonomic model has no heading
    constraint, heading and yaw-rate are irrelevant — control lives entirely
    in the (x, y) velocity and acceleration space.

    Control law:
      1. Speed control  — desired speed proportional to distance, clamped to
                          max_speed (mirrors PIDPointMassController.Kp_speed).
      2. Velocity control — desired velocity vector points toward target at
                           the desired speed.
      3. Acceleration PD — PD law on velocity error drives vx/vy to desired,
                           with a derivative term to dampen overshoot.

    Works with:
      - dynamics_holonomic.py (HolonomicDynamics): updates state using (ax, ay).
      - plan_holonomic.py (HolonomicPlanner): supplies the current target waypoint.
    """

    def __init__(self, dt) -> None:
        super().__init__(dt=dt)
        # Proportional gain: maps distance to a desired-speed magnitude.
        self.Kp_speed: float = 0.5
        # Proportional gain on velocity error → acceleration command.
        self.Kp_accel: float = 2.0
        # Derivative gain on velocity error to dampen oscillations.
        self.Kd_accel: float = 0.1
        # Previous velocity errors for the derivative term.
        self._prev_err_vx: float = 0.0
        self._prev_err_vy: float = 0.0

    def get_control_action(self, uav, target_pos) -> tuple[float, float]:
        """Compute world-frame acceleration (ax, ay) to steer toward target_pos.

        Args:
            uav:        UAV object with attributes current_position (.x, .y),
                        vx, vy, max_speed, dt.
            target_pos: Shapely Point — current target waypoint from the planner.

        Returns:
            Tuple (ax, ay) — world-frame acceleration commands [m/s²].
        """
        dx = target_pos.x - uav.current_position.x
        dy = target_pos.y - uav.current_position.y
        distance = np.hypot(dx, dy)

        # 1. Desired speed — slow down proportionally as we approach the target.
        desired_speed = min(uav.max_speed, distance * self.Kp_speed)

        # 2. Desired velocity vector — directed toward target at desired_speed.
        if distance > 1e-6:
            desired_vx = desired_speed * (dx / distance)
            desired_vy = desired_speed * (dy / distance)
        else:
            desired_vx = 0.0
            desired_vy = 0.0

        # 3. Velocity errors.
        err_vx = desired_vx - uav.vx
        err_vy = desired_vy - uav.vy

        # 4. PD acceleration commands.
        ax = self.Kp_accel * err_vx + self.Kd_accel * (err_vx - self._prev_err_vx) / self.dt
        ay = self.Kp_accel * err_vy + self.Kd_accel * (err_vy - self._prev_err_vy) / self.dt

        self._prev_err_vx = err_vx
        self._prev_err_vy = err_vy

        return ax, ay

    def set_control_action(self) -> None:
        pass

    def reset(self) -> None:
        """Reset derivative state between episodes."""
        self._prev_err_vx = 0.0
        self._prev_err_vy = 0.0
