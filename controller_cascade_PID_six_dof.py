import numpy as np
from controller_template import Controller


class CascadedPIDSixDOFController(Controller):
    """Cascaded PID controller for SixDOF (3D) UAV dynamics.

    Works with:
      - dynamics_six_dof.py (SixDOF): consumes the 4-component action produced here.
      - plan_six_dof_pid.py (SixDOFPIDPlanner): target_pos is a 3D Shapely Point
        from the minimum-snap trajectory evaluated at the current timestep.

    Two-loop cascaded structure:

      Outer loop — position → desired velocity (P, 3D):
        v_des = Kp_pos * pos_error_3d,  magnitude clamped to uav.max_speed

      Inner loop — velocity error → acceleration command (PD, 3D):
        err_v   = v_des - v_actual
        a_cmd   = Kp_vel * err_v  +  Kd_vel * d(err_v)/dt

    Independent yaw loop (PD on heading error):
        yaw_rate_cmd = Kp_yaw * yaw_err  +  Kd_yaw * d(yaw_err)/dt

    Output: (ax, ay, az, yaw_rate_cmd)  — 4-tuple consumed by SixDOF.step()
    """

    def __init__(self, dt: float):
        super().__init__(dt=dt)

        # Outer loop: position error → desired speed
        self.Kp_pos: float = 0.5
        # Inner loop: velocity error → acceleration
        self.Kp_vel: float = 2.0
        self.Kd_vel: float = 0.1
        # Yaw loop
        self.Kp_yaw: float = 2.0
        self.Kd_yaw: float = 0.1

        # Previous velocity errors (for derivative term in inner loop)
        self._prev_err_vx: float = 0.0
        self._prev_err_vy: float = 0.0
        self._prev_err_vz: float = 0.0
        # Previous yaw error (for derivative term in yaw loop)
        self._prev_yaw_err: float = 0.0

    def get_control_action(self, uav, target_pos) -> tuple:
        """Compute (ax, ay, az, yaw_rate_cmd) to drive the UAV toward target_pos.

        Args:
            uav:        UAV object with state attributes (current_position, vx, vy, vz,
                        current_heading, max_speed, max_acceleration).
            target_pos: Shapely Point — 3D reference from SixDOFPIDPlanner.get_plan()[0].
                        z component accessed safely via getattr (2D Points have no .z).

        Returns:
            Tuple (ax, ay, az, yaw_rate_cmd) — 3 world-frame accelerations + yaw rate.
        """
        # Position errors (3D)
        dx = target_pos.x - uav.current_position.x
        dy = target_pos.y - uav.current_position.y
        dz = getattr(target_pos, 'z', 0.0) - uav.pz

        dist_3d = np.sqrt(dx*dx + dy*dy + dz*dz)

        # --- Outer loop: position → desired velocity ---
        des_speed = min(uav.max_speed, dist_3d * self.Kp_pos)
        if dist_3d > 1e-6:
            scale = des_speed / dist_3d
            des_vx = dx * scale
            des_vy = dy * scale
            des_vz = dz * scale
        else:
            des_vx = des_vy = des_vz = 0.0

        # --- Inner loop: velocity error → acceleration ---
        err_vx = des_vx - uav.vx
        err_vy = des_vy - uav.vy
        err_vz = des_vz - uav.vz

        ax = self.Kp_vel * err_vx + self.Kd_vel * (err_vx - self._prev_err_vx) / self.dt
        ay = self.Kp_vel * err_vy + self.Kd_vel * (err_vy - self._prev_err_vy) / self.dt
        az = self.Kp_vel * err_vz + self.Kd_vel * (err_vz - self._prev_err_vz) / self.dt

        self._prev_err_vx = err_vx
        self._prev_err_vy = err_vy
        self._prev_err_vz = err_vz

        # --- Yaw loop: PD on heading error ---
        target_heading = np.arctan2(dy, dx)
        yaw_err = (target_heading - uav.current_heading + np.pi) % (2 * np.pi) - np.pi
        yaw_rate_cmd = (self.Kp_yaw * yaw_err
                        + self.Kd_yaw * (yaw_err - self._prev_yaw_err) / self.dt)
        self._prev_yaw_err = yaw_err

        return ax, ay, az, yaw_rate_cmd

    def set_control_action(self):
        pass

    def reset(self) -> None:
        """Reset controller state between episodes."""
        self._prev_err_vx = 0.0
        self._prev_err_vy = 0.0
        self._prev_err_vz = 0.0
        self._prev_yaw_err = 0.0
