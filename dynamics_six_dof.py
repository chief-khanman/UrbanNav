import math
import numpy as np
from shapely import Point
from dynamics_template import Dynamics
from uav import UAV
from uav_template import UAV_template


class SixDOF(Dynamics):
    """Simplified planar 6-DOF dynamics for a multirotor UAV.

    Models translation in x, y, z plus roll/pitch/yaw attitudes.
    Designed for the 3D simulator mode.

    State (stored in UAV object):
        position:    px, py, pz          — world-frame position (metres)
        velocity:    vx, vy, vz          — world-frame velocity (m/s)
        attitude:    roll, pitch, yaw    — Euler angles (rad)
        angular rate: roll_dot, pitch_dot, yaw_dot

    Control input (from CascadedPIDSixDOFController):
        (ax, ay, az, yaw_rate_cmd)
        ax, ay, az      — world-frame acceleration commands (m/s²)
        yaw_rate_cmd    — desired yaw rate (rad/s)

    Physical model:
        - ax, ay, az produce translational acceleration (Euler integration)
        - 3D speed capped at uav.max_speed
        - roll  = -ay / g  (bank angle coupling, small-angle linearisation)
        - pitch =  ax / g  (pitch angle coupling, small-angle linearisation)
        - yaw driven by yaw_rate_cmd
        - current_position is kept 2D (x, y) for compatibility with the collision
          detection and sensor systems; pz is stored separately for 3D controllers.
    """

    GRAVITY: float = 9.81          # m/s²
    MAX_TILT: float = math.pi / 6  # 30° max roll/pitch

    def __init__(self):
        super().__init__()

    def step(self, action, uav: UAV | UAV_template) -> None:
        ax_cmd, ay_cmd, az_cmd, yaw_rate_cmd = action

        # Clip acceleration commands to UAV limits
        ax = np.clip(ax_cmd, -uav.max_acceleration, uav.max_acceleration)
        ay = np.clip(ay_cmd, -uav.max_acceleration, uav.max_acceleration)
        az = np.clip(az_cmd, -uav.max_acceleration, uav.max_acceleration)

        # Integrate velocity
        uav.vx += ax * self.dt
        uav.vy += ay * self.dt
        uav.vz += az * self.dt

        # Enforce 3D speed cap
        speed_3d = math.sqrt(uav.vx**2 + uav.vy**2 + uav.vz**2)
        if speed_3d > uav.max_speed:
            scale = uav.max_speed / speed_3d
            uav.vx *= scale
            uav.vy *= scale
            uav.vz *= scale
            speed_3d = uav.max_speed

        # Horizontal speed (used for 2D rendering and collision detection)
        speed_xy = math.hypot(uav.vx, uav.vy)
        uav.current_speed = speed_xy

        # Heading derived from horizontal velocity; yaw driven by commanded rate
        uav.yaw += yaw_rate_cmd * self.dt
        uav.yaw = (uav.yaw + math.pi) % (2 * math.pi) - math.pi
        uav.yaw_dot = yaw_rate_cmd
        # Sync current_heading (used by controller and renderer)
        uav.current_heading = uav.yaw

        # Attitude coupling: roll and pitch reflect lateral accelerations
        # (small-angle linearisation of multirotor attitude-to-force coupling)
        uav.roll  = np.clip(-ay / self.GRAVITY, -self.MAX_TILT, self.MAX_TILT)
        uav.pitch = np.clip( ax / self.GRAVITY, -self.MAX_TILT, self.MAX_TILT)
        uav.roll_dot  = 0.0   # simplified: instantaneous attitude response
        uav.pitch_dot = 0.0

        # Integrate position
        new_x = uav.current_position.x + uav.vx * self.dt
        new_y = uav.current_position.y + uav.vy * self.dt
        uav.pz += uav.vz * self.dt

        # current_position kept as 2D Point for collision system compatibility
        uav.current_position = Point(new_x, new_y)
        uav.px = new_x
        uav.py = new_y

    def update(self, uav_id: str, action) -> None:
        super().update(uav_id, action)
