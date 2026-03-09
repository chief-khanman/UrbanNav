import numpy as np
from controller_template import Controller


class PIDPointMassController(Controller):
    def __init__(self, dt):
        super().__init__(dt=dt)
        # Gains for Speed (Proportional only for simplicity)
        self.Kp_speed = 0.5
        # Gains for Heading (PD Controller to prevent overshooting turns)
        self.Kp_yaw = 2.0
        self.Kd_yaw = 0.1
        self.prev_yaw_error = 0

    def get_control_action(self, uav, target_pos):
        dx = target_pos.x - uav.px
        dy = target_pos.y - uav.py
        distance = np.hypot(dx, dy)
        
        # 1. Speed Control: Slow down as we approach the target
        target_speed = min(uav.max_speed, distance * self.Kp_speed)
        accel_cmd = self.Kp_speed * (target_speed - uav.current_speed)

        # 2. Heading Control: Calculate angle to target
        target_heading = np.arctan2(dy, dx)
        # Normalize error to [-pi, pi] to prevent 360-degree spins
        yaw_error = (target_heading - uav.current_heading + np.pi) % (2 * np.pi) - np.pi
        
        # PD Law for yaw rate
        yaw_rate_cmd = (self.Kp_yaw * yaw_error) + (self.Kd_yaw * (yaw_error - self.prev_yaw_error) / self.dt) #! need to pass dt from sim_manager 
        self.prev_yaw_error = yaw_error

        return accel_cmd, yaw_rate_cmd
    
    def set_control_action(self,):
        pass

    def reset(self,):
        pass