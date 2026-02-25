
from uav_template import UAV_template

class UAV(UAV_template):
    """Standard UAV with controller"""

    def __init__(self,
                 radius,
                 nmac_radius,
                 detection_radius,
                 _id,
                 ):
        super().__init__(radius, nmac_radius, detection_radius)

    def assign_start_end(self, start, end, *args, **kwargs):
        return super().assign_start_end(start, end, *args, **kwargs)

    def get_mission_status(self):
        return super().get_mission_status()

    def set_mission_complete_status(self, mission_complete_status):
        return super().set_mission_complete_status(mission_complete_status)

    def get_state(self):
        return super().get_state()

    # def get_sensor_data(self):
    #     return super().get_sensor_data()

    # def get_obs(self):
    #     return super().get_obs()

