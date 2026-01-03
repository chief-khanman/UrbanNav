
from uav_v2_template import UAV_v2_template

class UAV_v2(UAV_v2_template):

    def __init__(self, controller, dynamics,sensor,radius, nmac_radius,detection_radius):
        super().__init__(controller, dynamics,sensor,radius, nmac_radius,detection_radius)

    def assign_start_end(self, start, end):
        return super().assign_start_end(start, end)
    
    def get_mission_status(self):
        return super().get_mission_status()
    
    def set_mission_complete_status(self, mission_complete_status):
        return super().set_mission_complete_status(mission_complete_status)
    
    def get_state(self):
        return super().get_state()
    
    def get_sensor_data(self):
        return super().get_sensor_data()
    
    def get_obs(self):
        return super().get_obs()

    def get_action(self, observation):
        action = self.controller(observation)
        return action

    