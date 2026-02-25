from typing import Dict 
from sensor_template import Sensor

# sensors will use collision detector -
# collision detector has two phases - 
#   broad phase and narrow phase 
#   
#   narrow phase will be performed three times 
#       1. detection 
#       2. nmac
#       3. collision
#       if detection only then check for nmac
#           if nmac only then check for collision  
# return format - 
#    for each UAV create a dict mapping: uav_id -> [detected uav id ]
#    for each UAV create a dict mapping: uav_id -> [nmac uav id ] 
#    for each UAV create a dict mapping: uav_id -> [collision uav id ]

class SensorEngine:
    def __init__(self, config, sensor_uav_map, uav_dict):
            self.config = config
            self.sensor_uav_map = sensor_uav_map
            self.uav_dict = uav_dict
            
            #! what is plan_dict doing here 
            self.plan_dict:Dict[str, Sensor]
        


    def get_detection_restricted_area(self, *args, **kwargs) -> Dict:
        ''''''
        detection_dict = {}
        return detection_dict

    def get_detection_other_uavS(self,*args, **kwargs) -> Dict:
        detection_dict = {}
        return detection_dict


    def get_nmac(self,*args, **kwargs) -> Dict:
        nmac_dict = {}
        return nmac_dict



    def get_collision_restricted_area(self,*args, **kwargs) -> Dict:
        collision_dict = {}
        return collision_dict


    def get_collision_uavS(self,*args, **kwargs) -> Dict:
        collision_dict = {}
        return collision_dict