from typing import Dict, List 
from sensor_template import Sensor
from component_schema import VALID_SENSORS
from sensor_partial import PartialSensor
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
SENSOR_CLASS_MAP:Dict[str, type] = {'PartialSensor': PartialSensor, 
                                    # implement camera, LiDAR abstractions
                                    }
class SensorEngine:
    def __init__(self, config, sensor_uav_map, uav_dict):
            self.config = config
            #
            self.sensor_uav_map = sensor_uav_map #mapping: sensor_name -> [uav_id, ...]
            self.uav_dict = uav_dict # mapping: uav_id-> UAV

            self.sensor_obj_map = {}     
        
            #TODO: should there be another attr for mapping UAV_id to sensor object,
            #  to store sensor data, only if sensors are stateful
            # BUT shouldn't be, since I am creating and returning the dicts for each of the methods defined below  


    def register_uav_sensors(self, *args, **kwargs):
         pass

    def get_detection_restricted_area(self, *args, **kwargs) -> Dict[int, List]:
        '''return a dict keyed by uav_id, 
        each value will contain a list of 
        detected RA for respective UAV/UAV_id or 
        empty list '''
        
        for uav_id in self.uav_dict:
             
            detection_dict = {}
        
        return detection_dict

    def get_detection_other_uavS(self,*args, **kwargs) -> Dict[int, List]:
        '''return a dict keyed by uav_id, 
        each value will contain a list of 
        detected UAVs for respective UAV/UAV_id or 
        empty list '''
        
        detection_dict = {}
        return detection_dict


    def get_nmac(self,*args, **kwargs) -> Dict[int, List]:
        '''return a dict keyed by uav_id, 
        each value will contain a list of 
        nmac UAVs for respective UAV/UAV_id or 
        empty list '''
        
        nmac_dict = {}
        return nmac_dict



    def get_collision_restricted_area(self,*args, **kwargs) -> Dict[int, List]:
        '''return a dict keyed by uav_id, 
        each value will contain a list of 
        collided RA - typically the list should only contain one RA for a collision
        for respective UAV/UAV_id or empty list '''
        
        collision_dict = {}
        return collision_dict


    def get_collision_uavS(self,*args, **kwargs) -> Dict[int, List]:
        '''return a dict keyed by uav_id, 
        each value will contain a list of 
        collided UAVs - typically the list should only contain one UAV for a collision
        for respective UAV/UAV_id or empty list '''
        
        collision_dict = {}
        return collision_dict