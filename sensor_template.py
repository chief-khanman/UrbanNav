from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Any


class Sensor(ABC):

    '''
    Collect other UAVs state information
    '''
    #FIX: 
    # set_data needs to be updated, currently we are only focused on dynamic obj, ie other_UAVs,
    # a sensor will sense any object, dynamic or static. 
    # proposed design - 
    # if intersection with object
    #   sensor will add label
    #       if sensor_label == UAV:
    #           add UAV to otherUAV_data_list
    #       else if sensor_label == restricted_airspace:
    #           add building to restricted_airspace_list
    #       else : 
    #           raise ERROR - this program at the moment should have these two objects only
    # 
    # 
    # The above structure is flexible for accepting different kinds of objects, using a label-string
    # and adding the object to a list for variable named label-string 



    @abstractmethod
    def __init__(self)->None:
        return None



    def get_sensor_data(self, uav_id:str, sensor_active_status:bool=True) -> Dict[str, List]: #Tuple[List, List]
        '''Combination of uav detection(List of Dict) and ra detection(List of Dict)'''
        
        if sensor_active_status:
            detect_uav_id_list = self.get_uav_detection(uav_id)
            detect_ra_id_list = self.get_ra_detection(uav_id)
            nmac_uav_id_list = self.get_nmac(uav_id)
            collision_uav_id_list = self.get_uav_collision(uav_id)
            collision_ra_id_list = self.get_ra_collision(uav_id)
        else: 
            detect_uav_id_list = []
            detect_ra_id_list = []
            nmac_uav_id_list = []
            collision_uav_id_list = []
            collision_ra_id_list = []
        
        sensor_data_packet = {
                            'detect_uav': detect_uav_id_list,
                            'detect_ra': detect_ra_id_list,
                            'detect_nmac': nmac_uav_id_list,
                            'collision_uav': collision_uav_id_list,
                            'collision_ra': collision_ra_id_list
                            }
        
        return sensor_data_packet



    # UAV     
    @abstractmethod
    def get_uav_detection(self, uav_id:str, *args) -> List[str]:
        # RUN broad phase collision detection 
        pass 
   
    @abstractmethod
    def get_nmac(self, uav_id:str) -> List[str]: #Tuple[bool, List]:
        '''Collect the time step and UAVs with who there was an NMAC'''
        pass
    
    @abstractmethod
    def get_uav_collision(self, uav_id:str)-> List[str] #Tuple[bool, Tuple|Any]:
        '''returns a bool if there is a collision along with UAV id'''
        pass


    # Restricted Airspace (ra)
    def get_ra_detection(self, uav_id:str)-> List[str]: #Tuple[bool, Dict]:
        pass

    def get_ra_collision(self, uav_id:str)-> List[str]: #Tuple[bool, Dict]:
        # RUN narrow phase 
        pass


    


