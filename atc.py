import time
import math
from copy import copy
import random
import numpy as np
from typing import List, Tuple, Type, Dict
from shapely import Point
from shapely.geometry import Polygon
from airspace import Airspace
from uav_template import UAV_template
from uav import UAV
from vertiport import Vertiport


class ATC():
    """
    Manages UAVs and their interactions with Vertiports in Airspace
    
    Args:
        airspace: Airspace object containing geographical data and restricted areas
        max_uavs: Maximum number of UAVs allowed in the space
        seed: Random seed for reproducibility
    """
    
    def __init__(self, 
                 airspace: Airspace, 
                 seed: int): #! use seed from CONFIG
        
        #! update seed - use seed from CONFIG
        self.seed:int = seed
        random.seed(self.seed)
        np.random.seed(self.seed)
        self._seed = seed
        
        self.airspace:Airspace = airspace
    
        self.uav_dict:Dict[str, UAV_template] = {}
        #!deprecate this attribute - use new uav_dict 
        self.uav_list:List[UAV_template] = []
        
        
   
    def get_state(self,):
        '''ATC state is current uav_list in simulation'''
        return self.uav_list

    #* Future task - start *#
    def connect_airspace(self, airspace):
        '''Must run this function to connect AIRSPACE with ATC'''
        self.airspace = airspace 
        return None
    #* Future task - end  *#

    
    #TODO: create necessary functions that will handle
    # 1. hold at vertiport OR
    # 2. reassignment for new mission
    # 3. ...   
    def update(self):

        # For now, check mission completion
        for uav in self.uav_list:
            if hasattr(uav, 'mission_complete') and uav.current_mission_complete_status:
                # Mark for future reassignment
                pass

        return None

    def _set_uav(self, uav:UAV_template):
        """
        Adds a UAV to the UAV list.

        Args:
            uav (UAV_v2_template): The UAV to add.
        
        Returns:
            None
        """
        
        self.uav_list.append(uav)
        print('Current number of UAVs in system: ', len(self.get_uav_list()))

    def get_uav_list(self) -> List[UAV_template]:
        """
        Returns the list of UAVs.

        Returns:
            List[UAV_v2_template]: The list of UAVs.
        """
        return self.uav_list
    
    def remove_uavs_by_id(self, ids_to_remove:List[int]):
        """
        Removes UAV objects from the list based on their id attribute.
        Used when a collision is detected between two UAVs.

        Args:
            ids_to_remove (set): A set of IDs to remove from the list.

        Returns:
            None
        """
        print(f'Current UAV list: {self.uav_list}')
        
        self.uav_list = [uav for uav in self.uav_list if uav.id not in ids_to_remove]
        
        time.sleep(1)
        print(f'Updated UAV list: {self.uav_list}')
        
        return None
    
    def create_uav(self,  
                   _id,
                   radius, 
                   nmac_radius, 
                   detection_radius,
                   ) -> None:
        # my space can have a max amount of agents
        # as UAVs are added, the space will keep track of the number of UAVs added 
        """
        Create UAV.

        Args:
            UAV constructor
        
        Returns:    
            None
        """

        uav = UAV(radius, nmac_radius, detection_radius, _id,)
        self._set_uav(uav)
        
        return None
    
    def create_uav_from_blueprint(self, uav_blueprint):
        
        return UAV
    
    def create_uavS_from_blueprint(self, uav_blueprint_list):
        for uav_blueprint in uav_blueprint_list:
            create_uav_from_blueprint(uav_blueprint)
        return None 
    
    def assign_vertiport_uav(self, uav:UAV_template, start:Vertiport, end:Vertiport )->None:
        """
        Assign start and end vertiports to a UAV, ensuring they're different.
        
        Args:
            uav: The UAV to assign vertiports to
            start: The starting vertiport
            end: The ending vertiport (should be different from start)
        """
        # print('Printing: In file atc.py')
        # print(f'end type: {type(end)}')
        # print(f'end: {end}')
        if start.id == end.id or start.location.equals(end.location):
            # Find a different end vertiport
            alternative_vertiports = [v for v in self.airspace.vertiport_list 
                                if v.id != start.id and not v.location.equals(start.location)]
            
            # If there are alternatives, choose one
            if alternative_vertiports:
                end = random.choice(alternative_vertiports)
                print(f"Found same vertiport for start and end, replaced with different end vertiport")
            else:
                print(f"WARNING: No alternative vertiports found, using same start and end")
        
        # Assign start and end vertiports to the UAV
        uav.assign_start_end(start.location, end.location)
        return None

    #FIX: #### START ####
    # bring the following up to date for use with current env so that we can start making updates to vertiport
    # and sound modeling 

    
    # NEED to run for every UAV at each step - unless i can change this to event driven 
    def has_reached_end_vertiport(self, uav: UAV_template) -> None:
        """Checks if a UAV has reached its end_vertiport.

        This method checks if a UAV has reached its end_vertiport. If it did reach,
        it calls the landing_procedure method to update relevant objects.

        Args:
            uav (UAV): The UAV object to check.

        Returns:
            None
        """

        if (uav.current_position.distance(uav.mission_end_point) <= uav.mission_complete_distance):
            # uav.reached_end_vertiport = True
            self._landing_procedure(uav)
        
        return None
        
    def has_left_start_vertiport(self, uav: UAV_template) -> None:
        """Checks if a UAV has left its start_vertiport.

        This method checks if a UAV has left its start_vertiport. If it did leave,
        then it calls the clearing_procedure to take care of updating objects.

        Args:
            uav (UAV): The UAV object to check.

        Returns:
            None
        """
        if (uav.current_position.distance(uav.mission_start_point) > 100):
            self._takeoff_procedure(uav)

        return None
    

    def _takeoff_procedure(
        self, outgoing_uav: UAV_template
        ) -> None: 
        """
        Performs the clearing procedure for a given UAV.
        Args:
            outgoing_uav (UAV): The UAV that is outgoing(leaving the start_vertiport).
        Returns:
            None
        Raises:
            None
        """
        outgoing_uav_id = outgoing_uav.id
        for uav in outgoing_uav.start_vertiport.uav_list:
            if uav.id == outgoing_uav_id:
                outgoing_uav.start_vertiport.uav_list.remove(uav)

        return None

    def _landing_procedure(self, landing_uav: UAV_template) -> None:
        """
        Performs the landing procedure for a given UAV.
        Args:
            landing_uav (UAV): The UAV that is landing.
        Returns:
            None
        Raises:
            None
        """
        landing_vertiport = landing_uav.end_vertiport
        landing_vertiport.uav_list.append(landing_uav)
        
        #! this method is to zero out velocity and other attributes and keep the UAV stable at current vertiport
        # function name refresh_uav needs to be changed to something more informative and intuitive 
        landing_uav.refresh_uav() 
        
        self._update_end_vertiport_of_uav(landing_uav)

        return None

    #### MISSION CONTROL ####
    
    #! FIX implementation 
    def get_vp_new_mission(self,) -> Vertiport: 
        vp = self.airspace.get_random_vertiport_from_region(1)[0]
        return vp

    def _update_end_vertiport_of_uav(self, uav_id) -> None:
        """Reassigns the end vertiport of a UAV.

        This method samples a vertiport from the ATC vertiport list.
        If the sampled vertiport is the same as the UAV's current start_vertiport, it resamples until a different vertiport is obtained.
        The sampled end_vertiport is then assigned as the UAV's end_vertiport.
        Finally, the UAV's end_point is updated.

        Args:
            uav (UAV): The UAV object for which the end vertiport needs to be reassigned.
        """
        uav = self.uav_list[uav_id]
        sample_end_vertiport = self.get_vp_new_mission()
        while sample_end_vertiport.location == uav.start_vertiport.location:
            sample_end_vertiport = self.get_vp_new_mission()
        uav.end_vertiport = sample_end_vertiport
        uav.update_end_point()


        return None

    def _update_start_vertiport_of_uav(
        self, vertiport: Vertiport, uav: UAV_template
    ) -> None:
        """This method accepts a vertiport (end-vertiport of uav)
        and updates the start_vertiport attribute of UAV
        to the provided vertiport. This method works in conjunction with landing_procedure.

        Args:
            vertiport (Vertiport): The vertiport representing the end-vertiport of the UAV.
            uav (UAV): The UAV whose start_vertiport attribute needs to be updated.

        Returns:
            None

        """
        uav.start_vertiport = vertiport
        uav.update_start_point()
    

        return None
    
    def assign_new_mission(self, uav_id):
        self._update_end_vertiport_of_uav(uav_id)
        self._update_start_vertiport_of_uav(uav_id)
        return None

    def wait_at_vertiport(self,):
        self._update_end_vertiport_of_uav()
        return None

    #### MISSION CONTROL ####
    





    #### PLANNER ####
    def uav_mission_planner(self, uav_id, some_planner, *args, **kwargs) -> List[Point]:
        '''Provide mission plan which must include start and end point'''
        # query map to find anchor point between start and end 
        # create mission plan 
        start_point, end_point = self.uav_dict[uav_id].mission_start_point, self.uav_dict[uav_id].mission_end_point
        mission_plan = some_planner(args, kwargs)
        return mission_plan

    def uav_path_planner(self,) -> List[Point]:
        '''Generate finer path plan for UAV using mission plan anchor points'''

        # coarse interpolation between all mission point

        start_point, end_point = Point(0,0), Point(1,1)
        mission_plan = [start_point, end_point]
        return mission_plan

    def uav_trajectory_planner(self,) -> List[Point]:
        '''Plan immediate points for UAV by interpolating between consecutive point from path plan'''
        
        # current path point
        # next path point
        # interpolate points based on dt

        start_point, end_point = Point(0,0), Point(1,1)
        mission_plan = [start_point, end_point]
        return mission_plan

    def set_plan_uav(self,uav_id):

        uav = self.uav_dict[uav_id]
        for plan_name, plan in uav.plan_dict.items():
            if plan_name == 'mission_plan':
                uav.plan_dict[plan_name] = self.uav_mission_planner(uav_id)
            elif plan_name == 'path_plan':
                uav.plan_dict['path_plan'] = self.uav_path_planner()
            elif plan_name == 'trajectory_plan':
                uav.plan_dict[plan_name] = self.uav_trajectory_planner()
            else:
                raise RuntimeError('Unknown plan name')

    
    

    #FIX: ####  END  ####


