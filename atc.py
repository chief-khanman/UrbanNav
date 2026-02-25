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
from component_schema import UAVBlueprint, UAVTypeConfig


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
                 seed: int): 
        

        # Single seed sourced from config passed by caller
        self.seed: int = seed
        random.seed(self.seed)
        np.random.seed(self.seed)
        self.airspace:Airspace = airspace
        self.airspace_mid_point_coord = self.airspace.location_utm_gdf.centroid
        self.uav_dict:Dict[int, UAV|UAV_template] = {}

        # dict(): dynamics/controller/planner/sensor_name(str) -> uav_id(int)
        self.dynamics_map:Dict[str, List[int]] = {}
        self.controller_map:Dict[str, List[int]] = {}
        self.planner_map:Dict[str, List[int]] = {}
        self.sensor_map:Dict[str, List[int]] = {}

        self.uav_id_index = 0
        
        
   
    def get_state(self,):
        '''ATC state is current uav_list in simulation'''
        return self.uav_dict

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
        for uav in self.uav_dict.values():
            if hasattr(uav, 'mission_complete') and uav.current_mission_complete_status:
                # Mark for future reassignment
                pass

        return None

    def _set_uav(self, uav:UAV_template) -> None:
        """
        Adds a UAV to the UAV list.

        Args:
            uav (UAV_v2_template): The UAV to add.
        
        Returns:
            None
        """
        
        # set uav id 
        uav.id_ = self.uav_id_index
        # set mapping: uav_id to uav 
        self.uav_dict[uav.id_] = uav
        # set mapping sensor/planner/controller/dynamics -> uav_id 
        dynamics_name = getattr(uav, 'dynamics_name', None)
        if dynamics_name is not None:
            self.dynamics_map.setdefault(dynamics_name, []).append(uav.id_)

        controller_name = getattr(uav, 'controller_name', None)
        if controller_name is not None:
            self.controller_map.setdefault(controller_name, []).append(uav.id_)

        sensor_name = getattr(uav, 'sensor_name', None)
        if sensor_name is not None:
            self.sensor_map.setdefault(sensor_name, []).append(uav.id_)

        planner_name = getattr(uav, 'planner_name', None)
        if planner_name is not None:
            self.planner_map.setdefault(planner_name, []).append(uav.id_)

        
        print('Current number of UAVs in system: ', len(self.uav_dict.values()))
        ## INCREMENT INDEX
        self.uav_id_index += 1
  
    def get_uav_dict(self,):
        return self.uav_dict
    
    def remove_uavs_by_id(self, ids_to_remove:List[int]):
        """
        Removes UAV objects from the list based on their id attribute.
        Used when a collision is detected between two UAVs.

        Args:
            ids_to_remove (set): A set of IDs to remove from the list.

        Returns:
            None
        """
        print(f'Current UAV list: {self.uav_dict.values()}')
        
        for uav_id in ids_to_remove:
            print('Removed UAV: ')
            self.uav_dict.pop(uav_id) 
        
        time.sleep(1)
        print(f'Updated UAV list: {self.uav_dict.values()}')
        
        return None
    
    def create_uav_from_blueprint(self, uav_blueprint: UAVBlueprint) -> None:
        """
        Creates a UAV instance from a resolved UAVBlueprint and registers it with ATC.

        The blueprint connects UAV_TYPE_REGISTRY (via UAVTypeConfig) with the
        UAVFleetInstanceConfig data to fully initialise the UAV's physical parameters
        and record its assigned dynamics, controller, and sensor for later wiring.

        Args:
            uav_blueprint: One resolved UAVBlueprint produced by build_fleet().

        Returns:
            UAV: The newly created and registered UAV instance.
        """
        type_cfg: UAVTypeConfig = uav_blueprint.type_config

        uav = UAV(
            radius=type_cfg.radius,
            nmac_radius=type_cfg.nmac_radius,
            detection_radius=type_cfg.detection_radius,
            _id=uav_blueprint.uav_id,
        )

        # Apply kinematic limits resolved from UAV_TYPE_REGISTRY via UAVTypeConfig
        uav.type_name = uav_blueprint.type_name
        uav.max_speed = type_cfg.max_speed
        uav.max_acceleration = type_cfg.max_acceleration
        uav.max_heading_change = type_cfg.max_heading_change
        uav.max_velocity = type_cfg.max_velocity

        # Store component assignments for SimulatorManager to wire up
        uav.dynamics_name = uav_blueprint.dynamics_name
        uav.controller_name = uav_blueprint.controller_name
        uav.sensor_name = uav_blueprint.sensor_name
        uav.planner_name = uav_blueprint.planner_name
        
        # set uav 
        self._set_uav(uav)

        return None
    
    def create_uavS_from_blueprint(self, uav_blueprint_list):
        for uav_blueprint in uav_blueprint_list:
            self.create_uav_from_blueprint(uav_blueprint)
        
        return None 
    
    # WORKING:  --- Feb 24, 2026
    # change use of UAV and replace with uav_id 
    def assign_mission_start_end_vertiport(self, uav_id:int, start:Vertiport, end:Vertiport )->None:
        """
        THIS IS THE MAIN FUNCTION THAT NEEDS TO BE CALLED WHEN A NEW MISSION IS ASSIGNED. 
        ALL OTHER FUNCTIONS MUST CALL THIS - 

        Assign start and end vertiports to a UAV, ensuring they're different.
        
        Args:
            uav: The UAV to assign vertiports to
            start: The starting vertiport
            end: The ending vertiport (should be different from start)
        """
        uav = self.uav_dict[uav_id]
        
        # Assign start and end vertiports to the UAV
        uav.assign_start_end(start, end, self.airspace_mid_point_coord) #    
        return None

    #FIX: #### START ####
    # bring the following up to date for use with current env so that we can start making updates to vertiport
    # and sound modeling 

    
    # NEED to run for every UAV at each step - unless i can change this to event driven 
    def has_reached_end_vertiport(self, uav_id:int) -> None:
        """Checks if a UAV has reached its end_vertiport.

        This method checks if a UAV has reached its end_vertiport. If it did reach,
        it calls the landing_procedure method to update relevant objects.

        Args:
            uav (UAV): The UAV object to check.

        Returns:
            None
        """
        uav = self.uav_dict[uav_id]
        
        if (uav.current_position.distance(uav.mission_end_point) <= uav.mission_complete_distance):
            # uav.reached_end_vertiport = True
            # space_avail = self.check_landing_space_vp()
            # if not space_avail:
            if not self.check_landing_space_vp(uav_id):
            #   hold_at_vertiport(self, uav_id)
                self.holding_pattern_at_vertiport(uav_id)   
            else: 
            #   _landing_procedure(self, landing_uav_id)
            #   
                self._landing_procedure(uav_id)

        
        return None
        
    def has_left_start_vertiport(self, uav_id:int) -> None:
        """Checks if a UAV has left its start_vertiport.

        This method checks if a UAV has left its start_vertiport. If it did leave,
        then it calls the clearing_procedure to take care of updating objects.

        Args:
            uav (UAV): The UAV object to check.

        Returns:
            None
        """
        uav = self.uav_dict[uav_id]

        if (uav.current_position.distance(uav.mission_start_point) > 100):
            self._takeoff_procedure(uav_id)

        return None
    

    def _takeoff_procedure(
        self, outgoing_uav_id:int
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
        
        outgoing_uav = self.uav_dict[outgoing_uav_id]
        
        outgoing_uav_id = outgoing_uav.id_
        # remove UAV from vertiport uav list 
        # this will clear a space in vertiport's vertipad and allow new UAV to land
        for uav in outgoing_uav.start_vertiport.uav_list:
            if uav.id == outgoing_uav_id:
                outgoing_uav.start_vertiport.uav_list.remove(uav)

        return None

    def _landing_procedure(self, landing_uav_id: int) -> None:
        """
        Once a landing spot is confirmed at Vertiport,
        performs the landing procedure for a given UAV.
        Args:
            landing_uav (UAV): The UAV that is landing.
        Returns:
            None
        Raises:
            None
        """
        # UAV
        landing_uav = self.uav_dict[landing_uav_id]
        #
        # Add UAV to Vertiport 
        landing_vertiport = landing_uav.end_vertiport
        landing_vertiport.uav_list.append(landing_uav)
        
        #! this method is to zero out velocity and other attributes and keep the UAV stable at current vertiport
        # function name refresh_uav needs to be changed to something more informative and intuitive 
        # landing_uav.refresh_uav() 
        new_mission = random.random() > 0.5 
        if new_mission: 
            end_vertiport = random.choice(self.airspace.vertiport_list)
            start_vertiport = landing_vertiport
            self.assign_mission_start_end_vertiport(landing_uav.id_, start_vertiport, end_vertiport)
        else:
            self.wait_at_vertiport(landing_uav.id_)

        return None

    #### MISSION CONTROL ####
    
    #! FIX implementation 
    def get_vp_new_mission(self,) -> Vertiport: 
        vp = self.airspace.get_random_vertiport_from_region(1)[0]
        return vp

    def _update_end_vertiport_of_uav(self, uav_id:int, vertiport:Vertiport) -> None:
        """Reassigns the end vertiport of a UAV.

        This method samples a vertiport from the ATC vertiport list.
        If the sampled vertiport is the same as the UAV's current start_vertiport, it resamples until a different vertiport is obtained.
        The sampled end_vertiport is then assigned as the UAV's end_vertiport.
        Finally, the UAV's end_point is updated.

        Args:
            uav (UAV): The UAV object for which the end vertiport needs to be reassigned.
        """
        uav = self.uav_dict[uav_id]
        if not vertiport:
            sample_end_vertiport = self.get_vp_new_mission()
            while sample_end_vertiport.location == uav.start_vertiport.location:
                sample_end_vertiport = self.get_vp_new_mission()
            uav.end_vertiport = sample_end_vertiport
            uav.update_end_point()
        else: 
            uav.end_vertiport = vertiport

        uav.update_end_point()
        return None

    def _update_start_vertiport_of_uav(
        self, uav_id:int, vertiport: Vertiport,
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
        uav = self.uav_dict[uav_id]
        
        if not vertiport:
            uav.start_vertiport = uav.end_vertiport
        
        else: 
            uav.start_vertiport = vertiport
        
        
        uav.update_start_point()
    

        return None
    
    def reassign_new_mission(self, uav_id:int):
        # for now use a threshold to either assign new mission or wait at vertiport




        uav = self.uav_dict[uav_id]
        
        start_vertiport = uav.end_vertiport
        self._update_end_vertiport_of_uav(uav_id, start_vertiport)

        # selecting a random vertiport - needs correct logic for selecting new vertiport
        end_vertiport = random.choice(self.airspace.vertiport_list)
        self._update_start_vertiport_of_uav(uav_id, end_vertiport)
        return None

    def wait_at_vertiport(self,uav_id):
        uav = self.uav_dict[uav_id]
        uav.start_vertiport, uav.end_vertiport = uav.end_vertiport, uav.end_vertiport

        self._update_start_vertiport_of_uav(uav_id, uav.start_vertiport)
        self._update_end_vertiport_of_uav(uav_id, uav.end_vertiport)
        return None

    #### MISSION CONTROL ####
    
    #FIX: ####  END  ####
    

    def check_landing_space_vp(self, incoming_uav_id:int):
        '''Returns True if there is landing space at end vertiport of UAV'''

        uav = self.uav_dict[incoming_uav_id]
        landing_vp = uav.end_vertiport
        if len(landing_vp.uav_list) < landing_vp.landing_takeoff_capacity:
            return True
        else:
            return False 

    def holding_pattern_at_vertiport(self, uav_id):
        uav = self.uav_dict[uav_id]
        uav.current_speed = 0
        uav.current_vel = (0,0)
        uav.current_position = uav.current_position
        print(f'UAV {uav_id}holding at: {uav.current_position}')
        #TODO: turn off sensors during hold - to avoid detection/nmac/collision
        raise PendingDeprecationWarning('This holding pattern will change')

    def assign_vertiports(self, assignment_type: str='random') -> None:
            """
            Assign start and end vertiports to UAVs.
            
            Args:
                assignment_type: Strategy for assigning vertiports ('random', 'optimal', etc.)
            """
            
            print(f'Number of vertiports in space: {len(self.airspace.vertiport_list)}')
            print(f'Number of UAVs in space: {len(self.uav_dict.values())}')
            
            if len(self.airspace.vertiport_list) < 2:
                raise ValueError("Need at least 2 vertiports to assign start and end points")
            
            self.assignment_type = assignment_type
            
            if assignment_type == 'random':
                # Each UAV gets a random pair of distinct vertiports
                available_starts = self.airspace.vertiport_list.copy()
                available_ends = self.airspace.vertiport_list.copy()
                
                for uav in self.uav_dict.values():
                    # Select random start vertiport
                    start_vertiport = random.choice(available_starts)
                    available_starts.remove(start_vertiport)
                    
                    # Create a temporary list excluding the start vertiport
                    temp_ends = [v for v in available_ends if v != start_vertiport]
                    
                    # If no valid end vertiports, reuse one
                    if not temp_ends:
                        temp_ends = [v for v in self.airspace.vertiport_list if v != start_vertiport]
                    
                    # Select random end vertiport
                    end_vertiport = random.choice(temp_ends)
                    if end_vertiport in available_ends:
                        available_ends.remove(end_vertiport)
                    
                    # Assign to UAV
                    uav.assign_start_end(start_vertiport, end_vertiport)
                    
                    # Replenish available vertiports if necessary
                    if not available_starts:
                        available_starts = self.airspace.vertiport_list.copy()
                    if not available_ends:
                        available_ends = self.airspace.vertiport_list.copy()
            
            elif assignment_type == 'optimal':
                # Assign vertiports to minimize total distance or conflicts
                # This is a more complex assignment strategy
                pass
            
            else:
                # Default to random assignment
                raise RuntimeError('Unknown assignment string')
            
            return None
    
       # WORKING:  --- Feb 24, 2026

    #! Order functions that need to run every step 
    # for uav_id in self.uav_dict.keys():
    #   has_left_start_vertiport(uav_id)
    #  
    #   has_reached_end_vertiport(uav_id)
    #       if reached_vp 
    #           check_vp_space(uav_id)
    #               if not vp_space:
    #                   hold_vp(uav_id)
    #               else:
    #                   _landing_procedure

    # inside step of simulator_manager 
    # for vp in self.airspace.vp_list:
    #   if take_off_queue():
    #       for uav in take_off_que:
    #           self.reassign_new_mission() 