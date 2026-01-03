import math
from copy import copy
import random
import numpy as np
from typing import List, Tuple
from space import Space
from shapely import Point
from shapely.geometry import Polygon
from airspace import Airspace
from uav_v2_template import UAV_v2_template
from auto_uav_v2 import Auto_UAV_v2
from vertiport import Vertiport


class ATC():
    """
    Manages UAVs and their interactions with Airspace, and 
    Vertiport module also manages geographical constraints.
    """
    
    def __init__(self, seed: int, airspace: Airspace, max_uavs: int = 40):
        """
        Initialize the MapSpace.
        
        Args:
            max_uavs: Maximum number of UAVs allowed in the space
            max_vertiports: Maximum number of vertiports allowed in the space
            seed: Random seed for reproducibility
            airspace: Airspace object containing geographical data and restricted areas
        """
        random.seed(seed)
        np.random.seed(seed)
        
        self.airspace = airspace
        self.uav_list:List = []
        
        #! ATC's vertiport list has a reference to airspace vertiport list
        self.vertiport_list:List = self.airspace.vertiport_list
        
        self.max_uavs = max_uavs
        self._seed = seed
   
    
    def _set_uav(self, uav:UAV_v2_template):
        """
        Adds a UAV to the UAV list.

        Args:
            uav (UAV_v2_template): The UAV to add.
        
        Returns:
            None
        """
        
        self.uav_list.append(uav)

    
        
    def get_uav_list(self) -> List[UAV_v2_template]:
        """
        Returns the list of UAVs.

        Returns:
            List[UAV_v2_template]: The list of UAVs.
        """
        return self.uav_list
    

    def remove_uavs_by_id(self, ids_to_remove):
        """
        Removes UAV objects from the list based on their id attribute.
        Used when a collision is detected between two UAVs.

        Args:
            ids_to_remove (set): A set of IDs to remove from the list.

        Returns:
            None
        """
        self.uav_list = [uav for uav in self.uav_list if uav.id not in ids_to_remove]

        return None
    

    def create_uav(self, uav_type:UAV_v2_template, controller, dynamics, sensor, radius, nmac_radius, detection_radius) -> None:
        # my space can have a max amount of agents
        # as UAVs are added, the space will keep track of the number of UAVs added 
        """
        Create UAV.

        Args:
            UAV constructor
        
        Returns:
            None
        """

        uav = uav_type(controller, dynamics, sensor, radius, nmac_radius, detection_radius)
        self._set_uav(uav)
        
        
        return None
        
    def create_auto_uav(self, dynamics, sensor, radius, nmac_radius, detection_radius) -> None:
        #FIX: why does AutoUAV not have a controller, this controller should be a (trained)model 
        auto_uav = Auto_UAV_v2(dynamics, sensor, radius, nmac_radius, detection_radius)
        self._set_uav(auto_uav)

    
    def assign_vertiport_uav(self, uav:UAV_v2_template, start:Vertiport, end:Vertiport )->None:
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
            alternative_vertiports = [v for v in self.vertiport_list 
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

    def assign_vertiport_from_lat_long(lat_long:Tuple) -> Vertiport:
        pass
    
    
    
    
    # def has_reached_end_vertiport(self, uav: UAVBasic | AutonomousUAV) -> None:
    #     """Checks if a UAV has reached its end_vertiport.

    #     This method checks if a UAV has reached its end_vertiport. If it did reach,
    #     it calls the landing_procedure method to update relevant objects.

    #     Args:
    #         uav (UAV): The UAV object to check.

    #     Returns:
    #         None
    #     """

    #     if (uav.current_position.distance(uav.end_point) <= uav.landing_proximity) and (
    #         uav.reaching_end_vertiport == False
    #     ):
    #         # uav.reached_end_vertiport = True
    #         self._landing_procedure(uav)
        
    #     return None
        
    # def has_left_start_vertiport(self, uav: UAVBasic | AutonomousUAV) -> None:
    #     """Checks if a UAV has left its start_vertiport.

    #     This method checks if a UAV has left its start_vertiport. If it did leave,
    #     then it calls the clearing_procedure to take care of updating objects.

    #     Args:
    #         uav (UAV): The UAV object to check.

    #     Returns:
    #         None
    #     """
    #     if (uav.current_position.distance(uav.start_point) > 100) and (
    #         uav.leaving_start_vertiport == False
    #     ):
    #         self._clearing_procedure(uav)
    #         uav.leaving_start_vertiport = True

    #     return None
    

    # def _clearing_procedure(
    #     self, outgoing_uav: UAVBasic | AutonomousUAV
    # ) -> None:  #! rename to _takeoff_procedure()
    #     """
    #     Performs the clearing procedure for a given UAV.
    #     Args:
    #         outgoing_uav (UAV): The UAV that is outgoing(leaving the start_vertiport).
    #     Returns:
    #         None
    #     Raises:
    #         None
    #     """
    #     outgoing_uav_id = outgoing_uav.id
    #     for uav in outgoing_uav.start_vertiport.uav_list:
    #         if uav.id == outgoing_uav_id:
    #             outgoing_uav.start_vertiport.uav_list.remove(uav)

    #     return None

    # def _landing_procedure(self, landing_uav: UAVBasic | AutonomousUAV) -> None:
    #     """
    #     Performs the landing procedure for a given UAV.
    #     Args:
    #         landing_uav (UAV): The UAV that is landing.
    #     Returns:
    #         None
    #     Raises:
    #         None
    #     """
    #     landing_vertiport = landing_uav.end_vertiport
    #     landing_vertiport.uav_list.append(landing_uav)
    #     landing_uav.refresh_uav()
    #     self._reassign_end_vertiport_of_uav(landing_uav)

    #     return None
        
    # def _reassign_end_vertiport_of_uav(self, uav: UAVBasic) -> None:
    #     """Reassigns the end vertiport of a UAV.

    #     This method samples a vertiport from the ATC vertiport list.
    #     If the sampled vertiport is the same as the UAV's current start_vertiport, it resamples until a different vertiport is obtained.
    #     The sampled end_vertiport is then assigned as the UAV's end_vertiport.
    #     Finally, the UAV's end_point is updated.

    #     Args:
    #         uav (UAV): The UAV object for which the end vertiport needs to be reassigned.
    #     """
    #     sample_end_vertiport = self.provide_vertiport()
    #     while sample_end_vertiport.location == uav.start_vertiport.location:
    #         sample_end_vertiport = self.provide_vertiport()
    #     uav.end_vertiport = sample_end_vertiport
    #     uav.update_end_point()


    #     return None

    # def _update_start_vertiport_of_uav(
    #     self, vertiport: Vertiport, uav: UAVBasic
    # ) -> None:
    #     """This method accepts a vertiport (end-vertiport of uav)
    #     and updates the start_vertiport attribute of UAV
    #     to the provided vertiport. This method works in conjunction with landing_procedure.

    #     Args:
    #         vertiport (Vertiport): The vertiport representing the end-vertiport of the UAV.
    #         uav (UAV): The UAV whose start_vertiport attribute needs to be updated.

    #     Returns:
    #         None

    #     """
    #     uav.start_vertiport = vertiport
    #     uav.update_start_point()
    

    #     return None
    

    # #FIX: ####  END  ####









    # # def assign_vertiport_agent(self, agent: UAV_v2_template) -> None:
    # #     """
    # #     Assign start and end vertiports to the learning agent.
        
    # #     Args:
    # #         agent: The learning agent (Auto_UAV_v2)
    # #     """
    # #     if len(self.vertiport_list) < 2:
    # #         raise ValueError("Need at least 2 vertiports to assign agent start and end points")
        
    # #     # Find vertiports that aren't used as starting points by other UAVs
    # #     used_starts = set()
    # #     for uav in self.uav_list:
    # #         if hasattr(uav, 'start'):
    # #             used_starts.add(uav.start)
        
    # #     available_starts = [v for v in self.vertiport_list if v not in used_starts]
        
    # #     # If all vertiports are used, select one randomly
    # #     if not available_starts:
    # #         available_starts = self.vertiport_list
        
    # #     # Select start vertiport
    # #     start_vertiport = random.choice(available_starts)
        
    # #     # Select end vertiport (different from start)
    # #     available_ends = [v for v in self.vertiport_list if v != start_vertiport]
    # #     end_vertiport = random.choice(available_ends)
        
    # #     # Assign to agent
    # #     agent.assign_start_end(start_vertiport, end_vertiport)
    # #     print(f"Agent assigned start: ({start_vertiport.x}, {start_vertiport.y}), end: ({end_vertiport.x}, {end_vertiport.y})")
    # #     return None

    

    

    


    


    
    



    
    
    
    
    
    
    # #FIX: update/upgrade this method to work with MAPPED_ENV
    
    # # def assign_vertiports(self, assignment_type: str) -> None:
    # #     """
    # #     Assign start and end vertiports to UAVs.
        
    # #     Args:
    # #         assignment_type: Strategy for assigning vertiports ('random', 'optimal', etc.)
    # #     """
        
    # #     print(f'Number of vertiports in space: {len(self.vertiport_list)}')
    # #     print(f'Number of UAVs in space: {len(self.uav_list)}')
        
    # #     if len(self.vertiport_list) < 2:
    # #         raise ValueError("Need at least 2 vertiports to assign start and end points")
        
    # #     self.assignment_type = assignment_type
        
    # #     if assignment_type == 'random':
    # #         # Each UAV gets a random pair of distinct vertiports
    # #         available_starts = self.vertiport_list.copy()
    # #         available_ends = self.vertiport_list.copy()
            
    # #         for uav in self.uav_list:
    # #             # Select random start vertiport
    # #             start_vertiport = random.choice(available_starts)
    # #             available_starts.remove(start_vertiport)
                
    # #             # Create a temporary list excluding the start vertiport
    # #             temp_ends = [v for v in available_ends if v != start_vertiport]
                
    # #             # If no valid end vertiports, reuse one
    # #             if not temp_ends:
    # #                 temp_ends = [v for v in self.vertiport_list if v != start_vertiport]
                
    # #             # Select random end vertiport
    # #             end_vertiport = random.choice(temp_ends)
    # #             if end_vertiport in available_ends:
    # #                 available_ends.remove(end_vertiport)
                
    # #             # Assign to UAV
    # #             uav.assign_start_end(start_vertiport, end_vertiport)
                
    # #             # Replenish available vertiports if necessary
    # #             if not available_starts:
    # #                 available_starts = self.vertiport_list.copy()
    # #             if not available_ends:
    # #                 available_ends = self.vertiport_list.copy()
        
    # #     elif assignment_type == 'optimal':
    # #         # Assign vertiports to minimize total distance or conflicts
    # #         # This is a more complex assignment strategy
    # #         pass
        
    # #     else:
    # #         # Default to random assignment
    # #         self.assign_vertiports('random')
        
    # #     return None


    # #FIX: this method needs to be removed - I strongly think this method is for Simple_ENV
    # # def assign_vertiports(self, assignment_type:str) -> None:
    # #     """
    # #     For a given space, assign start and end coordinates to UAVs.
        
    # #     Args: 
    # #         assignment_type (str): This string determines how the start-end points are assigned.
    # #                                Options are 'opposite', 'consecutive', 'random'.
                
    # #     Returns:
    # #         None  
    # #     """
    # #     print(f'Number of vertiports in space is {len(self.vertiport_list)}')
    # #     print(f'Number of UAVs in space is {len(self.uav_list)}')
        
    # #     if self.vertiport_pattern == 'circular':
    # #         print(f'Vertiport pattern in space is {self.vertiport_pattern}')
    # #     elif self.vertiport_pattern == 'random':
    # #         print(f'Vertiport pattern in space is {self.vertiport_pattern}')
    # #     else:
    # #         print('No vertiport pattern set')
    # #         raise ValueError('No vertiport pattern set')
        
    # #     self.assignment_type = assignment_type
    # #     coords_list_middle = int(len(self.vertiport_list)/2)
    # #     coords_list_len = int(len(self.vertiport_list))

        
    # #     local_veriport_list = self.vertiport_list.copy()
    # #     local_uav_list = self.uav_list.copy()

    # #     if self.vertiport_pattern == 'circular':
    # #         if self.assignment_type == 'opposite':
    # #             for i in range(len(self.uav_list)):
    # #                 uav = self.uav_list[i]
    # #                 start_idx = i % len(self.vertiport_list)
    # #                 end_idx = (start_idx + coords_list_middle) % coords_list_len
    # #                 start = self.vertiport_list[start_idx]
    # #                 end = self.vertiport_list[end_idx]
    # #                 uav.assign_start_end(start, end)
                
    # #         elif self.assignment_type == 'consecutive':
    # #             pass
    # #         elif self.assignment_type == 'random':
    # #             _start_list = self.vertiport_list.copy()
    # #             _end_list = self.vertiport_list.copy()
    # #             for i in range(len(self.uav_list)):
    # #                 uav = self.uav_list[i]
    # #                 start = random.choice(_start_list)
    # #                 end = random.choice(_end_list)
    # #                 uav.assign_start_end(start, end)
    # #                 _start_list.remove(start)
    # #                 _end_list.remove(end)
                    
    # #         else:
    # #             pass
        
    # #     elif self.vertiport_pattern == 'random':
    # #         random.seed(seed = self._seed)
    # #         _start_list = self.vertiport_list.copy()
    # #         _end_list = self.vertiport_list.copy()

    # #         for i in range(len(self.uav_list)):
    # #             uav = self.uav_list[i]
    # #             start = random.choice(_start_list)
    # #             end = random.choice(_end_list)
    # #             uav.assign_start_end(start, end)
    # #             _start_list.remove(start)
    # #             _end_list.remove(end)


    # #     return None

    #     #FIX: this method is for SIMPLE_ENV 
    # # def assign_vertiport_agent(self, agent: UAV_v2_template) -> None:
    # #     """
    # #     Assigns a start and end vertiport to the agent UAV in a way that avoids
    # #     conflicts with other UAVs.
        
    # #     Args:
    # #         agent (UAV_v2_template): The agent AutoUAV.
        
    # #     Returns:
    # #         None
    # #     """
    # #     coords_list_middle = int(len(self.vertiport_list)/2)
    # #     coords_list_len = int(len(self.vertiport_list))
        
    # #     # Instead of checking UAV start attributes, track assigned vertiports
    # #     assigned_vertiports = set()
    # #     for uav in self.uav_list:
    # #         if hasattr(uav, 'start'):  # Check if start has been assigned
    # #             assigned_vertiports.add(uav.start)
        
    # #     # Find first unassigned vertiport
    # #     for i in range(len(self.vertiport_list)):
    # #         potential_start = self.vertiport_list[i]
    # #         if potential_start not in assigned_vertiports:
    # #             if self.vertiport_pattern == 'circular':
    # #                 # Use the opposite point pattern
    # #                 end_idx = (i + coords_list_middle) % coords_list_len
    # #                 end = self.vertiport_list[end_idx]
    # #             else:  # random pattern
    # #                 # Choose from remaining vertiports
    # #                 remaining = [v for v in self.vertiport_list 
    # #                         if v != potential_start]
    # #                 end = random.choice(remaining)
                    
    # #             agent.assign_start_end(potential_start, end)
    # #             return None
                
    # #     raise RuntimeError("No available vertiports for agent - all are assigned")