from abc import ABC, abstractmethod
from typing import List, Dict, Tuple
import math
import numpy as np
from shapely import Point
from vertiport import Vertiport

class UAV_template(ABC):
    """
    Abstract Base Class (ABC) for a UAV (Unmanned Aerial Vehicle) template.

    This class defines the structure and behavior expected for UAV implementations. 
    It includes attributes and methods for assigning missions, retrieving states, 
    sensing the environment, and controlling UAV actions.

    Attributes:
        radius (float): Radius of the UAV's physical body.
        nmac_radius (float): Radius for Near Mid-Air Collision (NMAC) detection.
        detection_radius (float): Radius for detecting other UAVs.
    """
    
    def __init__(self, radius, nmac_radius, detection_radius):
        """
        Initialize the UAV with its controller, dynamics, sensor, and parameters.

        Args:
            radius (float): Radius of the UAV's physical body.
            nmac_radius (float): NMAC detection radius.
            detection_radius (float): Radius for detecting other UAVs.
        """
        # ID 
        # self.int_id = id(self) #id can be thought of as tail number -  need to convert/add new id that starts at 0 
        # self.id:str = id_ # this is the id that will be used when creating UAV and has strings like uav_0, uav_1 etc
        self.id_ : int
        # UAV type 
        self.type_name:str
        
        self.sensor_name:str
        self.planner_name:str
        self.controller_name:str
        self.dynamics_name:str
        
        # UAV sensor range
        self.radius = radius
        self.nmac_radius = nmac_radius
        self.detection_radius = detection_radius
        
        # UAV physics limit - WILL BE EXTRACTED FROM CONFIG - REMOVE DEFAULT VALUE 
        self.max_speed:float = 80
        self.max_velocity:float
        self.max_acceleration:float = 1 # Passed to DynamicsPointMass for action renormalization
        self.max_heading_change:float = math.pi # Passed to DynamicsPointMass for action renormalization
        
        # UAV incidence counter/metric
        self.nmac_count:int = 0

        # VERTIPORT DATA
        self.start_vertiport: Vertiport
        self.end_vertiport: Vertiport

        # POSITION PLAN
        self.mission_start_point:Point
        self.mission_end_point:Point
        self.plan_dict:Dict[str,List[Point]] # when UAV is initiated fill the waypoint with end, at each time step collect index 0 item and set as current_end_point, if current_end_point == end_point, run that new method which will reset attrs of UAV and keep the UAV in vertiport que attr
        
        self.current_position:Point
        self.next_position:Point
        
        # VELOCITY PLAN
        self.current_vel:Tuple
        self.next_vel:Tuple
        self.velocity_plan:Dict[str, List[Tuple]]
        
        # UAV kinematics state 
        self.current_speed:float = 0
        self.current_heading:float = 0
        
        # UAV global vector state - for rendering position of UAV 
        self.px:float 
        self.py:float 
        self.pz:float         

        # NED is simply the modified current location 
        # modified_current_location = current location - map_centeroid
        # NED frame helps simplify the calculations - thats all 
        self.n:float 
        self.e:float 
        self.d:float 

        self.vx:float 
        self.vy:float 
        self.vz:float 
        
        self.pitch:float
        self.roll:float
        self.yaw:float 

        self.pitch_dot:float
        self.roll_dot:float
        self.yaw_dot:float
        


        # UAV mission completion epsilon distance
        self.mission_complete_distance = 40 # Increased from 10 to 40 to account for UAV overshotting goal between updates
        self.current_mission_complete_status: bool
        
        # UAV operational_status
        # mission_plan, path_plan,trajectory_plan can be added before new_mission/during uav_in_flight 
        # once UAV reaches end point - many attrs of UAV are changes to OFF/False 
        # have a new method update attrs once UAV reaches end, 
        self.operational:bool 
       
        # UAV state - at vertiport or in flight 
        self.uav_in_flight:bool #




        
        


    @abstractmethod
    def assign_start_end(self, start: Vertiport, end: Vertiport, *args, **kwargs):
        # Since this method is setting many attrs, 
        # instead of regular set_*(), we are calling it assign_*

        """
        This method is setting a lot of attributes which 
        are dependent on start and end point of UAV. 
        

        Args:
            start (Point): Starting position of the UAV.
            end (Point): Target position of the UAV.
        """
        # TODO: 
        # collect start-end information from vertiport location
        # adjust the arguments to accept two vertiports
        # from each vertiport collect their location, and assign that as start/end point  
        
        # WORKING:  --- Feb 24, 2026
        # VERTIPORT DATA
        self.start_vertiport: Vertiport = start
        self.end_vertiport: Vertiport = end

        self.mission_start_point:Point = self.start_vertiport.location
        self.mission_end_point:Point = self.end_vertiport.location
        
        self.current_position:Point = self.mission_start_point
        
        # POSITION PLAN
        #! NOT set yet - requires PLAN
        self.plan_dict:Dict[str,List[Point]] # when UAV is initiated fill the waypoint with end, at each time step collect index 0 item and set as current_end_point, if current_end_point == end_point, run that new method which will reset attrs of UAV and keep the UAV in vertiport que attr
        #! NOT set yet - requires PLAN
        self.next_position:Point
        # VELOCITY PLAN
        #! Not set yet - requires PLAN
        self.current_vel:Tuple 
        self.next_vel:Tuple
        self.velocity_plan:Dict[str, List[Tuple]]
        
        
        # UAV kinematics state 
        self.current_speed:float = 0
        # TODO: change current heading at start to a random direction. let controller change current heading over time
        # performing this change will have impact on ORCA agent visualization
        # TODO: fix arrow visualization for ORCA agents - the arrow of ORCA agents should get updated as current heading is changes  
        #self.current_heading = math.atan2((end.y - start.y), (end.x - start.x))
        self.current_heading = np.random.uniform(-math.pi, math.pi)
        
        # UAV global vector state - for rendering position of UAV 
        self.px:float = self.current_position.x
        self.py:float = self.current_position.y
        self.pz:float = self.current_position.z         

        # NED is simply the modified current location 
        # modified_current_location = current location - map_centeroid
        # NED frame helps simplify the calculations - thats all 
        self.n:float = self.current_position.x # subtract airspace mid-point coord
        self.e:float = self.current_position.y 
        self.d:float = self.current_position.z 

        self.vx:float = 0.0
        self.vy:float = 0.0
        self.vz:float = 0.0
        
        self.pitch:float = 0.0
        self.roll:float = 0.0
        self.yaw:float = self.current_heading

        self.pitch_dot:float = 0.0
        self.roll_dot:float = 0.0
        self.yaw_dot:float = 0.0
        # WORKING:  --- Feb 24, 2026
        self.current_mission_complete_status = False
        # UAV operational_status
        # mission_plan, path_plan,trajectory_plan can be added before new_mission/during uav_in_flight 
        # once UAV reaches end point - many attrs of UAV are changes to OFF/False 
        # have a new method update attrs once UAV reaches end, 
        self.operational:bool = True
       
        # UAV state - at vertiport or in flight 
        self.uav_in_flight:bool = True
        
        # ADDED odometer attr:
        self.previous_position = start.location
        
        # artifact from UAM_v2
        self.final_heading = math.atan2((end.y - self.current_position.y), (end.x - self.current_position.x))
        
        return None

    @abstractmethod
    def get_mission_status(self) -> bool:
        """
        Check whether the mission is complete.

        Returns:
            bool: True if the UAV is within the mission completion distance from the target, False otherwise.
        """
        if self.current_position.distance(self.mission_start_point) <= self.mission_complete_distance:
            self.current_mission_complete_status = True
        else:
            self.current_mission_complete_status = False
        return self.current_mission_complete_status

    @abstractmethod
    def set_mission_complete_status(self, mission_complete_status: bool) -> None:
        """
        Set the mission completion status.

        Args:
            mission_complete_status (bool): True if the mission is complete, False otherwise.
        """
        self.current_mission_complete_status = mission_complete_status
        return None

    @abstractmethod
    def get_state(self) -> Dict:
        """
        Retrieve the current state of the UAV.

        Returns:
            Dict: A dictionary containing state information such as distance to goal, current speed, heading, and radius.
        """
        ref_prll, ref_orth = self.get_ref()
        return {'id':self.id_,
                'current_position':self.current_position, #global position
                'distance_to_end': self.mission_start_point.distance(self.mission_start_point), #mission_distance
                'distance_covered': self.mission_start_point.distance(self.current_position), #TODO: wrong def 
                'distance_to_goal': self.current_position.distance(self.mission_start_point),
                'max_dist': self.mission_start_point.distance(self.mission_start_point),#mission_distance
                'min_dist': 0,
                'min_speed': 0,
                'max_speed': self.max_speed, 
                'current_speed': self.current_speed,
                'current_heading': self.current_heading, # map_north - body_frame_north
                'final_heading': self.final_heading, # goal_deviation = goal_vector - body_frame_north
                'end':self.mission_start_point,
                'radius': self.radius,
                'ref_prll':ref_prll,
                'ref_orth':ref_orth,
                'detection_radius': self.detection_radius
                }


    def get_ref(self):
        """
        Calculate and return the reference directions (parallel and orthogonal) 
        for the UAV relative to its goal.

        Returns:
            Tuple[np.ndarray, np.ndarray]: Parallel and orthogonal reference directions.
        """
        goal_dir = np.array([self.mission_start_point.x - self.current_position.x, 
                             self.mission_start_point.y - self.current_position.y])
        
        self.dist_to_goal = self.current_position.distance(self.mission_start_point)

        if self.dist_to_goal > 1e-8:
            ref_prll = goal_dir / self.dist_to_goal
        else:
            ref_prll = goal_dir
        
        ref_orth = np.array([-ref_prll[1], ref_prll[0]])

        return ref_prll, ref_orth
    
    def reset_odometer(self) -> None:
        self.odometer_reading = 0
        return None
    
    def update_odometer(self) -> None:
        distance = self.current_position.distance(self.previous_position)
        self.odometer_reading += distance
        self.previous_position = self.current_position

        def update_start_point(self,):
            pass

    def update_start_point(self,):
        pass

    def update_end_point(self,):
        pass

    def refresh_uav(self,):
        pass

    
    # @abstractmethod
    # def get_sensor_data(self):
    #     """
    #     Collect data from the sensor about other UAVs and restricted airspace in the environment.

    #     Returns:
    #         Tuple[List, List]: Sensor data about other UAVs, and restricted area data.
    #     """
    #     pass

    # def get_obs(self):  
    #     """
    #     Retrieve the observation, combining the UAV's state with sensor data.

    #     Returns:
    #         Tuple[Dict, Tuple[List, List]]
    #         Tuple containing Dict which contains own state informati 
    #         and another Tuple that has two lists, one for other_uavs, and another for restricted airspaces
    #     """
    
    # # get self information
    # own_data = self.get_state()
    # # add self obs with other_uav obs
    # sensor_data = self.get_sensor_data()
    
    # pass # TODO: named tuple would be better for sensor data.