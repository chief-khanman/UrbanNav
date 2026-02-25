import datetime
import random
from shapely import Point
from typing import Dict, List, Tuple, Any
from airspace import Airspace
from vertiport import Vertiport
from atc import ATC
from uav import UAV
from component_schema import UAMConfig
from collision_detection import AdaptiveCollisionDetector
from sensor_engine import SensorEngine
from planner_engine import PlannerEngine
from aer_bus import AerBus
from dynamics_engine import DynamicsEngine
from component_schema import UAVCommandBundle, ActionType, SimulatorState, build_fleet


class SimulatorManager:
    '''Primary class that orchestrates and manages assets/data_classes,
      calculation, networking, collision handler for UAM simulator. '''
    
    def __init__(self, config:UAMConfig):
        '''Initialize simulator manager 
            Args: 
                config:
                seed:
                dt: 
            Returns:
                None'''
        
        # UAM config object 
        self.config = config
        # seed for all simulator related task 
        self.seed = self.config.simulator.seed
        # dt 
        self.dt = self.config.simulator.dt 
        
        return None 

    def _init_airspace(self,):
            return Airspace(location_name=self.config.airspace.location_name,
                            number_of_vertiports=self.config.airspace.number_of_vertiports,
                            vertiport_tag_list=self.config.airspace.vertiport_tag_list,
                            airspace_restricted_area_tag_list=self.config.airspace.airspace_restricted_area_tag_list
                            )
    
    def _init_atc(self):
        return ATC(self.airspace, self.seed)

    def _initiate_simulator_assets(self,):

        
        ## AIRSPACE 
        # create airspace
        self.airspace = self._init_airspace()

        ## ATC 
        # create atc
        self.atc = self._init_atc()
        
        

        return None
    
    def _initiate_simulator_components(self,):
        ##### Core components #####
        
        ### sensor ###
        # collision_detector for COLLISION DETECTION/RESOLUTION
        self.sensor_module = SensorEngine(self.config, self.atc.uav_dict)
        
        # use config to send data to statemanager 
        ### Planner ###
        self.planner_module = PlannerEngine(self.config, self.atc.uav_dict)
        
        ### Controller ###
        # AER_BUS handles state export and action import
        # AER_BUS for NETWORKING 
        #! should controller take in plan
        self.controller_module = AerBus(self.config, self.atc.uav_dict, mode='deployment') #! should I pass the CONFIG 
        
        ### Physics/Dynamics ###
        # Dynamics_Engine for PHYSICS CALCULATION 
        # make dynamics engine handle different kind/form of dynamics
        #! should dynamics take in controller_action 
        self.dynamics_module = DynamicsEngine(self.config, self.atc.uav_dict) 
          
    #! NEED to rework this function 
    def _build_assets(self,):
        #### ----------- VP ----------- ####
        # create vertiports
        num_vp = self.config.airspace.number_of_vertiports * self.config.vertiport.number_of_landing_pad
        # change this method to build vertiports in different ways 
        self.airspace.add_n_random_vps_to_vplist(num_vertiports=num_vp)
        
        ## ----> VP_LIST <---- ##
        # ----> self.airspace.vertiport_list <---- 
        
        #TODO: store the created VPs in a database with low memory AND fast query, fetch, and update process - 
        ## VP ID LIST ## 
        #! what is the use of this 
        self.sim_vp_id_list = self.airspace.get_vp_id_list()
        
        #### ----------- UAV ----------- ####
        self.uav_blueprints = build_fleet(self.config) # num_uav comes from total in fleet composition 
        # create uavs
        # use these uav_blueprints and build uavs using uav_template OR method from atc that builds UAVs from UAV blueprints
        self.atc.create_uavS_from_blueprint(self.uav_blueprints) 
        # assign vertiports to UAVs 
        self.atc.assign_vertiports()
        # initiate external systems
        self.external_systems = self.initiate_external_systems()

    def _create_data_class_state(self,):
        self.airspace_state = self.airspace.get_state() #! change list vertiport -> dict[id:int, vp:Vertiport]
        self.atc_state = self.atc.get_state()

    def reset(self, ):
        self.timestamp = datetime.datetime.now().strftime('%m %d %Y -- %H:%M')
        
        self.currentstep = 0
        
        # dyn_engine, aer_bus, path_planner
        self._initiate_simulator_components()
        # airspace, atc, 
        self._initiate_simulator_assets()
        # uav  
        self._build_assets()
        # atc state, airspace state, ...
        self._create_data_class_state()
        
        
        #! why do i need this 
        # state 
        self._state = SimulatorState(self.timestamp,
                                     self.currentstep,
                                     self.airspace_state,  
                                     self.atc_state,
                                     self.external_systems)
        return None
    
    



    def initiate_external_systems(self,):
        return {'No external system':None}    

    def get_state(self,):
        return self._state
    
    def set_state(self, state):
        self._state = state
        return None
   
    def step(self, external_control_actions_dict:UAVCommandBundle)->None:
        '''Update 
        1. timestamp
        2. currentstep
        3. airspace_state
        4. atc_state
        5. external_systems
        '''
        ##### update: current_state.STEP
        self._state.currentstep += 1

        ##### update: current_state.UAVs #####
        ## STEPPER : control_action -> dynamics -> state update          
        restricted_area_detect, uavs_detect, nmac, restricted_area_collision, uavs_collision = self.step_uavS(external_action_dict=external_control_actions_dict)

        #* PROCESS OF REACHING VERTIPROT
        #*            LEAVING VERTIPORT 
        #*            CHECKING SPACE AT VERTIPORT 
        #*            HOLDING PATTERN 
        #*            LANDING 
        #*            TAKEOFF 
        #*            NEW ASSIGNMENT 
        #* all of the above task should be in parallel and asynchronous 
        # check if there is any UAV waiting in landing_queue
        for vertiport in self.airspace.vertiport_list:
            if vertiport.get_landing_queue():
                for uav_id in vertiport.get_landing_queue():
                    self.atc.has_reached_end_vertiport(uav_id) 
            # reassign mission for UAVs sitting at vertiports 
            for uav_id in vertiport.uav_id_list:
                new_mission =  random.random() > 0.5 
                if new_mission:
                    self.atc.reassign_new_mission(uav_id)
                else:
                    self.atc.wait_at_vertiport(uav_id) 

        # handle UAVs that have reached vertiports
        #                       or left vertiports 
        for uav_id in self.atc.uav_dict.keys():
            self.atc.has_left_start_vertiport(uav_id)
            self.atc.has_reached_end_vertiport(uav_id) #! this function shall add the uav to vertiports landing queue

        # update: current_state.EXTERNAL_SYSTEMS
            
        


        
    def map_actions_to_uavs(self, internal_actions_dict, external_ids_actions_dict:UAVCommandBundle) -> Dict:
        '''This function will use internal actions dict and external_ids_actions_dict to form a complete action_dict'''
        

        #TODO:
        # the control_action_dict will have UAVs with both internal_actions, and external_actions 
        # map internal actions to internal uav_ids 
        # map external actions to external uav_ids - like LEARNING  
        # from the CommandBundle extract the CONTROL action 
        #! fix the code below based on logic described above 
        # need to unpack only the CONTROL action and place inside return elegantly 
        # check if the implementation below is correct 
        updated_external_actions_dict = {}
        for uav_id, uav_command_list in external_ids_actions_dict.items():
            for uav_command in uav_command_list:
                if uav_command.action_type == ActionType.CONTROL:
                    updated_external_actions_dict[uav_id] = uav_command.payload

        return {**internal_actions_dict, **updated_external_actions_dict} # since internal is already unpacked, the control actions from external should be a single dict that will be unpacked in return 

    def map_plans_to_uavs(self, internal_plans_dict, external_ids_plans_dict:UAVCommandBundle)->Dict:
        '''This function will use internal actions dict and external_ids_actions_dict to form a complete action_dict'''
        

        #TODO:
        # the control_action_dict will have UAVs with both internal_actions, and external_actions 
        # map internal actions to internal uav_ids 
        # map external actions to external uav_ids - like LEARNING  
        # from the CommandBundle extract the CONTROL action 
        #! fix the code below based on logic described above 
        # need to unpack only the CONTROL action and place inside return elegantly 
        # check if the implementation below is correct 
        updated_external_plans_dict = {}
        for uav_id, uav_command_list in external_ids_plans_dict.items():
            for uav_command in uav_command_list:
                if uav_command.action_type == ActionType.MISSION_PLAN:
                    updated_external_plans_dict[uav_id] = uav_command.payload

        return {**internal_plans_dict, **updated_external_plans_dict} # since internal is already unpacked, the control actions from external should be a single dict that will be unpacked in return 
    
    def step_uavS(self, external_action_dict: UAVCommandBundle) -> Tuple[Dict,Dict,Dict,Dict,Dict]:
        '''Bring all sort of updates and execute them in this function '''
        

        #! WHERE IS THE UAV_ID TO UAV MAP
        ### MOVE UAV ###
        
        # PLAN
        plan_dict = self.planner_module.get_plans()
        #updated_plan_dict = self.map_plans_to_uavs(plan_dict, external_ids_actions_dict=external_action_dict) 
        
        # CONTROL ACTION
        control_actions_dict = self.controller_module.get_actions(plan_dict)
        updated_control_actions_dict = self.map_actions_to_uavs(control_actions_dict, external_ids_actions_dict=external_action_dict) 
        
        # DYNAMICS 
        self.dynamics_module.step(actions_dict=updated_control_actions_dict)

        ### CHECK COLLISION ###
        #TODO: update collision module to return UAV_id list 
        # the uav_list needs to be updated here 
        detection_dict_restricted_area = self.sensor_module.get_detection_restricted_area() # <- pass uav_id_list
        detection_dict_uavS = self.sensor_module.get_detection_other_uavS()
        nmac_dict = self.sensor_module.get_nmac()
        collision_dict_restricted_area = self.sensor_module.get_collision_restricted_area()
        collision_dict_uavS = self.sensor_module.get_collision_uavS()

        ### REMOVE UAV ###
        # remove UAVs that have collided 
        self.atc.remove_uavs_by_id(collision_dict_restricted_area)
        self.atc.remove_uavs_by_id(collision_dict_uavS)
        # record their stats/metrics 
        
        return detection_dict_restricted_area, detection_dict_uavS, nmac_dict, collision_dict_restricted_area, collision_dict_uavS
        






    # do we directly add the command to the UAV
    # OR do collect these external commands and then combine with internal commands and dispatch at the same time 
    # def dispatch_commands(self, commands: UAVCommandBundle):
    #     for uav_id, cmd_list in commands.items():
    #         for cmd in cmd_list:
    #             match cmd.action_type:
    #                 case ActionType.MISSION_PLAN:
    #                     self.planner_module.set_plans(uav_id, cmd.payload)
    #                 case ActionType.PATH:
    #                     self.planner_module.set_plans(uav_id, cmd.payload)
    #                 case ActionType.TRAJECTORY:
    #                     self.planner_module.set_plans(uav_id, cmd.payload)
    #                 case ActionType.CONTROL:
    #                     {uav_id:cmd.payload}
    #                 case _:
    #                     raise ValueError(f"Unknown action type: {cmd.action_type}")

