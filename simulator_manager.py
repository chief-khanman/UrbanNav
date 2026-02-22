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
        ##### Collision #####
        # collision_detector for COLLISION DETECTION/RESOLUTION
        self.collision_detector = AdaptiveCollisionDetector(
                                                            sensor_radius=5.0,
                                                            mode='3D'
                                                            )
        
        # use config to send data to statemanager 
        
        # AER_BUS handles state export and action import
        # AER_BUS for NETWORKING 
        self.aer_bus = AerBus(mode='deployment') 
        
        ##### Physics/Dynamics #####
        # Dynamics_Engine for PHYSICS CALCULATION 
        # Dynamics engine will ingest config.fleet_composition and create instances of active dynamics in config
        self.dynamics_engine = DynamicsEngine(self.config) # make dynamics engine handle different kind/form of dynamics

    
    # WORKING:  --- Feb 19, 2026     
    #! NEED to rework this function 
    
    def _build_assets(self,):
        #### VP ####
        
        ## VP LIST ##
        # create vertiports
        num_vp = self.config.airspace.number_of_vertiports * self.config.vertiport.number_of_landing_pad
        # change this method to build vertiports in different ways 
        self.airspace.add_n_random_vps_to_vplist(num_vertiports=num_vp)
        # VP_LIST -> self.airspace.vertiport_list
        #TODO: store the created VPs in a database with low memory AND fast query, fetch, and update process - 
        ## VP ID LIST ## 
        self.sim_vp_id_list = self.airspace.get_vp_id_list()
        #### UAV ####
        self.uav_blueprints = build_fleet(self.config) # num_uav comes from total in fleet composition 
        # create uavs
        self.atc.create_uav(self.uav_blueprints) 
        # uav assignment - simulator_manager 
        
        ## UAV LIST  ##
        # use these uav_blueprints and build uavs using uav_template OR method from atc that builds UAVs from UAV blueprints
        ## UAV ID LIST ##
        # build a uav_id_list - the uav_id_list and uav_list need to be connected
        # DURING SIMULATION ON THE UAV_id_LIST is carried, it will be used to access all the UAVs in UAV_LIST, and update their attrs
        
        # and connect them to atc.uav_list
        
        # build similar component like dynamics_engine that will handle sensor, planner, controller
        # controller is already handled by aer_bus 
        # build the dict that hold mapping from uav_id to dynamics, controller, and sensor 

        # print and pause for user to verify the vertiports are correct and uavs are correct 


        # initiate external systems
        self.external_systems = self.initiate_external_systems()


    
    
    

    # WORKING:  --- Feb 19, 2026
        
    
    def _create_data_class_state(self,):
        self.airspace_state = self.airspace.get_state()
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
    
    def get_plan(self, ) -> Dict[str, List]:
        '''Use uav_id to access planner_dict,
        and extract plan for UAV and append to dict. '''
        # plan structure: 'uav_id_xx': [(time, plan), (time, plan), ......]
        plan_dict = {}
        for uav_id in self.uav_dict.keys():
            uav_plan = self.planner_dict[uav_id]
            plan_dict[uav_id] = uav_plan
        return plan_dict

    def get_control_actions(self, plan_dict):
        # loop through uav_id and get control_actions for each uav 
        # action structure: 'uav_id_xx': [(time, action), (time, action), ......]
        control_actions = {}
        for uav_id in self.uav_ids:
            controller = self.aer_bus.controllers[uav_id]
            uav_state = self.uavs.get_uav(uav_id)
            control_action = controller.compute_action(uav_state)
            self.control_actions[uav_id] = control_action
            control_actions[uav_id] = control_action
        # store control actions in self._control_actions = {'uav_id_i': control_action}
        return control_actions
   
    def update_simulator_state(self, control_actions:Dict[str,float])->None:
        '''Update 
        1. timestamp
        2. currentstep
        3. airspace_state
        4. atc_state
        5. external_systems
        '''
        ##### update: current_state.STEP
        current_state.step += 1

        ##### update: current_state.UAVs #####
        # already updated using dynamics.step()

        ##### update: current_state.AIRSPACE #####

        ##### update: current_state.VERTIPORTS #####
        for vertiport in current_state.vertiports:
            # if UAV landed at vertiport == True
            # add UAV to vertiport queue 
            vertiport.update_queue()
        
        ##### update: current_state.ATC_STATE #####
        # atc_state.update() will loop through all the vertiports in atc
        # for vertiport in atc.vertiport_list:
        # if vertiport.queue is not empty:
        # for uav in vertiport.queue:
        # assign new_mission/new_goal to uav   
        current_state.atc_state.update()


        ## UAV start tasks ##
        # if multiple uavs start from the same vp do not throw collision detection 
        # ...
        ## UAV end task 

        ## collision_check

        ## UAV - ATC - VERTIPORT COMM : hold patter, etc

        ## STEPPER : control_action -> dynamics -> state update 

        ##

        
        ##### update: current_state.EXTERNAL_SYSTEMS #####
        # loop through uav_id and get dynamics_model
        # for uav_id_i: use dynamics_model with -> control_action uav_id_i 
        # collect updated state 
        # update state of uav_id_i 
        for uav_id in uav_dict.keys():
            uav = self.uavs.get_uav(uav_id)
            
            dynamics_model = self.dynamics_engine.get_dynamics_model(uav_id)
            
            dynamics_model.step(uav, action)

    def map_action_to_uavs(self, external_ids_actions_dict:Dict[str, Any]) -> None:
        for uav_id, control_action in external_ids_actions_dict.items():
            self.control_actions[uav_id] =  control_action 

    #! POSSIBLE ENTRY FOR EXTERNAL ACTIONS 
    def step(self, external_action_dict):
        '''Bring all sort of updates and execute them in this function '''
        ### MOVE UAV ###
        plan_dict = self.get_plan()
        
        control_actions = self.get_control_actions(plan_dict=plan_dict)
        self. map_action_to_uavs(external_ids_actions_dict=external_action_dict)
        
        self.update_simulator_state(control_actions)

        ### CHECK COLLISION ###
        # the uav_list needs to be updated here 
        self.collision_detector() # <- pass uav_id_list


        ### REMOVE UAV ###
        # remove UAVs that have collided 
        # record their stats/metrics 

        ### MISSION UPDATE UAV ###
        # check uavs that have left vp 
        # check uavs that have reached vp 
        # check uavs that have reached vp - and vp has space to land 
        # hold uavs at vp where no space to land
        # assign new mission to uavs that are in vp

    def dispatch_commands(self, commands: UAVCommandBundle):
        for uav_id, cmd_list in commands.items():
            for cmd in cmd_list:
                match cmd.action_type:
                    case ActionType.MISSION_PLAN:
                        self.uavs[uav_id].set_mission_plan(cmd.payload)
                    case ActionType.PATH:
                        self.uavs[uav_id].set_path(cmd.payload)
                    case ActionType.TRAJECTORY:
                        self.uavs[uav_id].set_trajectory(cmd.payload)
                    case ActionType.CONTROL:
                        self.uavs[uav_id].set_control_action(cmd.payload)
                    case _:
                        raise ValueError(f"Unknown action type: {cmd.action_type}")


    
    ##### PLANNER ##### 
    #TODO: create a planner class - with three sub classes MissionPlanner, PathPlanner, TrajectoryPlanner
    # MisisonPlanner - 
    # PathPlanner - 
    # TrajectoryPlanner - 
    def _initialize_uav_plan(self) -> None:
        """Assign start/end vertiports to UAVs"""
        
        strategy = self.config.atc.vertiport_init_strategy
        uav_list = self.atc.get_uav_list()
        vertiport_list = self.airspace.get_vertiport_list()
        
        # mission plan 
        if strategy == 'random':
            for uav in uav_list:
                start_vp = random.choice(vertiport_list)
                end_vp = random.choice([vp for vp in vertiport_list if vp.id != start_vp.id])
                #! lets create ATC_STATE and ATC - 
                # atc will have functions 
                # atc_state will be a data class 
                #! update to ATC 
                self.atc_state.assign_vertiport_uav(uav, start_vp, end_vp)

        elif strategy == 'SOMETHING':
            pass
        # path plan 
        # trajectory plan 

    def update_uav_position_plan(self, plan_type:str, plan:List[Point]):
        
        uav.position_plan[plan_type] = plan
        
        return None

    def update_uav_velocity_plan(self, plan_type, plan):
        pass

    ##### PLANNER ##### 