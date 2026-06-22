import datetime
import random
from typing import Dict, List, Tuple, Any, Optional
import numpy as np
from urbannav.airspace import Airspace
from urbannav.atc import ATC
from urbannav.component_schema import UAMConfig
from urbannav.sensor_engine import SensorEngine
from urbannav.planner_engine import PlannerEngine
from urbannav.aer_bus import AerBus
from urbannav.dynamics_engine import DynamicsEngine
from urbannav.component_schema import UAVCommandBundle, ActionType, SimulatorState, build_fleet_blueprint
from urbannav.component_schema import RESERVED_TYPE_SINGLE_AGENT_LEARNING, RESERVED_TYPE_MULTI_AGENT_LEARNING
from urbannav.demand_model import DemandModelMixin

class SimulatorManager(DemandModelMixin):
    '''Primary class that orchestrates and manages assets/data_classes,
      calculation, networking, collision handler for UAM simulator. '''

    def __init__(self,
                 config:UAMConfig,
                 lambda_matrix: Optional[np.ndarray] = None,
                 vertiport_region_map: Optional[Dict[int, int]] = None,
                 zone_region_map: Optional[Dict] = None):
        '''Initialize simulator manager
            Args:
                config: UAMConfig — all existing simulator config.
                lambda_matrix: np.ndarray of shape (N, N), units trips/min.
                    lambda_matrix[i][j] is the Poisson rate for trip requests
                    from region i to region j. If None (default), the
                    demand-model path is off entirely and the simulator runs
                    the original random mission-assignment behavior.
                vertiport_region_map: Dict mapping vertiport_id (int) to
                    region_id (int, 0-indexed). Built by the RL environment
                    when it selects vertiports (one per region); updated each
                    episode via update_vertiport_region_map().
                zone_region_map: Static zone_id -> region_id mapping used by
                    the RL environment to build vertiport_region_map. None in
                    standalone/mission-sim use.
            Returns:
                None'''

        # UAM config object
        self.config = config
        # seed for all simulator related task
        self.seed = self.config.simulator.seed
        # dt
        self.dt = self.config.simulator.dt

        # Demand-model state. lambda_matrix is None -> demand-mode off,
        # reset()/step() run zero demand-model code (default mission-sim path
        # is untouched).
        self.lambda_matrix = lambda_matrix
        self.vertiport_region_map = vertiport_region_map if vertiport_region_map is not None else {}
        self.n_regions = lambda_matrix.shape[0] if lambda_matrix is not None else 0
        self._demand_rng = np.random.default_rng(self.seed)
        self.zone_region_map: Dict = zone_region_map if zone_region_map is not None else {}

        return None

    def _init_airspace(self,):
            return Airspace(location_name=self.config.airspace.location_name,
                            number_of_vertiports=self.config.airspace.number_of_vertiports,
                            vertiport_tag_list=self.config.airspace.vertiport_tag_list,
                            airspace_restricted_area_tag_list=self.config.airspace.airspace_restricted_area_tag_list,
                            seed=self.seed
                            )
    
    def _init_atc(self):
        return ATC(airspace=self.airspace, 
                   seed=self.seed)

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
        self.sensor_module = SensorEngine(self.config, 
                                          self.atc.sensor_map, 
                                          self.atc.uav_dict,
                                          airspace=self.airspace)
        
        # use config to send data to statemanager 
        ### Planner ###
        self.planner_module = PlannerEngine(self.config, 
                                            self.atc.planner_map, 
                                            self.atc.uav_dict)
        
        ### Controller ###
        # AER_BUS handles state export and action import
        # AER_BUS for NETWORKING 
        #! should controller take in plan
        self.controller_module = AerBus(self.config, 
                                        self.atc.controller_map, 
                                        self.atc.uav_dict, 
                                        mode='deployment')
        
        ### Physics/Dynamics ###
        # Dynamics_Engine for PHYSICS CALCULATION 
        # make dynamics engine handle different kind/form of dynamics
        #! should dynamics take in controller_action 
        self.dynamics_module = DynamicsEngine(self.config, 
                                              self.atc.dynamics_map, 
                                              self.atc.uav_dict) 
          
    def _build_vertiports_random(self,):
        """Populate self.airspace.vertiport_list with config.airspace.number_of_vertiports
        random vertiports. Skipped when reset(rebuild_airspace=False) — the
        vertiport_list is then assumed to have been set externally (e.g. by
        VertiportDesignEnv via airspace.set_vertiport_list_vp_design)."""
        num_vp = self.config.airspace.number_of_vertiports
        self.airspace.add_n_random_vps_to_vplist(num_vertiports=num_vp)

    def _build_fleet_and_externals(self,):
        """Build ATC fleet + external systems. Runs every reset() regardless
        of rebuild_airspace — UAVs are always rebuilt fresh per episode."""
        ## VP ID LIST ##
        self.sim_vp_id_list = self.airspace.get_vp_id_list()

        #### ----------- UAV ----------- ####
        self.uav_blueprints = build_fleet_blueprint(self.config) # num_uav comes from total in fleet composition
        # create uavs
        # use these uav_blueprints and build uavs using uav_template OR method from atc that builds UAVs from UAV blueprints
        self.atc.create_uavS_from_blueprint(self.uav_blueprints)
        # assign vertiports to UAVs
        self.atc.assign_vertiports()
        # initiate external systems
        self.external_systems = self.initiate_external_systems()

    #! NEED to rework this function
    def _build_assets(self,):
        """Default-path asset build: random vertiports + fresh fleet."""
        self._build_vertiports_random()
        self._build_fleet_and_externals()

    def _create_data_class_state(self,):
        self.airspace_state = self.airspace.get_state() #! change list vertiport -> dict[id:int, vp:Vertiport]
        self.atc_state = self.atc.get_state()

    def reset(self, rebuild_airspace: bool = True):
        """Reset the simulator for a new episode.

        Args:
            rebuild_airspace: When True (default), tear down and rebuild
                self.airspace (OSM fetch) and populate it with fresh random
                vertiports via add_n_random_vps_to_vplist. Mission-sim and the
                single/multi-agent RL paths use this.
                When False, REUSE the existing self.airspace and its current
                vertiport_list — only ATC, engines, and the UAV fleet are
                rebuilt. This is the soft-reset path used by the vertiport-
                design RL env: VertiportDesignEnv calls
                airspace.set_vertiport_list_vp_design(new_list) before each
                soft reset, so the agent's selected vertiports survive the
                reset and the expensive OSM/region geometry is built only
                once at env construction.
        """
        self.timestamp = datetime.datetime.now().strftime('%m %d %Y -- %H:%M')

        self.currentstep = 0

        if rebuild_airspace:
            # airspace, atc — fresh build (default path)
            self._initiate_simulator_assets()
        else:
            # Soft reset: reuse self.airspace as-is; only rebuild ATC.
            # self.airspace.vertiport_list must already be populated by the
            # caller (e.g. via airspace.set_vertiport_list_vp_design(...)).
            self.atc = self._init_atc()
        # dyn_engine, aer_bus, path_planner, sensor
        self._initiate_simulator_components()
        # uav
        if rebuild_airspace:
            self._build_assets()
        else:
            # Skip _build_vertiports_random — vertiport_list is the caller's
            # selection, not a fresh random sample.
            self._build_fleet_and_externals()
        # atc state, airspace state, ...
        self._create_data_class_state()
        
        self.sensor_module.register_uav_sensors()
        self.planner_module.register_uav_planners()
        self.controller_module.register_uav_controllers()
        self.dynamics_module.register_uav_dynamics()

        #! why do i need this
        # state
        self._state = SimulatorState(self.timestamp,
                                     self.currentstep,
                                     self.airspace_state,
                                     self.atc_state,
                                     self.external_systems)

        # Initialise demand-side state for this episode. Only runs when a
        # lambda_matrix was supplied at construction — otherwise the
        # simulator behaves exactly as it did before demand-model support.
        if self.lambda_matrix is not None:
            self._init_demand_state()

        return None




    def initiate_external_systems(self,):
        return {'No external system':None}    

    def get_state(self,):
        return self._state
    
    def set_state(self, state):
        self._state = state
        return None
   
    def step(self, external_control_actions_dict:UAVCommandBundle):
        '''Update 
        1. timestamp
        2. currentstep
        3. airspace_state
        4. atc_state
        5. external_systems
        '''
        # update: current_state.STEP
        #print(f'current_timestep: {self._state.currentstep}')
        self._state.currentstep += 1

        # ── DEMAND GENERATION (opt-in) ──────────────────────────────────────
        # Runs before _step_uavS so trips spawned this step are immediately
        # eligible for dispatch this step. No-op (and no behavior change) when
        # no lambda_matrix was supplied at construction.
        if self.lambda_matrix is not None and self.vertiport_region_map:
            self._generate_demand()

        # stepS_uav: control_action -> dynamics -> state update
        restricted_area_detect, uavs_detect, nmac, restricted_area_collision, uavs_collision = self._step_uavS(external_action_dict=external_control_actions_dict)

        #### ---------- ATC-UAV-Vertiport Mission Cycle ----------  ####
        #* PROCESS OF REACHING VERTIPROT
        #*            LEAVING VERTIPORT
        #*            CHECKING SPACE AT VERTIPORT
        #*            HOLDING PATTERN
        #*            LANDING
        #*            TAKEOFF
        #*            NEW ASSIGNMENT
        #* all of the above task should be in parallel and asynchronous
        # handle UAVs that have reached vertiports
        #                       or left vertiports

        for uav_id in self.atc.uav_dict.keys():
            self.atc.has_left_start_vertiport(uav_id)

            #! How can the same uav_id leave and reach at the same time
            #* logic for has_reached_end_vertiport() changed so this makes sense
            # has_reached_end_vertiport() -> check implementation
            self.atc.has_reached_end_vertiport(uav_id) #! this function shall add the uav to vertiports landing queue

            # Log queue-entry step for completed-trip metrics (opt-in; no-op
            # when demand-mode is off since _trip_log is never populated).
            if self.lambda_matrix is not None and self.vertiport_region_map:
                self._log_arrive_airspace(uav_id)

        #### delete after debug ####
        # `for vertiport in self.airspace.vertiport_list:
        #     print(f'Vertiport: {vertiport.id} has uav_id :{vertiport.uav_id_list}')`
        #### delete after debug ####

        for vertiport in self.airspace.vertiport_list:
            # print(f'Vertiport: {vertiport.id} has uav_id :{vertiport.uav_id_list}')
            if vertiport.uav_id_list:
                for uav_id in vertiport.uav_id_list:
                    if not self.atc.uav_dict[uav_id].operational and not self.atc.uav_dict[uav_id].uav_in_flight:
                        # ── DEMAND-DRIVEN MISSION ASSIGNMENT (opt-in) ──────
                        # Falls back to the original random coin-flip
                        # assignment when no lambda_matrix is supplied.
                        if self.lambda_matrix is not None and self.vertiport_region_map:
                            queue = self.departure_queues.get(vertiport.id, [])
                            if queue:
                                dest_vp_id, enqueue_step = queue.pop(0)
                                self._dispatch_mission(uav_id, vertiport.id, dest_vp_id, enqueue_step)
                            else:
                                self.atc.wait_at_vertiport(uav_id)
                        else:
                            new_mission = random.random() > 0.5
                            if new_mission:
                                # print(f'ATC uav ids: {self.atc.uav_dict} ')
                                # print(f'UAV id: {uav_id}, of UAV: {self.atc.uav_dict[uav_id]}')
                                self.atc.reassign_new_mission(uav_id) #! getting added to vertiport.uav_id_list
                            else:
                                self.atc.wait_at_vertiport(uav_id) #! getting added to vertiport.uav_id_list

            if vertiport.check_landing_space() and vertiport.get_landing_queue():
                landing_uav_id = vertiport.landing_queue[0]
                self.atc.landing_procedure(landing_uav_id)
                # Record completed trip for OD metrics (opt-in; no-op when
                # demand-mode is off since _trip_log is never populated).
                if self.lambda_matrix is not None and self.vertiport_region_map:
                    self._log_land(landing_uav_id, vertiport.id)

        ####  ---------- ATC-UAV-Vertiport Mission Cycle ----------  ####

        # Step-level metric accumulation (opt-in; cheap no-op-equivalent
        # bookkeeping when demand-mode is off, since get_episode_metrics()
        # is only ever called by the demand-aware RL env).
        if self.lambda_matrix is not None:
            self._accumulate_step_metrics()

        # update: current_state.EXTERNAL_SYSTEMS

        return restricted_area_detect, uavs_detect, nmac, restricted_area_collision, uavs_collision



        
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
    
    def _merge_collision_dicts(
        self,
        collision_dict_uavS: Dict[int, set[int]],
        collision_dict_ra: Dict[int, set[int]]
    ) -> List[int]:
        """
        Extract all UAV IDs to remove from both collision dicts.

        collision_dict_uavS: { uav_id -> [colliding_uav_id, ...] }
            Detection is symmetric — every colliding UAV appears as a value
            in its partner's entry, so iterating values captures all IDs.

        collision_dict_ra: { uav_id -> [ra_id, ...] }
            Values are restricted area IDs (not UAV IDs). Only keys where
            the value list is non-empty are UAV IDs to remove.
        """
        remove_uav_ids: set = set()

        # UAV-UAV: values are UAV IDs (symmetric detection covers all involved UAVs)
        for colliding_uavs in collision_dict_uavS.values():
            if colliding_uavs:
                remove_uav_ids.update(colliding_uavs)

        # Restricted area: values are RA IDs — use keys only
        for uav_id, ra_ids in collision_dict_ra.items():
            if ra_ids:
                remove_uav_ids.add(uav_id)

        return list(remove_uav_ids)

    def _step_uavS(self, external_action_dict: UAVCommandBundle) -> Tuple[Dict,Dict,Dict,Dict,Dict]:
        '''Bring all sort of updates and execute them in this function '''
        

        #! WHERE IS THE UAV_ID TO UAV MAP
        ### MOVE UAV ###
        
        # PLAN
        plan_dict = self.planner_module.get_plans()
        #updated_plan_dict = self.map_plans_to_uavs(plan_dict, external_ids_actions_dict=external_action_dict)

        # CONTROL ACTION
        control_actions_dict = self.controller_module.get_actions(plan_dict)
        updated_control_actions_dict = self.map_actions_to_uavs(control_actions_dict, external_ids_actions_dict=external_action_dict)

        # Skip dynamics for collided UAVs that are persisted (frozen in place)
        if self.config.simulator.persist_collided_uavs:
            collided_ids = self._get_collided_uav_ids()
            updated_control_actions_dict = {
                uid: act for uid, act in updated_control_actions_dict.items()
                if uid not in collided_ids
            }

        # DYNAMICS
        self.dynamics_module.step(actions_dict=updated_control_actions_dict)

        ### CHECK COLLISION ###
        #TODO: run get_collision once - from collision operation stream out detection, nmac, and collision
        # collision runs nmac which runs detection, so running these three means I am running detection 3 times, nmac twice and collsion once
        detection_dict_restricted_area = self.sensor_module.get_detection_restricted_area() # <- pass uav_id_list
        detection_dict_uavS = self.sensor_module.get_detection_other_uavS()
        nmac_dict = self.sensor_module.get_nmac()
        collision_dict_restricted_area = self.sensor_module.get_collision_restricted_area()
        collision_dict_uavS = self.sensor_module.get_collision_uavS()
        
        #print(f'Collision ids: {collision_dict_uavS}')
        ### REMOVE UAV ###
        # remove UAVs that have collided
        #! check vertiports
        uavs_to_remove = self._merge_collision_dicts(collision_dict_uavS, collision_dict_restricted_area)
        if self.config.simulator.persist_collided_uavs:
            self._mark_uavs_collided(uavs_to_remove)
        else:
            self.atc.remove_uavs_by_id(uavs_to_remove)
        
        # record their stats/metrics 
        
        return detection_dict_restricted_area, detection_dict_uavS, nmac_dict, collision_dict_restricted_area, collision_dict_uavS
        


    def _get_collided_uav_ids(self) -> set:
        """Return set of UAV ids that have been marked as collided."""
        convention = self.config.simulator.collision_status_convention
        collided_value = 0 if convention == "active_high" else 1
        return {
            uid for uid, uav in self.atc.uav_dict.items()
            if getattr(uav, "collision_status", None) == collided_value
        }

    def _mark_uavs_collided(self, uav_ids: List[int]) -> None:
        """Mark UAVs as collided without removing them from uav_dict.

        Sets collision_status according to the configured convention:
        - active_high: 1=active → 0=collided
        - collided_high: 0=active → 1=collided
        Collided UAVs remain in the dict but are frozen (excluded from
        dynamics/planning/control in future steps).
        """
        if not uav_ids:
            return
        convention = self.config.simulator.collision_status_convention
        collided_value = 0 if convention == "active_high" else 1
        for uid in uav_ids:
            uav = self.atc.uav_dict.get(uid)
            if uav is not None:
                uav.collision_status = collided_value
                uav.operational = False
                uav.vx = 0.0
                uav.vy = 0.0
                uav.vz = 0.0
                uav.current_speed = 0.0

    def get_learning_uav_id(self) -> Optional[int]:
        """
        Return the integer id of the first LEARNING UAV in atc.uav_dict.
        Returns None if no LEARNING UAV is present (e.g. before the first reset).
        """
        for uid, uav in self.atc.uav_dict.items():
            if getattr(uav, 'type_name', None) == RESERVED_TYPE_SINGLE_AGENT_LEARNING:
                return uid
        return None

    def get_multi_agent_uav_ids(self) -> Dict[str, List[int]]:
        """
        Return {policy_id: [uav_id, ...]} for all MULTI_AGENT_LEARNING UAVs in
        atc.uav_dict, grouped by their shared policy_id. Returns an empty dict
        if no MULTI_AGENT_LEARNING UAVs are present (e.g. before the first reset).
        """
        policy_uav_map: Dict[str, List[int]] = {}
        for uid, uav in self.atc.uav_dict.items():
            if getattr(uav, 'type_name', None) == RESERVED_TYPE_MULTI_AGENT_LEARNING:
                policy_id = getattr(uav, 'policy_id', None)
                if policy_id is not None:
                    policy_uav_map.setdefault(policy_id, []).append(uid)
        return policy_uav_map



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

