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
from urbannav.component_schema import RESERVED_TYPE_LEARNING


class SimulatorManagerVPDesign:
    '''Primary class that orchestrates and manages assets/data_classes,
      calculation, networking, collision handler for UAM simulator. '''

    def __init__(self,
                 config: UAMConfig,
                 lambda_matrix: Optional[np.ndarray] = None,
                 vertiport_region_map: Optional[Dict[int, int]] = None,
                 zone_region_map: Optional[Dict] = None):
        '''Initialize simulator manager.

        Args:
            config              : UAMConfig — all existing simulator config.
            lambda_matrix       : np.ndarray of shape (N, N), units trips/min.
                                  lambda_matrix[i][j] is the Poisson rate for
                                  trip requests from region i to region j.
                                  Diagonal must be 0 (no intra-region demand).
                                  Produced by Band 1 pipeline. If None,
                                  simulator falls back to original random
                                  assignment.
            vertiport_region_map: Dict mapping vertiport_id (int) to region_id
                                  (int, 0-indexed). Defines which region each
                                  selected vertiport belongs to. Must be
                                  consistent with the K-means clustering used
                                  in Band 1. Built by the RL environment when
                                  it selects vertiports (one per region) and
                                  passes them to reset().
                                  Note: Vertiport.region already stores this
                                  per vertiport; this dict mirrors it at the
                                  manager level for fast lookup and is updated
                                  each episode by the RL env via
                                  update_vertiport_region_map().
            zone_region_map     : Static Band 1 output mapping zone_id to
                                  region_id. Does not change between episodes.
                                  Loaded once from zone_region_map.csv. Used
                                  by the RL environment to translate selected
                                  vertiport zones into region assignments when
                                  building vertiport_region_map per episode.
                                  None if not supplied (standalone test mode).
        Returns:
            None
        '''
        # UAM config object
        self.config = config
        # seed for all simulator related tasks
        self.seed = self.config.simulator.seed
        # dt
        self.dt = self.config.simulator.dt

        # Store OD demand matrix and vertiport→region mapping.
        # lambda_matrix: (N, N) trips/min. None → demand-unaware fallback.
        self.lambda_matrix = lambda_matrix
        # vertiport_region_map: {vertiport_id: region_id}. Built per episode
        # by the RL environment when it commits to a vertiport selection.
        self.vertiport_region_map = vertiport_region_map if vertiport_region_map is not None else {}

        # Derive region count from lambda_matrix shape.
        # n_regions is 0 when no lambda_matrix is supplied (fallback mode).
        self.n_regions = lambda_matrix.shape[0] if lambda_matrix is not None else 0

        # Seeded numpy RNG for Poisson draws — separate from Python's random
        # module used by the rest of the simulator so that demand stochasticity
        # is independently reproducible.
        self._demand_rng = np.random.default_rng(self.seed)

        # Static zone→region mapping from Band 1. Does not change between
        # episodes. Used by the RL env to build vertiport_region_map each
        # episode. Empty dict when not supplied (standalone test mode).
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
        self.sensor_module = SensorEngine(self.config, self.atc.sensor_map, self.atc.uav_dict,
                                          airspace=self.airspace)
        
        # use config to send data to statemanager 
        ### Planner ###
        self.planner_module = PlannerEngine(self.config, self.atc.planner_map, self.atc.uav_dict)
        
        ### Controller ###
        # AER_BUS handles state export and action import
        # AER_BUS for NETWORKING 
        #! should controller take in plan
        self.controller_module = AerBus(self.config, self.atc.controller_map, self.atc.uav_dict, mode='deployment') #! should I pass the CONFIG 
        
        ### Physics/Dynamics ###
        # Dynamics_Engine for PHYSICS CALCULATION 
        # make dynamics engine handle different kind/form of dynamics
        #! should dynamics take in controller_action 
        self.dynamics_module = DynamicsEngine(self.config, self.atc.dynamics_map, self.atc.uav_dict) 
          
    #! NEED to rework this function 
    def _build_assets(self,):
        #### ----------- VP ----------- ####
        # create vertiports
        num_vp = self.config.airspace.number_of_vertiports 
        # change this method to build vertiports in different ways 
        self.airspace.add_n_random_vps_to_vplist(num_vertiports=num_vp)
        
        ## ----> VP_LIST <---- ##
        # ----> self.airspace.vertiport_list <---- 
        
        #TODO: store the created VPs in a database with low memory AND fast query, fetch, and update process - 
        ## VP ID LIST ## 
        #! what is the use of this 
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

    def _create_data_class_state(self,):
        self.airspace_state = self.airspace.get_state() #! change list vertiport -> dict[id:int, vp:Vertiport]
        self.atc_state = self.atc.get_state()

    def reset(self, ):
        self.timestamp = datetime.datetime.now().strftime('%m %d %Y -- %H:%M')
        
        self.currentstep = 0
        
        # airspace, atc
        self._initiate_simulator_assets()
        # dyn_engine, aer_bus, path_planner, sensor
        self._initiate_simulator_components()
        # uav  
        self._build_assets()
        # atc state, airspace state, ...
        self._create_data_class_state()
        
        self.sensor_module.register_uav_sensors()
        self.planner_module.register_uav_planners()
        self.controller_module.register_uav_controllers()
        self.dynamics_module.register_uav_dynamics()

        # Initialise all demand-side state for this episode.
        # Called after _build_assets() so that vertiport_region_map is stable
        # and vertiport_list is populated.
        self._init_demand_state()

        #! why do i need this
        # state
        self._state = SimulatorState(self.timestamp,
                                     self.currentstep,
                                     self.airspace_state,
                                     self.atc_state,
                                     self.external_systems)
        return None

    def _init_demand_state(self):
        '''Initialise per-episode demand tracking structures.
        Called by reset() at the start of every episode.

        Structures initialised:
          departure_queues    — one FIFO list per vertiport; each entry is a
                               (dest_vertiport_id, enqueue_step) tuple.
          _trip_log           — list of dicts, one per COMPLETED trip.
          _demand_gen_od      — (N, N) int array counting trips GENERATED this
                               episode per OD region pair. Denominator of B3.
          _trips_completed_od — (N, N) int array counting trips COMPLETED.
          _pads_occupied_sum  — (n_vp,) float; running sum of pads_occupied.
          _queue_length_sum   — (n_vp,) float; running sum of uavs_waiting.
          _uavs_in_flight_sum — int; running sum of in-flight count → B6.
        Step count uses self._state.currentstep (set by step()).
        '''

        n_vp = len(self.airspace.vertiport_list)

        # Departure queues: vertiport_id → list of (dest_vp_id, enqueue_step)
        # Trips generated by the Poisson process are pushed here and popped
        # when a UAV becomes available for a mission.
        self.departure_queues: Dict[int, List[Tuple[int, int]]] = {
            vp.id: [] for vp in self.airspace.vertiport_list
        }

        # Trip completion log: one dict per completed trip.
        # Filled in step() when a UAV lands. Keys match the UrbanNav
        # instrumentation spec in the MDP formulation §3.3.
        self._trip_log: List[Dict] = []

        # OD trip counters at region level (N×N).
        # Demand is defined at the region level (lambda_matrix is N×N).
        # Vertiport-level trips are mapped back to region pairs for metric
        # computation.
        if self.n_regions > 0:
            self._demand_gen_od      = np.zeros((self.n_regions, self.n_regions), dtype=int)
            self._trips_completed_od = np.zeros((self.n_regions, self.n_regions), dtype=int)
        else:
            # Fallback: no OD matrix supplied — OD metrics will be empty.
            self._demand_gen_od      = np.zeros((0, 0), dtype=int)
            self._trips_completed_od = np.zeros((0, 0), dtype=int)

        # Step-level accumulators for time-averaged node features (B4, B5, B6).
        # Indexed by position in airspace.vertiport_list, not by vertiport id,
        # to keep numpy operations simple. The mapping vp_list_index →
        # vertiport_id is stable within a single episode.
        self._vp_index_to_id: List[int] = [vp.id for vp in self.airspace.vertiport_list]
        
        #TODO: remove once airspace has defined attr
        self._vp_id_to_vp: Dict[int, Any] = {vp.id: vp for vp in self.airspace.vertiport_list}
        self._pads_occupied_sum  = np.zeros(n_vp, dtype=float)
        self._queue_length_sum   = np.zeros(n_vp, dtype=float)
        # UAVs in flight is always a whole number; int avoids float accumulation.
        self._uavs_in_flight_sum = 0



    def initiate_external_systems(self,):
        return {'No external system':None}    

    def get_state(self,):
        return self._state
    
    def set_state(self, state):
        self._state = state
        return None
   
    def step(self, external_control_actions_dict: UAVCommandBundle):
        '''Update
        1. timestamp
        2. currentstep
        3. airspace_state
        4. atc_state
        5. external_systems
        '''
        # update: current_state.STEP
        self._state.currentstep += 1

        # ── DEMAND GENERATION ────────────────────────────────────────────────
        # Run Poisson trip generation before mission assignment so that trips
        # spawned this step are immediately eligible for dispatch this step.
        #
        # Mechanism (Method A — per-timestep Bernoulli approximation):
        #   For each OD region pair (i, j) with i ≠ j:
        #     p_ij = lambda_ij * dt / 60
        #       lambda_ij : trips/min from region i to region j
        #       dt        : simulator timestep in seconds (default 0.1 s)
        #       /60       : converts dt from seconds to minutes
        #   Draw u ~ U(0,1); if u < p_ij → generate one trip request.
        #   This is a first-order discretisation of a Poisson process. For
        #   typical lambda_ij values (~0.1–5 trips/min) and dt=0.1 s the
        #   probability p_ij is O(1e-4)–O(1e-2), so the Bernoulli→Poisson
        #   approximation error is negligible.
        #
        # Why before _step_uavS?
        #   UAV dynamics are computed inside _step_uavS. New trips generated
        #   here are queued at origin vertiports. Mission assignment (below)
        #   then checks those queues for idle UAVs at the same timestep.
        #   This matches the causality in the MDP formulation §3.3.1.
        # ─────────────────────────────────────────────────────────────────────
        if self.lambda_matrix is not None and self.vertiport_region_map:
            self._generate_demand()

        # stepS_uav: control_action -> dynamics -> state update
        restricted_area_detect, uavs_detect, nmac, restricted_area_collision, uavs_collision = \
            self._step_uavS(external_action_dict=external_control_actions_dict)

        #### ---------- ATC-UAV-Vertiport Mission Cycle ----------  ####
        #* PROCESS OF REACHING VERTIPORT
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
            # has_reached_end_vertiport() places the UAV into the vertiport
            # landing queue. The UAV's arrival also triggers arrive_airspace_step
            # logging inside _log_arrive_airspace() called immediately after,
            # so the landing queue length at the moment of arrival (= wait time
            # start) is captured before any pad is freed.
            self.atc.has_reached_end_vertiport(uav_id)
            # Log the step at which this UAV entered the landing queue
            # (= arrived in the airspace above the vertiport).
            # _log_arrive_airspace checks internally whether the UAV is now
            # in a landing queue; it is a no-op if the UAV has not yet arrived.
            self._log_arrive_airspace(uav_id)

        for vertiport in self.airspace.vertiport_list:
            if vertiport.uav_id_list:
                for uav_id in vertiport.uav_id_list:
                    if not self.atc.uav_dict[uav_id].operational and not self.atc.uav_dict[uav_id].uav_in_flight:

                        # ── DEMAND-DRIVEN MISSION ASSIGNMENT ──────────────────
                        # Original: new_mission = random.random() > 0.5
                        #   A coin flip with no awareness of demand. Replaced
                        #   entirely when an OD matrix is available.
                        #
                        # Demand-driven logic:
                        #   1. Identify the region this vertiport belongs to.
                        #   2. Check the departure queue at this vertiport.
                        #   3. If the queue is non-empty, pop the front entry
                        #      (FIFO) and assign the UAV that mission.
                        #   4. If the queue is empty, the UAV waits.
                        #
                        # Why FIFO? Matches the queueing model in the MDP
                        # formulation: oldest requests served first (standard
                        # M/M/c queue discipline).
                        #
                        # Fallback: if no lambda_matrix is provided, retain
                        # original random assignment so the simulator runs
                        # unchanged without a demand matrix.
                        # ──────────────────────────────────────────────────────
                        if self.lambda_matrix is not None and self.vertiport_region_map:
                            queue = self.departure_queues.get(vertiport.id, [])
                            if queue:
                                dest_vp_id, enqueue_step = queue.pop(0)
                                self._dispatch_mission(uav_id, vertiport.id,
                                                       dest_vp_id, enqueue_step)
                            else:
                                self.atc.wait_at_vertiport(uav_id)
                        else:
                            # Original random assignment — kept intact so the
                            # simulator runs without a lambda_matrix.
                            new_mission = random.random() > 0.5
                            if new_mission:
                                self.atc.reassign_new_mission(uav_id)
                            else:
                                self.atc.wait_at_vertiport(uav_id)

            if vertiport.check_landing_space() and vertiport.get_landing_queue():
                landing_uav_id = vertiport.landing_queue[0]
                self.atc.landing_procedure(landing_uav_id)
                # Log the step at which this UAV lands and record the
                # completed trip in the OD counters.
                self._log_land(landing_uav_id, vertiport.id)

        # ── STEP-LEVEL METRIC ACCUMULATION ───────────────────────────────────
        # Accumulate pad occupancy, queue lengths, and in-flight count at every
        # step. Dividing by step count at episode end gives time-averaged values
        # for B4 (utilization), B5 (queue_length), B6 (fleet_utilization).
        # This must run at the end of each step after all state updates above.
        self._accumulate_step_metrics()

        ####  ---------- ATC-UAV-Vertiport Mission Cycle ----------  ####

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
        
        # DYNAMICS 
        self.dynamics_module.step(actions_dict=updated_control_actions_dict)

        ### CHECK COLLISION ###
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
        self.atc.remove_uavs_by_id(uavs_to_remove)
        
        # record their stats/metrics 
        
        return detection_dict_restricted_area, detection_dict_uavS, nmac_dict, collision_dict_restricted_area, collision_dict_uavS
        


    def get_learning_uav_id(self) -> Optional[int]:
        """
        Return the integer id of the first LEARNING UAV in atc.uav_dict.
        Returns None if no LEARNING UAV is present (e.g. before the first reset).
        """
        for uid, uav in self.atc.uav_dict.items():
            if getattr(uav, 'type_name', None) == RESERVED_TYPE_LEARNING:
                return uid
        return None


    def _generate_demand(self):
        '''Poisson trip generation — runs once per step.
        Pushes trip requests into vertiport departure queues.

        For each OD region pair (i, j) with i ≠ j:
          p_ij = lambda_matrix[i,j] * dt / 60   (Bernoulli probability)
          Draw u ~ U(0,1); if u < p_ij: push a trip request into the departure
          queue at the vertiport serving region i.

        Mapping from region → vertiport:
          vertiport_region_map is {vp_id: region_id}. The RL environment
          selects exactly one vertiport per region, so this map is already 1:1.
          We invert it to get {region_id: vp_id} — one vertiport per region
          (the Level 1 constraint). This inversion is cached in _region_to_vp
          after the first call and is stable for the duration of the episode.

        Why a departure queue entry stores (dest_vp_id, enqueue_step):
          dest_vp_id  : used by _dispatch_mission to set the UAV's target.
          enqueue_step: used to compute wait time. Currently stored but not
                        used for request-dropping; feeds potential Level 3
                        extensions.
        '''

        # Build region-to-vertiport lookup once per episode (stable after reset).
        if not hasattr(self, '_region_to_vp') or self._region_to_vp is None:
            self._region_to_vp: Dict[int, int] = {
                region: vp_id
                for vp_id, region in self.vertiport_region_map.items()
            }

        dt_min = self.dt / 60.0   # convert simulator dt (seconds) to minutes

        for i in range(self.n_regions):
            # vertiport_region_map has exactly one vertiport per region
            # (the RL agent's selection for this episode). _region_to_vp is
            # its inverse, so this lookup gives the single origin vertiport
            # for region i.
            origin_vp_id = self._region_to_vp.get(i)
            if origin_vp_id is None:
                continue   # region i has no selected vertiport this episode

            for j in range(self.n_regions):
                if i == j:
                    continue   # no intra-region demand (Level 1 rule)

                dest_vp_id = self._region_to_vp.get(j)
                if dest_vp_id is None:
                    continue

                # lambda <- lambda_matrix[i,j]
                # lambda is trips/min from i to j
                rate = self.lambda_matrix[i, j]   # trips/min
                if rate <= 0.0:
                    continue

                # Bernoulli approximation of Poisson process at this timestep.
                p_ij = rate * dt_min
                u = self._demand_rng.random()
                if u < p_ij:
                    # Trip generated — push to departure queue at origin vp.
                    self.departure_queues[origin_vp_id].append(
                        (dest_vp_id, self._state.currentstep)
                    )
                    # Increment region-level generated counter.
                    self._demand_gen_od[i, j] += 1

    def _dispatch_mission(self,
                          uav_id: int,
                          origin_vp_id: int,
                          dest_vp_id: int,
                          enqueue_step: int):
        '''Assign a queued demand trip to an idle UAV and open a trip log entry.

        Args:
            uav_id        : id of the UAV being assigned the mission.
            origin_vp_id  : vertiport id where the UAV currently is.
            dest_vp_id    : vertiport id the UAV is flying to.
            enqueue_step  : simulator step when the trip request was generated.

        What it writes to _trip_log:
          Each entry is a dict with keys matching the UrbanNav instrumentation
          spec (MDP formulation §3.3). Entries are completed (land_step,
          arrive_airspace_step) later by _log_arrive_airspace/_log_land.
          The key used to correlate later updates is uav_id — one active trip
          per UAV at a time (enforced by existing ATC assignment logic).

        Targeted assignment is implemented by calling _uav.assign_start_end()
        directly (same path as reassign_new_mission but with specific vertiports).
        A dedicated atc.assign_mission_to_target() is not needed.
        '''
        # Resolve region pair for this trip.
        o_region = self.vertiport_region_map.get(origin_vp_id, -1)
        d_region = self.vertiport_region_map.get(dest_vp_id, -1)

        # Assign targeted mission: set origin as start, dest as end.
        # Uses assign_start_end directly (mirrors reassign_new_mission but
        # with specific vertiports instead of random selection).
        # Must NOT use assign_mission_start_end_vertiport here: that method
        # appends to uav_id_list, but the UAV is already in origin_vp.uav_id_list
        # from landing_procedure. A duplicate entry would leave a ghost after
        # _takeoff_procedure removes only the first occurrence via list.remove().
        _uav = self.atc.uav_dict[uav_id]
        # temp-check: after landing at origin_vp, uav.end_vertiport must equal origin_vp.
        assert _uav.end_vertiport == self._vp_id_to_vp[origin_vp_id]
        _uav.assign_start_end(
            self._vp_id_to_vp[origin_vp_id],
            self._vp_id_to_vp[dest_vp_id],
            self.atc.airspace_mid_point_coord
        )

        # Open a trip log entry; arrive_airspace_step and land_step are
        # filled in by _log_arrive_airspace() and _log_land() respectively.
        self._trip_log.append({
            'uav_id'              : uav_id,
            'o_region'            : o_region,
            'd_region'            : d_region,
            'origin_vp_id'        : origin_vp_id,
            'dest_vp_id'          : dest_vp_id,
            'enqueue_step'        : enqueue_step,
            'depart_step'         : self._state.currentstep,
            'arrive_airspace_step': None,   # filled by _log_arrive_airspace
            'land_step'           : None,   # filled by _log_land
        })

    def _log_arrive_airspace(self, uav_id: int):
        '''Log the step at which uav_id entered the landing queue.

        Checks internally whether the UAV is currently in a landing queue;
        this is a no-op if the UAV has not yet reached its destination vertiport.
        Fills arrive_airspace_step in the open trip log entry for uav_id.

        In the current free-flight model, has_reached_end_vertiport() places
        the UAV directly into the landing queue. Therefore arrive_airspace_step
        equals land_step when there is no queue, and arrive_airspace_step <
        land_step when the UAV must wait for a free pad.

        Note on the None guard below: uav_id comes from iterating
        self.atc.uav_dict.keys() in step(), so the UAV should always be
        present. The guard is retained as a safety net against edge cases
        where has_left_start_vertiport() or has_reached_end_vertiport() might
        modify the dict during iteration.
        '''
        uav = self.atc.uav_dict.get(uav_id)
        if uav is None:
            return
        # Only log if the UAV is now in a landing queue (not still in flight).
        for vertiport in self.airspace.vertiport_list:
            if uav_id in vertiport.landing_queue:
                # Find the most recent open log entry for this UAV.
                for entry in reversed(self._trip_log):
                    if entry['uav_id'] == uav_id and entry['arrive_airspace_step'] is None:
                        entry['arrive_airspace_step'] = self._state.currentstep
                break

    def _log_land(self,
                  uav_id: int,
                  vertiport_id: int):
        '''Record completed landing and increment OD counters.

        Called immediately after landing_procedure() in step(). At this point
        the UAV has physically landed and its trip is complete:
          1. Fills land_step in the matching trip log entry.
          2. Increments _trips_completed_od[o_region, d_region].

        If arrive_airspace_step was never set (UAV landed without queuing),
        backfills it with land_step so that wait_time = 0 for that trip.
        '''
        for entry in reversed(self._trip_log):
            if entry['uav_id'] == uav_id and entry['land_step'] is None:
                entry['land_step'] = self._state.currentstep
                # Backfill arrive_airspace_step if UAV landed without queuing.
                if entry['arrive_airspace_step'] is None:
                    entry['arrive_airspace_step'] = self._state.currentstep
                # Increment completed trip counter for the OD region pair.
                o = entry['o_region']
                d = entry['d_region']
                if 0 <= o < self.n_regions and 0 <= d < self.n_regions:
                    self._trips_completed_od[o, d] += 1
                break

    def _accumulate_step_metrics(self):
        '''Accumulate step-level values for time-averaged metrics
        B4 (utilization), B5 (queue_length), B6 (fleet_utilization).

        pads_occupied[k]: number of UAVs currently landed at vertiport k.
          Computed as len(vertiport.uav_id_list) — UAVs in uav_id_list are
          those on the ground (not in-flight, not in landing queue).
        uavs_waiting[k]: len(vertiport.landing_queue) — UAVs hovering waiting
          for a free pad.
        uavs_in_flight: count of UAVs where uav.uav_in_flight is True.

        Step count is read from self._state.currentstep (incremented at the
        start of step() before this method is called).
        '''
        for k, vp_id in enumerate(self._vp_index_to_id):
            # O(1) lookup via _vp_id_to_vp (built in _init_demand_state).
            vertiport = self._vp_id_to_vp.get(vp_id)
            if vertiport is None:
                continue
            self._pads_occupied_sum[k] += len(vertiport.uav_id_list)
            # get_landing_queue() is a method on Vertiport that returns landing_queue.
            self._queue_length_sum[k]  += len(vertiport.get_landing_queue())

        in_flight = sum(
            1 for uav in self.atc.uav_dict.values()
            if getattr(uav, 'uav_in_flight', False)
        )
        self._uavs_in_flight_sum += in_flight

    def get_episode_metrics(self) -> Dict:
        '''Aggregate all step-level logs into Group B observation features
        for the RL environment. Call after an episode ends.

        Returns:
            dict with keys:
                pair_avg_trip_time      : np.ndarray (N, N) seconds
                pair_avg_wait_time      : np.ndarray (N, N) seconds
                pair_demand_served_ratio: np.ndarray (N, N) ratio [0, 1]
                demand_generated_od     : np.ndarray (N, N) int  (trips)
                trips_completed_od      : np.ndarray (N, N) int  (trips)
                utilization             : np.ndarray (n_vp,) ratio [0, 1]
                queue_length            : np.ndarray (n_vp,) vehicles (mean)
                fleet_utilization       : float ratio [0, 1]
                demand_served_ratio     : float ratio [0, 1]
                vp_index_to_id          : list[int]  vertiport id per index k

        Metric derivations:
          B1 pair_avg_trip_time[i,j]:
            Mean of (land_step - depart_step) * dt for completed trips on (i,j).
          B2 pair_avg_wait_time[i,j]:
            Mean of (land_step - arrive_airspace_step) * dt.
          B3 pair_demand_served_ratio[i,j]:
            trips_completed_od[i,j] / demand_gen_od[i,j].
            Set to 1.0 when demand_gen_od[i,j] == 0.
          B4 utilization[k]:
            pads_occupied_sum[k] / (step_count * pad_capacity).
          B5 queue_length[k]:
            queue_length_sum[k] / step_count.
          B6 fleet_utilization:
            uavs_in_flight_sum / (step_count * total_uavs).
          B7 demand_served_ratio:
            sum(trips_completed_od) / sum(demand_gen_od).
        '''
        N   = self.n_regions
        # n_vp documents the number of active vertiports this episode.
        # Available for shape assertions: utilization.shape == (n_vp,),
        # queue_length.shape == (n_vp,). Retained for clarity and extension.
        n_vp = len(self._vp_index_to_id)
        # Use self._state.currentstep as the step count — it is incremented at
        # the start of each step() call and matches the number of steps taken.
        T   = max(self._state.currentstep, 1)   # guard against division by zero

        # ── B1 / B2 / B3: per-OD-pair metrics from trip log ──────────────────
        trip_time_sum   = np.zeros((N, N), dtype=float)
        wait_time_sum   = np.zeros((N, N), dtype=float)
        completed_count = np.zeros((N, N), dtype=int)

        for entry in self._trip_log:
            o, d = entry['o_region'], entry['d_region']
            if not (0 <= o < N and 0 <= d < N):
                continue
            if entry['land_step'] is None:
                continue   # trip not completed — skip (counts as unserved)

            # total trip time(start VP to end VP + wait_time at end VP)
            trip_steps = entry['land_step'] - entry['depart_step']

            # wait time before landing at end VP
            wait_steps = entry['land_step'] - entry['arrive_airspace_step']

            trip_time_sum[o, d]   += trip_steps * self.dt   # seconds
            wait_time_sum[o, d]   += wait_steps * self.dt   # seconds
            completed_count[o, d] += 1

        # B1: pair_avg_trip_time — mean flight time per OD pair
        pair_avg_trip_time = np.where(
            completed_count > 0,
            trip_time_sum / np.maximum(completed_count, 1),
            #Research_Question: should 'condition y' be set to 0, or a very high number like 999_999 to indicate non-completion of trip
            0.0   # 0 flags "no completed trips" for RL env fallback
        )

        # B2: pair_avg_wait_time — mean pad wait per OD pair
        pair_avg_wait_time = np.where(
            completed_count > 0,
            wait_time_sum / np.maximum(completed_count, 1),
            #Research_Question: should 'condition y' be set to 0, or a very high number like 999_999 to indicate non-completion of trip
            0.0
        )

        # B3: pair_demand_served_ratio = completed / generated per OD pair
        pair_demand_served_ratio = np.where(
            self._demand_gen_od > 0,
            self._trips_completed_od / np.maximum(self._demand_gen_od, 1),
            #Research_Question: should 'condition y' be set to 0, or a very high number like 999_999 to indicate non-completion of trip
            1.0   # no demand on this pair → define as fully served
        )

        # ── B4: utilization — time-averaged pad occupancy / capacity ─────────
        pad_capacity = getattr(self.config.airspace, 'pad_capacity', 1)
        utilization  = self._pads_occupied_sum / (T * max(pad_capacity, 1))
        utilization  = np.clip(utilization, 0.0, 1.0)

        # ── B5: queue_length — time-averaged hovering UAVs per vertiport ─────
        queue_length = self._queue_length_sum / T

        # ── B6: fleet_utilization — fraction of fleet airborne on average ────
        total_uavs = max(len(self.atc.uav_dict), 1)
        fleet_utilization = self._uavs_in_flight_sum / (T * total_uavs)
        fleet_utilization = float(np.clip(fleet_utilization, 0.0, 1.0))

        # ── B7: demand_served_ratio — system-level aggregate ─────────────────
        total_generated = int(self._demand_gen_od.sum())
        total_completed = int(self._trips_completed_od.sum())
        demand_served_ratio = (total_completed / total_generated
                               if total_generated > 0 else 1.0)

        return {
            'pair_avg_trip_time'       : pair_avg_trip_time,
            'pair_avg_wait_time'       : pair_avg_wait_time,
            'pair_demand_served_ratio' : pair_demand_served_ratio,
            'demand_generated_od'      : self._demand_gen_od.copy(),
            'trips_completed_od'       : self._trips_completed_od.copy(),
            'utilization'              : utilization,
            'queue_length'             : queue_length,
            'fleet_utilization'        : fleet_utilization,
            'demand_served_ratio'      : demand_served_ratio,
            'vp_index_to_id'           : list(self._vp_index_to_id),
        }

    def update_vertiport_region_map(self, vertiport_region_map: Dict[int, int]):
        '''Update the vertiport→region mapping for the next episode.
        Call before reset().

        The RL agent selects a new action (different vertiports) at each MDP
        step. The vertiport_region_map changes each episode because different
        candidates may be selected. This method provides a clean entry point
        for that update before reset() is called.

        Args:
            vertiport_region_map: {vertiport_id: region_id}
        '''
        self.vertiport_region_map = vertiport_region_map
        # Invalidate cached region→vertiport inverse so _generate_demand
        # rebuilds it on the first step of the new episode.
        self._region_to_vp = None














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
