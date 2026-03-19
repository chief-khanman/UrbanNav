from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from uam_simulator import UAMSimulator
from component_schema import ActionType, UAVCommand, SimulatorState, RESERVED_TYPE_LEARNING
from obs_space_definitions import OBS_SPACE, POLICY_ARCH, get_obs_space
from utils import euclidean_distance


class UAMSimEnv(gym.Env):
    """
    Single-agent Gymnasium wrapper around UAMSimulator.

    One UAV in the fleet must be configured with controller_name=None and
    type_name='LEARNING' (set in sample_config.yaml + component_schema.py).
    This env supplies that UAV's action each step and returns its observation,
    reward, and termination signal.

    The constructor only needs a simulator instance, an obs-space type string,
    and optional n_intruder / reward_type knobs.  Everything else (learning UAV
    id, dynamics name, action space shape) is resolved automatically — either
    from simulator.config at construction time or from the ATC state after
    the first call to reset().

    Args:
        simulator:   A fully-constructed (but not yet reset) UAMSimulator.
        obs_type:    One of the strings in OBS_SPACE (obs_space_definitions.py).
        n_intruder:  Number of intruder slots for 'AGENT-N-INTRUDER*' types.
                     Ignored for non-N obs types.  Default 3.
        reward_type: Composable reward mode.
                     'r1'       — goal-reaching only
                     'r1r2'     — + speed incentive
                     'r1r2r3'   — + intruder avoidance
                     'r1r2r3r4' — + RA avoidance
                     Default 'r1'.
    """

    metadata = {"render_modes": []}

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        simulator: UAMSimulator,
        obs_type: str, 
        n_intruder: int = 3, 
        reward_type: str = 'r1',
        # throw error if these three are not SET 
    ):
        super().__init__()

        self.uam_simulator: UAMSimulator = simulator

        # Observation config
        if obs_type not in OBS_SPACE:
            raise ValueError(
                f"Unknown obs_type '{obs_type}'. Valid options: {OBS_SPACE}"
            )
        self._obs_type: str = obs_type
        self._n_intruder: int = int(n_intruder)
        self._reward_type: str = reward_type

        # Derive dynamics name for the LEARNING UAV from the already-loaded config.
        # This does NOT require reset() to have been called.
        # IMPORTANT -  this structure is for SINGLE AGENT ONLY - NEED TO FORMULATE A GENERAL EFFICIENT PROCESS THAT COVERS BOTH  SINGLE AND MULTI AGENT PARADIGM 
        learning_entry = next(
            (e for e in simulator.config.fleet_composition # need to use generator - when running simulator with 1k + UAVs we can iterate throough the learning list
             if e.type_name == RESERVED_TYPE_LEARNING),
            None,
        )
        if learning_entry is None:
            raise ValueError(
                "No LEARNING entry found in fleet_composition. "
                "Add a type_name: LEARNING block to your config yaml."
            )
        self._dynamics_name: str = learning_entry.dynamics

        # Learning UAV integer id — discovered in reset() after ATC builds the fleet
        self._learning_uav_id: Optional[int] = None


        # Gymnasium spaces — set at construction time (Gymnasium requirement)
        self.observation_space = get_obs_space(self._obs_type, self._n_intruder)
        self.action_space = self._create_action_space(self._dynamics_name)

    # ------------------------------------------------------------------
    # Space definitions
    # ------------------------------------------------------------------

    def _create_action_space(self, dynamics_name: str) -> spaces.Box:
        """
        Build a normalized [-1, 1]^n action Box.
        _format_action() rescales to physical units at each step.

        PointMass            → (2,)  [accel_norm, yaw_rate_norm]
        TwoDVector-Holonomic → (2,)  [ax_norm, ay_norm]
        SixDOF               → (4,)  [ax_norm, ay_norm, az_norm, yaw_rate_norm]
        """
        shape_map = {
            'PointMass':             2,
            'TwoDVector-Holonomic':  2,
            'SixDOF':                4,
        }
        if dynamics_name not in shape_map:
            raise ValueError(
                f"Unknown dynamics_name '{dynamics_name}'. "
                f"Valid options: {list(shape_map.keys())}"
            )
        n = shape_map[dynamics_name]
        return spaces.Box(
            low=np.full(n, -1.0, dtype=np.float32),
            high=np.full(n,  1.0, dtype=np.float32),
            dtype=np.float32,
        )

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------

    def reset(
        self,
        options: Optional[Dict[str, Any]] = None,
        seed: Optional[int] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)

        if seed: 
            self.seed = seed
            print(f'UAM Simulator seed updated to use gym.env seed: {seed}')
            self.uam_simulator.simulator_manager.seed = seed

        self.uam_simulator.reset()
        


        # Discover LEARNING UAV id now that ATC has built the full fleet
        self._learning_uav_id = self.uam_simulator.simulator_manager.get_learning_uav_id()
        if self._learning_uav_id is None:
            raise RuntimeError(
                "No LEARNING UAV found in simulator after reset. "
                "Ensure fleet_composition has a type_name: LEARNING entry."
            )

        state = self.uam_simulator.get_state()
        obs   = self._extract_observation(state, collisions=None)
        info  = self._build_info(state, collisions=None)

        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        Execute one environment step.

        Args:
            action: normalized numpy array from RL policy (matches action_space)

        Returns:
            (observation, reward, terminated, truncated, info)
        """
        sim_action = self._format_action(action)
        collisions = self.uam_simulator.step(sim_action)   # 5-tuple
        state      = self.uam_simulator.get_state()
        obs        = self._extract_observation(state, collisions)
        info       = self._build_info(state, collisions)
        reward     = self._compute_reward(state, action, info)
        terminated = self._check_terminated(state, collisions)
        truncated  = (
            self.uam_simulator.simulator_manager._state.currentstep
            >= self.uam_simulator.total_timestep
        )

        return obs, reward, terminated, truncated, info

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _extract_observation(
        self,
        state: SimulatorState,
        collisions: Optional[Tuple],
    ) -> np.ndarray:
        """
        Build a flat float32 observation vector for the learning UAV.
        Returns zero-vector if the learning UAV is no longer in atc_state.
        """
        
        uav = state.atc_state.get(self._learning_uav_id, None)
        if uav is None:
            return np.zeros(self.observation_space.shape, dtype=np.float32)

        dr = euclidean_distance(uav.mission_start_point, uav.mission_end_point) # a normalizing factor 
        ms = uav.max_speed

        # ---- self obs (9 features) ----
        goal_x = uav.mission_end_point.x
        goal_y = uav.mission_end_point.y
        goal_z = uav.mission_end_point.z if uav.mission_end_point.has_z else 0.0

        self_obs: List[float] = [
            (goal_x - uav.current_position.x) / dr,
            (goal_y - uav.current_position.y) / dr,
            (goal_z - uav.pz) / dr,
            uav.vx / ms,
            uav.vy / ms,
            uav.vz / ms,
            math.cos(uav.current_heading), #TODO: placeholder for final_heading - current_heading
            math.sin(uav.current_heading),
            uav.current_speed / ms,
        ]

        # ---- determine which blocks are needed ----
        needs_intruders = self._obs_type in (
            'AGENT-INTRUDER', 'AGENT-N-INTRUDER',
            'AGENT-INTRUDER-RA', 'AGENT-N-INTRUDER-RA',
        )
        needs_ra = self._obs_type in (
            'AGENT-INTRUDER-RA', 'AGENT-RA', 'AGENT-N-INTRUDER-RA',
        )
        n_slots = (
            1                 if self._obs_type in ('AGENT-INTRUDER', 'AGENT-INTRUDER-RA')
            else self._n_intruder if needs_intruders
            else 0
        )

        # ---- intruder obs ----
        intruder_obs: List[float] = []

        if needs_intruders:
            # Detected UAV IDs for our agent (uav_detect dict = collisions[1])
            detected_ids: set = (
                collisions[1].get(self._learning_uav_id, set())
                if collisions is not None else set()
            )
            # Keep only IDs still present in atc_state
            valid_ids = [i for i in detected_ids if i in state.atc_state]

            # Sort ascending by 3D distance (closest = most dangerous)
            def _dist3d(iid: int) -> float:
                iv = state.atc_state[iid]
                return math.sqrt(
                    (iv.current_position.x - uav.current_position.x) ** 2
                    + (iv.current_position.y - uav.current_position.y) ** 2
                    + (iv.pz - uav.pz) ** 2
                )

            sorted_ids = sorted(valid_ids, key=_dist3d)

            for intruder_id in sorted_ids[:n_slots]:
                iv = state.atc_state[intruder_id]
                intruder_obs += [
                    (iv.current_position.x - uav.current_position.x) / dr,
                    (iv.current_position.y - uav.current_position.y) / dr,
                    (iv.pz - uav.pz) / dr,
                    (iv.vx - uav.vx) / ms,
                    (iv.vy - uav.vy) / ms,
                    (iv.vz - uav.vz) / ms,
                ]

            # Pad remaining slots with zeros
            filled = min(len(sorted_ids), n_slots)
            intruder_obs += [0.0] * (6 * (n_slots - filled))

        # ---- RA obs (1 feature) ----
        ra_obs: List[float] = []
        if needs_ra:
            ra_safe = (
                0.0
                if (collisions is not None
                    and collisions[0].get(self._learning_uav_id))
                else 1.0
            )
            ra_obs = [ra_safe]

        full_obs = self_obs + intruder_obs + ra_obs
        return np.array(full_obs, dtype=np.float32)

    def _format_action(self, action: np.ndarray) -> Dict[int, List[UAVCommand]]:
        """
        Denormalize the [-1,1]^n policy action to physical units and wrap in
        a UAVCommandBundle for simulator_manager.map_actions_to_uavs().
        """
        uav    = self.uam_simulator.simulator_manager.atc.uav_dict[self._learning_uav_id]
        max_a  = uav.max_acceleration
        max_yr = uav.max_heading_change

        if self._dynamics_name == 'PointMass':
            payload = np.array(
                [action[0] * max_a, action[1] * max_yr],
                dtype=np.float32,
            )
        elif self._dynamics_name == 'TwoDVector-Holonomic':
            payload = np.array(
                [action[0] * max_a, action[1] * max_a],
                dtype=np.float32,
            )
        elif self._dynamics_name == 'SixDOF':
            payload = np.array(
                [action[0] * max_a, action[1] * max_a,
                 action[2] * max_a, action[3] * max_yr],
                dtype=np.float32,
            )
        else:
            raise ValueError(
                f"Unknown dynamics_name '{self._dynamics_name}' in _format_action."
            )

        return {self._learning_uav_id: [UAVCommand(ActionType.CONTROL, payload)]}

    def _build_info(
        self,
        state: SimulatorState,
        collisions: Optional[Tuple],
    ) -> Dict[str, Any]:
        """
        Build the info dict returned by reset() and step().

        Keys:
            ra_detected      : RA sensor triggered for learning UAV
            uav_detected     : another UAV detected within detection radius
            nmac             : NMAC threshold breached
            collision_ra     : actual collision with restricted airspace
            collision_uav    : actual collision with another UAV
            mission_complete : UAV reached goal vertiport
            dist_to_goal     : 3-D Euclidean distance to mission end point (metres)
            current_step     : simulator time step
        """
        lid = self._learning_uav_id
        uav = state.atc_state.get(lid) if lid is not None else None

        def _has(col_idx: int) -> bool:
            # very terse but the meaning is -
            #   collisions - is a tuple, that contains 5 dicts - ra_deteted_dict, uav_detected_dict, nmac_dict, ra_collision_dict, uav_collision_dict
            #                using _has() function we are determining if these dicts are available
            #                they are only available if there is a learning id: lid, collisions(tuple), and we can access those dicts using learning id,
            #                <>.get(lid) return None if lid not present in dict  
            return bool(
                collisions is not None and lid is not None
                and collisions[col_idx].get(lid)
            )

        if uav is None:
            dist_3d = 0.0
        else:
            ex = uav.mission_end_point.x
            ey = uav.mission_end_point.y
            ez = uav.mission_end_point.z if uav.mission_end_point.has_z else 0.0
            dist_3d = math.sqrt(
                (uav.current_position.x - ex) ** 2
                + (uav.current_position.y - ey) ** 2
                + (uav.pz - ez) ** 2
            )

        return {
            'ra_detected':      _has(0),
            'uav_detected':     _has(1),
            'nmac':             _has(2),
            'collision_ra':     _has(3),
            'collision_uav':    _has(4),
            'mission_complete': bool(uav and uav.current_mission_complete_status),
            'dist_to_goal':     dist_3d,
            'current_step':     state.currentstep,
        }

    def _check_terminated(
        self,
        state: SimulatorState,
        collisions: Optional[Tuple],
    ) -> bool:
        """
        Episode is done (terminated) when:
          - learning UAV was removed from atc_state (collision system culled it)
          - UAV-UAV physical collision
          - RA physical collision
          - mission waypoint reached
        """
        lid = self._learning_uav_id
        
        # self collision with other uav
        if lid is None or lid not in state.atc_state:
            return True                            # never started or culled
        
        if collisions is not None:
            if collisions[4].get(lid):             # uav-uav collision
                return True
            if collisions[3].get(lid):             # ra collision
                return True

        uav = state.atc_state[lid]
        if uav.current_mission_complete_status:    # reached goal
            return True

        return False

    def _compute_reward(
        self,
        state: SimulatorState,
        action: np.ndarray,
        info: Dict[str, Any],
    ) -> float:
        """
        Composable reward with four additive components.

        r1 — goal reaching:
            -normalised_dist_to_goal - 0.1·heading_err_normalised
            +10 bonus on mission complete
        r2 — speed incentive:
            +current_speed / max_speed
        r3 — intruder avoidance:
            -5 on NMAC; -0.5 when any UAV detected
        r4 — RA avoidance:
            -10 on RA collision; -0.5 when RA detected

        reward_type composition:
            'r1'       → r1
            'r1r2'     → r1 + r2
            'r1r2r3'   → r1 + r2 + r3
            'r1r2r3r4' → r1 + r2 + r3 + r4

        Terminal collision penalties (-100) are always applied on top,
        regardless of reward_type.
        """
        uav = state.atc_state.get(self._learning_uav_id) if self._learning_uav_id is not None else None

        # ---- r1: goal reaching ----
        if uav is not None:
            dist_norm = info['dist_to_goal'] / euclidean_distance(uav.mission_start_point, uav.mission_end_point)
            dx = uav.mission_end_point.x - uav.current_position.x
            dy = uav.mission_end_point.y - uav.current_position.y
            heading_to_goal = math.atan2(dy, dx)
            heading_err = abs(
                math.atan2(
                    math.sin(uav.current_heading - heading_to_goal),
                    math.cos(uav.current_heading - heading_to_goal),
                )
            ) / math.pi   # normalised to [0, 1]

            r1 = -dist_norm - 0.1 * heading_err
            if info['mission_complete']:
                r1 += 10.0
        else:
            r1 = 0.0

        # ---- r2: speed incentive ----
        r2 = (uav.current_speed / uav.max_speed) if uav is not None else 0.0

        # ---- r3: intruder avoidance ----
        if info['nmac']:
            r3 = -5.0
        elif info['uav_detected']:
            r3 = -0.5
        else:
            r3 = 0.0

        # ---- r4: RA avoidance ----
        if info['collision_ra']:
            r4 = -10.0
        elif info['ra_detected']:
            r4 = -0.5
        else:
            r4 = 0.0

        # Compose
        reward = {
            'r1':       r1,
            'r1r2':     r1 + r2,
            'r1r2r3':   r1 + r2 + r3,
            'r1r2r3r4': r1 + r2 + r3 + r4,
        }.get(self._reward_type, r1)

        # Terminal collision penalties — always applied regardless of reward_type
        if info['collision_uav']:
            reward -= 100.0
        if info['collision_ra']:
            reward -= 100.0

        return float(reward)
