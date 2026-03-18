from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from uam_simulator import UAMSimulator
from component_schema import ActionType, UAVCommand, SimulatorState


class UAMSimEnv(gym.Env):
    """
    Single-agent Gymnasium wrapper around UAMSimulator.

    One UAV in the fleet must be configured with controller_name=None and
    type_name='LEARNING'.  This env supplies that UAV's action each step and
    returns its observation, reward, and termination signal.

    Args:
        simulator:           A fully-constructed (but not yet reset) UAMSimulator.
        controlled_uav_ids:  List[int] — seed IDs; index-0 is the primary learning
                             UAV.  The env re-discovers the true ID after each reset
                             because ATC rebuilds IDs from 0 every episode.
        observation_config:  Dict with keys:
                               obs_type  : 'goal_only' | 'single_intruder' |
                                           'n_intruder' | 'single_intruder_ra' |
                                           'n_intruder_ra'
                               n_intruder: int  (default 3, used by n_intruder* types)
        action_config:       Dict with keys:
                               dynamics_name: 'PointMass' | 'TwoDVector-Holonomic' |
                                              'SixDOF'
                               reward_type  : 'r1' | 'r1r2' | 'r1r2r3' | 'r1r2r3r4'
                                              (default 'r1')
    """

    metadata = {"render_modes": []}

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        simulator: UAMSimulator,
        #TODO: remove the arguments below 
        controlled_uav_ids: List[int], 
        observation_config: Dict[str, Any], #collect this from UAM simulator config
        action_config: Dict[str, Any], #collect this from UAM simulator config
    ):
        super().__init__()

        self.uam_simulator: UAMSimulator = simulator
        
        # TODO: move this to reset - reset in UAMSimulator creates the UAVs, collect LEARNING UAV id after simulator reset 
        self.controlled_uav_ids: List[int] = controlled_uav_ids 
        # Primary learning UAV id (re-resolved after every reset)
        self._learning_uav_id: int = controlled_uav_ids[0]
        
        self._last_collisions: Optional[Tuple] = None

        # Observation config
        self._obs_type: str = observation_config.get("obs_type", "goal_only")
        self._n_intruder: int = int(observation_config.get("n_intruder", 3))

        # Action config
        self._dynamics_name: str = action_config.get("dynamics_name", "PointMass")
        self._reward_type: str = action_config.get("reward_type", "r1")

        # Gymnasium spaces
        self.observation_space = self._create_observation_space(observation_config)
        self.action_space = self._create_action_space(action_config)

    # ------------------------------------------------------------------
    # Space definitions
    # ------------------------------------------------------------------

    def _create_observation_space(self, config: Dict[str, Any]) -> spaces.Box:
        """
        Build the observation Box based on obs_type.

        Self obs  — 9 features:
            [dx_goal/dr, dy_goal/dr, dz_goal/dr,
             vx/ms, vy/ms, vz/ms,
             cos(yaw), sin(yaw), speed/ms]

        Per-intruder obs — 6 features each:
            [rel_x/dr, rel_y/dr, rel_z/dr, dvx/ms, dvy/ms, dvz/ms]

        RA obs — 1 feature:
            0.0 = RA detected (danger), 1.0 = safe

        obs_type shapes:
            goal_only          →  9
            single_intruder    → 15
            n_intruder         →  9 + 6·N
            single_intruder_ra → 16
            n_intruder_ra      →  9 + 6·N + 1
        """
        obs_type = config.get("obs_type", "goal_only")
        n = int(config.get("n_intruder", 3))

        shape_map = {
            "goal_only":          9,
            "single_intruder":    15,
            "n_intruder":         9 + 6 * n,
            "single_intruder_ra": 16,
            "n_intruder_ra":      9 + 6 * n + 1,
        }

        if obs_type not in shape_map:
            raise ValueError(
                f"Unknown obs_type '{obs_type}'. "
                f"Valid options: {list(shape_map.keys())}"
            )

        n_features = shape_map[obs_type]
        return spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(n_features,),
            dtype=np.float32,
        )

    def _create_action_space(self, config: Dict[str, Any]) -> spaces.Box:
        """
        All actions normalized to [-1, 1]^n.
        _format_action() rescales them to physical units.

        PointMass          → (2,)  [accel_norm, yaw_rate_norm]
        TwoDVector-Holonomic → (2,) [ax_norm, ay_norm]
        SixDOF             → (4,)  [ax_norm, ay_norm, az_norm, yaw_rate_norm]
        """
        dynamics = config.get("dynamics_name", "PointMass")
        shape_map = {
            "PointMass":             (2,),
            "TwoDVector-Holonomic":  (2,),
            "SixDOF":                (4,),
        }
        if dynamics not in shape_map:
            raise ValueError(
                f"Unknown dynamics_name '{dynamics}'. "
                f"Valid options: {list(shape_map.keys())}"
            )
        n = shape_map[dynamics][0]
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
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)

        self.uam_simulator.reset()
        self._last_collisions = None

        # TODO: collect learning uav id here
        # ATC rebuilds UAV IDs from 0 each episode — re-discover learning UAV
        self._refresh_learning_uav_id()

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
            action: normalized numpy array from RL policy

        Returns:
            (observation, reward, terminated, truncated, info)
        """
        sim_action = self._format_action(action)
        collisions = self.uam_simulator.step(sim_action)   # 5-tuple
        self._last_collisions = collisions

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

    def _refresh_learning_uav_id(self) -> None:
        """
        Re-discover the LEARNING UAV id from the freshly-reset ATC state.
        ATC increments uav_id_index from 0 each reset, so the id may differ
        between episodes if fleet composition changes.  Falls back to
        controlled_uav_ids[0] if no LEARNING UAV is found.
        """
        state = self.uam_simulator.get_state()
        for uid, uav in state.atc_state.items():
            if getattr(uav, "type_name", None) == "LEARNING":
                self._learning_uav_id = uid
                return
        # Fallback: use seed value supplied at construction
        self._learning_uav_id = self.controlled_uav_ids[0]

    def _extract_observation(
        self,
        state: SimulatorState,
        collisions: Optional[Tuple],
    ) -> np.ndarray:
        """
        Build a flat float32 observation vector for the learning UAV.

        Returns zero-vector if the learning UAV is no longer in atc_state.
        """
        uav = state.atc_state.get(self._learning_uav_id)
        if uav is None:
            return np.zeros(self.observation_space.shape, dtype=np.float32)

        dr = uav.detection_radius
        ms = uav.max_speed

        # ---- self obs (9 features) ----
        goal_x = uav.mission_end_point.x
        goal_y = uav.mission_end_point.y
        goal_z = uav.mission_end_point.z if uav.mission_end_point.has_z else 0.0

        dx = (goal_x - uav.current_position.x) / dr
        dy = (goal_y - uav.current_position.y) / dr
        dz = (goal_z - uav.pz) / dr

        self_obs: List[float] = [
            dx, dy, dz,
            uav.vx / ms, uav.vy / ms, uav.vz / ms,
            math.cos(uav.current_heading),
            math.sin(uav.current_heading),
            uav.current_speed / ms,
        ]

        # ---- intruder obs ----
        intruder_obs: List[float] = []

        obs_type = self._obs_type
        needs_intruders = obs_type in (
            "single_intruder", "n_intruder",
            "single_intruder_ra", "n_intruder_ra",
        )

        if needs_intruders:
            n_slots = 1 if obs_type in ("single_intruder", "single_intruder_ra") \
                      else self._n_intruder

            # Detected UAV IDs for our agent (from uav_detect dict)
            detected_ids: set = (
                collisions[1].get(self._learning_uav_id, set())
                if collisions is not None else set()
            )
            # Keep only IDs still present in atc_state
            valid_ids = [i for i in detected_ids if i in state.atc_state]

            # Sort by ascending 3D distance to learning UAV (closest = most dangerous)
            def _dist(iid: int) -> float:
                iv = state.atc_state[iid]
                return math.sqrt(
                    (iv.current_position.x - uav.current_position.x) ** 2
                    + (iv.current_position.y - uav.current_position.y) ** 2
                    + (iv.pz - uav.pz) ** 2
                )

            sorted_ids = sorted(valid_ids, key=_dist)

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
            filled = len(sorted_ids[:n_slots])
            intruder_obs += [0.0] * (6 * (n_slots - filled))

        # ---- RA obs (1 feature) ----
        ra_obs: List[float] = []
        if obs_type in ("single_intruder_ra", "n_intruder_ra"):
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
        Denormalize the [-1,1]^n policy action to physical units and wrap it
        in a UAVCommandBundle expected by simulator_manager.map_actions_to_uavs().
        """
        uav = self.uam_simulator.simulator_manager.atc.uav_dict[self._learning_uav_id]
        max_a  = uav.max_acceleration
        max_yr = uav.max_heading_change

        if self._dynamics_name == "PointMass":
            payload = np.array(
                [action[0] * max_a, action[1] * max_yr],
                dtype=np.float32,
            )
        elif self._dynamics_name == "TwoDVector-Holonomic":
            payload = np.array(
                [action[0] * max_a, action[1] * max_a],
                dtype=np.float32,
            )
        elif self._dynamics_name == "SixDOF":
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

        Fields:
            ra_detected     : RA sensor triggered for learning UAV
            uav_detected    : another UAV detected within detection radius
            nmac            : NMAC threshold breached
            collision_ra    : actual collision with restricted airspace
            collision_uav   : actual collision with another UAV
            mission_complete: UAV reached goal
            dist_to_goal    : 3-D Euclidean distance to mission end point (metres)
            current_step    : simulator time step
        """
        lid = self._learning_uav_id
        uav = state.atc_state.get(lid)

        def _has(col_idx: int) -> bool:
            return bool(
                collisions is not None and collisions[col_idx].get(lid)
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
            "ra_detected":      _has(0),
            "uav_detected":     _has(1),
            "nmac":             _has(2),
            "collision_ra":     _has(3),
            "collision_uav":    _has(4),
            "mission_complete": bool(uav and uav.current_mission_complete_status),
            "dist_to_goal":     dist_3d,
            "current_step":     state.currentstep,
        }

    def _check_terminated(
        self,
        state: SimulatorState,
        collisions: Optional[Tuple],
    ) -> bool:
        """
        Episode is done (terminated) when:
          - learning UAV was removed from atc_state (collision system culled it), OR
          - UAV-UAV physical collision, OR
          - RA physical collision, OR
          - mission waypoint reached.
        """
        lid = self._learning_uav_id

        if lid not in state.atc_state:
            return True                          # removed by collision system

        if collisions is not None:
            if collisions[4].get(lid):           # uav-uav collision
                return True
            if collisions[3].get(lid):           # ra collision
                return True

        uav = state.atc_state[lid]
        if uav.current_mission_complete_status:  # reached goal
            return True

        return False

    def _compute_reward(
        self,
        state: SimulatorState,
        action: np.ndarray,
        info: Dict[str, Any],
    ) -> float:
        """
        Composable reward function with four additive components.

        r1 — goal reaching:
            penalise normalised distance to goal + heading misalignment;
            bonus +10 on mission complete.

        r2 — speed incentive:
            reward for flying fast (normalised current_speed).

        r3 — intruder avoidance:
            -5 on NMAC; -0.5 when any UAV is detected.

        r4 — RA avoidance:
            -10 on RA collision; -0.5 when RA detected.

        Composite modes (reward_type):
            'r1'       → r1
            'r1r2'     → r1 + r2
            'r1r2r3'   → r1 + r2 + r3
            'r1r2r3r4' → r1 + r2 + r3 + r4

        Terminal collision penalties (always applied on top):
            collision_uav → -100
            collision_ra  → -100
        """
        uav = state.atc_state.get(self._learning_uav_id)

        # ---- r1: goal reaching ----
        if uav is not None:
            dist_norm = info["dist_to_goal"] / uav.detection_radius

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
            if info["mission_complete"]:
                r1 += 10.0
        else:
            r1 = 0.0

        # ---- r2: speed incentive ----
        r2 = (uav.current_speed / uav.max_speed) if uav is not None else 0.0

        # ---- r3: intruder avoidance ----
        if info["nmac"]:
            r3 = -5.0
        elif info["uav_detected"]:
            r3 = -0.5
        else:
            r3 = 0.0

        # ---- r4: RA avoidance ----
        if info["collision_ra"]:
            r4 = -10.0
        elif info["ra_detected"]:
            r4 = -0.5
        else:
            r4 = 0.0

        # Compose according to reward_type
        reward = {
            "r1":       r1,
            "r1r2":     r1 + r2,
            "r1r2r3":   r1 + r2 + r3,
            "r1r2r3r4": r1 + r2 + r3 + r4,
        }.get(self._reward_type, r1)

        # Terminal collision penalties — always applied regardless of reward_type
        if info["collision_uav"]:
            reward -= 100.0
        if info["collision_ra"]:
            reward -= 100.0

        return float(reward)
