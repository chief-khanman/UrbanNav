from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

import numpy as np
from gymnasium import spaces
from pettingzoo import ParallelEnv

from uam_simulator import UAMSimulator
from component_schema import ActionType, UAVCommand, SimulatorState, RESERVED_TYPE_LEARNING
from obs_space_definitions import OBS_SPACE, get_obs_space
from utils import euclidean_distance


class UAMMultiAgentEnv(ParallelEnv):
    """
    Multi-agent PettingZoo ParallelEnv wrapping UAMSimulator.

    All UAVs configured with type_name='LEARNING' are controlled agents.
    Use a single fleet entry with count > 1 to create multiple learning UAVs:

        fleet_composition:
          - type_name: LEARNING
            count: 3          # 3 independent learning agents
            dynamics: PointMass
            controller: null
            sensor: PartialSensor
            planner: N/A
            mode: TRAIN

    Agent IDs are "agent_0", "agent_1", …, "agent_{N-1}" where N is the
    count in the LEARNING fleet entry.  Agent IDs are stable across episodes —
    the same agent name always maps to the same simulator UAV slot (sorted by
    the integer uav_id ATC assigns at reset).

    Observation, action, and reward structures mirror single_agent_gym_env.py.
    All agents share the same obs/action space (homogeneous fleet).
    """

    metadata = {"name": "uam_multi_agent_v0", "render_modes": []}

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(
        self,
        simulator: UAMSimulator,
        obs_type: str,
        n_intruder: int = 3,
        reward_type: str = 'r1',
    ):
        super().__init__()

        self.uam_simulator = simulator

        if obs_type not in OBS_SPACE:
            raise ValueError(
                f"Unknown obs_type '{obs_type}'. Valid options: {OBS_SPACE}"
            )
        self._obs_type = obs_type
        self._n_intruder = n_intruder
        self._reward_type = reward_type

        # Resolve dynamics name and agent count from the LEARNING fleet entry
        learning_entry = next(
            (e for e in simulator.config.fleet_composition
             if e.type_name == RESERVED_TYPE_LEARNING),
            None,
        )
        if learning_entry is None:
            raise ValueError(
                "No LEARNING entry found in fleet_composition. "
                "Add a type_name: LEARNING block to your config yaml."
            )
        self._dynamics_name: str = learning_entry.dynamics
        self._n_agents: int = learning_entry.count

        # PettingZoo: possible_agents must be known at construction time
        self.possible_agents: List[str] = [
            f"agent_{i}" for i in range(self._n_agents)
        ]

        # Populated by reset() — tracks which agents are still active this episode
        self.agents: List[str] = []
        # Maps "agent_i" → simulator integer uav_id (assigned in reset())
        self._agent_to_uav_id: Dict[str, int] = {}

        # Spaces — identical for all agents (homogeneous LEARNING fleet)
        _obs_space = get_obs_space(self._obs_type, self._n_intruder)
        _act_space = self._create_action_space(self._dynamics_name)
        self.observation_spaces: Dict[str, spaces.Space] = {
            a: _obs_space for a in self.possible_agents
        }
        self.action_spaces: Dict[str, spaces.Space] = {
            a: _act_space for a in self.possible_agents
        }

    # ------------------------------------------------------------------
    # PettingZoo space accessors (required by ParallelEnv API)
    # ------------------------------------------------------------------

    def observation_space(self, agent: str) -> spaces.Space:
        return self.observation_spaces[agent]

    def action_space(self, agent: str) -> spaces.Space:
        return self.action_spaces[agent]

    # ------------------------------------------------------------------
    # PettingZoo API
    # ------------------------------------------------------------------

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        if seed is not None:
            self.uam_simulator.simulator_manager.seed = seed

        self.uam_simulator.reset()

        # Discover all LEARNING UAV ids now that ATC has built the fleet
        learning_ids = self._get_learning_uav_ids()
        if len(learning_ids) != self._n_agents:
            raise RuntimeError(
                f"Expected {self._n_agents} LEARNING UAVs after reset, "
                f"found {len(learning_ids)}. "
                "Check that the LEARNING fleet entry count matches."
            )

        # Stable mapping: sort uav_ids so agent_0 always gets the lowest id
        self.agents = list(self.possible_agents)
        self._agent_to_uav_id = {
            agent: uid
            for agent, uid in zip(self.possible_agents, sorted(learning_ids))
        }

        state = self.uam_simulator.get_state()
        observations = {
            a: self._extract_observation(a, state, collisions=None)
            for a in self.agents
        }
        infos = {
            a: self._build_info(a, state, collisions=None)
            for a in self.agents
        }
        return observations, infos

    def step(
        self, actions: Dict[str, np.ndarray]
    ) -> tuple[Dict, Dict, Dict, Dict, Dict]:
        """
        Execute one environment step for all active agents simultaneously.

        Args:
            actions: Dict mapping agent_id → normalized action array.
                     Agents not present in the dict receive no command
                     (the simulator's zero-hold injection handles them).

        Returns:
            (observations, rewards, terminations, truncations, infos)
            All dicts are keyed by the agents active at the START of this step.
        """
        # Snapshot which agents are active before we modify self.agents
        current_agents = list(self.agents)

        # Build simulator command bundle from all active agents
        sim_actions: Dict = {}
        for agent in current_agents:
            action = actions.get(agent)
            if action is not None:
                sim_actions.update(self._format_action(agent, action))

        collisions = self.uam_simulator.step(sim_actions)
        state = self.uam_simulator.get_state()

        is_truncated = state.currentstep >= self.uam_simulator.total_timestep

        observations: Dict[str, np.ndarray] = {}
        rewards: Dict[str, float] = {}
        terminations: Dict[str, bool] = {}
        truncations: Dict[str, bool] = {}
        infos: Dict[str, Any] = {}

        for agent in current_agents:
            info = self._build_info(agent, state, collisions)
            observations[agent] = self._extract_observation(agent, state, collisions)
            rewards[agent] = self._compute_reward(agent, state, actions.get(agent), info)
            terminations[agent] = self._check_terminated(agent, state, collisions)
            truncations[agent] = is_truncated
            infos[agent] = info

        # Remove done agents from the active list for the next step
        self.agents = [
            a for a in current_agents
            if not terminations[a] and not truncations[a]
        ]

        return observations, rewards, terminations, truncations, infos

    def render(self) -> None:
        """Save offline animation for the current episode."""
        self.uam_simulator.render()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_learning_uav_ids(self) -> List[int]:
        """Return all simulator uav_ids tagged as RESERVED_TYPE_LEARNING."""
        return [
            uid
            for uid, uav in self.uam_simulator.simulator_manager.atc.uav_dict.items()
            if getattr(uav, 'type_name', None) == RESERVED_TYPE_LEARNING
        ]

    def _create_action_space(self, dynamics_name: str) -> spaces.Box:
        """Normalized [-1, 1]^n action Box. _format_action rescales to physical units."""
        shape_map = {
            'PointMass':            2,   # (accel_norm, yaw_rate_norm)
            'TwoDVector-Holonomic': 2,   # (ax_norm, ay_norm)
            'SixDOF':               4,   # (ax_norm, ay_norm, az_norm, yaw_rate_norm)
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

    def _format_action(self, agent: str, action: np.ndarray) -> Dict:
        """Denormalize policy action to physical units and wrap in a UAVCommandBundle."""
        uav_id = self._agent_to_uav_id[agent]
        uav = self.uam_simulator.simulator_manager.atc.uav_dict.get(uav_id)
        if uav is None:
            return {}

        max_a  = uav.max_acceleration
        max_yr = uav.max_heading_change

        if self._dynamics_name == 'PointMass':
            payload = np.array(
                [action[0] * max_a, action[1] * max_yr], dtype=np.float32
            )
        elif self._dynamics_name == 'TwoDVector-Holonomic':
            payload = np.array(
                [action[0] * max_a, action[1] * max_a], dtype=np.float32
            )
        elif self._dynamics_name == 'SixDOF':
            payload = np.array(
                [action[0] * max_a, action[1] * max_a,
                 action[2] * max_a, action[3] * max_yr],
                dtype=np.float32,
            )
        else:
            raise ValueError(f"Unknown dynamics_name '{self._dynamics_name}'.")

        return {uav_id: [UAVCommand(ActionType.CONTROL, payload)]}

    def _extract_observation(
        self,
        agent: str,
        state: SimulatorState,
        collisions,
    ) -> np.ndarray:
        """Build a flat float32 observation for the given agent's UAV."""
        uav_id = self._agent_to_uav_id[agent]
        uav = state.atc_state.get(uav_id)

        if uav is None:
            return np.zeros(self.observation_spaces[agent].shape, dtype=np.float32)

        dr = euclidean_distance(uav.mission_start_point, uav.mission_end_point)
        ms = uav.max_speed
        goal_z = uav.mission_end_point.z if uav.mission_end_point.has_z else 0.0

        self_obs: List[float] = [
            (uav.mission_end_point.x - uav.current_position.x) / dr,
            (uav.mission_end_point.y - uav.current_position.y) / dr,
            (goal_z - uav.pz) / dr,
            uav.vx / ms,
            uav.vy / ms,
            uav.vz / ms,
            math.cos(uav.current_heading),
            math.sin(uav.current_heading),
            uav.current_speed / ms,
        ]

        needs_intruders = self._obs_type in (
            'AGENT-INTRUDER', 'AGENT-N-INTRUDER',
            'AGENT-INTRUDER-RA', 'AGENT-N-INTRUDER-RA',
        )
        needs_ra = self._obs_type in (
            'AGENT-INTRUDER-RA', 'AGENT-RA', 'AGENT-N-INTRUDER-RA',
        )
        n_slots = (
            1                   if self._obs_type in ('AGENT-INTRUDER', 'AGENT-INTRUDER-RA')
            else self._n_intruder if needs_intruders
            else 0
        )

        intruder_obs: List[float] = []
        if needs_intruders:
            detected_ids: set = (
                collisions[1].get(uav_id, set())
                if collisions is not None else set()
            )
            valid_ids = [i for i in detected_ids if i in state.atc_state]

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
            filled = min(len(sorted_ids), n_slots)
            intruder_obs += [0.0] * (6 * (n_slots - filled))

        ra_obs: List[float] = []
        if needs_ra:
            ra_safe = (
                0.0
                if collisions is not None and collisions[0].get(uav_id)
                else 1.0
            )
            ra_obs = [ra_safe]

        return np.array(self_obs + intruder_obs + ra_obs, dtype=np.float32)

    def _build_info(
        self,
        agent: str,
        state: SimulatorState,
        collisions,
    ) -> Dict[str, Any]:
        uav_id = self._agent_to_uav_id[agent]
        uav = state.atc_state.get(uav_id)

        def _has(col_idx: int) -> bool:
            return bool(
                collisions is not None and collisions[col_idx].get(uav_id)
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
            'uav_id':           uav_id,
        }

    def _check_terminated(
        self,
        agent: str,
        state: SimulatorState,
        collisions,
    ) -> bool:
        uav_id = self._agent_to_uav_id[agent]

        if uav_id not in state.atc_state:
            return True  # UAV was removed by the collision system

        if collisions is not None:
            if collisions[4].get(uav_id):  # UAV-UAV physical collision
                return True
            if collisions[3].get(uav_id):  # RA physical collision
                return True

        uav = state.atc_state[uav_id]
        if uav.current_mission_complete_status:
            return True

        return False

    def _compute_reward(
        self,
        agent: str,
        state: SimulatorState,
        action: Optional[np.ndarray],
        info: Dict[str, Any],
    ) -> float:
        """
        Composable per-agent reward matching single_agent_gym_env.py:
            r1 — goal reaching + heading alignment
            r2 — speed incentive
            r3 — intruder avoidance
            r4 — RA avoidance
        Terminal collision penalty (-100) always applied on top.
        """
        uav_id = self._agent_to_uav_id[agent]
        uav = state.atc_state.get(uav_id)

        if uav is not None:
            dist_norm = info['dist_to_goal'] / euclidean_distance(
                uav.mission_start_point, uav.mission_end_point
            )
            dx = uav.mission_end_point.x - uav.current_position.x
            dy = uav.mission_end_point.y - uav.current_position.y
            heading_to_goal = math.atan2(dy, dx)
            heading_err = abs(
                math.atan2(
                    math.sin(uav.current_heading - heading_to_goal),
                    math.cos(uav.current_heading - heading_to_goal),
                )
            ) / math.pi
            r1 = -dist_norm - 0.1 * heading_err
            if info['mission_complete']:
                r1 += 10.0
        else:
            r1 = 0.0

        r2 = (uav.current_speed / uav.max_speed) if uav is not None else 0.0
        r3 = -5.0 if info['nmac'] else (-0.5 if info['uav_detected'] else 0.0)
        r4 = -10.0 if info['collision_ra'] else (-0.5 if info['ra_detected'] else 0.0)

        reward = {
            'r1':       r1,
            'r1r2':     r1 + r2,
            'r1r2r3':   r1 + r2 + r3,
            'r1r2r3r4': r1 + r2 + r3 + r4,
        }.get(self._reward_type, r1)

        if info['collision_uav']:
            reward -= 100.0
        if info['collision_ra']:
            reward -= 100.0

        return float(reward)
