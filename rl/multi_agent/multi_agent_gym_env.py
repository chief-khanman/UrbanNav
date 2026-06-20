from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from pettingzoo.utils.env import ParallelEnv

from urbannav.uam_simulator import UAMSimulator
from urbannav.component_schema import RESERVED_TYPE_MULTI_AGENT_LEARNING, build_fleet_blueprint

from rl.common.obs_space_definitions import OBS_SPACE
from rl.common import agent_logic


class UAMMultiAgentEnv(ParallelEnv):
    """
    Multi-agent PettingZoo ParallelEnv wrapper around UAMSimulator.

    All UAVs whose fleet_composition entry has type_name='MULTI_AGENT_LEARNING'
    are exposed as agents. Agents sharing one policy_id (see agent_policy_id)
    are meant to be trained/deployed as one shared policy by the caller — this
    env itself doesn't enforce parameter sharing, it just reports the grouping.

    Per-agent obs/action/reward/termination logic is shared with
    rl/single_agent/single_agent_gym_env.py via rl/common/agent_logic.py.

    PettingZoo agent ids are the *blueprint* string ids (e.g.
    'MULTI_AGENT_LEARNING_2'), which are stable across resets — unlike the
    runtime integer uav_id ATC assigns each reset(), which is only discovered
    after the fleet is rebuilt.
    """

    metadata = {"render_modes": [], "name": "uam_multi_agent_v0"}

    def __init__(
        self,
        simulator: UAMSimulator,
        obs_type: str,
        n_intruder: int = 3,
        reward_type: str = 'r1',
    ):
        self.uam_simulator = simulator

        if obs_type not in OBS_SPACE:
            raise ValueError(
                f"Unknown obs_type '{obs_type}'. Valid options: {OBS_SPACE}"
            )
        self._obs_type = obs_type
        self._n_intruder = int(n_intruder)
        self._reward_type = reward_type

        ma_blueprints = [
            b for b in build_fleet_blueprint(simulator.config)
            if b.type_name == RESERVED_TYPE_MULTI_AGENT_LEARNING
        ]
        if not ma_blueprints:
            raise ValueError(
                "No MULTI_AGENT_LEARNING entries found in fleet_composition. "
                "Add one or more type_name: MULTI_AGENT_LEARNING blocks to your config yaml."
            )

        self.possible_agents: List[str] = [b.uav_id for b in ma_blueprints]
        self.agents: List[str] = []

        # Static per-agent metadata, resolved once from config.
        self._dynamics_by_agent: Dict[str, str] = {
            b.uav_id: b.dynamics_name for b in ma_blueprints
        }
        self.agent_policy_id: Dict[str, str] = {
            b.uav_id: b.policy_id for b in ma_blueprints
        }

        # agent (blueprint string id) -> runtime int uav_id, rediscovered each reset().
        self._agent_to_uav_id: Dict[str, int] = {}

        self._observation_spaces = {
            agent: agent_logic.create_observation_space(self._obs_type, self._n_intruder)
            for agent in self.possible_agents
        }
        self._action_spaces = {
            agent: agent_logic.create_action_space(self._dynamics_by_agent[agent])
            for agent in self.possible_agents
        }

    # PettingZoo requires these to return the SAME object on every call for a
    # given agent (so e.g. action_space seeding works as expected).
    def observation_space(self, agent: str):
        return self._observation_spaces[agent]

    def action_space(self, agent: str):
        return self._action_spaces[agent]

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Dict]]:
        if seed is not None:
            self.uam_simulator.simulator_manager.seed = seed

        self.uam_simulator.reset()

        # ATC assigns runtime int ids in the same order build_fleet_blueprint()
        # produced the blueprints, so zipping the two (both type-filtered,
        # iteration-order-preserving) ids them correctly.
        runtime_ma_ids = [
            uid for uid, uav in self.uam_simulator.simulator_manager.atc.uav_dict.items()
            if getattr(uav, 'type_name', None) == RESERVED_TYPE_MULTI_AGENT_LEARNING
        ]
        if len(runtime_ma_ids) != len(self.possible_agents):
            raise RuntimeError(
                f"Expected {len(self.possible_agents)} MULTI_AGENT_LEARNING UAVs "
                f"after reset, found {len(runtime_ma_ids)}."
            )
        self._agent_to_uav_id = dict(zip(self.possible_agents, runtime_ma_ids))
        self.agents = list(self.possible_agents)

        state = self.uam_simulator.get_state()
        obs = {
            agent: agent_logic.extract_observation(
                state, None, self._agent_to_uav_id[agent], self._obs_type, self._n_intruder
            )
            for agent in self.agents
        }
        infos = {
            agent: agent_logic.build_info(state, None, self._agent_to_uav_id[agent])
            for agent in self.agents
        }
        return obs, infos

    def step(
        self, actions: Dict[str, np.ndarray]
    ) -> Tuple[
        Dict[str, np.ndarray], Dict[str, float], Dict[str, bool], Dict[str, bool], Dict[str, Dict]
    ]:
        uav_dict = self.uam_simulator.simulator_manager.atc.uav_dict

        sim_action: Dict[int, List] = {}
        for agent, action in actions.items():
            uav_id = self._agent_to_uav_id[agent]
            sim_action.update(
                agent_logic.format_action(action, uav_id, self._dynamics_by_agent[agent], uav_dict)
            )

        collisions = self.uam_simulator.step(sim_action)
        state = self.uam_simulator.get_state()

        truncated_all = (
            self.uam_simulator.simulator_manager._state.currentstep
            >= self.uam_simulator.total_timestep
        )

        obs, rewards, terminations, truncations, infos = {}, {}, {}, {}, {}
        for agent in self.agents:
            uav_id = self._agent_to_uav_id[agent]
            obs[agent] = agent_logic.extract_observation(
                state, collisions, uav_id, self._obs_type, self._n_intruder
            )
            infos[agent] = agent_logic.build_info(state, collisions, uav_id)
            rewards[agent] = agent_logic.compute_reward(
                state, infos[agent], uav_id, self._reward_type
            )
            terminations[agent] = agent_logic.check_terminated(state, collisions, uav_id)
            truncations[agent] = truncated_all

        # PettingZoo ParallelEnv convention: drop agents that terminated/truncated
        # this step from self.agents so the next step()/training loop stops
        # supplying them actions.
        self.agents = [
            agent for agent in self.agents
            if not (terminations[agent] or truncations[agent])
        ]

        return obs, rewards, terminations, truncations, infos

    def render(self) -> None:
        return None

    def close(self) -> None:
        pass
