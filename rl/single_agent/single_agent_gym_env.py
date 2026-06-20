from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

import numpy as np
import gymnasium as gym

from urbannav.uam_simulator import UAMSimulator
from urbannav.component_schema import RESERVED_TYPE_SINGLE_AGENT_LEARNING

from rl.common.obs_space_definitions import OBS_SPACE
from rl.common import agent_logic


class UAMSimEnv(gym.Env):
    """
    Single-agent Gymnasium wrapper around UAMSimulator.

    One UAV in the fleet must be configured with
    type_name='SINGLE_AGENT_LEARNING' (set in sample_config.yaml + component_schema.py).
    This env supplies that UAV's action each step and returns its observation,
    reward, and termination signal.

    Per-agent obs/action/reward/termination logic lives in rl/common/agent_logic.py,
    shared with rl/multi_agent/multi_agent_gym_env.py — this class is just the
    single-agent Gymnasium-shaped wiring around it.

    Args:
        simulator:   A fully-constructed (but not yet reset) UAMSimulator.
        obs_type:    One of the strings in OBS_SPACE (rl/common/obs_space_definitions.py).
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
    ):
        super().__init__()

        self.uam_simulator: UAMSimulator = simulator

        if obs_type not in OBS_SPACE:
            raise ValueError(
                f"Unknown obs_type '{obs_type}'. Valid options: {OBS_SPACE}"
            )
        self._obs_type: str = obs_type
        self._n_intruder: int = int(n_intruder)
        self._reward_type: str = reward_type

        # Derive dynamics name for the LEARNING UAV from the already-loaded config.
        # This does NOT require reset() to have been called.
        learning_entry = next(
            (e for e in simulator.config.fleet_composition
             if e.type_name == RESERVED_TYPE_SINGLE_AGENT_LEARNING),
            None,
        )
        if learning_entry is None:
            raise ValueError(
                "No SINGLE_AGENT_LEARNING entry found in fleet_composition. "
                "Add a type_name: SINGLE_AGENT_LEARNING block to your config yaml."
            )
        self._dynamics_name: str = learning_entry.dynamics

        # Learning UAV integer id — discovered in reset() after ATC builds the fleet
        self._learning_uav_id: Optional[int] = None

        # Gymnasium spaces — set at construction time (Gymnasium requirement)
        self.observation_space = agent_logic.create_observation_space(self._obs_type, self._n_intruder)
        self.action_space = agent_logic.create_action_space(self._dynamics_name)

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
                "No SINGLE_AGENT_LEARNING UAV found in simulator after reset. "
                "Ensure fleet_composition has a type_name: SINGLE_AGENT_LEARNING entry."
            )

        state = self.uam_simulator.get_state()
        obs = agent_logic.extract_observation(
            state, None, self._learning_uav_id, self._obs_type, self._n_intruder
        )
        info = agent_logic.build_info(state, None, self._learning_uav_id)

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
        sim_action = agent_logic.format_action(
            action, self._learning_uav_id, self._dynamics_name,
            self.uam_simulator.simulator_manager.atc.uav_dict,
        )
        collisions = self.uam_simulator.step(sim_action)   # 5-tuple
        state = self.uam_simulator.get_state()
        obs = agent_logic.extract_observation(
            state, collisions, self._learning_uav_id, self._obs_type, self._n_intruder
        )
        info = agent_logic.build_info(state, collisions, self._learning_uav_id)
        reward = agent_logic.compute_reward(state, info, self._learning_uav_id, self._reward_type)
        terminated = agent_logic.check_terminated(state, collisions, self._learning_uav_id)
        truncated = (
            self.uam_simulator.simulator_manager._state.currentstep
            >= self.uam_simulator.total_timestep
        )

        return obs, reward, terminated, truncated, info
