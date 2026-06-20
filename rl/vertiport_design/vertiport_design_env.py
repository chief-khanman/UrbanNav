"""
vertiport_design_env.py
========================
Single-agent Gymnasium env for the vertiport-placement RL problem.

Action: one discrete pick per region — which candidate vertiport in that
region becomes the selected vertiport for this design step.

Observation: graph representation of the vertiport configuration (see
GraphBuilder + VP_OBS_SPACE['GRAPH']).

Reward: distance-based dense improvement — `previous_total_distance -
current_total_distance` between consecutive design steps. Working baseline
from the user's prior validated GNN-RL code; metric-based reward composition
from `SimulatorManager.get_episode_metrics()` is a follow-on.

Inner-sim driver: `simulator.step({})` (empty command bundle) for
`simulator_step` ticks per design step — internal controllers + demand
model run the inner sim; the RL agent only chooses vertiport placement, not
UAV control.

Reset flow: the airspace + regions_dict are built ONCE during __init__.
`simulator.reset(rebuild_airspace=False)` is used every design step so the
selected vertiports survive and the OSM/synthetic region build is not
repeated (see SimulatorManager.reset() docstring).
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from gymnasium import Env

from urbannav.uam_simulator import UAMSimulator

from rl.vertiport_design.graph_builder import GraphBuilder
from rl.vertiport_design.vp_action_space_definitions import get_vp_action_space
from rl.vertiport_design.vp_obs_space_definitions import get_vp_obs_space


class VertiportDesignEnv(Env):
    """Gymnasium env for vertiport placement via per-region discrete picks."""

    metadata = {"render_modes": [], "name": "uam_vertiport_design_v0"}

    def __init__(
        self,
        config_path: str,
        simulator_step: int = 3,
        total_episode_timestep: int = 100,
        obs_type: str = 'GRAPH',
        reward_type: str = 'distance_improvement',
        region_mode: str = 'synthetic',
        region_kwargs: Optional[Dict[str, Any]] = None,
    ):
        """
        Args:
            config_path:            path to a UAMConfig yaml for the inner sim.
                                    For the demand-driven inner loop, the yaml
                                    should be wired with the OD-matrix CLI of
                                    UAMSimulator (od_matrix_path /
                                    zone_region_map_path); without it the
                                    inner sim still runs but uses random
                                    mission assignment.
            simulator_step:         inner sim ticks per RL design step.
            total_episode_timestep: design steps before truncation.
            obs_type:               key into VP_OBS_SPACE (default 'GRAPH').
            reward_type:            'distance_improvement' is the only mode
                                    implemented.
            region_mode:            'synthetic' (default, OSM-free, fast) or
                                    'osm' (uses make_regions_dict, requires
                                    network).
            region_kwargs:          forwarded to the chosen region-builder.
                                    Defaults: synthetic -> {} (uses method
                                    defaults). osm -> {'tag_str':'commercial',
                                    'num_regions':4}.
        """
        super().__init__()

        self.config_path = config_path
        self.simulator_step = int(simulator_step)
        self.total_episode_timestep = int(total_episode_timestep)
        self.obs_type = obs_type
        self.reward_type = reward_type
        self.region_mode = region_mode

        # ── Bootstrap: build the inner sim + regions ONCE ───────────────────
        # The expensive OSM fetch (when applicable) happens here, never inside
        # the RL design loop. reset() and step() reuse the airspace via
        # simulator.reset(rebuild_airspace=False).
        self.uam_simulator = UAMSimulator(config_path=self.config_path)
        # First reset builds airspace + ATC + a random vertiport_list.
        self.uam_simulator.reset()

        airspace = self.uam_simulator.simulator_manager.airspace
        if region_mode == 'synthetic':
            airspace.make_regions_dict_synthetic(**(region_kwargs or {}))
        elif region_mode == 'osm':
            kwargs = region_kwargs or {'tag_str': 'commercial', 'num_regions': 4}
            airspace.make_regions_dict(**kwargs)
        else:
            raise ValueError(
                f"Unknown region_mode '{region_mode}'. Use 'synthetic' or 'osm'."
            )

        self.graph_builder = GraphBuilder(airspace=airspace, connectivity_type='full')

        # Action / observation spaces resolved against the just-built regions.
        # The obs space uses the GraphBuilder's candidate count, NOT
        # len(airspace.vertiport_list) — the latter includes any pre-existing
        # random vertiports from the bootstrap reset, while GraphBuilder only
        # ever traverses regions_dict candidates.
        self.action_space = get_vp_action_space('PER_REGION_DISCRETE', airspace)
        num_vertiports = len(self.graph_builder.vertiport_to_idx)
        self.num_regions = airspace.num_regions
        self.observation_space = get_vp_obs_space(
            obs_type, num_vertiports=num_vertiports, num_regions=self.num_regions,
        )

        # Per-episode state — initialised in reset().
        self.current_time_step: int = 0
        self.current_selected_vertiports: Optional[List] = None
        self.previous_selected_vertiports: Optional[List] = None
        self.current_total_distance: Optional[float] = None
        self.previous_total_distance: Optional[float] = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _calculate_total_distance(self, selected_vertiports: List) -> float:
        """Sum of pairwise Euclidean distances between selected vertiports."""
        total = 0.0
        for i, vp_i in enumerate(selected_vertiports):
            for j, vp_j in enumerate(selected_vertiports):
                if i < j:
                    total += self.graph_builder.compute_distance(vp_i, vp_j)
        return float(total)

    def _build_observation(self, action: np.ndarray) -> Dict[str, np.ndarray]:
        """Build the 'GRAPH' obs Dict from current_selected_vertiports + action."""
        x, edge_index, edge_attr = self.graph_builder.build_graph(
            self.current_selected_vertiports
        )
        return {
            'node_feat': x.numpy().astype(np.float32),
            'edge_index': edge_index.numpy().astype(np.int64),
            'edge_attr': edge_attr.numpy().astype(np.float32),
            'selected_actions': np.asarray(action, dtype=np.int64),
        }

    def _apply_selection_and_inner_reset(self, selected_vertiports: List):
        """Push the selection into the airspace and soft-reset the inner sim."""
        airspace = self.uam_simulator.simulator_manager.airspace
        airspace.set_vertiport_list_vp_design(selected_vertiports)
        self.uam_simulator.reset(rebuild_airspace=False)

    def _run_inner_sim(self):
        """Tick the inner sim simulator_step times with an empty command bundle.
        Internal controllers + (when configured) demand model run the inner sim;
        no random UAV-control actions, no per-step RL command injection."""
        for _ in range(self.simulator_step):
            self.uam_simulator.step({})

    def _compute_reward(self) -> float:
        """Distance-based dense improvement reward.
        previous_total_distance - current_total_distance.
        Returns 0 on the first design step after reset()."""
        if self.previous_total_distance is None:
            return 0.0
        return float(self.previous_total_distance - self.current_total_distance)

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        super().reset(seed=seed)
        self.current_time_step = 0

        # Initial selection: first candidate per region.
        airspace = self.uam_simulator.simulator_manager.airspace
        self.current_selected_vertiports = [
            airspace.regions_dict[r][0] for r in sorted(airspace.regions_dict.keys())
        ]
        self.previous_selected_vertiports = None
        self.previous_total_distance = None

        # initial_action selects index 0 in every region (matches the initial
        # selection above; agent sees this in the obs).
        initial_action = np.zeros(self.num_regions, dtype=np.int64)

        self._apply_selection_and_inner_reset(self.current_selected_vertiports)
        self._run_inner_sim()

        self.current_total_distance = self._calculate_total_distance(
            self.current_selected_vertiports
        )

        obs = self._build_observation(initial_action)
        info = {
            'selected_vertiports': self.current_selected_vertiports,
            'initial_total_distance': self.current_total_distance,
            'current_total_distance': self.current_total_distance,
            'reward': 0.0,
            'timestep': self.current_time_step,
        }
        return obs, info

    def step(
        self, action: np.ndarray
    ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
        self.current_time_step += 1

        self.previous_selected_vertiports = list(self.current_selected_vertiports)
        self.previous_total_distance = self.current_total_distance

        new_vp_list = self.graph_builder.region_idx_to_vertiport(action)
        self.current_selected_vertiports = new_vp_list

        self._apply_selection_and_inner_reset(new_vp_list)
        self._run_inner_sim()

        self.current_total_distance = self._calculate_total_distance(new_vp_list)
        reward = self._compute_reward()

        obs = self._build_observation(action)

        terminated = False
        truncated = self.current_time_step >= self.total_episode_timestep

        info = {
            'selected_vertiports': self.current_selected_vertiports,
            'previous_total_distance': self.previous_total_distance,
            'current_total_distance': self.current_total_distance,
            'reward': reward,
            'timestep': self.current_time_step,
        }
        return obs, reward, terminated, truncated, info

    def render(self) -> None:
        return None

    def close(self) -> None:
        pass
