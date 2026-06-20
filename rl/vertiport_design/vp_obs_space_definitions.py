"""
vp_obs_space_definitions.py
============================
Observation space definitions for VertiportDesignEnv.

Mirrors the OBS_SPACE / get_obs_space pattern in
rl/common/obs_space_definitions.py.

'GRAPH' is a direct port of the user's previously-validated GNN-RL obs space
(node_feat / edge_index / edge_attr / selected_actions), simplified per that
code's own TODOs to just [is_selected, normalized_x, normalized_y] node
features and [normalized_distance, both_endpoints_selected] edge features —
dropping the edge-type one-hot and placeholder flow-metric features, which
were never wired to anything besides constants.

'METRICS' and 'GRAPH-METRICS' are stubs until reward/obs composition from
SimulatorManager.get_episode_metrics() is designed (deferred — see plan).
"""

from __future__ import annotations

from typing import List

import numpy as np
from gymnasium import spaces

VP_OBS_SPACE: List[str] = [
    'GRAPH',          # node_feat/edge_index/edge_attr/selected_actions — implemented
    'METRICS',        # flat Box from get_episode_metrics() — NotImplementedError stub
    'GRAPH-METRICS',  # Dict of the two above — NotImplementedError stub
]

# Feature dimensions for 'GRAPH' obs — fixed by the simplified feature set
# described above (see GraphBuilder._build_node_features / _compute_edge_features).
NODE_FEAT_DIM = 3   # [is_selected, normalized_x, normalized_y]
EDGE_FEAT_DIM = 2   # [normalized_distance, both_endpoints_selected]


def obs_space_graph(num_vertiports: int, num_regions: int) -> spaces.Dict:
    """
    Graph-structured obs Dict for the 'GRAPH' obs_type.

    Args:
        num_vertiports: total candidate vertiports across all regions
                         (len(airspace.vertiport_list) after make_regions_dict).
        num_regions:    number of regions (== action_space.shape[0] for
                         PER_REGION_DISCRETE).
    """
    max_edges = num_vertiports * (num_vertiports - 1)
    return spaces.Dict({
        'node_feat': spaces.Box(
            low=-10_000, high=10_000,
            shape=(num_vertiports, NODE_FEAT_DIM),
            dtype=np.float32,
        ),
        'edge_index': spaces.Box(
            low=-10, high=max(num_vertiports, 10),
            shape=(2, int(max_edges)),
            dtype=np.int64,
        ),
        'edge_attr': spaces.Box(
            low=-1_000_000.0, high=1_000_000.0,
            shape=(int(max_edges), EDGE_FEAT_DIM),
            dtype=np.float32,
        ),
        'selected_actions': spaces.Box(
            low=0, high=num_vertiports,
            shape=(num_regions,),
            dtype=np.int64,
        ),
    })


def get_vp_obs_space(
    obs_type: str,
    num_vertiports: int,
    num_regions: int,
) -> spaces.Space:
    """
    Return the Gymnasium observation space for the given obs_type.

    Raises:
        ValueError          — unknown obs_type.
        NotImplementedError — obs_type is 'METRICS' or 'GRAPH-METRICS' (not yet implemented).
    """
    if obs_type == 'GRAPH':
        return obs_space_graph(num_vertiports, num_regions)

    if obs_type in ('METRICS', 'GRAPH-METRICS'):
        raise NotImplementedError(
            f"obs_type='{obs_type}' is not yet implemented — reward/obs composition "
            "from SimulatorManager.get_episode_metrics() is deferred. "
            "Use obs_type='GRAPH' for now."
        )

    raise ValueError(
        f"Unknown obs_type '{obs_type}'. Valid options: {VP_OBS_SPACE}"
    )
