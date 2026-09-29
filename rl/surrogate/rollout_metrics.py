"""
rollout_metrics.py
====================
Adapters that convert an autoregressive surrogate-model rollout (a list of
predicted graphs, no simulator involved) into an episode_metrics.json-shaped
dict, matching MetricsCollector.get_metrics()'s schema:
total_steps, total_nmac_events, total_uav_collision_events,
total_ra_collision_events, unique_uavs, avg_missions_completed,
peak_active_uavs, avg_speed, min_speed, max_speed, avg_dist_to_goal_final.

This is the "drop-in replacement for the simulator" step: seed from a real
initial graph, call model.predict_graph_next_state()/predict_dual_graph_next_state()
autoregressively for N steps with no simulator involved, then feed the
resulting rollout through the matching adapter below instead of running
MetricsCollector._calculate_metrics() against real simulator state.

dual_graph retains real per-UAV identity across a rollout (graph_flow
structurally can't), so its avg_missions_completed/avg_dist_to_goal_final are
EXACT per-UAV reconstructions, not graph_flow's network-wide proxies -- made
possible by DualGraphDataset's fixed-size per-episode padding + mask (a
removed UAV's slot freezes rather than disappearing, so max_uavs is constant
across a whole episode/rollout and unique_uavs is exactly that constant).
"""

from __future__ import annotations

from typing import Any, Dict, List

import torch
from torch_geometric.data import Data, HeteroData

from rl.surrogate.backbones.graph_flow_gnn import GraphFlowGNN
from rl.surrogate.datasets.dual_graph_dataset import UAV_NODE_KEYS

# Column layout, matching graph_flow_dataset.py's NODE_SPEED_KEYS/EDGE_SPEED_KEYS/
# EDGE_STATIC_KEYS ordering (avg_speed, speed_min, speed_max are the first three
# entries of both NODE_NONNEG_INDICES and EDGE_NONNEG_INDICES; nmac_count is the
# trailing edge column beyond that).
_NODE_SPEED_AVG_IDX, _NODE_SPEED_MIN_IDX, _NODE_SPEED_MAX_IDX = GraphFlowGNN.NODE_NONNEG_INDICES
_EDGE_SPEED_AVG_IDX, _EDGE_SPEED_MIN_IDX, _EDGE_SPEED_MAX_IDX, _EDGE_NMAC_IDX = (
    GraphFlowGNN.EDGE_NONNEG_INDICES
)
_EDGE_DISTANCE_IDX = 2  # EDGE_DYNAMIC_KEYS (2 cols) precede EDGE_STATIC_KEYS

# dual_graph UAV column indices, derived from UAV_NODE_KEYS rather than
# hardcoded (mirrors DualGraphGNN's own class constants).
_UAV_X_IDX = UAV_NODE_KEYS.index("x")
_UAV_Y_IDX = UAV_NODE_KEYS.index("y")
_UAV_SPEED_IDX = UAV_NODE_KEYS.index("speed")
_UAV_IN_NMAC_IDX = UAV_NODE_KEYS.index("in_nmac_event")
_UAV_IN_COLLISION_IDX = UAV_NODE_KEYS.index("in_collision_event")
_UAV_IN_RA_COLLISION_IDX = UAV_NODE_KEYS.index("in_ra_collision_event")
_UAV_MISSIONS_COMPLETED_IDX = UAV_NODE_KEYS.index("missions_completed")
_UAV_TARGET_X_IDX = UAV_NODE_KEYS.index("target_x")
_UAV_TARGET_Y_IDX = UAV_NODE_KEYS.index("target_y")


def graph_flow_rollout_to_episode_metrics(
    rollout: List[Data],
    initial_total_uavs: float,
) -> Dict[str, Any]:
    """Convert a GraphFlowGNN/GraphFlowRecurrentGNN autoregressive rollout into
    an episode_metrics.json-shaped dict (MetricsCollector.get_metrics()'s schema).

    Args:
        rollout: Consecutive predicted graphs [g_1, ..., g_N], each the output
            of model.predict_graph_next_state() fed back into itself with no
            simulator involved (g_1 = model(seed_graph), g_2 = model(g_1),
            ...). The pre-rollout seed graph itself is NOT included, matching
            how MetricsCollector counts total_steps as len(_steps) recorded
            during the episode, not counting an initial pre-episode state.
        initial_total_uavs: The TRUE fleet size before rollout started, read
            from the real seed graph's `total_uavs` (i.e. the last
            ground-truth graph, before any model prediction) -- NOT
            reconstructed from the model's own predicted `num_removed`, so
            that `unique_uavs` below stays exact regardless of how accurate
            the model's removal predictions are.

    Returns:
        Dict with the same 10 keys as MetricsCollector.get_metrics().

    Fidelity notes (see the surrogate plan doc's Part 6 for full reasoning):
        - total_steps, unique_uavs, peak_active_uavs are exact given accurate
          total_uavs/num_removed predictions; unique_uavs specifically is
          always exact (it's `initial_total_uavs`, a caller-supplied ground
          truth, not a model prediction), since the fleet is
          fixed-then-shrinking and never grows mid-episode.
        - avg_speed/min_speed/max_speed reconstruct exactly (given accurate
          predictions) from the per-node/per-edge avg_speed/speed_min/
          speed_max columns: avg_speed is a count-weighted mean (exact, since
          min/max/mean all decompose exactly across a disjoint partition of
          UAVs into nodes/edges), min_speed/max_speed are the min/max over all
          per-partition extrema.
        - total_nmac_events is exact up to double-counting if an nmac_pair
          spanned two different edges (see MetricsCollector.record()'s
          docstring) -- a rare edge case.
        - total_uav_collision_events/total_ra_collision_events are the model's
          own predicted per-step global-scalar counts, summed -- only as
          accurate as those two scalar heads.
        - avg_missions_completed and avg_dist_to_goal_final are network-wide
          aggregate PROXIES, not exact per-UAV reconstructions: graph_flow has
          no per-UAV identity, so these approximate the true per-UAV means
          using network-wide totals divided by initial_total_uavs.
          avg_dist_to_goal_final specifically underestimates whenever a
          grounded UAV already has a newly reassigned mission with nonzero
          remaining distance -- graph_flow has no way to know about pending
          mission assignments for grounded UAVs (treated as contributing 0).
    """
    if not rollout:
        raise ValueError("rollout must contain at least one predicted step")
    if initial_total_uavs <= 0:
        raise ValueError(f"initial_total_uavs must be > 0, got {initial_total_uavs}")

    total_steps = len(rollout)
    peak_active_uavs = max(float(g.total_uavs.item()) for g in rollout)

    total_nmac_events = sum(float(g.edge_attr[:, _EDGE_NMAC_IDX].sum().item()) for g in rollout)
    total_uav_collision_events = sum(float(g.total_collision_events.item()) for g in rollout)
    total_ra_collision_events = sum(float(g.total_ra_collision_events.item()) for g in rollout)

    cumulative_completions = sum(float(g.new_mission_completions.item()) for g in rollout)
    avg_missions_completed = cumulative_completions / initial_total_uavs

    weighted_speed_sum = 0.0
    weighted_count_sum = 0.0
    speed_mins: List[float] = []
    speed_maxs: List[float] = []
    for g in rollout:
        node_count = g.x[:, 0] + g.x[:, 1]  # n_grounded + n_landing_queue
        weighted_speed_sum += float((g.x[:, _NODE_SPEED_AVG_IDX] * node_count).sum().item())
        weighted_count_sum += float(node_count.sum().item())
        node_mask = node_count > 0
        if node_mask.any():
            speed_mins.append(float(g.x[node_mask, _NODE_SPEED_MIN_IDX].min().item()))
            speed_maxs.append(float(g.x[node_mask, _NODE_SPEED_MAX_IDX].max().item()))

        edge_count = g.edge_attr[:, 0]  # n_in_transit
        weighted_speed_sum += float((g.edge_attr[:, _EDGE_SPEED_AVG_IDX] * edge_count).sum().item())
        weighted_count_sum += float(edge_count.sum().item())
        edge_mask = edge_count > 0
        if edge_mask.any():
            speed_mins.append(float(g.edge_attr[edge_mask, _EDGE_SPEED_MIN_IDX].min().item()))
            speed_maxs.append(float(g.edge_attr[edge_mask, _EDGE_SPEED_MAX_IDX].max().item()))

    avg_speed = weighted_speed_sum / weighted_count_sum if weighted_count_sum > 0 else 0.0
    min_speed = min(speed_mins) if speed_mins else 0.0
    max_speed = max(speed_maxs) if speed_maxs else 0.0

    # Final-step-only: weighted remaining distance for in-transit UAVs, treating
    # grounded/landing-queue UAVs as contributing 0 (see docstring caveat above).
    last = rollout[-1]
    n_in_transit = last.edge_attr[:, 0]
    avg_progress = last.edge_attr[:, 1]
    edge_distance = last.edge_attr[:, _EDGE_DISTANCE_IDX]
    remaining_distance_sum = float(((1.0 - avg_progress) * edge_distance * n_in_transit).sum().item())
    avg_dist_to_goal_final = remaining_distance_sum / initial_total_uavs

    return {
        "total_steps": total_steps,
        "total_nmac_events": total_nmac_events,
        "total_uav_collision_events": total_uav_collision_events,
        "total_ra_collision_events": total_ra_collision_events,
        "unique_uavs": initial_total_uavs,
        "avg_missions_completed": avg_missions_completed,
        "peak_active_uavs": peak_active_uavs,
        "avg_speed": avg_speed,
        "min_speed": min_speed,
        "max_speed": max_speed,
        "avg_dist_to_goal_final": avg_dist_to_goal_final,
    }


def dual_graph_rollout_to_episode_metrics(rollout: List[HeteroData]) -> Dict[str, Any]:
    """Convert a DualGraphGNN autoregressive rollout into an
    episode_metrics.json-shaped dict (MetricsCollector.get_metrics()'s schema).

    Args:
        rollout: Consecutive predicted graphs [g_1, ..., g_N], each the output
            of model.predict_dual_graph_next_state() fed back into itself with
            no simulator involved -- each call's returned `uav_mask` must be
            written back onto the next input graph's `data["uav"].mask`
            before the next call (see DualGraphGNN.predict_dual_graph_next_state's
            docstring), so the "dead stays dead" invariant threads forward
            without ground truth. The pre-rollout seed graph itself is NOT
            included (same convention as graph_flow_rollout_to_episode_metrics).

    Returns:
        Dict with the same 10 keys as MetricsCollector.get_metrics().

    Fidelity notes:
        - unique_uavs is exact and structural, not a model prediction: it's
          simply max_uavs, the constant UAV row count fixed for the whole
          episode by DualGraphDataset (that episode's own initial fleet size,
          since the fleet only shrinks, never grows, mid-episode).
        - avg_missions_completed and avg_dist_to_goal_final are EXACT per-UAV
          reconstructions (not graph_flow's network-wide proxies), since
          dual_graph retains real UAV identity across the whole rollout:
          avg_missions_completed averages the final step's missions_completed
          column over ALL max_uavs slots (matching MetricsCollector's own
          "every UAV ever present, using its last known value" semantics);
          avg_dist_to_goal_final averages distance(position, target) over
          only mask==1 slots at the final step (matching MetricsCollector's
          own "only UAVs still in uav_dict at the last step" semantics --
          confirmed from _calculate_metrics()'s `if uav_id in
          last_step['uavs']` gate).
        - total_nmac_events/total_uav_collision_events/total_ra_collision_events
          are exact sums of the corresponding per-UAV, per-step event flags
          (in_nmac_event/in_collision_event/in_ra_collision_event), each of
          which fires on exactly the UAVs/steps the real event occurred on --
          no aggregate-scalar-head approximation needed, unlike graph_flow
          (which has no per-UAV identity to attribute RA-collisions/collisions
          to at all).
        - avg_speed/min_speed/max_speed are exact, mask-weighted over only
          real slots at each step (matching MetricsCollector's own per-UAV,
          per-step `all_speeds` list, which only ever contains snapshots from
          steps where that UAV was actually present).
    """
    if not rollout:
        raise ValueError("rollout must contain at least one predicted step")

    max_uavs = rollout[0]["uav"].x.shape[0]
    total_steps = len(rollout)
    unique_uavs = float(max_uavs)

    peak_active_uavs = max(float(g["uav"].mask.sum().item()) for g in rollout)

    total_nmac_events = sum(
        float(g["uav"].x[:, _UAV_IN_NMAC_IDX].sum().item()) for g in rollout
    )
    total_uav_collision_events = sum(
        float(g["uav"].x[:, _UAV_IN_COLLISION_IDX].sum().item()) for g in rollout
    )
    total_ra_collision_events = sum(
        float(g["uav"].x[:, _UAV_IN_RA_COLLISION_IDX].sum().item()) for g in rollout
    )

    last = rollout[-1]
    avg_missions_completed = float(
        last["uav"].x[:, _UAV_MISSIONS_COMPLETED_IDX].mean().item()
    )

    last_real = last["uav"].mask > 0.5
    if last_real.any():
        pos = last["uav"].x[last_real][:, [_UAV_X_IDX, _UAV_Y_IDX]]
        target = last["uav"].x[last_real][:, [_UAV_TARGET_X_IDX, _UAV_TARGET_Y_IDX]]
        avg_dist_to_goal_final = float(torch.norm(pos - target, dim=-1).mean().item())
    else:
        avg_dist_to_goal_final = 0.0

    speed_sum = 0.0
    speed_count = 0
    speed_mins: List[float] = []
    speed_maxs: List[float] = []
    for g in rollout:
        real = g["uav"].mask > 0.5
        if real.any():
            speeds = g["uav"].x[real, _UAV_SPEED_IDX]
            speed_sum += float(speeds.sum().item())
            speed_count += int(real.sum().item())
            speed_mins.append(float(speeds.min().item()))
            speed_maxs.append(float(speeds.max().item()))
    avg_speed = speed_sum / speed_count if speed_count > 0 else 0.0
    min_speed = min(speed_mins) if speed_mins else 0.0
    max_speed = max(speed_maxs) if speed_maxs else 0.0

    return {
        "total_steps": total_steps,
        "total_nmac_events": total_nmac_events,
        "total_uav_collision_events": total_uav_collision_events,
        "total_ra_collision_events": total_ra_collision_events,
        "unique_uavs": unique_uavs,
        "avg_missions_completed": avg_missions_completed,
        "peak_active_uavs": peak_active_uavs,
        "avg_speed": avg_speed,
        "min_speed": min_speed,
        "max_speed": max_speed,
        "avg_dist_to_goal_final": avg_dist_to_goal_final,
    }
