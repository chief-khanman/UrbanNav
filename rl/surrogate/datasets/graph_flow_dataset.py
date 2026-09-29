"""
graph_flow_dataset.py
======================
PyTorch Geometric Dataset for graph-level surrogate Model 1.

Nodes = vertiports, edges = flight paths between vertiport pairs.
Node attributes: [n_grounded, n_landing_queue, capacity, avg_speed, speed_min, speed_max].
Edge attributes: [n_in_transit, avg_progress, edge_distance, avg_speed, speed_min,
speed_max, nmac_count].
Global per-step scalars (not edge/node-localized): total_uavs, step, num_removed,
new_mission_completions, total_collision_events, total_ra_collision_events.

Each sample is a (graph_t, graph_t+1) pair for next-state prediction.
Supports three edge topology types via ``edge_type`` parameter:
  - "full_mesh":          all N*(N-1) directed vertiport pairs
  - "demand_driven":      only OD pairs observed in the logged data
  - "distance_threshold": pairs within ``distance_threshold`` of each other
"""

from __future__ import annotations

import json
import math
import os
import warnings
from itertools import product
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import torch
from torch_geometric.data import Data

# The end goal of this surrogate dynamics model is: state_t -> model -> state_t+1,
# rolled out autoregressively for a full episode, so episode-level metrics
# (collision/NMAC counts, mission completions, speed stats) can be derived from the
# rollout instead of running the simulator. Episode length and end-episode metrics
# already exist in the simulator core (metadata.json's total_timestep,
# episode_metrics.json from MetricsCollector) and don't need new plumbing here.
# Collision/NMAC data is wired in below (EDGE_EVENT_KEYS) plus global per-step
# scalars read directly off the Data object (see _step_to_graph) -- see
# rl/surrogate/rollout_metrics.py for how a rollout is turned into episode metrics.
#
# Temporal signal: avg_progress is a straight-line distance-progress fraction, not
# elapsed transit time -- it only implicitly encodes time under a constant-speed,
# straight-path assumption. An explicit `avg_transit_steps` edge feature (elapsed
# steps since departure, averaged over in-transit UAVs) would need new persistent
# per-UAV departure-step tracking in MetricsCollector and is deferred to a follow-up
# pass, not implemented here.
NODE_ATTR_KEYS: Tuple[str, ...] = ("n_grounded", "n_landing_queue", "capacity")
# n_grounded is really "landed and awaiting a new mission assignment", not grounded
# in an operational/maintenance sense -- renaming deferred to a follow-up pass
# (touches metrics_collector.py, dual_graph_dataset.py, and one test assertion).
NODE_SPEED_KEYS: Tuple[str, ...] = ("avg_speed", "speed_min", "speed_max")
NODE_ATTR_DIM: int = len(NODE_ATTR_KEYS) + len(NODE_SPEED_KEYS)

# avg_progress is the mean, over all UAVs currently in transit on this edge, of each
# UAV's individual straight-line progress fraction min(dist(pos, src_vp)/edge_distance, 1.0).
EDGE_DYNAMIC_KEYS: Tuple[str, ...] = ("n_in_transit", "avg_progress")
EDGE_STATIC_KEYS: Tuple[str, ...] = ("edge_distance",)
# edge_distance is straight-line vertiport separation, not a UAM route length --
# renaming deferred to a follow-up pass (same blast radius as n_grounded above).
EDGE_SPEED_KEYS: Tuple[str, ...] = ("avg_speed", "speed_min", "speed_max")
# NMAC never removes a UAV (unlike a UAV-UAV or restricted-area collision), so it's
# the one collision-type event that's safely attributable to a specific edge --
# see MetricsCollector.record()'s docstring for why collision/RA-collision counts
# are step-level global scalars instead (read off step["num_removed_this_step"]
# etc. in _step_to_graph below), not edge-local columns.
EDGE_EVENT_KEYS: Tuple[str, ...] = ("nmac_count",)
EDGE_ATTR_DIM: int = (
    len(EDGE_DYNAMIC_KEYS) + len(EDGE_STATIC_KEYS) + len(EDGE_SPEED_KEYS) + len(EDGE_EVENT_KEYS)
)

VALID_EDGE_TYPES = {"full_mesh", "demand_driven", "distance_threshold"}


def _build_edge_index_full_mesh(n_nodes: int) -> torch.Tensor:
    src, dst = [], []
    for i, j in product(range(n_nodes), repeat=2):
        if i != j:
            src.append(i)
            dst.append(j)
    return torch.tensor([src, dst], dtype=torch.long)


def _build_edge_index_demand_driven(
    n_nodes: int,
    all_edge_snapshots: List[Dict[str, Dict[str, Any]]],
) -> torch.Tensor:
    """Edges for every OD pair that appears at least once across all steps.

    Symmetrized: if a corridor was only ever observed carrying traffic i->j in
    the source logs, j->i is added too, so every OD pair has both directions
    (matching full_mesh/distance_threshold, where both directions are always
    present) -- otherwise a node could have no way to receive information
    about return traffic on a corridor that happened to be one-directional in
    this particular dataset.
    """
    pairs: set = set()
    for snap in all_edge_snapshots:
        for entry in snap.values():
            pairs.add((entry["src"], entry["dst"]))
    pairs |= {(dst, src) for src, dst in pairs}
    if not pairs:
        return torch.zeros((2, 0), dtype=torch.long)
    src, dst = zip(*sorted(pairs))
    return torch.tensor([list(src), list(dst)], dtype=torch.long)


def _build_edge_index_distance_threshold(
    vp_positions: np.ndarray,
    threshold: float,
) -> torch.Tensor:
    n = len(vp_positions)
    src, dst = [], []
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            dist = np.linalg.norm(vp_positions[i] - vp_positions[j])
            if dist <= threshold:
                src.append(i)
                dst.append(j)
    return torch.tensor([src, dst], dtype=torch.long)


def _compute_pairwise_distances(vp_positions: np.ndarray) -> Dict[Tuple[int, int], float]:
    n = len(vp_positions)
    dists: Dict[Tuple[int, int], float] = {}
    for i in range(n):
        for j in range(n):
            if i != j:
                dists[(i, j)] = float(np.linalg.norm(vp_positions[i] - vp_positions[j]))
    return dists


def _step_to_graph(
    step: Dict[str, Any],
    edge_index: torch.Tensor,
    n_nodes: int,
    pairwise_dists: Dict[Tuple[int, int], float],
) -> Data:
    """Convert one step record into a PyG Data object."""
    vp_snap = step.get("vertiports") or {}
    edge_snap = step.get("edges") or {}

    # Node features: [n_grounded, n_landing_queue, capacity, avg_speed, speed_min, speed_max]
    x = torch.zeros((n_nodes, NODE_ATTR_DIM), dtype=torch.float32)
    for idx_str, info in vp_snap.items():
        idx = int(idx_str)
        if idx < n_nodes:
            n_grounded = float(info.get("n_grounded", 0))
            n_landing_queue = float(info.get("n_landing_queue", 0))
            n_present = n_grounded + n_landing_queue
            x[idx, 0] = n_grounded
            x[idx, 1] = n_landing_queue
            x[idx, 2] = float(info.get("capacity", 0))
            x[idx, 3] = info.get("speed_sum", 0.0) / n_present if n_present > 0 else 0.0
            x[idx, 4] = float(info.get("speed_min", 0.0))
            x[idx, 5] = float(info.get("speed_max", 0.0))
        else:
            warnings.warn(
                f"vertiport idx {idx} >= n_nodes {n_nodes} (topology fixed from step 0) "
                "-- dropping this vertiport's data for this step instead of silently "
                "ignoring it; check for a topology-size mismatch across steps."
            )

    # Build a lookup for edge snapshots: (src, dst) -> snapshot
    edge_lookup: Dict[Tuple[int, int], Dict[str, Any]] = {}
    for entry in edge_snap.values():
        key = (entry["src"], entry["dst"])
        edge_lookup[key] = entry

    # Edge features: [n_in_transit, avg_progress, edge_distance, avg_speed, speed_min,
    # speed_max, nmac_count]
    n_edges = edge_index.shape[1]
    edge_attr = torch.zeros((n_edges, EDGE_ATTR_DIM), dtype=torch.float32)
    for e in range(n_edges):
        src_i = int(edge_index[0, e])
        dst_i = int(edge_index[1, e])
        snap = edge_lookup.get((src_i, dst_i))
        if snap is not None:
            n_transit = snap["n_in_transit"]
            edge_attr[e, 0] = float(n_transit)
            edge_attr[e, 1] = snap["progress_sum"] / n_transit if n_transit > 0 else 0.0
        edge_attr[e, 2] = pairwise_dists.get((src_i, dst_i), 0.0)
        if snap is not None:
            edge_attr[e, 3] = snap.get("speed_sum", 0.0) / n_transit if n_transit > 0 else 0.0
            edge_attr[e, 4] = float(snap.get("speed_min", 0.0))
            edge_attr[e, 5] = float(snap.get("speed_max", 0.0))
            edge_attr[e, 6] = float(snap.get("nmac_count", 0))

    # Global: total UAVs (sum of grounded + landing_queue + all in-transit)
    total_grounded = x[:, 0].sum().item() + x[:, 1].sum().item()
    total_transit = edge_attr[:, 0].sum().item()
    total_uavs = total_grounded + total_transit

    data = Data(
        x=x, # Node feature matrix
        edge_index=edge_index, # edge index
        edge_attr=edge_attr, # edge feature matrix
        total_uavs=torch.tensor([total_uavs], dtype=torch.float32),
        # step["step"] = SimulatorState.currentstep, incremented once per
        # SimulatorManager.step() call -- a 1:1 simulator timestep index.
        step=torch.tensor([step.get("step", 0)], dtype=torch.long),
        # Global per-step event scalars (not edge/node-localized -- see
        # MetricsCollector.record()'s docstring for why collision/RA-collision
        # counts can't be attributed to a specific edge like nmac_count can).
        num_removed=torch.tensor([step.get("num_removed_this_step", 0)], dtype=torch.float32),
        new_mission_completions=torch.tensor(
            [step.get("new_mission_completions_this_step", 0)], dtype=torch.float32
        ),
        total_collision_events=torch.tensor(
            [step.get("total_collision_events_this_step", 0)], dtype=torch.float32
        ),
        total_ra_collision_events=torch.tensor(
            [step.get("total_ra_collision_events_this_step", 0)], dtype=torch.float32
        ),
    )
    return data


class GraphFlowDataset(torch.utils.data.Dataset):
    """(graph_t, graph_t+1) pairs for graph-level next-state prediction.

    Args:
        episode_dirs: Directories containing step_history.json with
            vertiport/edge snapshots (from extended MetricsCollector).
        edge_type: One of "full_mesh", "demand_driven", "distance_threshold".
        distance_threshold: Required when edge_type="distance_threshold".
    """

    def __init__(
        self,
        episode_dirs: List[str],
        edge_type: str = "full_mesh",
        distance_threshold: float = 0.0,
    ):
        if edge_type not in VALID_EDGE_TYPES:
            raise ValueError(f"edge_type must be one of {VALID_EDGE_TYPES}, got '{edge_type}'")
        if edge_type == "distance_threshold" and distance_threshold <= 0:
            raise ValueError("distance_threshold must be > 0 for edge_type='distance_threshold'")

        self.edge_type = edge_type
        self.distance_threshold = distance_threshold

        self._pairs: List[Tuple[Data, Data]] = []

        for ep_dir in episode_dirs:
            self._load_episode(ep_dir)

    def _load_episode(self, episode_dir: str) -> None:
        path = os.path.join(episode_dir, "step_history.json")
        with open(path, "r") as f:
            steps: List[Dict[str, Any]] = json.load(f)
        steps.sort(key=lambda s: s["step"])

        # Filter to steps that have vertiport data
        steps = [s for s in steps if s.get("vertiports")]
        if len(steps) < 2:
            return

        # Determine graph topology from first step
        first_vp = steps[0]["vertiports"]
        n_nodes = len(first_vp)
        if n_nodes == 0:
            return

        # Extract vertiport positions for distance calculations
        vp_positions = np.zeros((n_nodes, 2), dtype=np.float64)
        for idx_str, info in first_vp.items():
            idx = int(idx_str)
            if idx < n_nodes:
                vp_positions[idx] = [info.get("x", 0.0), info.get("y", 0.0)]

        pairwise_dists = _compute_pairwise_distances(vp_positions)

        # Build edge_index based on edge_type
        all_edge_snaps = [s.get("edges") or {} for s in steps]
        if self.edge_type == "full_mesh":
            edge_index = _build_edge_index_full_mesh(n_nodes)
        elif self.edge_type == "demand_driven":
            edge_index = _build_edge_index_demand_driven(n_nodes, all_edge_snaps)
        else:
            edge_index = _build_edge_index_distance_threshold(vp_positions, self.distance_threshold)

        # Build consecutive (graph_t, graph_t+1) pairs
        for t in range(len(steps) - 1):
            g_t = _step_to_graph(steps[t], edge_index, n_nodes, pairwise_dists)
            g_tp1 = _step_to_graph(steps[t + 1], edge_index, n_nodes, pairwise_dists)
            self._pairs.append((g_t, g_tp1))

    def __len__(self) -> int:
        return len(self._pairs)

    def __getitem__(self, idx: int) -> Tuple[Data, Data]:
        return self._pairs[idx]

    @classmethod
    def from_logs_root(
        cls, logs_root: Union[str, List[str]], **kwargs
    ) -> "GraphFlowDataset":
        """Build from every episode under logs_root that has step_history.json.

        Args:
            logs_root: Single path or list of paths.  When a list is given,
                all directories are scanned and their episodes are merged
                into one dataset — this enables mixing data from different
                simulator configurations in a single training run.
        """
        if isinstance(logs_root, str):
            logs_root = [logs_root]

        dirs: List[str] = []
        for root_path in logs_root:
            root = Path(root_path)
            if not root.exists():
                continue
            dirs.extend(
                sorted(
                    str(p)
                    for p in root.iterdir()
                    if p.is_dir() and (p / "step_history.json").exists()
                )
            )
        return cls(dirs, **kwargs)
