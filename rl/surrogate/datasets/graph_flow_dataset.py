"""
graph_flow_dataset.py
======================
PyTorch Geometric Dataset for graph-level surrogate Model 1.

Nodes = vertiports, edges = flight paths between vertiport pairs.
Node attributes: [n_grounded, n_landing_queue, capacity].
Edge attributes: [n_in_transit, avg_progress, edge_distance].

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
from itertools import product
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch
from torch_geometric.data import Data

NODE_ATTR_KEYS: Tuple[str, ...] = ("n_grounded", "n_landing_queue", "capacity")
NODE_ATTR_DIM: int = len(NODE_ATTR_KEYS)

EDGE_DYNAMIC_KEYS: Tuple[str, ...] = ("n_in_transit", "avg_progress")
EDGE_STATIC_KEYS: Tuple[str, ...] = ("edge_distance",)
EDGE_ATTR_DIM: int = len(EDGE_DYNAMIC_KEYS) + len(EDGE_STATIC_KEYS)

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
    """Edges for every OD pair that appears at least once across all steps."""
    pairs: set = set()
    for snap in all_edge_snapshots:
        for entry in snap.values():
            pairs.add((entry["src"], entry["dst"]))
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

    # Node features: [n_grounded, n_landing_queue, capacity]
    x = torch.zeros((n_nodes, NODE_ATTR_DIM), dtype=torch.float32)
    for idx_str, info in vp_snap.items():
        idx = int(idx_str)
        if idx < n_nodes:
            x[idx, 0] = float(info.get("n_grounded", 0))
            x[idx, 1] = float(info.get("n_landing_queue", 0))
            x[idx, 2] = float(info.get("capacity", 0))

    # Build a lookup for edge snapshots: (src, dst) -> snapshot
    edge_lookup: Dict[Tuple[int, int], Dict[str, Any]] = {}
    for entry in edge_snap.values():
        key = (entry["src"], entry["dst"])
        edge_lookup[key] = entry

    # Edge features: [n_in_transit, avg_progress, edge_distance]
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

    # Global: total UAVs (sum of grounded + landing_queue + all in-transit)
    total_grounded = x[:, 0].sum().item() + x[:, 1].sum().item()
    total_transit = edge_attr[:, 0].sum().item()
    total_uavs = total_grounded + total_transit

    data = Data(
        x=x,
        edge_index=edge_index,
        edge_attr=edge_attr,
        total_uavs=torch.tensor([total_uavs], dtype=torch.float32),
        step=torch.tensor([step.get("step", 0)], dtype=torch.long),
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
    def from_logs_root(cls, logs_root: str, **kwargs) -> "GraphFlowDataset":
        """Build from every episode under logs_root that has step_history.json."""
        root = Path(logs_root)
        if not root.exists():
            return cls([], **kwargs)
        dirs = sorted(
            str(p)
            for p in root.iterdir()
            if p.is_dir() and (p / "step_history.json").exists()
        )
        return cls(dirs, **kwargs)
