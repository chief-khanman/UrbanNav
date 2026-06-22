"""
dual_graph_dataset.py
======================
PyTorch Geometric HeteroData Dataset for surrogate Model 2 (UAV-as-node).

Two graphs composed into a single heterogeneous graph:

1. **Static vertiport graph**: nodes = vertiports, edges = spatial connections.
   Topology is fixed per episode; node features (UAV counts) update each step.

2. **Dynamic UAV graph**: nodes = UAVs, edges = inter-UAV connections.
   Topology changes every step (distance-threshold or fully-connected).

Cross-graph edges connect UAVs to their current/target vertiports.

Edge feature tensor scaling:
  - Per-edge feature dim is 4: [relative_distance, dx, dy, dz].
  - Total tensor size depends on connectivity mode:
    - Fully connected: N*(N-1) edges -> shape [N*(N-1), 4] — O(N^2).
    - Distance threshold: ~N*K edges (K = avg neighbors in sensor range)
      -> shape [~N*K, 4] — roughly linear.

Each sample is a (hetero_graph_t, hetero_graph_t+1) pair for next-state
prediction.  Loads from the same step_history.json files produced by
MetricsCollector — no new logging format needed.

Supports multiple log directories for cross-config data mixing.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import torch
from torch_geometric.data import HeteroData
from torch.utils.data import Dataset

# --- Feature dimensions ---
UAV_NODE_KEYS: Tuple[str, ...] = (
    "x", "y", "z", "vx", "vy", "vz", "speed", "heading", "collision_status",
)
UAV_NODE_DIM: int = len(UAV_NODE_KEYS)

UAV_EDGE_DIM: int = 4  # [relative_distance, dx, dy, dz]

VP_NODE_KEYS: Tuple[str, ...] = ("x", "y", "capacity", "n_grounded", "n_landing_queue")
VP_NODE_DIM: int = len(VP_NODE_KEYS)

VALID_UAV_EDGE_TYPES = {"distance_threshold", "fully_connected"}


def _build_uav_edges_fully_connected(n: int) -> torch.Tensor:
    if n < 2:
        return torch.zeros((2, 0), dtype=torch.long)
    src, dst = [], []
    for i in range(n):
        for j in range(n):
            if i != j:
                src.append(i)
                dst.append(j)
    return torch.tensor([src, dst], dtype=torch.long)


def _build_uav_edges_distance(
    positions: np.ndarray, threshold: float
) -> torch.Tensor:
    n = len(positions)
    if n < 2:
        return torch.zeros((2, 0), dtype=torch.long)
    src, dst = [], []
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            dist = np.linalg.norm(positions[i] - positions[j])
            if dist <= threshold:
                src.append(i)
                dst.append(j)
    if not src:
        return torch.zeros((2, 0), dtype=torch.long)
    return torch.tensor([src, dst], dtype=torch.long)


def _compute_uav_edge_attr(
    positions: np.ndarray, edge_index: torch.Tensor
) -> torch.Tensor:
    n_edges = edge_index.shape[1]
    if n_edges == 0:
        return torch.zeros((0, UAV_EDGE_DIM), dtype=torch.float32)
    attr = torch.zeros((n_edges, UAV_EDGE_DIM), dtype=torch.float32)
    for e in range(n_edges):
        si = int(edge_index[0, e])
        di = int(edge_index[1, e])
        diff = positions[di] - positions[si]
        attr[e, 0] = float(np.linalg.norm(diff))
        attr[e, 1] = float(diff[0])
        attr[e, 2] = float(diff[1])
        attr[e, 3] = float(diff[2]) if len(diff) > 2 else 0.0
    return attr


def _step_to_hetero(
    step: Dict[str, Any],
    uav_edge_type: str,
    uav_edge_distance: float,
    vp_edge_index: torch.Tensor,
) -> HeteroData:
    """Convert one step record into a HeteroData graph."""
    data = HeteroData()

    # --- Vertiport nodes ---
    vp_snap = step.get("vertiports") or {}
    n_vp = len(vp_snap)
    vp_x = torch.zeros((n_vp, VP_NODE_DIM), dtype=torch.float32)
    for idx_str, info in vp_snap.items():
        idx = int(idx_str)
        if idx < n_vp:
            vp_x[idx, 0] = float(info.get("x", 0.0))
            vp_x[idx, 1] = float(info.get("y", 0.0))
            vp_x[idx, 2] = float(info.get("capacity", 0))
            vp_x[idx, 3] = float(info.get("n_grounded", 0))
            vp_x[idx, 4] = float(info.get("n_landing_queue", 0))
    data["vertiport"].x = vp_x

    # --- Vertiport edges (static topology, passed in) ---
    data["vertiport", "connected_to", "vertiport"].edge_index = vp_edge_index

    # --- UAV nodes ---
    uav_snap = step.get("uavs") or {}
    uav_ids = sorted(uav_snap.keys(), key=lambda k: int(k))
    n_uav = len(uav_ids)
    uav_x = torch.zeros((n_uav, UAV_NODE_DIM), dtype=torch.float32)
    uav_positions = np.zeros((n_uav, 3), dtype=np.float64)

    for local_idx, uid_str in enumerate(uav_ids):
        snap = uav_snap[uid_str]
        for feat_idx, key in enumerate(UAV_NODE_KEYS):
            uav_x[local_idx, feat_idx] = float(snap.get(key, 0.0))
        uav_positions[local_idx] = [
            float(snap.get("x", 0.0)),
            float(snap.get("y", 0.0)),
            float(snap.get("z", 0.0)),
        ]
    data["uav"].x = uav_x

    # --- UAV-UAV edges ---
    if uav_edge_type == "fully_connected":
        uav_ei = _build_uav_edges_fully_connected(n_uav)
    else:
        uav_ei = _build_uav_edges_distance(uav_positions, uav_edge_distance)
    data["uav", "communicates_with", "uav"].edge_index = uav_ei
    data["uav", "communicates_with", "uav"].edge_attr = _compute_uav_edge_attr(
        uav_positions, uav_ei
    )

    # --- Cross-graph edges: UAV -> vertiport ---
    # Each UAV is connected to vertiports it is associated with (start/end).
    # step_history doesn't directly store vertiport indices per UAV, but we
    # can infer from the edge snapshots or fall back to nearest-vertiport.
    # For now: connect each UAV to its nearest vertiport by position.
    cross_src, cross_dst = [], []
    if n_uav > 0 and n_vp > 0:
        vp_positions = vp_x[:, :2].numpy()
        for ui in range(n_uav):
            uav_pos_2d = uav_positions[ui, :2]
            dists = np.linalg.norm(vp_positions - uav_pos_2d, axis=1)
            nearest = int(np.argmin(dists))
            cross_src.append(ui)
            cross_dst.append(nearest)
            # Also connect to second-nearest if available (proxy for target vp)
            if n_vp > 1:
                sorted_vps = np.argsort(dists)
                second = int(sorted_vps[1])
                cross_src.append(ui)
                cross_dst.append(second)
    if cross_src:
        cross_ei = torch.tensor([cross_src, cross_dst], dtype=torch.long)
    else:
        cross_ei = torch.zeros((2, 0), dtype=torch.long)
    data["uav", "assigned_to", "vertiport"].edge_index = cross_ei

    # Reverse cross-graph edges: vertiport -> UAV
    if cross_ei.shape[1] > 0:
        rev_ei = torch.stack([cross_ei[1], cross_ei[0]])
    else:
        rev_ei = torch.zeros((2, 0), dtype=torch.long)
    data["vertiport", "hosts", "uav"].edge_index = rev_ei

    # --- Global metadata ---
    data.total_uavs = torch.tensor([n_uav], dtype=torch.float32)
    data.step = torch.tensor([step.get("step", 0)], dtype=torch.long)

    return data


class DualGraphDataset(Dataset):
    """(hetero_graph_t, hetero_graph_t+1) pairs for dual-graph next-state prediction.

    Args:
        episode_dirs: Directories containing step_history.json.
        uav_edge_type: "distance_threshold" or "fully_connected".
        uav_edge_distance: Threshold distance for UAV-UAV edges (only used
            when uav_edge_type="distance_threshold").
        vp_edge_type: Edge topology for vertiport graph. Same options as
            GraphFlowDataset: "full_mesh", "distance_threshold".
        vp_edge_distance: Threshold for vertiport distance-based edges.
    """

    def __init__(
        self,
        episode_dirs: List[str],
        uav_edge_type: str = "distance_threshold",
        uav_edge_distance: float = 200.0,
        vp_edge_type: str = "full_mesh",
        vp_edge_distance: float = 0.0,
    ):
        if uav_edge_type not in VALID_UAV_EDGE_TYPES:
            raise ValueError(
                f"uav_edge_type must be one of {VALID_UAV_EDGE_TYPES}, got '{uav_edge_type}'"
            )

        self.uav_edge_type = uav_edge_type
        self.uav_edge_distance = uav_edge_distance
        self.vp_edge_type = vp_edge_type
        self.vp_edge_distance = vp_edge_distance

        self._pairs: List[Tuple[HeteroData, HeteroData]] = []

        for ep_dir in episode_dirs:
            self._load_episode(ep_dir)

    def _load_episode(self, episode_dir: str) -> None:
        path = os.path.join(episode_dir, "step_history.json")
        with open(path, "r") as f:
            steps: List[Dict[str, Any]] = json.load(f)
        steps.sort(key=lambda s: s["step"])
        steps = [s for s in steps if s.get("vertiports") and s.get("uavs")]
        if len(steps) < 2:
            return

        # Build static vertiport edge index from first step
        first_vp = steps[0]["vertiports"]
        n_vp = len(first_vp)
        if n_vp == 0:
            return

        vp_positions = np.zeros((n_vp, 2), dtype=np.float64)
        for idx_str, info in first_vp.items():
            idx = int(idx_str)
            if idx < n_vp:
                vp_positions[idx] = [info.get("x", 0.0), info.get("y", 0.0)]

        if self.vp_edge_type == "full_mesh":
            vp_edge_index = _build_uav_edges_fully_connected(n_vp)
        else:
            vp_edge_index = _build_uav_edges_distance(
                vp_positions, self.vp_edge_distance
            )

        for t in range(len(steps) - 1):
            g_t = _step_to_hetero(
                steps[t], self.uav_edge_type, self.uav_edge_distance, vp_edge_index
            )
            g_tp1 = _step_to_hetero(
                steps[t + 1], self.uav_edge_type, self.uav_edge_distance, vp_edge_index
            )
            self._pairs.append((g_t, g_tp1))

    def __len__(self) -> int:
        return len(self._pairs)

    def __getitem__(self, idx: int) -> Tuple[HeteroData, HeteroData]:
        return self._pairs[idx]

    @classmethod
    def from_logs_root(
        cls, logs_root: Union[str, List[str]], **kwargs
    ) -> "DualGraphDataset":
        """Build from every episode under logs_root that has step_history.json.

        Args:
            logs_root: Single path or list of paths.  When a list is given,
                episodes from all directories are merged into one dataset.
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
