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

UAV node count is fixed per episode at max_uavs (that episode's own initial
fleet size — a UAV-UAV or restricted-area collision permanently removes a UAV
from ATC.uav_dict under the actual persist_collided_uavs=False data-collection
config, and the fleet only ever shrinks, never grows, mid-episode). Every step
in an episode carries the same max_uavs UAV rows, with a `mask` tensor marking
which slots are still real (1.0) vs removed (0.0) as of that step -- a removed
slot's row is frozen at its last real values (velocities zeroed, collision_status
forced to 0) rather than dropped, so every (g_t, g_tp1) pair in an episode has
identical shapes. This is what makes train_dual_graph.py's per-pair MSE loss
well-defined even across a collision-removal transition.

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
    "in_nmac_event", "in_collision_event", "in_ra_collision_event",
    "missions_completed", "target_x", "target_y",
)
# in_nmac_event: 1.0 if this UAV appears in step["collisions"]["nmac_pairs"] this
# step, else 0.0 -- computed specially in _uav_row_from_snapshot (NMAC never
# removes a UAV, so it's safely attributable per-UAV every step it's real).
#
# in_collision_event/in_ra_collision_event: 1.0 for exactly the one step a UAV
# is removed (appears in step["collisions"]["uav_collision_pairs"] or
# ["ra_collision_ids"]), else 0.0. Unlike graph_flow (where these events can
# only be tracked as aggregate global scalars, since a removed UAV leaves no
# trace to attribute to a specific edge), the fixed-size padding here keeps a
# removed UAV's *slot* alive (frozen), so its cause-of-removal can be attributed
# exactly, per-UAV, on the single transition step -- set in _step_to_hetero's
# freeze path (_freeze_removed), not in _uav_row_from_snapshot, since the event
# fires on the first step the UAV is ALREADY gone from step["uavs"].
#
# missions_completed: the already-exact uav_snapshots[...]['num_missions_completed']
# MetricsCollector writes -- monotonically non-decreasing, so freezing it forward
# for a removed slot already gives that UAV's correct final mission count.
#
# target_x/target_y: the UAV's true mission-destination vertiport position
# (uav_snapshots[...]['target_vertiport_idx'], resolved against that step's own
# vertiport snapshot), replacing the old nearest/second-nearest position
# heuristic for the "assigned_to" cross-graph edge. Defaults to the UAV's own
# current position when no end_vertiport is assigned yet (idle/no-mission
# convention: 0 remaining distance, matching graph_flow's grounded-UAV proxy).
#
# Removed-slot freezing (see _freeze_removed): position/heading/speed/target
# frozen at the last real snapshot: vx/vy/vz/speed forced to 0, collision_status
# forced to 0. This is the concrete fix for collision_status never actually
# flipping under the real (persist_collided_uavs=False) data-collection config --
# ground truth is now derived from the real ATC.remove_uavs_by_id() removal
# event via the dataset's own mask tracking, not the (unwired) UAV attribute.

UAV_NODE_DIM: int = len(UAV_NODE_KEYS)
_TARGET_X_IDX = UAV_NODE_KEYS.index("target_x")
_TARGET_Y_IDX = UAV_NODE_KEYS.index("target_y")

UAV_EDGE_DIM: int = 4  # [relative_distance, dx, dy, dz]

VP_NODE_KEYS: Tuple[str, ...] = ("x", "y", "capacity", "n_grounded", "n_landing_queue")
VP_NODE_DIM: int = len(VP_NODE_KEYS)

VALID_UAV_EDGE_TYPES = {"distance_threshold", "fully_connected"}


def _build_uav_edges_fully_connected(n: int) -> torch.Tensor:
    """All N*(N-1) directed pairs among n local indices [0, n)."""
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


def _build_uav_edges_among(
    real_slots: np.ndarray,
    positions: np.ndarray,
    edge_type: str,
    threshold: float,
) -> torch.Tensor:
    """UAV-UAV edges restricted to real (mask==1) slots, remapped back to
    global slot indices -- removed slots get zero edges (isolated nodes;
    _HomoMessagePassingBlock's degree-normalization already handles 0-in-degree
    nodes safely via its .clamp(min=1))."""
    n_real = len(real_slots)
    if n_real < 2:
        return torch.zeros((2, 0), dtype=torch.long)
    sub_positions = positions[real_slots]
    if edge_type == "fully_connected":
        sub_ei = _build_uav_edges_fully_connected(n_real)
    else:
        sub_ei = _build_uav_edges_distance(sub_positions, threshold)
    if sub_ei.shape[1] == 0:
        return sub_ei
    remap = torch.as_tensor(real_slots, dtype=torch.long)
    return remap[sub_ei]


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


def _resolve_target_xy(
    snap: Dict[str, Any], vp_snap: Dict[str, Any]
) -> Tuple[float, float]:
    """True mission-destination position, or the UAV's own current position
    when no end_vertiport is assigned yet (idle/no-mission convention)."""
    target_idx = snap.get("target_vertiport_idx")
    if target_idx is not None:
        info = vp_snap.get(str(target_idx))
        if info is not None:
            return float(info.get("x", 0.0)), float(info.get("y", 0.0))
    return float(snap.get("x", 0.0)), float(snap.get("y", 0.0))


def _uav_row_from_snapshot(
    uid_str: str,
    snap: Dict[str, Any],
    vp_snap: Dict[str, Any],
    uav_ids_in_nmac: set,
) -> Dict[str, float]:
    """Build one real UAV's feature row from its step snapshot."""
    target_x, target_y = _resolve_target_xy(snap, vp_snap)
    return {
        "x": float(snap.get("x", 0.0)),
        "y": float(snap.get("y", 0.0)),
        "z": float(snap.get("z", 0.0)),
        "vx": float(snap.get("vx", 0.0)),
        "vy": float(snap.get("vy", 0.0)),
        "vz": float(snap.get("vz", 0.0)),
        "speed": float(snap.get("speed", 0.0)),
        "heading": float(snap.get("heading", 0.0)),
        "collision_status": float(snap.get("collision_status", 1.0)),
        "in_nmac_event": 1.0 if uid_str in uav_ids_in_nmac else 0.0,
        "in_collision_event": 0.0,  # only ever 1.0 on the transition step -- see _freeze_removed
        "in_ra_collision_event": 0.0,
        "missions_completed": float(snap.get("num_missions_completed", 0.0)),
        "target_x": target_x,
        "target_y": target_y,
    }


def _freeze_removed(
    row: Dict[str, float],
    in_collision: bool,
    in_ra_collision: bool,
) -> Dict[str, float]:
    """Carry a removed UAV's row forward: position/heading/target frozen at
    its last real values, velocity/speed zeroed, collision_status forced to 0
    (the ground-truth "dead" signal -- see module docstring).

    `in_collision`/`in_ra_collision` should only be True on the single step
    this call represents the *first* frozen step for this uav_id (checked by
    the caller via the previous collision_status) -- so the flags correctly
    fire exactly once, on the transition step, not on every subsequent frozen
    step.
    """
    frozen = dict(row)
    frozen["vx"] = 0.0
    frozen["vy"] = 0.0
    frozen["vz"] = 0.0
    frozen["speed"] = 0.0
    frozen["collision_status"] = 0.0
    frozen["in_nmac_event"] = 0.0
    frozen["in_collision_event"] = 1.0 if in_collision else 0.0
    frozen["in_ra_collision_event"] = 1.0 if in_ra_collision else 0.0
    return frozen


def _step_to_hetero(
    step: Dict[str, Any],
    uav_id_to_slot: Dict[str, int],
    max_uavs: int,
    last_known: Dict[str, Dict[str, float]],
    uav_edge_type: str,
    uav_edge_distance: float,
    vp_edge_index: torch.Tensor,
) -> Tuple[HeteroData, Dict[str, Dict[str, float]]]:
    """Convert one step record into a fixed-size ([max_uavs] UAV rows)
    HeteroData graph, plus the updated last-known-values dict to carry into
    the next step's call (episode-scoped, threaded by the caller)."""
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

    # --- UAV nodes: fixed [max_uavs, UAV_NODE_DIM], one row per episode slot ---
    uav_snap = step.get("uavs") or {}
    collisions = step.get("collisions") or {}
    uav_ids_in_nmac = {str(uid) for pair in (collisions.get("nmac_pairs") or []) for uid in pair}
    uav_ids_in_collision = {
        str(uid) for pair in (collisions.get("uav_collision_pairs") or []) for uid in pair
    }
    uav_ids_in_ra_collision = {str(uid) for uid in (collisions.get("ra_collision_ids") or [])}

    uav_x = torch.zeros((max_uavs, UAV_NODE_DIM), dtype=torch.float32)
    mask = torch.zeros(max_uavs, dtype=torch.float32)
    uav_positions = np.zeros((max_uavs, 3), dtype=np.float64)
    new_last_known: Dict[str, Dict[str, float]] = dict(last_known)

    for uid_str, slot in uav_id_to_slot.items():
        snap = uav_snap.get(uid_str)
        if snap is not None:
            row = _uav_row_from_snapshot(uid_str, snap, vp_snap, uav_ids_in_nmac)
            mask[slot] = 1.0
        else:
            # Removed at or before this step. Every uid in uav_id_to_slot came
            # from the episode's first step, so it always has a real row
            # (in new_last_known) before it can ever be removed. The
            # in_collision_event/in_ra_collision_event flags should only fire
            # on the single transition step -- i.e. only if the PREVIOUS
            # stored row was still alive (collision_status >= 0.5); once
            # already frozen, these are always 0 on every subsequent step.
            prev_row = new_last_known.get(uid_str, {k: 0.0 for k in UAV_NODE_KEYS})
            just_removed = prev_row.get("collision_status", 0.0) >= 0.5
            row = _freeze_removed(
                prev_row,
                in_collision=just_removed and uid_str in uav_ids_in_collision,
                in_ra_collision=just_removed and uid_str in uav_ids_in_ra_collision,
            )
            mask[slot] = 0.0
        new_last_known[uid_str] = row

        for feat_idx, key in enumerate(UAV_NODE_KEYS):
            uav_x[slot, feat_idx] = row[key]
        uav_positions[slot] = [row["x"], row["y"], row["z"]]

    data["uav"].x = uav_x
    data["uav"].mask = mask

    # --- UAV-UAV edges (only among real slots) ---
    real_slots = mask.nonzero(as_tuple=True)[0].numpy()
    uav_ei = _build_uav_edges_among(real_slots, uav_positions, uav_edge_type, uav_edge_distance)
    data["uav", "communicates_with", "uav"].edge_index = uav_ei
    data["uav", "communicates_with", "uav"].edge_attr = _compute_uav_edge_attr(
        uav_positions, uav_ei
    )

    # --- Cross-graph edges: UAV -> vertiport (only real slots) ---
    # First edge: nearest vertiport by current position (spatial context for an
    # in-flight UAV, which has no single discrete "current vertiport").
    # Second edge: true target vertiport (replaces the old nearest/second-nearest
    # position heuristic) -- found by nearest-match against target_x/target_y,
    # which is exact since those columns hold the target vertiport's own
    # coordinates whenever a real target is resolved.
    cross_src, cross_dst = [], []
    if len(real_slots) > 0 and n_vp > 0:
        vp_positions_arr = vp_x[:, :2].numpy()
        for ui in real_slots:
            ui = int(ui)
            uav_pos_2d = uav_positions[ui, :2]
            nearest = int(np.argmin(np.linalg.norm(vp_positions_arr - uav_pos_2d, axis=1)))
            cross_src.append(ui)
            cross_dst.append(nearest)

            target_xy = np.array(
                [uav_x[ui, _TARGET_X_IDX].item(), uav_x[ui, _TARGET_Y_IDX].item()]
            )
            target_vp = int(np.argmin(np.linalg.norm(vp_positions_arr - target_xy, axis=1)))
            cross_src.append(ui)
            cross_dst.append(target_vp)
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
    data.total_uavs = mask.sum().unsqueeze(0)  # live count this step, not max_uavs
    data.step = torch.tensor([step.get("step", 0)], dtype=torch.long)

    return data, new_last_known


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

        # Per-episode UAV slot mapping, fixed for the whole episode: the fleet
        # is fixed-then-shrinking (ATC only creates UAVs at reset, only removes
        # during step()), so the first step's UAV set is exactly max_uavs -- the
        # true total ever present in this episode.
        first_uav_ids = sorted(steps[0]["uavs"].keys(), key=int)
        uav_id_to_slot = {uid: slot for slot, uid in enumerate(first_uav_ids)}
        max_uavs = len(uav_id_to_slot)
        if max_uavs == 0:
            return

        last_known: Dict[str, Dict[str, float]] = {}
        graphs: List[HeteroData] = []
        for step in steps:
            g, last_known = _step_to_hetero(
                step, uav_id_to_slot, max_uavs, last_known,
                self.uav_edge_type, self.uav_edge_distance, vp_edge_index,
            )
            graphs.append(g)

        for t in range(len(graphs) - 1):
            self._pairs.append((graphs[t], graphs[t + 1]))

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
