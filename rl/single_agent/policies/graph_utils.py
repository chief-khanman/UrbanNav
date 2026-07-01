"""
graph_utils.py
==============
Shared obs-slicing and per-sample star-graph construction for the
RNN/GNN/GNN+RNN feature extractors in this package (rnn_extractor.py,
gat_encoder.py, gnn_extractor.py, gnn_rnn_extractor.py).

Single source of truth for the existing flat-vector obs layout produced by
rl/common/agent_logic.py:extract_observation (self_obs(9) +
intruder_obs(6*n_intruder) [+ ra_obs(1)]), so the extractor modules don't
each re-derive the same index arithmetic.

This module intentionally holds no learnable parameters — only tensor
slicing / graph-construction logic. It is the seam plan_02/03's future
graph-based KnowledgeBase is expected to extend (adding KB nodes/edges to
build_star_graph) without touching any GNN/RNN forward pass.
"""
from __future__ import annotations

from typing import Tuple

import torch

SELF_DIM = 9
INTRUDER_SLOT_DIM = 6
RA_DIM = 1
REAL_SLOT_EPS = 1e-8


def split_obs_blocks(
    obs: torch.Tensor, n_intruder: int, has_ra: bool,
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Slice a batch of flat obs vectors into (self_block, intruder_block, ra_scalar).

    Args:
        obs: [batch, D] float tensor, D = 9 + 6*n_intruder [+ 1].
        n_intruder: number of intruder slots N encoded in the obs.
        has_ra: whether the trailing RA scalar is present in `obs`.

    Returns:
        self_block:     [batch, 9]
        intruder_block: [batch, n_intruder, 6]
        ra_scalar:      [batch, 1] (zeros if has_ra is False)
    """
    self_block = obs[:, :SELF_DIM]
    intruder_end = SELF_DIM + INTRUDER_SLOT_DIM * n_intruder
    intruder_block = obs[:, SELF_DIM:intruder_end].reshape(-1, n_intruder, INTRUDER_SLOT_DIM)
    if has_ra:
        ra_scalar = obs[:, intruder_end:intruder_end + RA_DIM]
    else:
        ra_scalar = torch.zeros(obs.shape[0], RA_DIM, dtype=obs.dtype, device=obs.device)
    return self_block, intruder_block, ra_scalar


def real_intruder_mask(intruder_block: torch.Tensor) -> torch.Tensor:
    """
    Recover which intruder slots are real vs zero-padded.

    agent_logic.extract_observation sorts detected intruders closest-first and
    zero-pads any unfilled trailing slots (rl/common/agent_logic.py:122-134) —
    there is no explicit mask channel in the obs space, so a slot is "real"
    iff its 6-feature row's L2 norm is above a small epsilon.

    Caveat: this misclassifies a real intruder that is simultaneously exactly
    co-located with ego AND has exactly zero relative velocity. The
    radius/collision system fires before that geometry occurs in practice, so
    it's treated as an acceptable approximation rather than special-cased.

    Args:
        intruder_block: [batch, n_intruder, 6]

    Returns:
        mask: [batch, n_intruder] bool tensor, True where the slot is real.
    """
    return intruder_block.norm(dim=-1) > REAL_SLOT_EPS


def build_star_graph(
    self_block_1: torch.Tensor,
    intruder_block_1: torch.Tensor,
    mask_1: torch.Tensor,
) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Build a single-sample ego-centric star graph from one row of obs blocks.

    Node 0 is always ego (raw feature = self_block_1, the 9-d self-obs row).
    Nodes 1..n_real are the real (non-padded) intruders, kept in their
    existing closest-first order. A node's raw feature is its relative-
    POSITION 3-d (rel_x, rel_y, rel_z) — "where" the other agent is. The
    matching relative-VELOCITY 3-d becomes the edge attribute of that node's
    two directed edges to/from ego instead of being duplicated onto the node
    — "how the relationship is changing" is a relational quantity, so it
    belongs on the edge. This node/edge split uses only data already present
    in the existing flat obs (no simulator/obs-schema change).

    Topology is star-only: ego<->each real intruder, no intruder-intruder
    edges (matches CADRL's ego-centric framing; a full mesh of
    intruder-intruder edges is a documented future extension, not built
    here).

    Returns raw (unencoded) features — callers apply their own per-node-type
    linear encoders before any graph convolution, since ego's raw width (9)
    and an intruder's raw width (3) differ and must be projected into a
    shared hidden width first (see gat_encoder.py).

    Args:
        self_block_1:     [9]            one ego self-obs row.
        intruder_block_1: [n_intruder, 6] one sample's intruder rows.
        mask_1:           [n_intruder]    bool, True where the row is real.

    Returns:
        ego_raw:           [9]            unchanged self_block_1.
        intruder_pos_raw:  [n_real, 3]    relative position of each real intruder.
        edge_index:        [2, 2*n_real]  bidirectional ego(node 0)<->intruder edges.
        edge_attr_raw:     [2*n_real, 3]  relative velocity, duplicated per direction.
    """
    device = self_block_1.device
    real_rows = intruder_block_1[mask_1]  # [n_real, 6], closest-first order preserved
    n_real = real_rows.shape[0]

    ego_raw = self_block_1
    intruder_pos_raw = real_rows[:, :3]
    rel_vel_raw = real_rows[:, 3:6]

    if n_real == 0:
        edge_index = torch.zeros((2, 0), dtype=torch.long, device=device)
        edge_attr_raw = torch.zeros((0, 3), dtype=self_block_1.dtype, device=device)
        return ego_raw, intruder_pos_raw, edge_index, edge_attr_raw

    intruder_node_ids = torch.arange(1, n_real + 1, device=device, dtype=torch.long)
    ego_node_id = torch.zeros(n_real, device=device, dtype=torch.long)
    # ego -> intruder and intruder -> ego, so attention can flow both ways
    src = torch.cat([ego_node_id, intruder_node_ids])
    dst = torch.cat([intruder_node_ids, ego_node_id])
    edge_index = torch.stack([src, dst], dim=0)
    edge_attr_raw = torch.cat([rel_vel_raw, rel_vel_raw], dim=0)

    return ego_raw, intruder_pos_raw, edge_index, edge_attr_raw
