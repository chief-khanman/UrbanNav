"""
dual_graph_gnn.py
==================
Surrogate Model 2: Dual-graph GNN with static vertiport graph and dynamic
UAV-UAV graph for next-step state prediction.

Architecture: Heterogeneous encode-process-decode GNN operating on two
composed graphs — a frozen vertiport spatial graph and a per-step dynamic
UAV interaction graph — with cross-graph message passing for information
exchange between the two levels.

**Papers and inspirations:**

- **MeshGraphNets** (Pfaff et al., ICLR 2021,
  https://arxiv.org/abs/2010.03409):
  Multi-mesh encode-process-decode with world-space mesh (static) and
  internal simulation mesh (dynamic).  Direct architectural template for
  our dual-graph composition: the vertiport graph is analogous to the
  world-space mesh (fixed topology, provides spatial context), while the
  UAV graph is analogous to the simulation mesh (dynamic topology, carries
  the evolving state).  Message-passing block structure and residual
  decoding follow MeshGraphNets conventions.

- **Interaction Networks** (Battaglia et al., NeurIPS 2016,
  https://arxiv.org/abs/1612.00222):
  Object-relation decomposition: objects (UAVs) interact via learned
  relation functions (edges), and external effects (vertiport context)
  modulate predictions.  Our cross-graph VP→UAV message passing implements
  this external-effect pathway.

- **Heterogeneous Graph Transformers** (Hu et al., WWW 2020,
  https://arxiv.org/abs/2003.01332):
  Meta-path-based attention across different node types and edge types in
  a unified heterogeneous graph.  Motivates our use of typed edges
  (UAV-UAV, VP-VP, UAV↔VP) within a single HeteroData structure, enabling
  type-specific encoders and message functions.

- **MultiScale MeshGraphNets** (Fortunato et al., ICML 2022,
  https://arxiv.org/abs/2210.00612):
  Coarse and fine resolution meshes with inter-mesh edges for multi-scale
  information exchange.  Our vertiport graph (coarse, spatial) and UAV
  graph (fine, agent-level) with cross-graph edges mirrors this multi-scale
  pattern: vertiports aggregate regional state, UAVs carry local dynamics.

- **DynamicalGraphNet** (Nauck et al., Nature Communications 2025,
  https://www.nature.com/articles/s41467-025-67802-5):
  Physics-informed GNN with conservation constraints on dynamic graphs.
  Informs our UAV count conservation projection, applied post-decode to
  enforce that total active UAVs are preserved across timesteps.

- **GNS / Learning to Simulate** (Sanchez-Gonzalez et al., ICML 2020,
  https://arxiv.org/abs/2002.09405):
  Noise injection during training for stable autoregressive rollout.
  Applied to UAV node features during training to improve multi-step
  prediction robustness.

Collision handling:
  The model predicts collision_status as part of UAV node features.
  Polarity is configurable (active_high: 1=active/0=collided, or
  collided_high: 0=active/1=collided).  Post-decode, collided nodes
  have velocity predictions zeroed out via masking.
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.data import HeteroData
from torch_geometric.utils import degree

from rl.surrogate.backbones.surrogate_template import SurrogateModel
from rl.surrogate.datasets.dual_graph_dataset import UAV_EDGE_DIM, UAV_NODE_DIM, VP_NODE_DIM


class _MLP(nn.Module):
    def __init__(self, in_dim: int, hidden_dim: int, out_dim: int, layers: int = 2):
        super().__init__()
        mods = [nn.Linear(in_dim, hidden_dim), nn.ReLU()]
        for _ in range(layers - 2):
            mods += [nn.Linear(hidden_dim, hidden_dim), nn.ReLU()]
        mods.append(nn.Linear(hidden_dim, out_dim))
        self.net = nn.Sequential(*mods)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class _HomoMessagePassingBlock(nn.Module):
    """One round of message passing on a single-type graph: edge update -> node update."""

    def __init__(self, hidden_dim: int):
        super().__init__()
        self.edge_mlp = _MLP(hidden_dim * 3, hidden_dim, hidden_dim)
        self.node_mlp = _MLP(hidden_dim * 2, hidden_dim, hidden_dim)
        self.edge_norm = nn.LayerNorm(hidden_dim)
        self.node_norm = nn.LayerNorm(hidden_dim)

    def forward(
        self,
        h_node: torch.Tensor,
        h_edge: torch.Tensor,
        edge_index: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        src, dst = edge_index[0], edge_index[1]

        edge_input = torch.cat([h_node[src], h_node[dst], h_edge], dim=-1)
        h_edge_new = self.edge_norm(h_edge + self.edge_mlp(edge_input))

        # Mean (not sum) over incoming edges: in-degree varies with both fleet
        # size (UAV-UAV graph) and vertiport count (VP-VP graph) across the
        # sweep dataset's varied configs -- an unnormalized sum would bias
        # aggregated message magnitude by graph size alone, hurting
        # generalization across configs (mirrors the same fix in graph_flow_gnn.py).
        agg = torch.zeros_like(h_node)
        agg.scatter_add_(0, dst.unsqueeze(-1).expand(-1, h_edge_new.shape[-1]), h_edge_new)
        in_degree = degree(dst, num_nodes=h_node.shape[0], dtype=h_edge_new.dtype).clamp(min=1).unsqueeze(-1)
        agg = agg / in_degree
        node_input = torch.cat([h_node, agg], dim=-1)
        h_node_new = self.node_norm(h_node + self.node_mlp(node_input))

        return h_node_new, h_edge_new


class _CrossGraphBlock(nn.Module):
    """Bidirectional cross-graph message passing: UAV <-> vertiport."""

    def __init__(self, hidden_dim: int):
        super().__init__()
        self.uav_to_vp_mlp = _MLP(hidden_dim * 2, hidden_dim, hidden_dim)
        self.vp_to_uav_mlp = _MLP(hidden_dim * 2, hidden_dim, hidden_dim)
        self.uav_norm = nn.LayerNorm(hidden_dim)
        self.vp_norm = nn.LayerNorm(hidden_dim)

    def forward(
        self,
        h_uav: torch.Tensor,
        h_vp: torch.Tensor,
        uav_to_vp_ei: torch.Tensor,
        vp_to_uav_ei: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        # UAV -> VP: aggregate UAV info at vertiport nodes (mean, not sum -- see
        # _HomoMessagePassingBlock for why: cross-graph in-degree also varies
        # with fleet/vertiport size across the sweep dataset's configs).
        if uav_to_vp_ei.shape[1] > 0:
            src, dst = uav_to_vp_ei[0], uav_to_vp_ei[1]
            agg_at_vp = torch.zeros_like(h_vp)
            agg_at_vp.scatter_add_(
                0, dst.unsqueeze(-1).expand(-1, h_uav.shape[-1]), h_uav[src]
            )
            vp_in_degree = degree(dst, num_nodes=h_vp.shape[0], dtype=h_uav.dtype).clamp(min=1).unsqueeze(-1)
            agg_at_vp = agg_at_vp / vp_in_degree
            vp_input = torch.cat([h_vp, agg_at_vp], dim=-1)
            h_vp = self.vp_norm(h_vp + self.uav_to_vp_mlp(vp_input))

        # VP -> UAV: aggregate vertiport context at UAV nodes
        if vp_to_uav_ei.shape[1] > 0:
            src, dst = vp_to_uav_ei[0], vp_to_uav_ei[1]
            agg_at_uav = torch.zeros_like(h_uav)
            agg_at_uav.scatter_add_(
                0, dst.unsqueeze(-1).expand(-1, h_vp.shape[-1]), h_vp[src]
            )
            uav_in_degree = degree(dst, num_nodes=h_uav.shape[0], dtype=h_vp.dtype).clamp(min=1).unsqueeze(-1)
            agg_at_uav = agg_at_uav / uav_in_degree
            uav_input = torch.cat([h_uav, agg_at_uav], dim=-1)
            h_uav = self.uav_norm(h_uav + self.vp_to_uav_mlp(uav_input))

        return h_uav, h_vp


def uav_conservation_projection(
    uav_status: torch.Tensor, total_active: torch.Tensor
) -> torch.Tensor:
    """Project predicted active-UAV counts to sum to total_active."""
    uav_status = F.relu(uav_status)
    current_sum = uav_status.sum()
    if current_sum < 1e-8:
        return uav_status
    scale = total_active.squeeze() / current_sum
    return uav_status * scale


class DualGraphGNN(SurrogateModel):
    """Dual-graph heterogeneous GNN for UAV-level next-state prediction.

    Operates on a HeteroData graph with two node types (uav, vertiport)
    and four edge types (uav-uav, vp-vp, uav->vp, vp->uav).

    Predicts residual changes to UAV node features and vertiport node
    features, then applies conservation projection.
    """

    COLLISION_STATUS_IDX = 8  # index of collision_status in UAV_NODE_KEYS
    VEL_INDICES = [3, 4, 5]  # vx, vy, vz indices in UAV_NODE_KEYS
    IN_NMAC_EVENT_IDX = 9  # index of in_nmac_event in UAV_NODE_KEYS
    IN_COLLISION_EVENT_IDX = 10  # index of in_collision_event in UAV_NODE_KEYS
    IN_RA_COLLISION_EVENT_IDX = 11  # index of in_ra_collision_event in UAV_NODE_KEYS
    MISSIONS_COMPLETED_IDX = 12  # index of missions_completed in UAV_NODE_KEYS
    EVENT_FLAG_INDICES = [9, 10, 11]  # in_nmac/in_collision/in_ra_collision -- all [0,1] flags

    def __init__(
        self,
        hidden_dim: int = 64,
        num_mp_rounds: int = 3,
        noise_std: float = 3e-4,
    ):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_mp_rounds = num_mp_rounds
        self.noise_std = noise_std

        # Encoders
        self.uav_encoder = _MLP(UAV_NODE_DIM, hidden_dim, hidden_dim)
        self.vp_encoder = _MLP(VP_NODE_DIM, hidden_dim, hidden_dim)
        self.uav_edge_encoder = _MLP(UAV_EDGE_DIM, hidden_dim, hidden_dim)

        # Message passing blocks
        self.uav_mp_blocks = nn.ModuleList(
            [_HomoMessagePassingBlock(hidden_dim) for _ in range(num_mp_rounds)]
        )
        self.vp_mp_blocks = nn.ModuleList(
            [_HomoMessagePassingBlock(hidden_dim) for _ in range(num_mp_rounds)]
        )
        self.cross_blocks = nn.ModuleList(
            [_CrossGraphBlock(hidden_dim) for _ in range(num_mp_rounds)]
        )

        # Decoders
        self.uav_decoder = _MLP(hidden_dim, hidden_dim, UAV_NODE_DIM)
        self.vp_decoder = _MLP(hidden_dim, hidden_dim, VP_NODE_DIM)

    def forward(self, data: HeteroData) -> Dict[str, torch.Tensor]:
        """Predict next-step node features for UAVs and vertiports.

        Args:
            data: HeteroData with node types 'uav' and 'vertiport'.

        Returns:
            Dict with 'uav_x' and 'vp_x' predicted feature tensors.
        """
        uav_x = data["uav"].x
        vp_x = data["vertiport"].x

        uav_uav_ei = data["uav", "communicates_with", "uav"].edge_index
        uav_uav_ea = data["uav", "communicates_with", "uav"].edge_attr
        vp_vp_ei = data["vertiport", "connected_to", "vertiport"].edge_index
        uav_to_vp_ei = data["uav", "assigned_to", "vertiport"].edge_index
        vp_to_uav_ei = data["vertiport", "hosts", "uav"].edge_index

        # GNS-style noise injection during training
        if self.training and self.noise_std > 0:
            uav_x = uav_x + torch.randn_like(uav_x) * self.noise_std

        # Encode
        h_uav = self.uav_encoder(uav_x)
        h_vp = self.vp_encoder(vp_x)

        # Encode UAV-UAV edges (VP edges use node embeddings directly)
        if uav_uav_ei.shape[1] > 0:
            h_uav_edge = self.uav_edge_encoder(uav_uav_ea)
        else:
            h_uav_edge = torch.zeros((0, self.hidden_dim), device=uav_x.device)

        # VP-VP edges: initialize from endpoint node embeddings
        if vp_vp_ei.shape[1] > 0:
            vp_edge_src = h_vp[vp_vp_ei[0]]
            vp_edge_dst = h_vp[vp_vp_ei[1]]
            h_vp_edge = (vp_edge_src + vp_edge_dst) / 2.0
        else:
            h_vp_edge = torch.zeros((0, self.hidden_dim), device=vp_x.device)

        # Process: N rounds of parallel message passing + cross-graph exchange
        for uav_mp, vp_mp, cross in zip(
            self.uav_mp_blocks, self.vp_mp_blocks, self.cross_blocks
        ):
            h_uav, h_uav_edge = uav_mp(h_uav, h_uav_edge, uav_uav_ei)
            h_vp, h_vp_edge = vp_mp(h_vp, h_vp_edge, vp_vp_ei)
            h_uav, h_vp = cross(h_uav, h_vp, uav_to_vp_ei, vp_to_uav_ei)

        # Decode: residual prediction
        uav_delta = self.uav_decoder(h_uav)
        vp_delta = self.vp_decoder(h_vp)

        pred_uav_x = uav_x + uav_delta
        pred_vp_x = vp_x + vp_delta

        return {"uav_x": pred_uav_x, "vp_x": pred_vp_x}

    def predict_next_state(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError("DualGraphGNN uses predict_dual_graph_next_state instead")

    def predict_episode_outcome(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        raise NotImplementedError("DualGraphGNN is a next-state model, not episode-outcome")

    def predict_dual_graph_next_state(self, data: HeteroData) -> Dict[str, torch.Tensor]:
        """Full prediction pipeline with conservation projection, collision
        masking, and the "dead stays dead" invariant for already-removed slots.
        """
        input_uav_x = data["uav"].x
        preds = self.forward(data)
        pred_uav = preds["uav_x"]

        # mask_in: which slots were still real going INTO this step. Ground
        # truth during teacher-forced training (data["uav"].mask, set by
        # DualGraphDataset); during autoregressive rollout there's no ground
        # truth, so callers should thread the previous call's returned mask
        # back in as data["uav"].mask (see rollout_metrics.py). Falls back to
        # "everything real" for graphs that predate this fixed-size padding
        # scheme (e.g. older hand-built test graphs).
        if hasattr(data["uav"], "mask"):
            mask_in = data["uav"].mask
        else:
            mask_in = torch.ones(pred_uav.shape[0], device=pred_uav.device)
        real_in = mask_in > 0.5
        dead_in = ~real_in

        # Conservation: ensure total active UAVs is preserved, restricted to
        # currently-real slots only -- an already-dead slot's predicted
        # collision_status is meaningless noise (it gets fully overridden
        # below anyway) and would otherwise pollute the rescale factor.
        if hasattr(data, "total_uavs") and real_in.any():
            status_pred_real = pred_uav[real_in, self.COLLISION_STATUS_IDX]
            status_proj_real = uav_conservation_projection(status_pred_real, data.total_uavs)
            pred_uav = pred_uav.clone()
            pred_uav[real_in, self.COLLISION_STATUS_IDX] = status_proj_real

        # Zero out velocities for collided UAVs (status near 0 for active_high)
        status = pred_uav[:, self.COLLISION_STATUS_IDX]
        collision_mask = (status < 0.5).unsqueeze(-1)
        if collision_mask.any():
            pred_uav = pred_uav.clone()
            for vi in self.VEL_INDICES:
                pred_uav[:, vi] = pred_uav[:, vi].masked_fill(collision_mask.squeeze(-1), 0.0)

        # Event flags (in_nmac/in_collision/in_ra_collision) are [0,1] flags,
        # not free-floating residuals -- sigmoid (not ReLU) since they're
        # boolean-like rather than count-valued (unlike graph_flow_gnn.py's
        # nmac_count, which is a per-edge count).
        pred_uav = pred_uav.clone()
        for idx in self.EVENT_FLAG_INDICES:
            pred_uav[:, idx] = torch.sigmoid(pred_uav[:, idx])

        # missions_completed can't go negative or decrease -- a UAV never
        # "un-completes" a mission. Clone the read *before* using it in
        # torch.maximum: writing the result back into pred_uav[:, IDX] in
        # place would otherwise corrupt the exact storage maximum's backward
        # needs to inspect (it's a view into pred_uav, not a copy).
        current_missions = pred_uav[:, self.MISSIONS_COMPLETED_IDX].clone()
        pred_uav = pred_uav.clone()
        pred_uav[:, self.MISSIONS_COMPLETED_IDX] = torch.maximum(
            current_missions, input_uav_x[:, self.MISSIONS_COMPLETED_IDX]
        )

        # "Dead stays dead": once a slot is removed, it must never come back --
        # force already-dead slots back to their exact (already-frozen) input
        # values, overriding anything the decoder predicted for them. This is
        # the model-side analog of GraphFlowGNN's monotonic total_uavs decrement.
        if dead_in.any():
            pred_uav = pred_uav.clone()
            pred_uav[dead_in] = input_uav_x[dead_in]
            # in_collision_event/in_ra_collision_event are one-shot, transition-step-
            # only signals in ground truth (DualGraphDataset._freeze_removed only
            # sets them True the single step a slot first goes dead, then 0
            # forever after) -- force them to 0 here too, rather than letting an
            # already-dead slot perpetuate a stale 1.0 forward indefinitely
            # across a multi-step rollout.
            pred_uav[dead_in, self.IN_COLLISION_EVENT_IDX] = 0.0
            pred_uav[dead_in, self.IN_RA_COLLISION_EVENT_IDX] = 0.0

        # Next step's mask: real_in AND not newly predicted to have collided
        # this step (predicted collision_status >= 0.5) -- monotonic, since
        # dead_in slots are already forced to status=0 above and real_in=False
        # forces this to 0 regardless of the (overridden, meaningless) status.
        next_mask = real_in.float() * (pred_uav[:, self.COLLISION_STATUS_IDX] >= 0.5).float()

        preds["uav_x"] = pred_uav
        preds["uav_mask"] = next_mask
        return preds
