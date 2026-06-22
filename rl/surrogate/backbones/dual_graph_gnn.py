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

        agg = torch.zeros_like(h_node)
        agg.scatter_add_(0, dst.unsqueeze(-1).expand(-1, h_edge_new.shape[-1]), h_edge_new)
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
        # UAV -> VP: aggregate UAV info at vertiport nodes
        if uav_to_vp_ei.shape[1] > 0:
            src, dst = uav_to_vp_ei[0], uav_to_vp_ei[1]
            agg_at_vp = torch.zeros_like(h_vp)
            agg_at_vp.scatter_add_(
                0, dst.unsqueeze(-1).expand(-1, h_uav.shape[-1]), h_uav[src]
            )
            vp_input = torch.cat([h_vp, agg_at_vp], dim=-1)
            h_vp = self.vp_norm(h_vp + self.uav_to_vp_mlp(vp_input))

        # VP -> UAV: aggregate vertiport context at UAV nodes
        if vp_to_uav_ei.shape[1] > 0:
            src, dst = vp_to_uav_ei[0], vp_to_uav_ei[1]
            agg_at_uav = torch.zeros_like(h_uav)
            agg_at_uav.scatter_add_(
                0, dst.unsqueeze(-1).expand(-1, h_vp.shape[-1]), h_vp[src]
            )
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
        """Full prediction pipeline with conservation projection and collision masking."""
        preds = self.forward(data)
        pred_uav = preds["uav_x"]

        # Conservation: ensure total active UAVs is preserved
        if hasattr(data, "total_uavs"):
            status_pred = pred_uav[:, self.COLLISION_STATUS_IDX]
            status_proj = uav_conservation_projection(status_pred, data.total_uavs)
            pred_uav = pred_uav.clone()
            pred_uav[:, self.COLLISION_STATUS_IDX] = status_proj

        # Zero out velocities for collided UAVs (status near 0 for active_high)
        status = pred_uav[:, self.COLLISION_STATUS_IDX]
        collision_mask = (status < 0.5).unsqueeze(-1)
        if collision_mask.any():
            pred_uav = pred_uav.clone()
            for vi in self.VEL_INDICES:
                pred_uav[:, vi] = pred_uav[:, vi].masked_fill(collision_mask.squeeze(-1), 0.0)

        preds["uav_x"] = pred_uav
        return preds
