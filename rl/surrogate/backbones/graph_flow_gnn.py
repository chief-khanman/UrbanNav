"""
graph_flow_gnn.py
==================
Surrogate Model 1: discrete-time graph network for vertiport-edge UAV flow.

Nodes = vertiports, edges = flight paths. Predicts next-step node and edge
attributes (UAV counts) with a hard conservation constraint ensuring total
UAV count is preserved across timesteps.

Architecture follows the encode-process-decode pattern (GNS / MeshGraphNets)
with a conservation projection layer (Beucler et al., 2019).

Two variants:
  - GraphFlowGNN: stateless (no temporal memory)
  - GraphFlowRecurrentGNN: per-node GRU hidden state across timesteps
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.data import Data

from rl.surrogate.backbones.surrogate_template import SurrogateModel
from rl.surrogate.datasets.graph_flow_dataset import EDGE_ATTR_DIM, NODE_ATTR_DIM


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------


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


class _MessagePassingBlock(nn.Module):
    """One round of message passing: edge update → node update → global update."""

    def __init__(self, hidden_dim: int):
        super().__init__()
        self.edge_mlp = _MLP(hidden_dim * 3 + hidden_dim, hidden_dim, hidden_dim)
        self.node_mlp = _MLP(hidden_dim * 2, hidden_dim, hidden_dim)
        self.global_mlp = _MLP(hidden_dim * 3, hidden_dim, hidden_dim)
        self.edge_norm = nn.LayerNorm(hidden_dim)
        self.node_norm = nn.LayerNorm(hidden_dim)

    def forward(
        self,
        h_node: torch.Tensor,
        h_edge: torch.Tensor,
        h_global: torch.Tensor,
        edge_index: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        src, dst = edge_index[0], edge_index[1]
        n_nodes = h_node.shape[0]

        # Edge update: f_e([h_src, h_dst, h_edge, h_global])
        h_global_exp = h_global.expand(h_edge.shape[0], -1)
        edge_input = torch.cat([h_node[src], h_node[dst], h_edge, h_global_exp], dim=-1)
        h_edge_new = self.edge_norm(h_edge + self.edge_mlp(edge_input))

        # Node update: f_v([h_node, agg_incoming_edges])
        agg = torch.zeros_like(h_node)
        agg.scatter_add_(0, dst.unsqueeze(-1).expand(-1, h_edge_new.shape[-1]), h_edge_new)
        node_input = torch.cat([h_node, agg], dim=-1)
        h_node_new = self.node_norm(h_node + self.node_mlp(node_input))

        # Global update: f_u([h_global, mean(h_node'), mean(h_edge')])
        global_input = torch.cat(
            [h_global, h_node_new.mean(dim=0, keepdim=True), h_edge_new.mean(dim=0, keepdim=True)],
            dim=-1,
        )
        h_global_new = h_global + self.global_mlp(global_input)

        return h_node_new, h_edge_new, h_global_new


# ---------------------------------------------------------------------------
# Conservation projection
# ---------------------------------------------------------------------------


def conservation_projection(
    node_uavs: torch.Tensor,
    edge_uavs: torch.Tensor,
    total_uavs: torch.Tensor,
) -> Tuple[torch.Tensor, torch.Tensor]:
    """Differentiable projection enforcing sum(node_uavs) + sum(edge_uavs) = total_uavs.

    Clamps negatives to zero, then redistributes the residual proportionally.
    """
    node_uavs = F.relu(node_uavs)
    edge_uavs = F.relu(edge_uavs)

    current_sum = node_uavs.sum() + edge_uavs.sum()
    if current_sum < 1e-8:
        return node_uavs, edge_uavs

    scale = total_uavs.squeeze() / current_sum
    return node_uavs * scale, edge_uavs * scale


# ---------------------------------------------------------------------------
# GraphFlowGNN (stateless)
# ---------------------------------------------------------------------------


class GraphFlowGNN(SurrogateModel):
    """Encode-process-decode GNN for vertiport-edge UAV flow prediction.

    Predicts residual changes in node/edge UAV counts and applies a hard
    conservation projection to ensure total UAV count is preserved.
    """

    # Indices into node/edge feature vectors for the UAV-count channels
    NODE_UAV_INDICES = [0, 1]  # n_grounded, n_landing_queue
    EDGE_UAV_INDEX = 0  # n_in_transit

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

        self.node_encoder = _MLP(NODE_ATTR_DIM, hidden_dim, hidden_dim)
        self.edge_encoder = _MLP(EDGE_ATTR_DIM, hidden_dim, hidden_dim)
        self.global_encoder = _MLP(3, hidden_dim, hidden_dim)  # [total_uavs, step, dt]

        self.mp_blocks = nn.ModuleList(
            [_MessagePassingBlock(hidden_dim) for _ in range(num_mp_rounds)]
        )

        self.node_decoder = _MLP(hidden_dim, hidden_dim, NODE_ATTR_DIM)
        self.edge_decoder = _MLP(hidden_dim, hidden_dim, EDGE_ATTR_DIM)

    def predict_graph_next_state(self, graph: Data) -> Data:
        x = graph.x  # [N, NODE_ATTR_DIM]
        edge_attr = graph.edge_attr  # [E, EDGE_ATTR_DIM]
        edge_index = graph.edge_index  # [2, E]
        total_uavs = graph.total_uavs  # [1]

        # GNS-style noise injection during training
        if self.training and self.noise_std > 0:
            x = x + torch.randn_like(x) * self.noise_std
            edge_attr = edge_attr + torch.randn_like(edge_attr) * self.noise_std

        # Encode
        h_node = self.node_encoder(x)
        h_edge = self.edge_encoder(edge_attr)

        step_val = graph.step.float() if hasattr(graph, "step") else torch.zeros(1, device=x.device)
        global_input = torch.cat(
            [total_uavs.view(1, 1), step_val.view(1, 1), torch.ones(1, 1, device=x.device)],
            dim=-1,
        )
        h_global = self.global_encoder(global_input)  # [1, hidden_dim]

        # Process
        for mp in self.mp_blocks:
            h_node, h_edge, h_global = mp(h_node, h_edge, h_global, edge_index)

        # Decode (residual)
        delta_node = self.node_decoder(h_node)
        delta_edge = self.edge_decoder(h_edge)
        next_x = x + delta_node
        next_edge_attr = edge_attr + delta_edge

        # Conservation projection on UAV-count channels only
        # Clamp sub-channels non-negative before computing fractions
        node_uavs = F.relu(next_x[:, self.NODE_UAV_INDICES].clone())
        edge_uavs = next_edge_attr[:, self.EDGE_UAV_INDEX].clone()

        proj_node, proj_edge = conservation_projection(
            node_uavs.sum(dim=-1),  # total UAVs at each node
            edge_uavs,
            total_uavs,
        )

        # Redistribute projected node UAVs back to grounded/queue proportionally
        node_total_raw = node_uavs.sum(dim=-1, keepdim=True).clamp(min=1e-8)
        node_fracs = node_uavs / node_total_raw
        next_x[:, self.NODE_UAV_INDICES] = node_fracs * proj_node.unsqueeze(-1)
        next_edge_attr[:, self.EDGE_UAV_INDEX] = proj_edge

        result = Data(
            x=next_x,
            edge_index=edge_index,
            edge_attr=next_edge_attr,
            total_uavs=total_uavs,
            step=graph.step + 1 if hasattr(graph, "step") else torch.tensor([1]),
        )
        return result

    def predict_next_state(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError("GraphFlowGNN operates on graph Data, not per-UAV tensors")

    def predict_episode_outcome(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        raise NotImplementedError("GraphFlowGNN does not support episode outcome prediction")


# ---------------------------------------------------------------------------
# GraphFlowRecurrentGNN (with per-node GRU temporal memory)
# ---------------------------------------------------------------------------


class GraphFlowRecurrentGNN(GraphFlowGNN):
    """GraphFlowGNN extended with per-node GRU for temporal memory.

    Maintains hidden states across timesteps to capture multi-step transit
    delays (a UAV takes several steps to traverse an edge).
    """

    def __init__(
        self,
        hidden_dim: int = 64,
        num_mp_rounds: int = 3,
        noise_std: float = 3e-4,
    ):
        super().__init__(hidden_dim=hidden_dim, num_mp_rounds=num_mp_rounds, noise_std=noise_std)
        self.node_gru = nn.GRUCell(hidden_dim, hidden_dim)
        self._h_node_prev: Optional[torch.Tensor] = None

    def reset_hidden(self) -> None:
        self._h_node_prev = None

    def predict_graph_next_state(self, graph: Data) -> Data:
        x = graph.x
        edge_attr = graph.edge_attr
        edge_index = graph.edge_index
        total_uavs = graph.total_uavs

        if self.training and self.noise_std > 0:
            x = x + torch.randn_like(x) * self.noise_std
            edge_attr = edge_attr + torch.randn_like(edge_attr) * self.noise_std

        h_node = self.node_encoder(x)
        h_edge = self.edge_encoder(edge_attr)

        step_val = graph.step.float() if hasattr(graph, "step") else torch.zeros(1, device=x.device)
        global_input = torch.cat(
            [total_uavs.view(1, 1), step_val.view(1, 1), torch.ones(1, 1, device=x.device)],
            dim=-1,
        )
        h_global = self.global_encoder(global_input)

        # Inject temporal memory via GRU before message passing
        if self._h_node_prev is not None and self._h_node_prev.shape[0] == h_node.shape[0]:
            h_node = self.node_gru(h_node, self._h_node_prev)
        else:
            h_node = self.node_gru(h_node, torch.zeros_like(h_node))

        for mp in self.mp_blocks:
            h_node, h_edge, h_global = mp(h_node, h_edge, h_global, edge_index)

        # Store for next timestep (detach to prevent BPTT across episodes)
        self._h_node_prev = h_node.detach()

        delta_node = self.node_decoder(h_node)
        delta_edge = self.edge_decoder(h_edge)
        next_x = x + delta_node
        next_edge_attr = edge_attr + delta_edge

        node_uavs = F.relu(next_x[:, self.NODE_UAV_INDICES].clone())
        edge_uavs = next_edge_attr[:, self.EDGE_UAV_INDEX].clone()

        proj_node, proj_edge = conservation_projection(
            node_uavs.sum(dim=-1),
            edge_uavs,
            total_uavs,
        )

        node_total_raw = node_uavs.sum(dim=-1, keepdim=True).clamp(min=1e-8)
        node_fracs = node_uavs / node_total_raw
        next_x[:, self.NODE_UAV_INDICES] = node_fracs * proj_node.unsqueeze(-1)
        next_edge_attr[:, self.EDGE_UAV_INDEX] = proj_edge

        return Data(
            x=next_x,
            edge_index=edge_index,
            edge_attr=next_edge_attr,
            total_uavs=total_uavs,
            step=graph.step + 1 if hasattr(graph, "step") else torch.tensor([1]),
        )
