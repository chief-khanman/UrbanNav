"""
gnn_backbone.py
================
Reference GNN backbone for the surrogate-model interface.

Implements both predict_next_state (per-UAV regression from state+action)
and predict_episode_outcome (graph-pooled regression from start conditions).
For predict_next_state, treats each UAV as an isolated node — the GNN
plumbing is structural (so future backbones can swap in true graph
interaction with intruders/RAs) without depending on neighbor data the
current logged step records don't carry per-UAV.

For predict_episode_outcome, builds a fully-connected graph over the
initial UAV positions, runs GATConv layers, mean-pools, and concatenates
the episode scalars before the final regression head.
"""

from __future__ import annotations

from typing import Dict

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GATConv

from rl.surrogate.backbones.surrogate_template import SurrogateModel
from rl.surrogate.datasets.episode_outcome_dataset import EPISODE_SCALAR_KEYS, START_POS_DIM
from rl.surrogate.datasets.trajectory_dataset import UAV_ACTION_DIM, UAV_STATE_DIM


class GNNSurrogateBackbone(SurrogateModel):
    """GAT-based reference backbone."""

    def __init__(
        self,
        hidden_dim: int = 32,
        num_gat_layers: int = 2,
        num_outcome_targets: int = 1,
        heads: int = 2,
    ):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_gat_layers = num_gat_layers

        # ── next-state head ────────────────────────────────────────────────
        # MLP over [state, action] → delta-state. Predicting the delta
        # (residual) is far more learnable than predicting absolute state.
        self.next_state_input_dim = UAV_STATE_DIM + UAV_ACTION_DIM
        self.next_state_mlp = nn.Sequential(
            nn.Linear(self.next_state_input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, UAV_STATE_DIM),
        )

        # ── episode-outcome head ───────────────────────────────────────────
        self.node_proj = nn.Linear(START_POS_DIM, hidden_dim)
        self.gat_layers = nn.ModuleList([
            GATConv(in_channels=hidden_dim, out_channels=hidden_dim, heads=heads, concat=False)
            for _ in range(num_gat_layers)
        ])
        self.gat_norms = nn.ModuleList([
            nn.LayerNorm(hidden_dim) for _ in range(num_gat_layers)
        ])
        self.outcome_head = nn.Sequential(
            nn.Linear(hidden_dim + len(EPISODE_SCALAR_KEYS), hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, num_outcome_targets),
        )

    # ------------------------------------------------------------------
    # SurrogateModel interface
    # ------------------------------------------------------------------

    def predict_next_state(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        x = torch.cat([state, action], dim=-1)
        delta = self.next_state_mlp(x)
        return state + delta

    def predict_episode_outcome(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        positions = batch['positions']     # [B, max_uavs, 3]
        mask = batch['mask']               # [B, max_uavs]
        scalars = batch['scalars']         # [B, n_scalars]

        B, max_uavs, _ = positions.shape
        outputs = []

        for b in range(B):
            valid = mask[b] > 0.5
            n_valid = int(valid.sum().item())
            if n_valid == 0:
                pooled = torch.zeros(self.hidden_dim, device=positions.device, dtype=positions.dtype)
            else:
                node_feat = self.node_proj(positions[b][valid])  # [n_valid, hidden_dim]
                # Fully-connected edge_index over valid nodes.
                if n_valid > 1:
                    idx = torch.arange(n_valid, device=positions.device)
                    src = idx.repeat_interleave(n_valid)
                    dst = idx.repeat(n_valid)
                    keep = src != dst
                    edge_index = torch.stack([src[keep], dst[keep]], dim=0)
                else:
                    edge_index = torch.zeros((2, 0), dtype=torch.long, device=positions.device)

                h = node_feat
                for conv, norm in zip(self.gat_layers, self.gat_norms):
                    h_new = conv(h, edge_index)
                    h_new = F.relu(h_new)
                    h = norm(h + h_new)
                pooled = h.mean(dim=0)  # [hidden_dim]

            outputs.append(pooled)

        pooled_batch = torch.stack(outputs, dim=0)  # [B, hidden_dim]
        combined = torch.cat([pooled_batch, scalars], dim=-1)
        return self.outcome_head(combined)
