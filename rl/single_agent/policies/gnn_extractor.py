"""
gnn_extractor.py
=================
GNN feature extractor for `AGENT-N-INTRUDER[-RA]` obs types: ego + detected
intruders form a star graph for the *current timestep only* (no temporal
component — see gnn_rnn_extractor.py for the spatio-temporal variant).

See gat_encoder.py's module docstring for the CADRL/GAT citations behind the
star-topology, ego-centric-readout design used here.
"""
from __future__ import annotations

import torch
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

from rl.single_agent.policies.gat_encoder import EgoIntruderGATEncoder
from rl.single_agent.policies.graph_utils import split_obs_blocks, real_intruder_mask


class IntruderGNNExtractor(BaseFeaturesExtractor):
    """
    GATConv-based feature extractor: ego + real intruders as a star graph,
    pooled by reading out ego's post-message-passing embedding.

    Args:
        observation_space: flat Box, shape (9 + 6*n_intruder [+ 1],).
        n_intruder: number of intruder slots N encoded in the obs.
        has_ra: whether the obs has a trailing RA scalar.
        hidden_dim: width nodes/edges are projected into and GATConv operates on.
        num_layers: number of stacked GATConv (+residual+LayerNorm) rounds.
        heads: number of attention heads per GATConv layer.
        features_dim: output feature width consumed by SB3's policy/value heads.
    """

    def __init__(
        self,
        observation_space: spaces.Box,
        n_intruder: int,
        has_ra: bool = False,
        hidden_dim: int = 64,
        num_layers: int = 2,
        heads: int = 4,
        features_dim: int = 64,
    ):
        super().__init__(observation_space, features_dim)

        expected_dim = 9 + 6 * n_intruder + (1 if has_ra else 0)
        if observation_space.shape[0] != expected_dim:
            raise ValueError(
                f"observation_space shape {observation_space.shape} does not match "
                f"n_intruder={n_intruder}, has_ra={has_ra} (expected dim {expected_dim})"
            )

        self.n_intruder = n_intruder
        self.has_ra = has_ra

        self.gat_encoder = EgoIntruderGATEncoder(
            hidden_dim=hidden_dim, num_layers=num_layers, heads=heads,
        )
        concat_dim = hidden_dim + (1 if has_ra else 0)
        self.output_proj = torch.nn.Sequential(
            torch.nn.Linear(concat_dim, features_dim), torch.nn.ReLU(),
        )

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        """
        Args:
            observations: [batch, 9 + 6*n_intruder [+ 1]]

        Returns:
            [batch, features_dim]
        """
        self_block, intruder_block, ra_scalar = split_obs_blocks(
            observations, self.n_intruder, self.has_ra,
        )
        mask = real_intruder_mask(intruder_block)

        ego_embedding = self.gat_encoder(self_block, intruder_block, mask)  # [batch, hidden_dim]

        if self.has_ra:
            combined = torch.cat([ego_embedding, ra_scalar], dim=-1)
        else:
            combined = ego_embedding
        return self.output_proj(combined)
