"""
gnn_rnn_extractor.py
======================
Combined spatio-temporal feature extractor: each timestep's ego+intruder
state is encoded into a graph (spatial, via the same `EgoIntruderGATEncoder`
used by gnn_extractor.py, weight-tied across time), and a GRU/LSTM runs over
a fixed-length window of those per-timestep graph embeddings (temporal).

**Papers and inspirations (in addition to gat_encoder.py's CADRL/GAT citations):**
- **Social-STGCNN** (Mohamed, Elhoseiny, Doersch, Lyon, CVPR 2020,
  https://arxiv.org/abs/2002.11927): spatial graph convolution per frame +
  temporal convolution/recurrence across frames for pedestrian trajectory
  forecasting — the spatial-GNN-per-frame, temporal-RNN-over-frames split
  here follows that pattern, adapted to a collision-avoidance policy rather
  than a forecasting model.

Expects a **frame-stacked** observation of shape (T * D,), produced by
`stable_baselines3.common.vec_env.VecFrameStack` wrapping the env with
`n_stack=T` — confirmed via `StackedObservations.compute_stacking` to
concatenate along the last axis with the newest frame at the tail
(oldest-first, newest-last) and to zero-fill frames before T steps have
elapsed in an episode, consistent with this repo's existing fixed-size
zero-padding philosophy (see graph_utils.real_intruder_mask). A bounded
window `T` was chosen over true cross-episode recurrence
(`sb3_contrib.RecurrentPPO`) to avoid a new dependency and BPTT/episode-
boundary-masking complexity (see plan_01's Context section).
"""
from __future__ import annotations

from typing import Literal

import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

from rl.single_agent.policies.gat_encoder import EgoIntruderGATEncoder
from rl.single_agent.policies.graph_utils import split_obs_blocks, real_intruder_mask


class IntruderGNNRNNExtractor(BaseFeaturesExtractor):
    """
    Spatial GNN per frame (weight-tied across the window) + temporal GRU/LSTM
    over the window, for `AGENT-N-INTRUDER[-RA]` obs types wrapped in
    `VecFrameStack(n_stack=temporal_window)`.

    Args:
        observation_space: flat Box, shape (temporal_window * (9 + 6*n_intruder [+ 1]),).
        n_intruder: number of intruder slots N encoded in each frame.
        has_ra: whether each frame has a trailing RA scalar.
        temporal_window: number of stacked frames T (must match the env's VecFrameStack n_stack).
        rnn_type: 'gru' or 'lstm', for the temporal recurrence over frame embeddings.
        hidden_dim: width of the spatial GNN and the temporal RNN hidden state.
        num_layers: number of stacked GATConv (+residual+LayerNorm) rounds in the spatial encoder.
        heads: number of attention heads per GATConv layer.
        features_dim: output feature width consumed by SB3's policy/value heads.
    """

    def __init__(
        self,
        observation_space: spaces.Box,
        n_intruder: int,
        has_ra: bool = False,
        temporal_window: int = 4,
        rnn_type: Literal['gru', 'lstm'] = 'gru',
        hidden_dim: int = 64,
        num_layers: int = 2,
        heads: int = 4,
        features_dim: int = 64,
    ):
        super().__init__(observation_space, features_dim)

        self.frame_dim = 9 + 6 * n_intruder + (1 if has_ra else 0)
        expected_dim = temporal_window * self.frame_dim
        if observation_space.shape[0] != expected_dim:
            raise ValueError(
                f"observation_space shape {observation_space.shape} does not match "
                f"temporal_window={temporal_window} * frame_dim={self.frame_dim} "
                f"(expected dim {expected_dim}). Did you wrap the env in "
                f"VecFrameStack(n_stack={temporal_window})?"
            )
        if rnn_type not in ('gru', 'lstm'):
            raise ValueError(f"Unknown rnn_type '{rnn_type}'. Valid options: 'gru', 'lstm'")

        self.n_intruder = n_intruder
        self.has_ra = has_ra
        self.temporal_window = temporal_window
        self.rnn_type = rnn_type

        # Shared (weight-tied) spatial encoder, called once per frame below.
        self.gat_encoder = EgoIntruderGATEncoder(
            hidden_dim=hidden_dim, num_layers=num_layers, heads=heads,
        )
        rnn_cls = nn.GRU if rnn_type == 'gru' else nn.LSTM
        self.temporal_rnn = rnn_cls(input_size=hidden_dim, hidden_size=hidden_dim, batch_first=True)

        concat_dim = hidden_dim + (1 if has_ra else 0)
        self.output_proj = nn.Sequential(nn.Linear(concat_dim, features_dim), nn.ReLU())

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        """
        Args:
            observations: [batch, temporal_window * frame_dim]

        Returns:
            [batch, features_dim]
        """
        batch_size = observations.shape[0]
        frames = observations.reshape(batch_size, self.temporal_window, self.frame_dim)

        frame_embeddings = []
        last_ra_scalar = None
        for t in range(self.temporal_window):
            self_block, intruder_block, ra_scalar = split_obs_blocks(
                frames[:, t, :], self.n_intruder, self.has_ra,
            )
            mask = real_intruder_mask(intruder_block)
            frame_embeddings.append(self.gat_encoder(self_block, intruder_block, mask))
            last_ra_scalar = ra_scalar  # oldest-first order -> last loop iter is newest frame

        # [batch, T, hidden_dim], chronological (oldest -> newest), matching
        # VecFrameStack's newest-last layout — this is genuine temporal
        # recurrence, NOT the farthest-first reversal used in rnn_extractor.py.
        sequence = torch.stack(frame_embeddings, dim=1)
        _, h_n = self.temporal_rnn(sequence)
        final_hidden = h_n[0] if self.rnn_type == 'gru' else h_n[0][0]

        if self.has_ra:
            combined = torch.cat([final_hidden, last_ra_scalar], dim=-1)
        else:
            combined = final_hidden
        return self.output_proj(combined)
