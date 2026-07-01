"""
rnn_extractor.py
=================
CADRL/GA3C-CADRL-style RNN feature extractor: encodes the variable number of
detected intruders within a *single timestep* as a sequence consumed by a
GRU or LSTM, rather than a fixed-order flat concatenation (the MLP path).

**Papers and inspirations:**
- **CADRL** (Chen, Liu, Kreiss, Pavone, ICRA 2017, https://arxiv.org/abs/1609.07845):
  decentralized, ego-centric collision avoidance via a value network over
  (own state, one other agent's state) pairs.
- **GA3C-CADRL** (Everett, Chen, How, IROS 2018, https://arxiv.org/abs/1805.01956;
  reference implementation: https://github.com/mit-acl/rl_collision_avoidance):
  extends CADRL to a variable number of other agents via an LSTM consuming
  per-agent state vectors sequentially. This module's farthest-first ->
  nearest-last feed order (see `forward`) is taken directly from that
  implementation's design: it exploits a vanilla RNN's recency bias so the
  final hidden state is dominated by the nearest, most safety-relevant
  neighbor rather than whichever agent happened to be detected first.
"""
from __future__ import annotations

from typing import Literal

import torch
import torch.nn as nn
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

from rl.single_agent.policies.graph_utils import split_obs_blocks, real_intruder_mask


class IntruderRNNExtractor(BaseFeaturesExtractor):
    """
    GRU/LSTM-based feature extractor for `AGENT-N-INTRUDER[-RA]` obs types.

    Args:
        observation_space: flat Box, shape (9 + 6*n_intruder [+ 1],).
        n_intruder: number of intruder slots N encoded in the obs.
        has_ra: whether the obs has a trailing RA scalar.
        rnn_type: 'gru' or 'lstm'.
        hidden_dim: width of the self-MLP and RNN hidden state.
        features_dim: output feature width consumed by SB3's policy/value heads.
    """

    def __init__(
        self,
        observation_space: spaces.Box,
        n_intruder: int,
        has_ra: bool = False,
        rnn_type: Literal['gru', 'lstm'] = 'gru',
        hidden_dim: int = 64,
        features_dim: int = 64,
    ):
        super().__init__(observation_space, features_dim)

        expected_dim = 9 + 6 * n_intruder + (1 if has_ra else 0)
        if observation_space.shape[0] != expected_dim:
            raise ValueError(
                f"observation_space shape {observation_space.shape} does not match "
                f"n_intruder={n_intruder}, has_ra={has_ra} (expected dim {expected_dim})"
            )
        if rnn_type not in ('gru', 'lstm'):
            raise ValueError(f"Unknown rnn_type '{rnn_type}'. Valid options: 'gru', 'lstm'")

        self.n_intruder = n_intruder
        self.has_ra = has_ra
        self.rnn_type = rnn_type

        self.self_mlp = nn.Sequential(nn.Linear(9, hidden_dim), nn.ReLU())
        rnn_cls = nn.GRU if rnn_type == 'gru' else nn.LSTM
        self.intruder_rnn = rnn_cls(input_size=6, hidden_size=hidden_dim, batch_first=True)

        concat_dim = hidden_dim + hidden_dim + (1 if has_ra else 0)
        self.output_proj = nn.Sequential(nn.Linear(concat_dim, features_dim), nn.ReLU())

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
        mask = real_intruder_mask(intruder_block)  # [batch, n_intruder]

        self_feat = self.self_mlp(self_block)  # [batch, hidden_dim]

        batch_size = self_block.shape[0]
        hidden_dim = self_feat.shape[-1]
        agent_feats = []
        for i in range(batch_size):
            real_rows = intruder_block[i][mask[i]]  # [n_real, 6], closest-first
            if real_rows.shape[0] == 0:
                agent_feats.append(
                    torch.zeros(hidden_dim, dtype=observations.dtype, device=observations.device)
                )
                continue
            # reverse closest-first -> farthest-first..nearest-last
            sequence = real_rows.flip(dims=(0,)).unsqueeze(0)  # [1, n_real, 6]
            _, h_n = self.intruder_rnn(sequence)
            final_hidden = h_n[0] if self.rnn_type == 'gru' else h_n[0][0]
            agent_feats.append(final_hidden.squeeze(0))
        agent_feat = torch.stack(agent_feats, dim=0)  # [batch, hidden_dim]

        if self.has_ra:
            combined = torch.cat([self_feat, agent_feat, ra_scalar], dim=-1)
        else:
            combined = torch.cat([self_feat, agent_feat], dim=-1)
        return self.output_proj(combined)
