"""
surrogate_template.py
======================
Abstract base class for surrogate models that learn from UrbanNav rollouts.

Two prediction interfaces:
- predict_next_state(state, action): autoregressive next-state dynamics
  (replaces the physics simulator in inner loops).
- predict_episode_outcome(start_condition): AlphaZero-style value estimator
  (predicts end-of-episode metrics from initial conditions, replaces a full
  rollout inside an MCTS/RL outer loop).

A concrete backbone may implement either or both. Backbones that only
target one path raise NotImplementedError for the other.
"""

from __future__ import annotations

from abc import abstractmethod
from typing import Any, Dict

import torch
import torch.nn as nn


class SurrogateModel(nn.Module):
    """Abstract base for all surrogate backbones registered in BACKBONE_REGISTRY."""

    def predict_graph_next_state(self, graph: Any) -> Any:
        """Predict next-step node and edge attributes on a vertiport graph.

        Args:
            graph: torch_geometric.data.Data with node/edge attributes
                   representing the current vertiport network state.

        Returns:
            torch_geometric.data.Data with predicted next-step attributes.
        """
        raise NotImplementedError

    @abstractmethod
    def predict_next_state(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        """Predict the per-UAV state at t+1.

        Args:
            state:  [batch, UAV_STATE_DIM] — current state.
            action: [batch, UAV_ACTION_DIM] — controller action.

        Returns:
            Predicted next state, same shape as `state`.
        """
        raise NotImplementedError

    @abstractmethod
    def predict_episode_outcome(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        """Predict end-of-episode metrics from start conditions.

        Args:
            batch: dict matching EpisodeOutcomeDataset.__getitem__:
                positions: [batch, max_uavs, 3]
                mask:      [batch, max_uavs]
                scalars:   [batch, n_scalars]

        Returns:
            Predicted target, shape [batch, n_targets].
        """
        raise NotImplementedError
