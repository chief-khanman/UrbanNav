"""End-to-end: train the reference GNN backbone for a few epochs against
fresh logged episodes; confirm train loss decreases for both tasks."""

import torch

from rl.surrogate.train import train


class TestNextStateTraining:
    def test_loss_decreases(self, episode_logs_root):
        torch.manual_seed(0)
        model, history = train(
            task='next_state',
            backbone='GNN',
            logs_root=episode_logs_root,
            epochs=5,
            batch_size=8,
            learning_rate=1e-2,
            val_fraction=0.0,
            seed=0,
            device='cpu',
            backbone_kwargs={'hidden_dim': 16},
            verbose=0,
        )
        assert len(history['train_loss']) == 5
        # Final epoch should beat the first; allow a tiny tolerance to absorb
        # noise on very small batches.
        assert history['train_loss'][-1] < history['train_loss'][0] * 0.95


class TestEpisodeOutcomeTraining:
    def test_loss_decreases(self, episode_logs_root):
        torch.manual_seed(0)
        model, history = train(
            task='episode_outcome',
            backbone='GNN',
            logs_root=episode_logs_root,
            epochs=20,
            batch_size=2,
            learning_rate=1e-2,
            val_fraction=0.0,
            seed=0,
            device='cpu',
            backbone_kwargs={'hidden_dim': 16},
            verbose=0,
        )
        # With only ~2 episodes available the dataset is tiny; just require
        # the loss to drop appreciably from initialization.
        assert history['train_loss'][-1] < history['train_loss'][0]
