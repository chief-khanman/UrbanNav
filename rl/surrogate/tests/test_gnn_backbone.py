import torch

from rl.surrogate.backbones import BACKBONE_REGISTRY, GNNSurrogateBackbone, SurrogateModel
from rl.surrogate.datasets.episode_outcome_dataset import START_POS_DIM
from rl.surrogate.datasets.trajectory_dataset import UAV_ACTION_DIM, UAV_STATE_DIM


class TestRegistry:
    def test_gnn_registered(self):
        assert 'GNN' in BACKBONE_REGISTRY
        assert BACKBONE_REGISTRY['GNN'] is GNNSurrogateBackbone

    def test_backbone_subclasses_template(self):
        model = GNNSurrogateBackbone()
        assert isinstance(model, SurrogateModel)


class TestPredictNextState:
    def test_output_shape_matches_state(self):
        model = GNNSurrogateBackbone(hidden_dim=8)
        state = torch.zeros(4, UAV_STATE_DIM)
        action = torch.zeros(4, UAV_ACTION_DIM)
        out = model.predict_next_state(state, action)
        assert out.shape == state.shape


class TestPredictEpisodeOutcome:
    def test_output_shape_matches_targets(self):
        model = GNNSurrogateBackbone(hidden_dim=8, num_outcome_targets=3)
        batch = {
            'positions': torch.zeros(2, 4, START_POS_DIM),
            'mask': torch.tensor([[1.0, 1.0, 0.0, 0.0], [1.0, 1.0, 1.0, 0.0]]),
            'scalars': torch.zeros(2, 2),
        }
        out = model.predict_episode_outcome(batch)
        assert out.shape == (2, 3)

    def test_handles_zero_valid_uavs(self):
        model = GNNSurrogateBackbone(hidden_dim=4, num_outcome_targets=1)
        batch = {
            'positions': torch.zeros(1, 2, START_POS_DIM),
            'mask': torch.zeros(1, 2),
            'scalars': torch.zeros(1, 2),
        }
        out = model.predict_episode_outcome(batch)
        assert out.shape == (1, 1)
        assert torch.isfinite(out).all()
