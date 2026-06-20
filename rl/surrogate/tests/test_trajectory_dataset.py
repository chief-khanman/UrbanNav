import torch

from rl.surrogate.datasets.trajectory_dataset import (
    UAV_ACTION_DIM,
    UAV_STATE_DIM,
    TrajectoryDataset,
    discover_episode_dirs,
)


class TestDiscovery:
    def test_discovers_episode_logs(self, episode_logs_root):
        dirs = discover_episode_dirs(episode_logs_root)
        assert len(dirs) >= 1

    def test_returns_empty_for_missing_root(self, tmp_path):
        assert discover_episode_dirs(str(tmp_path / 'nope')) == []


class TestTrajectoryDataset:
    def test_dataset_loads_transitions(self, episode_dirs):
        ds = TrajectoryDataset(episode_dirs)
        assert len(ds) > 0

    def test_item_shapes(self, episode_dirs):
        ds = TrajectoryDataset(episode_dirs)
        state, action, next_state = ds[0]
        assert state.shape == (UAV_STATE_DIM,)
        assert action.shape == (UAV_ACTION_DIM,)
        assert next_state.shape == (UAV_STATE_DIM,)
        assert state.dtype == torch.float32

    def test_source_lookup(self, episode_dirs):
        ds = TrajectoryDataset(episode_dirs)
        src = ds.get_source(0)
        assert src[0] in episode_dirs
        assert isinstance(src[1], int) and isinstance(src[2], int)

    def test_from_logs_root(self, episode_logs_root):
        ds = TrajectoryDataset.from_logs_root(episode_logs_root)
        assert len(ds) > 0
