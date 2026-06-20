import torch

from rl.surrogate.datasets.episode_outcome_dataset import (
    EPISODE_SCALAR_KEYS,
    START_POS_DIM,
    EpisodeOutcomeDataset,
)


class TestEpisodeOutcomeDataset:
    def test_dataset_loads_episodes(self, episode_dirs):
        ds = EpisodeOutcomeDataset(episode_dirs)
        assert len(ds) >= 1
        assert ds.max_uavs >= 1

    def test_item_shapes(self, episode_dirs):
        ds = EpisodeOutcomeDataset(episode_dirs)
        item = ds[0]
        assert item['positions'].shape == (ds.max_uavs, START_POS_DIM)
        assert item['mask'].shape == (ds.max_uavs,)
        assert item['scalars'].shape == (len(EPISODE_SCALAR_KEYS),)
        assert item['target'].shape == (1,)
        assert item['positions'].dtype == torch.float32

    def test_mask_marks_only_present_uavs(self, episode_dirs):
        ds = EpisodeOutcomeDataset(episode_dirs)
        for i in range(len(ds)):
            mask = ds[i]['mask']
            assert mask.sum().item() > 0
            assert mask.sum().item() <= ds.max_uavs

    def test_normalize_stores_stats(self, episode_dirs):
        ds = EpisodeOutcomeDataset(episode_dirs, normalize=True)
        assert 'target_mean' in ds.stats
        assert 'target_std' in ds.stats
