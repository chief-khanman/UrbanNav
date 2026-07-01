"""Forward-pass shape tests for the RNN/GNN/GNN+RNN feature extractors in
rl/single_agent/policies/. These are not training-correctness tests — they
only assert that each extractor accepts the obs shape SB3 will actually feed
it (batch size 1 and >1, since SB3 always batches even for a single env) and
returns a tensor of shape (batch, features_dim)."""
import numpy as np
import pytest
import torch
from gymnasium import spaces

from rl.single_agent.policies.graph_utils import (
    split_obs_blocks, real_intruder_mask, build_star_graph,
)
from rl.single_agent.policies.gat_encoder import EgoIntruderGATEncoder
from rl.single_agent.policies.rnn_extractor import IntruderRNNExtractor
from rl.single_agent.policies.gnn_extractor import IntruderGNNExtractor
from rl.single_agent.policies.gnn_rnn_extractor import IntruderGNNRNNExtractor


def _box(n: int) -> spaces.Box:
    return spaces.Box(low=-np.inf, high=np.inf, shape=(n,), dtype=np.float32)


def _make_obs(batch_size: int, n_intruder: int, has_ra: bool, n_real: int) -> torch.Tensor:
    """Self-block all ones, n_real real (nonzero) intruder rows followed by
    zero-padded rows, optional RA scalar — mirrors agent_logic's real-first,
    zero-padded-tail layout."""
    dim = 9 + 6 * n_intruder + (1 if has_ra else 0)
    obs = torch.zeros(batch_size, dim)
    obs[:, :9] = 1.0
    for slot in range(n_real):
        start = 9 + 6 * slot
        obs[:, start:start + 6] = torch.arange(1, 7, dtype=torch.float32) * (slot + 1)
    if has_ra:
        obs[:, -1] = 1.0
    return obs


class TestGraphUtils:
    def test_split_obs_blocks_shapes(self):
        obs = _make_obs(batch_size=3, n_intruder=4, has_ra=True, n_real=2)
        self_block, intruder_block, ra_scalar = split_obs_blocks(obs, n_intruder=4, has_ra=True)
        assert self_block.shape == (3, 9)
        assert intruder_block.shape == (3, 4, 6)
        assert ra_scalar.shape == (3, 1)

    def test_real_intruder_mask_detects_padding(self):
        obs = _make_obs(batch_size=2, n_intruder=4, has_ra=False, n_real=2)
        _, intruder_block, _ = split_obs_blocks(obs, n_intruder=4, has_ra=False)
        mask = real_intruder_mask(intruder_block)
        assert mask.shape == (2, 4)
        assert mask[:, :2].all()
        assert not mask[:, 2:].any()

    @pytest.mark.parametrize("n_real", [0, 1, 3])
    def test_build_star_graph_node_edge_counts(self, n_real):
        obs = _make_obs(batch_size=1, n_intruder=3, has_ra=False, n_real=n_real)
        self_block, intruder_block, _ = split_obs_blocks(obs, n_intruder=3, has_ra=False)
        mask = real_intruder_mask(intruder_block)
        ego_raw, intruder_pos_raw, edge_index, edge_attr_raw = build_star_graph(
            self_block[0], intruder_block[0], mask[0],
        )
        assert ego_raw.shape == (9,)
        assert intruder_pos_raw.shape == (n_real, 3)
        assert edge_index.shape == (2, 2 * n_real)
        assert edge_attr_raw.shape == (2 * n_real, 3)


class TestGATEncoder:
    @pytest.mark.parametrize("batch_size,n_real", [(1, 0), (1, 3), (4, 2)])
    def test_forward_shape(self, batch_size, n_real):
        n_intruder = 3
        encoder = EgoIntruderGATEncoder(hidden_dim=16, num_layers=2, heads=2)
        obs = _make_obs(batch_size, n_intruder, has_ra=False, n_real=n_real)
        self_block, intruder_block, _ = split_obs_blocks(obs, n_intruder, has_ra=False)
        mask = real_intruder_mask(intruder_block)
        out = encoder(self_block, intruder_block, mask)
        assert out.shape == (batch_size, 16)


class TestIntruderRNNExtractor:
    @pytest.mark.parametrize("rnn_type", ["gru", "lstm"])
    @pytest.mark.parametrize("has_ra", [False, True])
    @pytest.mark.parametrize("batch_size,n_real", [(1, 0), (1, 3), (5, 2)])
    def test_forward_shape(self, rnn_type, has_ra, batch_size, n_real):
        n_intruder = 3
        obs_space = _box(9 + 6 * n_intruder + (1 if has_ra else 0))
        extractor = IntruderRNNExtractor(
            obs_space, n_intruder=n_intruder, has_ra=has_ra,
            rnn_type=rnn_type, hidden_dim=16, features_dim=8,
        )
        obs = _make_obs(batch_size, n_intruder, has_ra, n_real)
        out = extractor(obs)
        assert out.shape == (batch_size, 8)
        assert torch.isfinite(out).all()

    def test_rejects_mismatched_obs_space(self):
        with pytest.raises(ValueError):
            IntruderRNNExtractor(_box(10), n_intruder=3, has_ra=False)


class TestIntruderGNNExtractor:
    @pytest.mark.parametrize("has_ra", [False, True])
    @pytest.mark.parametrize("batch_size,n_real", [(1, 0), (1, 3), (5, 2)])
    def test_forward_shape(self, has_ra, batch_size, n_real):
        n_intruder = 3
        obs_space = _box(9 + 6 * n_intruder + (1 if has_ra else 0))
        extractor = IntruderGNNExtractor(
            obs_space, n_intruder=n_intruder, has_ra=has_ra,
            hidden_dim=16, num_layers=2, heads=2, features_dim=8,
        )
        obs = _make_obs(batch_size, n_intruder, has_ra, n_real)
        out = extractor(obs)
        assert out.shape == (batch_size, 8)
        assert torch.isfinite(out).all()

    def test_rejects_mismatched_obs_space(self):
        with pytest.raises(ValueError):
            IntruderGNNExtractor(_box(10), n_intruder=3, has_ra=False)


class TestIntruderGNNRNNExtractor:
    @pytest.mark.parametrize("rnn_type", ["gru", "lstm"])
    @pytest.mark.parametrize("has_ra", [False, True])
    @pytest.mark.parametrize("batch_size", [1, 4])
    def test_forward_shape(self, rnn_type, has_ra, batch_size):
        n_intruder = 3
        temporal_window = 4
        frame_dim = 9 + 6 * n_intruder + (1 if has_ra else 0)
        obs_space = _box(temporal_window * frame_dim)
        extractor = IntruderGNNRNNExtractor(
            obs_space, n_intruder=n_intruder, has_ra=has_ra,
            temporal_window=temporal_window, rnn_type=rnn_type,
            hidden_dim=16, num_layers=2, heads=2, features_dim=8,
        )
        # stack temporal_window frames with varying n_real per frame
        frames = [
            _make_obs(batch_size, n_intruder, has_ra, n_real=t % (n_intruder + 1))
            for t in range(temporal_window)
        ]
        obs = torch.cat(frames, dim=-1)
        out = extractor(obs)
        assert out.shape == (batch_size, 8)
        assert torch.isfinite(out).all()

    def test_rejects_mismatched_obs_space(self):
        with pytest.raises(ValueError):
            IntruderGNNRNNExtractor(_box(10), n_intruder=3, has_ra=False, temporal_window=4)
