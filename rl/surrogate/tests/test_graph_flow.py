"""Tests for the graph-flow surrogate model (Model 1).

Covers: GraphFlowDataset construction, GraphFlowGNN forward pass,
conservation projection correctness, and autoregressive rollout stability.
"""

import os

import pytest
import torch
from torch_geometric.data import Data

from rl.surrogate.backbones.graph_flow_gnn import (
    GraphFlowGNN,
    GraphFlowRecurrentGNN,
    conservation_projection,
)
from rl.surrogate.datasets.graph_flow_dataset import (
    EDGE_ATTR_DIM,
    NODE_ATTR_DIM,
    GraphFlowDataset,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _make_synthetic_graph(n_nodes=4, n_uavs_total=10.0):
    """Build a small fully-connected graph with known UAV counts."""
    src, dst = [], []
    for i in range(n_nodes):
        for j in range(n_nodes):
            if i != j:
                src.append(i)
                dst.append(j)
    edge_index = torch.tensor([src, dst], dtype=torch.long)
    n_edges = edge_index.shape[1]

    x = torch.zeros(n_nodes, NODE_ATTR_DIM)
    x[:, 0] = torch.tensor([3.0, 2.0, 1.0, 0.0])  # grounded
    x[:, 1] = torch.tensor([1.0, 0.0, 1.0, 0.0])   # landing queue
    x[:, 2] = torch.tensor([4.0, 4.0, 4.0, 4.0])   # capacity

    edge_attr = torch.zeros(n_edges, EDGE_ATTR_DIM)
    edge_attr[0, 0] = 1.0  # 1 UAV in transit on first edge
    edge_attr[3, 0] = 1.0  # 1 UAV in transit on fourth edge

    total = x[:, 0].sum() + x[:, 1].sum() + edge_attr[:, 0].sum()
    return Data(
        x=x, edge_index=edge_index, edge_attr=edge_attr,
        total_uavs=total.unsqueeze(0), step=torch.tensor([0]),
    )


# ---------------------------------------------------------------------------
# Conservation projection
# ---------------------------------------------------------------------------


class TestConservationProjection:
    def test_preserves_total(self):
        node = torch.tensor([3.0, 2.0, 1.0])
        edge = torch.tensor([2.0, 1.0, 0.5])
        total = torch.tensor([10.0])
        pn, pe = conservation_projection(node, edge, total)
        assert abs((pn.sum() + pe.sum()).item() - 10.0) < 1e-5

    def test_clamps_negatives(self):
        node = torch.tensor([-1.0, 3.0])
        edge = torch.tensor([-2.0, 5.0])
        total = torch.tensor([6.0])
        pn, pe = conservation_projection(node, edge, total)
        assert (pn >= 0).all()
        assert (pe >= 0).all()
        assert abs((pn.sum() + pe.sum()).item() - 6.0) < 1e-5

    def test_zero_input(self):
        node = torch.tensor([0.0, 0.0])
        edge = torch.tensor([0.0])
        total = torch.tensor([0.0])
        pn, pe = conservation_projection(node, edge, total)
        assert pn.sum().item() == 0.0
        assert pe.sum().item() == 0.0


# ---------------------------------------------------------------------------
# GraphFlowGNN
# ---------------------------------------------------------------------------


class TestGraphFlowGNN:
    def test_output_shapes(self):
        g = _make_synthetic_graph()
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        with torch.no_grad():
            pred = model.predict_graph_next_state(g)
        assert pred.x.shape == g.x.shape
        assert pred.edge_attr.shape == g.edge_attr.shape
        assert pred.edge_index.shape == g.edge_index.shape

    def test_conservation_exact(self):
        g = _make_synthetic_graph()
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        with torch.no_grad():
            pred = model.predict_graph_next_state(g)
        pred_total = pred.x[:, 0].sum() + pred.x[:, 1].sum() + pred.edge_attr[:, 0].sum()
        assert abs(pred_total.item() - g.total_uavs.item()) < 1e-5

    def test_no_negative_uav_counts(self):
        g = _make_synthetic_graph()
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        with torch.no_grad():
            pred = model.predict_graph_next_state(g)
        assert (pred.x[:, 0] >= -1e-6).all(), "n_grounded went negative"
        assert (pred.x[:, 1] >= -1e-6).all(), "n_landing_queue went negative"
        assert (pred.edge_attr[:, 0] >= -1e-6).all(), "n_in_transit went negative"

    def test_multi_step_rollout_conservation(self):
        g = _make_synthetic_graph()
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        original_total = g.total_uavs.item()
        current = g
        with torch.no_grad():
            for _ in range(20):
                current = model.predict_graph_next_state(current)
        final_total = current.x[:, 0].sum() + current.x[:, 1].sum() + current.edge_attr[:, 0].sum()
        assert abs(final_total.item() - original_total) < 1e-4

    def test_raises_on_per_uav_interface(self):
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        with pytest.raises(NotImplementedError):
            model.predict_next_state(torch.zeros(1, 8), torch.zeros(1, 2))
        with pytest.raises(NotImplementedError):
            model.predict_episode_outcome({})


# ---------------------------------------------------------------------------
# GraphFlowRecurrentGNN
# ---------------------------------------------------------------------------


class TestGraphFlowRecurrentGNN:
    def test_hidden_state_changes_prediction(self):
        g = _make_synthetic_graph()
        model = GraphFlowRecurrentGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        with torch.no_grad():
            model.reset_hidden()
            pred1 = model.predict_graph_next_state(g)
            pred2 = model.predict_graph_next_state(g)
        assert not torch.allclose(pred1.x, pred2.x)

    def test_reset_hidden_restores_determinism(self):
        g = _make_synthetic_graph()
        model = GraphFlowRecurrentGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        with torch.no_grad():
            model.reset_hidden()
            pred_a = model.predict_graph_next_state(g)
            model.reset_hidden()
            pred_b = model.predict_graph_next_state(g)
        assert torch.allclose(pred_a.x, pred_b.x)

    def test_conservation_with_recurrence(self):
        g = _make_synthetic_graph()
        model = GraphFlowRecurrentGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        original_total = g.total_uavs.item()
        model.reset_hidden()
        current = g
        with torch.no_grad():
            for _ in range(10):
                current = model.predict_graph_next_state(current)
        final_total = current.x[:, 0].sum() + current.x[:, 1].sum() + current.edge_attr[:, 0].sum()
        assert abs(final_total.item() - original_total) < 1e-4


# ---------------------------------------------------------------------------
# GraphFlowDataset (integration, uses simulator-generated logs)
# ---------------------------------------------------------------------------


class TestGraphFlowDataset:
    def test_dataset_loads_from_sim(self, episode_logs_root):
        ds = GraphFlowDataset.from_logs_root(episode_logs_root, edge_type="full_mesh")
        if len(ds) == 0:
            pytest.skip("No vertiport snapshots in logs (MetricsCollector extension not active)")
        g_t, g_tp1 = ds[0]
        assert isinstance(g_t, Data)
        assert g_t.x.shape[1] == NODE_ATTR_DIM
        assert g_t.edge_attr.shape[1] == EDGE_ATTR_DIM

    def test_all_edge_types_loadable(self, episode_logs_root):
        for etype in ["full_mesh", "demand_driven"]:
            ds = GraphFlowDataset.from_logs_root(episode_logs_root, edge_type=etype)
            if len(ds) > 0:
                g_t, _ = ds[0]
                assert g_t.edge_index.shape[0] == 2

    def test_conservation_in_data(self, episode_logs_root):
        ds = GraphFlowDataset.from_logs_root(episode_logs_root, edge_type="full_mesh")
        if len(ds) == 0:
            pytest.skip("No vertiport snapshots in logs")
        for i in range(min(len(ds), 5)):
            g_t, g_tp1 = ds[i]
            total_t = g_t.x[:, 0].sum() + g_t.x[:, 1].sum() + g_t.edge_attr[:, 0].sum()
            assert g_t.total_uavs.item() == pytest.approx(total_t.item(), abs=1e-4)

    def test_empty_logs_returns_empty_dataset(self, tmp_path):
        ds = GraphFlowDataset.from_logs_root(str(tmp_path))
        assert len(ds) == 0
