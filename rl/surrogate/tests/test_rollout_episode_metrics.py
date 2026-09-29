"""Tests for autoregressive rollout -> episode-metrics adapters (surrogate plan
doc): confirms both GraphFlowGNN and DualGraphGNN can be rolled out N steps
with no simulator involved and plugged into logic that reproduces
MetricsCollector.get_metrics()'s schema.

Uses freshly-initialized (untrained) models -- this is a structural/plumbing
test suite, not an accuracy test: it checks that the rollout -> adapter
pipeline produces the right shape/schema/invariants, not that an untrained
model's predictions are numerically close to ground truth.
"""

import json
import os

import pytest
import torch

from rl.surrogate.backbones.dual_graph_gnn import DualGraphGNN
from rl.surrogate.backbones.graph_flow_gnn import GraphFlowGNN
from rl.surrogate.datasets.dual_graph_dataset import DualGraphDataset, UAV_NODE_KEYS
from rl.surrogate.datasets.graph_flow_dataset import GraphFlowDataset
from rl.surrogate.rollout_metrics import (
    dual_graph_rollout_to_episode_metrics,
    graph_flow_rollout_to_episode_metrics,
)

EXPECTED_METRIC_KEYS = {
    "total_steps",
    "total_nmac_events",
    "total_uav_collision_events",
    "total_ra_collision_events",
    "unique_uavs",
    "avg_missions_completed",
    "peak_active_uavs",
    "avg_speed",
    "min_speed",
    "max_speed",
    "avg_dist_to_goal_final",
}


def _rollout(model, seed_graph, n_steps):
    model.eval()
    rollout = []
    current = seed_graph
    with torch.no_grad():
        for _ in range(n_steps):
            current = model.predict_graph_next_state(current)
            rollout.append(current)
    return rollout


def _seed_graph(episode_logs_root):
    ds = GraphFlowDataset.from_logs_root(episode_logs_root, edge_type="full_mesh")
    if len(ds) == 0:
        pytest.skip("No vertiport snapshots in logs")
    seed_graph, _ = ds[0]
    return seed_graph


# ---------------------------------------------------------------------------
# graph_flow: full end-to-end rollout -> episode metrics
# ---------------------------------------------------------------------------


class TestGraphFlowRolloutToMetrics:
    def test_rollout_to_metrics_end_to_end(self, episode_logs_root):
        seed_graph = _seed_graph(episode_logs_root)
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        rollout = _rollout(model, seed_graph, n_steps=10)

        metrics = graph_flow_rollout_to_episode_metrics(
            rollout, initial_total_uavs=seed_graph.total_uavs.item()
        )

        # Same schema as MetricsCollector.get_metrics() -- the literal
        # "plug into end-episode-metrics logic" ask.
        assert set(metrics.keys()) == EXPECTED_METRIC_KEYS
        assert metrics["total_steps"] == 10
        # unique_uavs is exact: it's the caller-supplied ground-truth initial
        # fleet size, not a model prediction (see rollout_metrics.py docstring).
        assert metrics["unique_uavs"] == pytest.approx(seed_graph.total_uavs.item())

        for key, value in metrics.items():
            assert isinstance(value, (int, float)), f"{key} is not a scalar: {value!r}"
            assert value == value, f"{key} is NaN"  # NaN != NaN
            assert value not in (float("inf"), float("-inf")), f"{key} is infinite"

    def test_rollout_total_uavs_never_increases(self, episode_logs_root):
        seed_graph = _seed_graph(episode_logs_root)
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        rollout = _rollout(model, seed_graph, n_steps=10)

        # Regression-pin for the "drop-in replacement" correctness property:
        # checkable even with random/untrained weights, since it's enforced by
        # the deterministic decrement/clamp arithmetic in
        # GraphFlowGNN._finalize, not by learned prediction accuracy.
        totals = [seed_graph.total_uavs.item()] + [g.total_uavs.item() for g in rollout]
        for earlier, later in zip(totals, totals[1:]):
            assert later <= earlier + 1e-5

        metrics = graph_flow_rollout_to_episode_metrics(
            rollout, initial_total_uavs=seed_graph.total_uavs.item()
        )
        assert metrics["peak_active_uavs"] == pytest.approx(
            max(g.total_uavs.item() for g in rollout)
        )
        assert metrics["peak_active_uavs"] <= metrics["unique_uavs"] + 1e-5

    def test_recurrent_variant_also_rolls_out(self, episode_logs_root):
        """GraphFlowRecurrentGNN shares _finalize with GraphFlowGNN -- confirm
        the rollout adapter works identically for it."""
        from rl.surrogate.backbones.graph_flow_gnn import GraphFlowRecurrentGNN

        seed_graph = _seed_graph(episode_logs_root)
        model = GraphFlowRecurrentGNN(hidden_dim=16, num_mp_rounds=2)
        model.reset_hidden()
        rollout = _rollout(model, seed_graph, n_steps=5)

        metrics = graph_flow_rollout_to_episode_metrics(
            rollout, initial_total_uavs=seed_graph.total_uavs.item()
        )
        assert set(metrics.keys()) == EXPECTED_METRIC_KEYS
        assert metrics["total_steps"] == 5

    def test_empty_rollout_raises(self):
        with pytest.raises(ValueError):
            graph_flow_rollout_to_episode_metrics([], initial_total_uavs=5.0)

    def test_zero_initial_uavs_raises(self, episode_logs_root):
        seed_graph = _seed_graph(episode_logs_root)
        model = GraphFlowGNN(hidden_dim=16, num_mp_rounds=2)
        rollout = _rollout(model, seed_graph, n_steps=1)
        with pytest.raises(ValueError):
            graph_flow_rollout_to_episode_metrics(rollout, initial_total_uavs=0.0)


# ---------------------------------------------------------------------------
# dual_graph: fixed-size padding/mask + rollout adapter
# ---------------------------------------------------------------------------


def _vp_snap():
    return {
        "0": {"x": 0.0, "y": 0.0, "capacity": 4, "n_grounded": 1, "n_landing_queue": 0},
        "1": {"x": 100.0, "y": 0.0, "capacity": 4, "n_grounded": 1, "n_landing_queue": 0},
    }


def _uav_snap(x, y, target_idx, missions=0):
    return {
        "x": x, "y": y, "z": 0.0, "vx": 1.0, "vy": 0.0, "vz": 0.0,
        "speed": 1.0, "heading": 0.0, "collision_status": 1,
        "mission_complete": False, "num_missions_completed": missions,
        "dist_to_goal": 10.0, "target_vertiport_idx": target_idx,
    }


def _write_synthetic_collision_episode(episode_dir: str) -> None:
    """A small, deterministic 3-step step_history.json with UAV '2' removed
    via a restricted-area collision transitioning into step 1 -- used instead
    of relying on chance from a real random episode (collisions are rare in a
    short, small random scenario)."""
    steps = [
        {
            "step": 0, "num_active_uavs": 3,
            "uavs": {
                "0": _uav_snap(1.0, 1.0, 1), "1": _uav_snap(2.0, 2.0, None),
                "2": _uav_snap(50.0, 0.0, 1, missions=1),
            },
            "collisions": {"nmac_pairs": [], "uav_collision_pairs": [], "ra_collision_ids": []},
            "vertiports": _vp_snap(), "edges": {},
        },
        {
            "step": 1, "num_active_uavs": 2,
            "uavs": {"0": _uav_snap(1.5, 1.5, 1), "1": _uav_snap(2.5, 2.5, None)},
            "collisions": {"nmac_pairs": [], "uav_collision_pairs": [], "ra_collision_ids": ["2"]},
            "vertiports": _vp_snap(), "edges": {},
        },
        {
            "step": 2, "num_active_uavs": 2,
            "uavs": {"0": _uav_snap(2.0, 2.0, 1), "1": _uav_snap(3.0, 3.0, None)},
            "collisions": {"nmac_pairs": [], "uav_collision_pairs": [], "ra_collision_ids": []},
            "vertiports": _vp_snap(), "edges": {},
        },
    ]
    with open(os.path.join(episode_dir, "step_history.json"), "w") as f:
        json.dump(steps, f)


def _dual_graph_rollout(model, seed_graph, n_steps):
    model.eval()
    rollout = []
    current = seed_graph
    with torch.no_grad():
        for _ in range(n_steps):
            preds = model.predict_dual_graph_next_state(current)
            current["uav"].x = preds["uav_x"]
            current["uav"].mask = preds["uav_mask"]
            current.total_uavs = preds["uav_mask"].sum().unsqueeze(0)
            rollout.append(current)
    return rollout


class TestDualGraphNodeCountStaysFixed:
    def test_node_count_stays_fixed_across_collision(self, tmp_path):
        """Regression test for the originally-discovered latent bug: a real
        UAV-UAV or restricted-area collision removes a UAV mid-episode
        (persist_collided_uavs defaults to False in component_schema.py, and
        isn't overridden by the surrogate sweep config), so a (g_t, g_tp1)
        pair spanning that transition used to have genuinely different UAV
        node counts, crashing train_dual_graph.py's loss_fn. Confirms the
        fixed-size padding/mask fix: every pair in an episode with an
        engineered collision has identical shapes, and the mask only ever
        turns 0, never back to 1.
        """
        _write_synthetic_collision_episode(str(tmp_path))
        ds = DualGraphDataset([str(tmp_path)])
        assert len(ds) == 2  # 3 steps -> 2 consecutive pairs

        max_uavs = ds[0][0]["uav"].x.shape[0]
        assert max_uavs == 3  # episode's initial (step 0) fleet size

        prev_mask = None
        for g_t, g_tp1 in ds:
            assert g_t["uav"].x.shape == (max_uavs, len(UAV_NODE_KEYS))
            assert g_tp1["uav"].x.shape == (max_uavs, len(UAV_NODE_KEYS))
            for mask in (g_t["uav"].mask, g_tp1["uav"].mask):
                if prev_mask is not None:
                    # Monotonic: a slot that was already 0 must stay 0.
                    assert torch.all(mask[prev_mask == 0] == 0)
                prev_mask = mask

        # The engineered collision actually happened: mask ends with one dead slot.
        _, g_last = ds[-1]
        assert g_last["uav"].mask.sum().item() == 2

    def test_removed_slot_values_freeze(self, tmp_path):
        """A removed UAV's row freezes at its last real snapshot (position/
        heading/target/missions_completed), with velocity zeroed and
        collision_status forced to 0 -- the ground-truth fix for
        collision_status never actually flipping under the real
        (persist_collided_uavs=False) data-collection config."""
        _write_synthetic_collision_episode(str(tmp_path))
        ds = DualGraphDataset([str(tmp_path)])
        g0, g1 = ds[0]  # step0 (uav '2' real) -> step1 (uav '2' removed)

        slot = 2  # uav ids sorted '0','1','2' -> slot 2
        row = dict(zip(UAV_NODE_KEYS, g1["uav"].x[slot].tolist()))
        assert row["x"] == pytest.approx(50.0)  # frozen at step0's real position
        assert row["y"] == pytest.approx(0.0)
        assert row["vx"] == 0.0 and row["vy"] == 0.0 and row["vz"] == 0.0
        assert row["speed"] == 0.0
        assert row["collision_status"] == 0.0
        assert row["missions_completed"] == pytest.approx(1.0)  # frozen from step0
        assert row["in_ra_collision_event"] == 1.0  # fires exactly on the transition step
        assert g1["uav"].mask[slot].item() == 0.0


class TestDualGraphRolloutToMetrics:
    def test_rollout_to_metrics_end_to_end(self, tmp_path):
        _write_synthetic_collision_episode(str(tmp_path))
        ds = DualGraphDataset([str(tmp_path)])
        seed_graph, _ = ds[0]
        max_uavs = seed_graph["uav"].x.shape[0]

        model = DualGraphGNN(hidden_dim=16, num_mp_rounds=2)
        rollout = _dual_graph_rollout(model, seed_graph, n_steps=8)

        metrics = dual_graph_rollout_to_episode_metrics(rollout)

        assert set(metrics.keys()) == EXPECTED_METRIC_KEYS
        assert metrics["total_steps"] == 8
        # unique_uavs is exact and structural (max_uavs), not a model prediction.
        assert metrics["unique_uavs"] == pytest.approx(max_uavs)

        for key, value in metrics.items():
            assert isinstance(value, (int, float)), f"{key} is not a scalar: {value!r}"
            assert value == value, f"{key} is NaN"
            assert value not in (float("inf"), float("-inf")), f"{key} is infinite"

    def test_mask_sum_never_increases_across_rollout(self, tmp_path):
        _write_synthetic_collision_episode(str(tmp_path))
        ds = DualGraphDataset([str(tmp_path)])
        seed_graph, _ = ds[0]

        model = DualGraphGNN(hidden_dim=16, num_mp_rounds=2)
        rollout = _dual_graph_rollout(model, seed_graph, n_steps=8)

        totals = [seed_graph["uav"].mask.sum().item()] + [
            g["uav"].mask.sum().item() for g in rollout
        ]
        for earlier, later in zip(totals, totals[1:]):
            assert later <= earlier + 1e-5

        metrics = dual_graph_rollout_to_episode_metrics(rollout)
        assert metrics["peak_active_uavs"] <= metrics["unique_uavs"] + 1e-5

    def test_dead_slot_frozen_across_multiple_rollout_steps(self, tmp_path):
        """The dead-stays-dead invariant: once a slot dies (predicted or real),
        it must never come back, and its one-shot event flags
        (in_collision_event/in_ra_collision_event) must not perpetuate forward."""
        _write_synthetic_collision_episode(str(tmp_path))
        ds = DualGraphDataset([str(tmp_path)])
        g1, _ = ds[1]  # step1: uav '2' already dead (ground truth)

        model = DualGraphGNN(hidden_dim=16, num_mp_rounds=2)
        rollout = _dual_graph_rollout(model, g1, n_steps=5)

        slot = 2
        ra_idx = UAV_NODE_KEYS.index("in_ra_collision_event")
        for g in rollout:
            assert g["uav"].mask[slot].item() == 0.0
            assert g["uav"].x[slot, ra_idx].item() == 0.0

    def test_empty_rollout_raises(self):
        with pytest.raises(ValueError):
            dual_graph_rollout_to_episode_metrics([])


class TestDualGraphStillPredictsOnFixedNodeCount:
    def test_dual_graph_still_predicts_on_fixed_node_count(self, episode_logs_root):
        """Sanity check that DualGraphGNN still works normally against real
        simulator-generated logs (the common case, most windows have no
        collision) -- confirms the event-flag columns, degree-normalized
        aggregation, and dead-stays-dead invariant didn't break the existing
        forward pass."""
        ds = DualGraphDataset.from_logs_root(episode_logs_root)
        if len(ds) == 0:
            pytest.skip("No UAV/vertiport snapshots in logs")
        g_t, _ = ds[0]

        model = DualGraphGNN(hidden_dim=16, num_mp_rounds=2)
        model.eval()
        with torch.no_grad():
            preds = model.predict_dual_graph_next_state(g_t)
        assert preds["uav_x"].shape == g_t["uav"].x.shape
        assert preds["vp_x"].shape == g_t["vertiport"].x.shape
