"""
Layer: Functional correctness test for the 5 collision-check lines in
SimulatorManager._step_uavS (simulator_manager.py, lines 329-333):
    get_detection_restricted_area, get_detection_other_uavS, get_nmac,
    get_collision_restricted_area, get_collision_uavS

Scenario: 3 UAVs built directly (bypassing Airspace/ATC/planner/dynamics),
positions scripted per step via conftest.set_scripted_positions:
  - UAV 0 (A) and UAV 1 (B) close head-on along the x-axis.
        detect@10, nmac@13, collision@15 (a single-step event -- distance
        passes through 0 and separates again by step 16).
  - UAV 2 (C) drifts in along the y-axis and only grazes NMAC range with
    A and B at step 18 (also a single step), never reaching the collision
    threshold -- a near-miss, not a collision.

Run in isolation:
    pytest tests/test_collision_scenario.py -v
"""
import pytest

from conftest import build_three_uav_rig, set_scripted_positions

STEPS = range(0, 21)  # t = 0..20 inclusive


@pytest.fixture(scope='module')
def step_history():
    """Run the scripted 3-UAV scenario once, recording the 5 lines' output per step.

    Returns:
        Dict[int, dict]: t -> {
            'detect_ra': {...}, 'detect_uav': {...}, 'nmac': {...},
            'collide_ra': {...}, 'collide_uav': {...}
        }
    """
    uav_dict, sensor_module = build_three_uav_rig()
    history = {}
    for t in STEPS:
        set_scripted_positions(uav_dict, t)
        history[t] = {
            'detect_ra': sensor_module.get_detection_restricted_area(),
            'detect_uav': sensor_module.get_detection_other_uavS(),
            'nmac': sensor_module.get_nmac(),
            'collide_ra': sensor_module.get_collision_restricted_area(),
            'collide_uav': sensor_module.get_collision_uavS(),
        }
    return history


# ── Smoke: no exceptions, correct shape ───────────────────────────────────────

class TestOutputShape:
    def test_all_five_dicts_keyed_by_every_uav(self, step_history):
        for t, snap in step_history.items():
            for key in ('detect_ra', 'detect_uav', 'nmac', 'collide_ra', 'collide_uav'):
                assert set(snap[key].keys()) == {0, 1, 2}, (
                    f"step {t}: {key} missing a UAV id"
                )

    def test_all_values_are_sets(self, step_history):
        for t, snap in step_history.items():
            for key in ('detect_ra', 'detect_uav', 'nmac', 'collide_ra', 'collide_uav'):
                for uav_id, value in snap[key].items():
                    assert isinstance(value, set), (
                        f"step {t}: {key}[{uav_id}] is {type(value)}, not set"
                    )


# ── Restricted area calls are no-ops (no RA geometry injected) ───────────────

class TestRestrictedAreaIsNoOp:
    def test_detect_ra_always_empty(self, step_history):
        for t, snap in step_history.items():
            for uav_id in (0, 1, 2):
                assert snap['detect_ra'][uav_id] == set(), f"step {t}, uav {uav_id}"

    def test_collide_ra_always_empty(self, step_history):
        for t, snap in step_history.items():
            for uav_id in (0, 1, 2):
                assert snap['collide_ra'][uav_id] == set(), f"step {t}, uav {uav_id}"


# ── Structural invariants: collision subset of nmac subset of detection ──────

class TestChainInvariants:
    def test_nmac_is_subset_of_detection(self, step_history):
        for t, snap in step_history.items():
            for uav_id in (0, 1, 2):
                assert snap['nmac'][uav_id] <= snap['detect_uav'][uav_id], (
                    f"step {t}, uav {uav_id}: nmac not subset of detection"
                )

    def test_collision_is_subset_of_nmac(self, step_history):
        for t, snap in step_history.items():
            for uav_id in (0, 1, 2):
                assert snap['collide_uav'][uav_id] <= snap['nmac'][uav_id], (
                    f"step {t}, uav {uav_id}: collision not subset of nmac"
                )

    def test_detection_is_symmetric(self, step_history):
        """All 3 UAVs share the same detection_radius, so i detects j iff j detects i."""
        for t, snap in step_history.items():
            for uav_id in (0, 1, 2):
                for other_id in snap['detect_uav'][uav_id]:
                    assert uav_id in snap['detect_uav'][other_id], (
                        f"step {t}: {uav_id}->{other_id} detection not symmetric"
                    )


# ── A-B detection sequence (head-on pair) ─────────────────────────────────────

class TestDetectionSequence:
    def test_not_detected_before_step_10(self, step_history):
        assert step_history[9]['detect_uav'][0] == set()
        assert step_history[9]['detect_uav'][1] == set()

    def test_detected_at_step_10(self, step_history):
        assert step_history[10]['detect_uav'][0] == {1}
        assert step_history[10]['detect_uav'][1] == {0}


# ── A-B NMAC sequence ──────────────────────────────────────────────────────

class TestNmacSequence:
    def test_no_nmac_before_step_13(self, step_history):
        assert step_history[12]['nmac'][0] == set()
        assert step_history[12]['nmac'][1] == set()

    def test_nmac_at_step_13(self, step_history):
        assert step_history[13]['nmac'][0] == {1}
        assert step_history[13]['nmac'][1] == {0}


# ── A-B collision sequence (single-step event: distance crosses 0 at t=15) ───

class TestCollisionSequence:
    def test_no_collision_before_step_15(self, step_history):
        assert step_history[14]['collide_uav'][0] == set()
        assert step_history[14]['collide_uav'][1] == set()

    def test_collision_at_step_15(self, step_history):
        assert step_history[15]['collide_uav'][0] == {1}
        assert step_history[15]['collide_uav'][1] == {0}

    def test_no_longer_colliding_after_step_15(self, step_history):
        """A and B pass through each other and separate again."""
        assert step_history[16]['collide_uav'][0] == set()
        assert step_history[16]['collide_uav'][1] == set()


# ── UAV C: near-miss with A and B, never a collision ──────────────────────────

class TestNearMissUAV:
    def test_c_never_collides_with_anyone(self, step_history):
        for t, snap in step_history.items():
            assert snap['collide_uav'][2] == set(), f"step {t}: C collided"
            for uav_id in (0, 1):
                assert 2 not in snap['collide_uav'][uav_id], (
                    f"step {t}: uav {uav_id} collided with C"
                )

    def test_c_grazes_nmac_only_at_step_18(self, step_history):
        assert step_history[17]['nmac'][2] == set()
        assert step_history[18]['nmac'][2] == {0, 1}
        assert step_history[19]['nmac'][2] == set()

    def test_c_detected_by_both_starting_step_13(self, step_history):
        assert step_history[12]['detect_uav'][2] == set()
        assert step_history[13]['detect_uav'][2] == {0, 1}
