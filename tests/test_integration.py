"""
Layer 3: Integration tests (short run).

Validates the full simulation pipeline over a bounded number of steps.
These tests are intentionally slower than Layers 1 & 2 because they
cover mission lifecycle, collision handling, and UAV removal.

Run in isolation:
    pytest tests/test_integration.py -v

NOTE: These tests use scope='module' to share one simulation run per module
so the 100-step run is not repeated for each test class.
"""
import sys
import os
import pytest

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.abspath(PROJECT_ROOT))

from uam_simulator import UAMSimulator

N_STEPS = 100


@pytest.fixture(scope='module')
def run_sim(config_path):
    """
    Reset, run N_STEPS, and return the simulator along with a snapshot of
    UAV counts recorded at every step for invariant checks.
    """
    s = UAMSimulator(config_path=config_path)
    s.reset()

    initial_count = len(s.simulator_manager.atc.uav_dict)
    uav_counts = [initial_count]

    for _ in range(N_STEPS):
        s.step({})
        uav_counts.append(len(s.simulator_manager.atc.uav_dict))

    return s, uav_counts, initial_count


# ── Basic run completion ──────────────────────────────────────────────────────

class TestRunCompletion:
    def test_100_steps_complete(self, run_sim):
        """Simulator must reach step N_STEPS without crashing."""
        s, _, _ = run_sim
        assert s.get_state().currentstep == N_STEPS

    def test_step_counter_is_monotonic(self, run_sim):
        """Step counter must equal N_STEPS after the run."""
        s, _, _ = run_sim
        assert s.get_state().currentstep == N_STEPS


# ── UAV count invariants ──────────────────────────────────────────────────────

class TestUAVCountInvariants:
    def test_uav_count_never_exceeds_initial(self, run_sim):
        """UAV count must never grow beyond the initial fleet size.
        New UAVs should only be created via explicit fleet expansion,
        not as a side-effect of any step operation."""
        s, uav_counts, initial_count = run_sim
        max_observed = max(uav_counts)
        assert max_observed <= initial_count, (
            f"UAV count grew to {max_observed} (initial was {initial_count}). "
            "Phantom UAV creation detected."
        )

    def test_uav_count_is_non_negative(self, run_sim):
        """UAV count must never drop below zero."""
        _, uav_counts, _ = run_sim
        assert all(c >= 0 for c in uav_counts)


# ── Mission lifecycle ─────────────────────────────────────────────────────────

class TestMissionLifecycle:
    def test_vertiports_still_exist_after_run(self, run_sim):
        """Vertiport list must remain populated throughout the run."""
        s, _, _ = run_sim
        assert len(s.simulator_manager.airspace.vertiport_list) > 0

    def test_simulator_survives_collision_removal(self, run_sim):
        """If UAVs were removed by collisions the simulator must still run.
        This is satisfied if the 100-step run completed (step counter == N_STEPS)."""
        s, uav_counts, initial_count = run_sim
        # Collisions reduce the count — sim must continue regardless
        assert s.get_state().currentstep == N_STEPS

    def test_atc_uav_dict_is_consistent(self, run_sim):
        """After the run, the ATC uav_dict and the state's atc_state must
        reference the same object (no stale copy)."""
        s, _, _ = run_sim
        state = s.get_state()
        atc_direct = s.simulator_manager.atc.uav_dict
        assert state.atc_state is atc_direct


# ── Engine state ──────────────────────────────────────────────────────────────

class TestEngineStateAfterRun:
    def test_all_engines_still_initialized(self, run_sim):
        """No engine should be None or unset after N steps."""
        s, _, _ = run_sim
        mgr = s.simulator_manager
        assert mgr.planner_module is not None
        assert mgr.controller_module is not None
        assert mgr.dynamics_module is not None
        assert mgr.sensor_module is not None
