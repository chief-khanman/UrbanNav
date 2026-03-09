"""
Layer 2: Single-step tests.

Validates that one simulation step executes correctly through the full
plan → control → dynamics → sensor → mission-lifecycle pipeline.

Run in isolation:
    pytest tests/test_step.py -v
"""
import sys
import os
import pytest

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.abspath(PROJECT_ROOT))

from uam_simulator import UAMSimulator


# Each test in this module gets a fresh sim so step count is always 0→1.
@pytest.fixture(scope='module')
def stepped_sim(config_path):
    """Reset sim, execute exactly one step, return it."""
    s = UAMSimulator(config_path=config_path)
    s.reset()
    s.step({})   # empty bundle — fleet uses internal planners/controllers
    return s


# ── Step counter ─────────────────────────────────────────────────────────────

class TestStepCounter:
    def test_step_increments_counter(self, stepped_sim):
        state = stepped_sim.get_state()
        assert state.currentstep == 1

    def test_step_does_not_crash(self, config_path):
        """Explicit smoke: step must not raise any exception."""
        s = UAMSimulator(config_path=config_path)
        s.reset()
        s.step({})  # should complete without exception


# ── State structure ───────────────────────────────────────────────────────────

class TestStateStructureAfterStep:
    def test_state_returned_by_get_state(self, stepped_sim):
        state = stepped_sim.get_state()
        assert state is not None

    def test_state_has_atc_state(self, stepped_sim):
        state = stepped_sim.get_state()
        assert state.atc_state is not None

    def test_state_has_airspace_state(self, stepped_sim):
        state = stepped_sim.get_state()
        assert state.airspace_state is not None

    def test_atc_state_is_dict(self, stepped_sim):
        """atc_state is the uav_dict: {uav_id: UAV}."""
        state = stepped_sim.get_state()
        assert isinstance(state.atc_state, dict)


# ── UAV movement ──────────────────────────────────────────────────────────────

class TestUAVMovement:
    def test_uav_positions_exist(self, stepped_sim):
        """After one step every active UAV must have a current_position."""
        uav_dict = stepped_sim.simulator_manager.atc.uav_dict
        for uav_id, uav in uav_dict.items():
            assert hasattr(uav, 'current_position'), (
                f"UAV {uav_id} missing current_position after one step"
            )

    def test_at_least_one_uav_active(self, stepped_sim):
        """UAV dict must not be empty after one step."""
        assert len(stepped_sim.simulator_manager.atc.uav_dict) > 0


# ── Pipeline wiring ───────────────────────────────────────────────────────────

class TestPipelineWiring:
    def test_planner_module_exists(self, stepped_sim):
        assert stepped_sim.simulator_manager.planner_module is not None

    def test_controller_module_exists(self, stepped_sim):
        assert stepped_sim.simulator_manager.controller_module is not None

    def test_dynamics_module_exists(self, stepped_sim):
        assert stepped_sim.simulator_manager.dynamics_module is not None

    def test_sensor_module_exists(self, stepped_sim):
        assert stepped_sim.simulator_manager.sensor_module is not None
