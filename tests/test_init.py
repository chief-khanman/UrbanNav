"""
Layer 1: Initialization tests.

Validates that the simulator can be configured, constructed, and reset
without errors. These tests must pass before any step-level tests are run.

Run in isolation:
    pytest tests/test_init.py -v
"""
import pytest
from urbannav.component_schema import UAMConfig, build_fleet_blueprint


# ── Config ──────────────────────────────────────────────────────────────────

class TestConfigLoads:
    def test_config_parses_without_error(self, config_path):
        """YAML is valid and all fields pass Pydantic validation."""
        config = UAMConfig.load_from_yaml(config_path)
        assert config is not None

    def test_simulator_fields(self, config_path):
        config = UAMConfig.load_from_yaml(config_path)
        assert config.simulator.dt > 0
        assert config.simulator.total_timestep > 0
        assert config.simulator.mode in ('2D', '3D')

    def test_fleet_composition_present(self, config_path):
        config = UAMConfig.load_from_yaml(config_path)
        assert len(config.fleet_composition) > 0

    def test_fleet_total_count(self, config_path):
        """Each fleet entry's count must parse as a positive int; total > 0."""
        config = UAMConfig.load_from_yaml(config_path)
        assert len(config.fleet_composition) > 0
        for entry in config.fleet_composition:
            assert isinstance(entry.count, int)
            assert entry.count > 0
        assert sum(e.count for e in config.fleet_composition) > 0

    def test_build_fleet_blueprint_produces_blueprints(self, config_path):
        config = UAMConfig.load_from_yaml(config_path)
        blueprints = build_fleet_blueprint(config)
        total = sum(e.count for e in config.fleet_composition)
        assert len(blueprints) == total


# ── Construction ─────────────────────────────────────────────────────────────

class TestSimulatorInit:
    def test_simulator_constructs(self, config_path):
        """UAMSimulator.__init__ completes and exposes expected attributes."""
        from urbannav.uam_simulator import UAMSimulator
        s = UAMSimulator(config_path=config_path)
        assert hasattr(s, 'simulator_manager')
        assert hasattr(s, 'total_timestep')
        assert s.total_timestep > 0

    def test_simulator_manager_created(self, config_path):
        from urbannav.uam_simulator import UAMSimulator
        s = UAMSimulator(config_path=config_path)
        assert s.simulator_manager is not None


# ── Reset ────────────────────────────────────────────────────────────────────

class TestSimulatorReset:
    def test_reset_populates_uav_dict(self, sim, config_path):
        """After reset(), ATC must hold exactly as many UAVs as the config requests."""
        expected_total = sum(e.count for e in UAMConfig.load_from_yaml(config_path).fleet_composition)
        uav_dict = sim.simulator_manager.atc.uav_dict
        assert len(uav_dict) == expected_total, (
            f"Expected {expected_total} UAVs per sample_config.yaml's "
            f"fleet_composition, got {len(uav_dict)}"
        )

    def test_reset_creates_vertiports(self, sim):
        """Airspace must have at least one vertiport after reset()."""
        vp_list = sim.simulator_manager.airspace.vertiport_list
        assert len(vp_list) > 0, "No vertiports were created during reset()"

    def test_reset_populates_dynamics_map(self, sim):
        dynamics_map = sim.simulator_manager.atc.dynamics_map
        assert len(dynamics_map) > 0, "dynamics_map is empty after reset()"

    def test_reset_populates_controller_map(self, sim):
        controller_map = sim.simulator_manager.atc.controller_map
        assert len(controller_map) > 0, "controller_map is empty after reset()"

    def test_reset_populates_planner_map(self, sim):
        planner_map = sim.simulator_manager.atc.planner_map
        assert len(planner_map) > 0, "planner_map is empty after reset()"

    def test_reset_populates_sensor_map(self, sim):
        sensor_map = sim.simulator_manager.atc.sensor_map
        assert len(sensor_map) > 0, "sensor_map is empty after reset()"

    def test_reset_initial_step_is_zero(self, sim):
        state = sim.get_state()
        assert state.currentstep == 0

    def test_state_has_required_fields(self, sim):
        state = sim.get_state()
        assert hasattr(state, 'timestamp')
        assert hasattr(state, 'currentstep')
        assert hasattr(state, 'airspace_state')
        assert hasattr(state, 'atc_state')
        assert hasattr(state, 'external_systems')
