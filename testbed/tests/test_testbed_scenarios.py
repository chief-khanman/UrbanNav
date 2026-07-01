import os

import pytest

from testbed.testbed_simulator import TestbedSimulator

CONFIGS_DIR = os.path.join(os.path.dirname(__file__), '..', 'configs')

_DENSE_SCENARIO_YAML = """
simulator:
  dt: 1.0
  total_timestep: 300
  mode: '3D'
  seed: 123

logging:
  enabled: false

rendering:
  enabled: false

testbed_airspace:
  pattern: 'ring'
  num_vertiports: 4
  center: [0.0, 0.0]
  radius: 1500.0
  start_angle: 0.7853981633974483
  landing_pad_capacity: 1
  altitude_range: [1800.0, 1800.0]
  buildings:
    - center: [0.0, 0.0]
      width: 300.0
      depth: 300.0
      buffer_radius: 200.0

fleet_composition:
  - type_name: STANDARD
    count: 4
    dynamics: PointMass
    controller: PIDPointMassController
    sensor: PartialSensor
    planner: PointMass-PID
"""


def _write_dense_scenario_config(tmp_path) -> str:
    """A dense square scenario (4 vertiports, 4 STANDARD UAVs, PID/no-avoidance
    controller) with rendering/logging disabled, for fast hermetic assertions."""
    path = tmp_path / 'dense_scenario.yaml'
    path.write_text(_DENSE_SCENARIO_YAML)
    return str(path)


class TestExampleConfigsLoadAndReset:
    @pytest.mark.parametrize('filename,expected_uav_count', [
        ('ring_4.yaml', 4),
        ('ring_8.yaml', 8),
        ('file_placement.yaml', 4),
    ])
    def test_loads_and_resets(self, filename, expected_uav_count):
        sim = TestbedSimulator(config_path=os.path.join(CONFIGS_DIR, filename))
        sim.config.rendering.enabled = False  # keep test hermetic, no disk writes
        sim.reset()
        assert len(sim.simulator_manager.atc.uav_dict) == expected_uav_count


class TestDenseScenarioProducesConflict:
    def test_nmac_or_collision_occurs(self, tmp_path):
        config_path = _write_dense_scenario_config(tmp_path)
        sim = TestbedSimulator(config_path=config_path)
        sim.reset()

        any_nmac_or_collision = False
        for _ in range(sim.total_timestep):
            if len(sim.simulator_manager.atc.uav_dict) < 2:
                break  # all but one UAV collided out — nothing left to conflict
            _, _, nmac_dict, ra_collision_dict, uav_collision_dict = sim.step({})
            event_occurred = (
                any(nmac_dict.values())
                or any(ra_collision_dict.values())
                or any(uav_collision_dict.values())
            )
            if event_occurred:
                any_nmac_or_collision = True
                break

        assert any_nmac_or_collision, (
            "Expected at least one NMAC or collision event in a dense 4-vertiport "
            "square scenario with non-avoidance PID controllers within "
            f"{sim.total_timestep} steps"
        )
