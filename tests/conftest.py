"""
Shared pytest fixtures for the UAM Simulator test suite.

Imports use the installed `urbannav` package (`pip install -e .` from repo root).
"""
import os
import pytest

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..')

from urbannav.uam_simulator import UAMSimulator

CONFIG_PATH = os.path.join(PROJECT_ROOT, 'sample_config.yaml')


@pytest.fixture(scope='module')
def config_path():
    return os.path.abspath(CONFIG_PATH)


@pytest.fixture(scope='module')
def sim(config_path):
    """
    Create and reset a UAMSimulator instance once per test module.
    scope='module' avoids re-fetching OSM map data for every test.
    """
    simulator = UAMSimulator(config_path=config_path)
    simulator.reset()
    return simulator


# ── 3-UAV collision-check rig ─────────────────────────────────────────────────
#
# Builds 3 UAV objects directly and wires them into a bare SensorEngine,
# bypassing Airspace/ATC/OSM/planner/dynamics entirely. This gives exact,
# scriptable control over per-step positions so the detection -> NMAC ->
# collision sequence in SimulatorManager._step_uavS (simulator_manager.py
# lines 329-333) can be exercised deterministically.
#
# Used by tests/test_collision_scenario.py and tests/test_collision_performance.py.

from shapely import Point
from urbannav.uav import UAV
from urbannav.vertiport import Vertiport
from urbannav.sensor_engine import SensorEngine

# Mirrors UAV_TYPE_REGISTRY['STANDARD'] in component_schema.py
STANDARD_RADIUS = 17.0
STANDARD_NMAC_RADIUS = 200.0
STANDARD_DETECTION_RADIUS = 500.0

# Far enough from the scripted flight paths (which stay within +/-1500 on
# each axis) that get_sensor_operational() never trips sensor_shutoff_distance
# (3 * radius = 51) and turns sensors off near start/end vertiports.
_FAR_VERTIPORT_LOCATION = Point(1_000_000.0, 1_000_000.0, 0.0)


def build_three_uav_rig():
    """Build 3 STANDARD-type UAVs (ids 0, 1, 2) and a registered SensorEngine.

    Returns:
        (uav_dict, sensor_module)
    """
    start_vp = Vertiport(_FAR_VERTIPORT_LOCATION)
    end_vp = Vertiport(_FAR_VERTIPORT_LOCATION)

    uav_dict = {}
    for uav_id in (0, 1, 2):
        uav = UAV(radius=STANDARD_RADIUS,
                   nmac_radius=STANDARD_NMAC_RADIUS,
                   detection_radius=STANDARD_DETECTION_RADIUS,
                   _id=uav_id)
        uav.id_ = uav_id
        uav.assign_start_end(start_vp, end_vp)
        uav_dict[uav_id] = uav

    sensor_module = SensorEngine(
        config=None,
        sensor_uav_map={'PartialSensor': [0, 1, 2]},
        uav_dict=uav_dict,
        airspace=None,
    )
    sensor_module.register_uav_sensors()
    return uav_dict, sensor_module


def set_scripted_positions(uav_dict, t: int) -> None:
    """Place UAV 0 (A), 1 (B), 2 (C) at their scripted position for step t.

    A and B close head-on along the x-axis at a combined 100 units/step:
        distance_AB(t) = 1500 - 100*t
            detect crossing   (<=500): t=10
            nmac crossing     (<=200): t=13
            collision crossing(<=34) : t=15 (distance hits 0 exactly)

    C drifts in along the y-axis more slowly and only grazes NMAC range
    with A and B (minimum distance ~192 around t=18) -- a near-miss that
    never reaches the 34-unit collision threshold.
    """
    positions = {
        0: (-750.0 + 50.0 * t, 0.0, 0.0),
        1: (750.0 - 50.0 * t, 0.0, 0.0),
        2: (0.0, 1200.0 - 60.0 * t, 0.0),
    }
    for uav_id, (x, y, z) in positions.items():
        uav = uav_dict[uav_id]
        uav.px, uav.py, uav.pz = x, y, z
        uav.current_position = Point(x, y, z)


@pytest.fixture
def three_uav_rig():
    """Function-scoped: fresh (uav_dict, sensor_module) 3-UAV rig."""
    return build_three_uav_rig()
