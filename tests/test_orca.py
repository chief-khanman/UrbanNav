"""
ORCA controller/dynamics tests on bare UAV objects (no Airspace/ATC/OSM).

Two STANDARD-size UAVs fly head-on at each other's start point. With ORCA they
must pass without overlapping (separation > r_A + r_B) and still reach their
targets; at different altitudes ORCA must ignore each other and fly straight.
"""
import math

import pytest
from pydantic import ValidationError
from shapely import Point

from conftest import (
    STANDARD_DETECTION_RADIUS,
    STANDARD_NMAC_RADIUS,
    STANDARD_RADIUS,
    _FAR_VERTIPORT_LOCATION,
)
from urbannav.component_schema import UAVFleetInstanceConfig
from urbannav.controller_orca import ORCAController
from urbannav.dynamics_orca import ORCADynamics
from urbannav.uav import UAV
from urbannav.vertiport import Vertiport

DT = 1.0
MAX_SPEED = 10.0
HALF_GAP = 500.0


def _build_head_on_pair(z_a: float = 0.0, z_b: float = 0.0):
    """Build UAVs 0 (at -HALF_GAP, 0) and 1 (at +HALF_GAP, 0), each targeting the other's
    start. Returns (uav_dict, targets, controllers, dynamics)."""
    vp = Vertiport(_FAR_VERTIPORT_LOCATION)  # far away: sensors stay operational
    starts = {0: (-HALF_GAP, 0.0, z_a), 1: (HALF_GAP, 0.0, z_b)}
    uav_dict, targets, controllers = {}, {}, {}
    for uav_id, (x, y, z) in starts.items():
        uav = UAV(radius=STANDARD_RADIUS, nmac_radius=STANDARD_NMAC_RADIUS,
                  detection_radius=STANDARD_DETECTION_RADIUS, _id=uav_id)
        uav.id_ = uav_id
        uav.max_speed = MAX_SPEED
        uav.assign_start_end(vp, vp)
        uav.current_position = Point(x, y)
        uav.px, uav.py, uav.pz = x, y, z
        uav_dict[uav_id] = uav
        targets[uav_id] = Point(-x, y)
        controller = ORCAController(DT)
        controller.bind_fleet(uav_dict)
        controllers[uav_id] = controller
    dynamics = ORCADynamics()
    dynamics.dt = DT
    return uav_dict, targets, controllers, dynamics


def _run(uav_dict, targets, controllers, dynamics, steps: int = 200):
    """Step the pair synchronously; return (min separation, max |y| deviation)."""
    min_sep, max_dev = math.inf, 0.0
    for _ in range(steps):
        # gather all actions first, then integrate (same order as SimulatorManager)
        actions = {uid: controllers[uid].get_control_action(uav, targets[uid])
                   for uid, uav in uav_dict.items()}
        for uid, action in actions.items():
            dynamics.step(action, uav_dict[uid])
        a, b = uav_dict[0], uav_dict[1]
        min_sep = min(min_sep, a.current_position.distance(b.current_position))
        max_dev = max(max_dev, abs(a.py), abs(b.py))
    return min_sep, max_dev


def test_head_on_pair_avoids_collision_and_reaches_goals():
    uav_dict, targets, controllers, dynamics = _build_head_on_pair()
    min_sep, max_dev = _run(uav_dict, targets, controllers, dynamics)

    assert min_sep > 2 * STANDARD_RADIUS
    assert max_dev > 1.0  # they actually swerved
    for uid, uav in uav_dict.items():
        assert uav.current_position.distance(targets[uid]) < 1.0
        assert uav.current_speed <= MAX_SPEED + 1e-9


def test_vertically_separated_pair_flies_straight():
    uav_dict, targets, controllers, dynamics = _build_head_on_pair(z_a=0.0, z_b=100.0)
    _, max_dev = _run(uav_dict, targets, controllers, dynamics)

    assert max_dev < 1e-9
    for uid, uav in uav_dict.items():
        assert uav.current_position.distance(targets[uid]) < 1.0


def test_orca_controller_requires_orca_dynamics():
    common = dict(type_name='ORCA', count=1, sensor='PartialSensor', planner='ORCA')
    UAVFleetInstanceConfig(dynamics='ORCA', controller='ORCA', **common)
    with pytest.raises(ValidationError):
        UAVFleetInstanceConfig(dynamics='PointMass', controller='ORCA', **common)
    with pytest.raises(ValidationError):
        UAVFleetInstanceConfig(dynamics='ORCA', controller='PIDPointMassController', **common)
