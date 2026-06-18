"""
Layer: Performance smoke-test for the 5 collision-check lines in
SimulatorManager._step_uavS (simulator_manager.py, lines 329-333):
    get_detection_restricted_area, get_detection_other_uavS, get_nmac,
    get_collision_restricted_area, get_collision_uavS

This test originally surfaced two performance bugs (since fixed):
  1. SpatialHash grid spacing was sized off UAV body radius (34m) instead of
     detection_radius (500m), so each broad-phase query scanned ~30x30x30
     cells instead of ~3x3x3. This was the dominant cost (~16ms/call with
     just 3 UAVs). Fixed in sensor_engine.py: spacing is now left as None
     and computed lazily as max(detection_radius) in PartialSensor.update().
  2. get_nmac() re-invokes detection internally, and get_collision_uavS()
     re-invokes nmac (which re-invokes detection) -- so across the 5-line
     block, UAV-UAV detection ran 3 times, NMAC ran 2 times, and collision
     ran once. Fixed in sensor_partial.py via a per-step memoization cache
     (cleared in update(), populated on first call, reused for the rest of
     the step).

Post-fix, only get_detection_other_uavS does real work on every call -- it's
the line that calls update() and rebuilds the spatial hash each time. The
other four lines compute once (cache miss) and then return cached results on
every subsequent repeat within this test (since nothing calls update() again
between repeats), so their measured per-call cost reflects cache-hit cost,
not fresh computation. That mirrors real per-step usage: SimulatorManager
calls update() once via get_detection_other_uavS, then the remaining 4 lines
reuse that step's cached results.

This is intentionally a *simple* wall-clock measurement -- no hard pass/fail
thresholds, no call-count instrumentation, no cProfile. Just report numbers.
A follow-up test can add stricter profiling once these findings are reviewed.

Run in isolation (need -s to see the printed report; pytest captures stdout
by default):
    pytest tests/test_collision_performance.py -v -s
"""
import sys
import os
import time
import pytest

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.abspath(PROJECT_ROOT))

from conftest import build_three_uav_rig, set_scripted_positions

# Post-fix, get_detection_other_uavS costs roughly 70us/call with 3 UAVs
# (rebuilds the spatial hash every call); the other 4 lines are sub-microsecond
# cache hits after their first call. 100 repeats keeps total runtime well
# under a second while still giving a stable average.
REPEATS = 100

# t=13: A-B are mutually in NMAC range (distance 200) and C has just become
# mutually detected by both A and B (distance ~431.7) -- all 3 pairs are
# "live" in the broad phase at once, making this the busiest snapshot in the
# scripted run (see test_collision_scenario.py for the full schedule).
BUSY_STEP = 13


@pytest.fixture(scope='module')
def busy_rig():
    uav_dict, sensor_module = build_three_uav_rig()
    set_scripted_positions(uav_dict, BUSY_STEP)
    return uav_dict, sensor_module


def _time_call(fn, repeats=REPEATS):
    """Run fn() `repeats` times, return (total_seconds, per_call_microseconds, last_result)."""
    start = time.perf_counter()
    result = None
    for _ in range(repeats):
        result = fn()
    total = time.perf_counter() - start
    return total, (total / repeats) * 1e6, result


class TestCollisionCheckLinesTiming:
    def test_lines_run_without_exception_and_report_timing(self, busy_rig, capsys):
        uav_dict, sensor_module = busy_rig

        lines = [
            ('get_detection_restricted_area', sensor_module.get_detection_restricted_area),
            ('get_detection_other_uavS', sensor_module.get_detection_other_uavS),
            ('get_nmac', sensor_module.get_nmac),
            ('get_collision_restricted_area', sensor_module.get_collision_restricted_area),
            ('get_collision_uavS', sensor_module.get_collision_uavS),
        ]

        report = []
        for name, fn in lines:
            total, per_call_us, result = _time_call(fn)
            assert isinstance(result, dict)
            for uav_id, value in result.items():
                assert isinstance(value, set), f"{name}[{uav_id}] is {type(value)}, not set"
            report.append((name, total, per_call_us))

        with capsys.disabled():
            spacing = sensor_module.sensor_obj_map[0]._spacing
            detection_radius = uav_dict[0].detection_radius
            print(f"\n--- Collision-check line timing "
                  f"({REPEATS} repeats each, step={BUSY_STEP}, 3 UAVs) ---")
            print(f"PartialSensor grid spacing = {spacing} "
                  f"(vs detection_radius = {detection_radius}); "
                  f"a single broad-phase query box spans roughly "
                  f"{round(2 * detection_radius / spacing) + 1} cells per axis.")
            baseline = report[1][2]  # get_detection_other_uavS per-call time
            for name, total, per_call_us in report:
                ratio = per_call_us / baseline if baseline else float('nan')
                print(f"{name:32s} total={total:9.4f}s  per_call={per_call_us:9.2f}us  "
                      f"({ratio:5.2f}x get_detection_other_uavS)")
