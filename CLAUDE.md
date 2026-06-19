# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

UrbanNav is a research simulation framework for Urban Air Mobility (UAM): multiple UAVs flying
between vertiports in a real-world airspace built from OpenStreetMap data (via `osmnx`), with
pluggable dynamics/controller/planner/sensor models, collision detection, optional RL training
(Gymnasium), logging, and offline/realtime rendering.

## Environment

Conda only — there is no pip/venv path.

```bash
conda config --add channels conda-forge
conda env create -f environment_ubuntu.yml
conda activate AAM_AMOD
```

Activate `AAM_AMOD` before running anything in this repo (scripts, tests, notebooks).

## Common commands

```bash
# Run the full test suite (conftest.py adds repo root to sys.path; no install step needed)
pytest tests/

# Run a single test file / test
pytest tests/test_init.py -v
pytest tests/test_collision_scenario.py::TestClassName::test_name -v

# Run the simulator end-to-end against sample_config.yaml
python -c "
from uam_simulator import UAMSimulator
sim = UAMSimulator(config_path='sample_config.yaml')
sim.reset()
for _ in range(sim.total_timestep):
    sim.step({})
sim.render()
"
```

There is no lint/format/build tooling configured (no `pyproject.toml`, `setup.cfg`, or pytest ini)
— `tests/conftest.py` manually inserts the repo root onto `sys.path` so flat-module imports work.

The `tests/` suite is layered and tests are meant to be read in order: `test_init.py` (config/build)
→ `test_step.py` → `test_collision_scenario.py` / `test_collision_performance.py` →
`test_integration.py` / `test_single_agent_env.py`. The shared `sim` fixture in `conftest.py` is
module-scoped specifically to avoid re-fetching OSM map data per test — don't change that scope
without considering test runtime. `conftest.py` also exposes a `three_uav_rig` fixture that builds
3 UAVs directly with scripted positions, bypassing Airspace/ATC/OSM, to deterministically exercise
the detect → NMAC → collision pipeline.

`Python-RVO2/` is a vendored third-party C++/Cython library (ORCA collision avoidance) with its own
CMake/setup.py build; it is not part of the main Python package and is wired in only as a future
`ORCA` dynamics/controller backend (see `VALID_DYNAMICS`/`VALID_CONTROLLERS` in `component_schema.py`).

## Architecture

### Orchestration chain

```
UAMSimulator (uam_simulator.py)
  └─ SimulatorManager (simulator_manager.py)   — owns the step loop
        ├─ Airspace (airspace.py)              — OSM geometry, vertiports, restricted areas
        ├─ ATC (atc.py)                        — owns uav_dict + per-component uav-id maps,
        │                                         vertiport assignment/landing/takeoff cycle
        ├─ SensorEngine (sensor_engine.py)      — detection / NMAC / collision queries
        ├─ PlannerEngine (planner_engine.py)    — produces waypoint plans per UAV
        ├─ AerBus (aer_bus.py)                  — controller orchestration (see below)
        └─ DynamicsEngine (dynamics_engine.py)  — physics integration per UAV
  ├─ Renderer (renderer.py)                     — realtime/offline 2D animation
  └─ Logger (logger.py) / MetricsCollector (metrics_collector.py) — per-episode logs under logs/
```

`UAMSimulator.reset()`/`.step()` is the public entry point used by both plain scripts and the gym
wrapper. `SimulatorManager.reset()` builds Airspace → ATC → the four Engine objects → UAV fleet (in
that order — each step depends on the previous), then calls each engine's
`register_uav_*()` to wire UAVs into per-component maps.

### Config-driven, registry-resolved components

Everything pluggable (dynamics, controller, sensor, planner) is selected in `sample_config.yaml`
by string name, validated by Pydantic models in `component_schema.py` against a `VALID_*` set, and
resolved to a concrete class through a `*_CLASS_MAP` dict living in the corresponding `*_engine.py`
/ `aer_bus.py` module (e.g. `DYNAMICS_CLASS_MAP` in `dynamics_engine.py`,
`CONTROLLER_CLASS_MAP` in `aer_bus.py`). To add a new implementation of any component:
1. Add its string key to the relevant `VALID_*` set in `component_schema.py`.
2. Implement the class against the matching `*_template.py` ABC (`dynamics_template.Dynamics`,
   `controller_template.Controller`, `plan_template.PlannerTemplate`, `sensor_template.Sensor`).
3. Register it in the module's `*_CLASS_MAP`.

UAV *physical* parameters (radius, max speed, etc.) are intentionally **not** in the yaml — they
live in code in `UAV_TYPE_REGISTRY` (`component_schema.py`), keyed by `type_name` (`STANDARD`,
`HEAVY`, `LEARNING`, `ORCA`). The yaml only assigns `fleet_composition` entries (`type_name`,
`count`, `dynamics`, `controller`, `sensor`, `planner`) which `build_fleet_blueprint()` expands into
one `UAVBlueprint` per UAV instance, consumed by `ATC.create_uavS_from_blueprint()`.

`ATC` maintains four `{component_name(str): [uav_id, ...]}` maps (`dynamics_map`, `controller_map`,
`planner_map`, `sensor_map`) built at fleet-creation time; each Engine's `register_uav_*()` reads
its map and instantiates one component object per *type actually present* in the current fleet
(not one per UAV), then keys it by every `uav_id` that uses it.

### RL integration

`type_name: LEARNING` is a reserved fleet entry (at most one per config, enforced by
`validate_fleet_composition()`) whose `controller` must be `None` — its action is supplied
externally instead of by `AerBus`. `mode: TRAIN|TEST` is required only for this entry.
`AerBus.RL_CONTROLLER_NAMES` (`{'RL'}`) marks which controller name signals this skip. The
Gymnasium wrapper `single_agent_gym_env.py::UAMSimEnv` finds the LEARNING UAV id via
`SimulatorManager.get_learning_uav_id()`, supplies its action each step as an `UAVCommand` with
`ActionType.CONTROL`, and `SimulatorManager.map_actions_to_uavs()` merges that external action
with the internally-generated ones before dispatch to `DynamicsEngine`. Observation space
definitions live in `obs_space_definitions.py` (`OBS_SPACE` registry).

### AerBus execution modes

`AerBus` supports three controller execution modes (`ExecutionMode` enum): `INLINE` (same process,
the common case — see `CONTROLLER_CLASS_MAP`), `PROCESS` (subprocess via `multiprocessing.Queue`),
and `EXTERNAL` (remote process over ZeroMQ REQ-REP, for non-Python controllers like MATLAB/ORCA).

### Per-step sequence (`SimulatorManager._step_uavS`)

Plan → control action (internal + external merge) → dynamics integration → sensor queries
(restricted-area detection, UAV-UAV detection, NMAC, restricted-area collision, UAV-UAV collision)
→ remove colliding UAVs via `ATC.remove_uavs_by_id()`. Detection/NMAC/collision are layered radii
checks (`detection_radius` > `nmac_radius` > `radius`) computed off live UAV positions each step —
note the existing inefficiency flagged in code comments: collision implicitly reruns NMAC/detection
internally, so calling all of `get_detection_*`, `get_nmac`, `get_collision_*` separately repeats
work.

After `_step_uavS`, `SimulatorManager.step()` runs the ATC–UAV–vertiport mission cycle: check
arrivals/departures (`has_left_start_vertiport`, `has_reached_end_vertiport`), assign new missions
or hold UAVs idle at a vertiport (`reassign_new_mission` / `wait_at_vertiport`), then process the
landing queue (`check_landing_space`, `landing_procedure`).

### State and commands (`component_schema.py`)

`SimulatorState` is the full serializable snapshot (`to_json`/`from_json`) used for save/replay.
`UAVCommand`/`UAVCommandBundle` (`Dict[uav_id, List[UAVCommand]]`) is the typed external-input
schema (`ActionType`: `CONTROL`, `MISSION_PLAN`, `TRAJECTORY`, `PATH`) used to inject actions or
plans into a step from outside the normal internal pipeline (e.g. from gym or a notebook).

### Dynamics models

Four physical model families exist with progressively more state: `dynamics_point_mass.py` (2D/3D
point mass), `dynamics_holonomic.py` / `dynamics_two_d_vector.py` (2D holonomic), and
`dynamics_six_dof.py` (full 6-DOF rigid body). Each has a matching `controller_*.py` and
`plan_*.py` pair (e.g. `dynamics_point_mass.py` + `controller_pid_point_mass.py` +
`plan_point_mass_pid.py`) — when adding a new dynamics model, add its controller/planner
counterparts together and wire all three into their respective `*_CLASS_MAP`s.

### Sensing

`sensor_engine.py` dispatches to per-UAV `Sensor` instances; `sensor_partial.py` implements
range-limited (`PartialSensor`) detection using `sensor_spatial_hash.py` for efficient
neighbor queries instead of all-pairs distance checks. Restricted-area geometry is injected
post-construction via `Sensor.set_restricted_area_data()` rather than passed at init, since it
comes from `Airspace` which is built after the sensor map exists.

### Two simulation variants

- **Mission/flight simulation** (`uam_simulator.py`, `simulator_manager.py`): UAVs flying between
  existing vertiports — the primary, actively-developed path.
- **Vertiport design simulation** (`uam_simulator_vp_design.py`, `simulator_manager_vp_design.py`,
  `vertiport_design_env.py`, `deployment_vp_design.py`): a separate, parallel pipeline for the
  vertiport *placement* problem (where to put vertiports given OD demand), built on
  region/zone data in `Austin_GEOID_data/` and `band1_output_*/`. These are independent entry
  points, not alternate code paths through the same classes — don't assume changes to
  `simulator_manager.py` propagate to `simulator_manager_vp_design.py` or vice versa.

## Conventions specific to this codebase

- Modules are flat (no `src/` package, no `__init__.py`) and import each other by bare module
  name (e.g. `from atc import ATC`) — this only works because `conftest.py` / scripts run from the
  repo root.
- `*_template.py` files are the ABCs for pluggable components; concrete implementations are named
  `<component>_<variant>.py` (e.g. `controller_pid_point_mass.py`), not nested under a package per
  component type.
- Config validity is enforced almost entirely through Pydantic validators in `component_schema.py`
  (`field_validator`/`model_validator`) rather than ad hoc `if` checks scattered through the
  simulator — when adding new config-driven behavior, add the constraint there first.
- The codebase carries many in-line `#TODO`, `#!`, and `#FIX` comments marking known design debt
  and half-finished refactors (e.g. `simulator_manager.py`'s `map_actions_to_uavs`/
  `map_plans_to_uavs`, `_build_assets`). Read the surrounding comments before refactoring these —
  they usually describe the intended direction already.
