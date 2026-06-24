# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

UrbanNav is a research simulation framework for Urban Air Mobility (UAM): multiple UAVs flying
between vertiports in a real-world airspace built from OpenStreetMap data (via `osmnx`), with
pluggable dynamics/controller/planner/sensor models, collision detection, optional RL training
(Gymnasium), logging, and offline/realtime rendering.

## Environment

Conda + an editable pip install — there is no plain pip/venv-only path.

```bash
conda config --add channels conda-forge
conda env create -f environment_ubuntu.yml
conda activate AAM_AMOD
pip install -e .          # installs src/urbannav and rl as editable packages
```

Activate `AAM_AMOD` before running anything in this repo (scripts, tests, notebooks). Re-run
`pip install -e .` any time a pull touches `pyproject.toml` or adds/removes packages.

## Common commands

```bash
# Run the full test suite (testpaths = tests/ and rl/, set in pyproject.toml)
pytest

# Run a single test file / test
pytest tests/test_init.py -v
pytest tests/test_collision_scenario.py::TestClassName::test_name -v
pytest rl/single_agent/tests/test_single_agent_env.py -v

# Run the simulator end-to-end against sample_config.yaml
python -c "
from urbannav.uam_simulator import UAMSimulator
sim = UAMSimulator(config_path='sample_config.yaml')
sim.reset()
for _ in range(sim.total_timestep):
    sim.step({})
sim.render()
"

# Quick manual runners (not pytest — see their module docstrings for flags)
python deployment.py                  # mission/flight sim smoke test + render
python src/urbannav/deployment_vp_design.py   # vertiport-design sim variant
python benchmarks/deployment_regression_analysis.py   # step-time/memory regression sweep
```

`pre-commit` is configured (`.pre-commit-config.yaml`): `black` (line-length 100), `isort`
(black profile), `flake8` (`--max-line-length=100 --extend-ignore=E203,W503`). Run
`pre-commit run --all-files` before committing if hooks aren't installed locally.

The `tests/` suite is layered and tests are meant to be read in order: `test_init.py` (config/build)
→ `test_step.py` → `test_collision_scenario.py` / `test_collision_performance.py` →
`test_integration.py`. RL-specific tests live alongside the RL code at
`rl/single_agent/tests/test_single_agent_env.py`, not under `tests/`. The shared `sim` fixture in
`tests/conftest.py` is module-scoped specifically to avoid re-fetching OSM map data per test —
don't change that scope without considering test runtime. `conftest.py` also exposes a
`three_uav_rig` fixture that builds 3 UAVs directly with scripted positions, bypassing
Airspace/ATC/OSM, to deterministically exercise the detect → NMAC → collision pipeline.

`Python-RVO2/` is a vendored third-party C++/Cython library (ORCA collision avoidance) with its own
CMake/setup.py build; it is not part of the `urbannav` package and is wired in only as a future
`ORCA` dynamics/controller backend (see `VALID_DYNAMICS`/`VALID_CONTROLLERS` in `component_schema.py`).

## Repo layout

```
src/urbannav/   — the installable `urbannav` package: simulator core, all pluggable components
rl/             — the installable `rl` package: Gymnasium/PettingZoo envs + training scripts,
                  grouped by rl/common (shared obs-space defs + per-agent obs/action/reward/
                  termination logic in agent_logic.py), rl/single_agent, rl/multi_agent,
                  rl/surrogate (stub), rl/vertiport_design
benchmarks/     — standalone perf/regression scripts, not part of either package, not under pytest
tests/          — pytest suite for the core simulator (see testpaths above)
sample_config.yaml, environment_ubuntu.yml, pyproject.toml — root-level, drive both packages
```

Both `src/urbannav` and `rl` are declared in `pyproject.toml`
(`[tool.setuptools.packages.find]`, `include = ["urbannav*", "rl*"]`) and installed together by
the single `pip install -e .`.

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

All of the above live in `src/urbannav/`. `UAMSimulator.reset()`/`.step()` is the public entry
point used by both plain scripts and the gym wrapper. `SimulatorManager.reset()` builds
Airspace → ATC → the four Engine objects → UAV fleet (in that order — each step depends on the
previous), then calls each engine's `register_uav_*()` to wire UAVs into per-component maps.

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
`HEAVY`, `SINGLE_AGENT_LEARNING`, `MULTI_AGENT_LEARNING`, `ORCA`). The yaml only assigns `fleet_composition` entries (`type_name`,
`count`, `dynamics`, `controller`, `sensor`, `planner`) which `build_fleet_blueprint()` expands into
one `UAVBlueprint` per UAV instance, consumed by `ATC.create_uavS_from_blueprint()`.

`ATC` maintains four `{component_name(str): [uav_id, ...]}` maps (`dynamics_map`, `controller_map`,
`planner_map`, `sensor_map`) built at fleet-creation time; each Engine's `register_uav_*()` reads
its map and instantiates one component object per *type actually present* in the current fleet
(not one per UAV), then keys it by every `uav_id` that uses it.

### RL integration

The RL code is a separate installable package (`rl/`) that depends on `urbannav` — it is not
inside `src/urbannav`. Two reserved `type_name`s exist (`RESERVED_TYPE_SINGLE_AGENT_LEARNING` /
`RESERVED_TYPE_MULTI_AGENT_LEARNING` in `component_schema.py`; `RESERVED_TYPE_LEARNING` is kept
as a backward-compat alias for the former): `SINGLE_AGENT_LEARNING` allows at most one fleet
entry (enforced by `validate_fleet_composition()`, which IS wired into `UAMConfig.load_from_yaml()`)
and `mode=TRAIN` forces `count==1` (one agent being trained; `mode=TEST` allows deploying the same
policy across `count>1` UAVs). `MULTI_AGENT_LEARNING` allows any number of fleet entries; every
entry of either LEARNING type requires a `policy_id` (forbidden on non-LEARNING entries), and all
entries sharing one `policy_id` must use identical dynamics/controller/sensor/planner.
`controller: RL` (not `None`) is the actual marker AerBus checks (`AerBus.RL_CONTROLLER_NAMES`,
`{'RL'}`) to skip internal action generation for a UAV — this applies identically regardless of
type_name, since routing is keyed off controller name. `AerBus.get_rl_policy_uav_map()` and
`SimulatorManager.get_multi_agent_uav_ids()` both expose `{policy_id: [uav_id, ...]}` groupings
(the UAV's `policy_id` attribute is set by `ATC.create_uav_from_blueprint()` from
`UAVBlueprint.policy_id`).

Per-agent obs/action/reward/termination logic lives in `rl/common/agent_logic.py` (free functions
keyed by `uav_id`, not methods), shared by both:
- `rl/single_agent/single_agent_gym_env.py::UAMSimEnv` (Gymnasium `Env`) — finds the
  `SINGLE_AGENT_LEARNING` UAV id via `SimulatorManager.get_learning_uav_id()`.
- `rl/multi_agent/multi_agent_gym_env.py::UAMMultiAgentEnv` (PettingZoo `ParallelEnv`, verified
  against `pettingzoo.test.parallel_api_test`) — one agent per `MULTI_AGENT_LEARNING` UAV, keyed
  by the UAV's *blueprint* string id (stable across resets), not its runtime int `uav_id` (which
  ATC reassigns each `reset()` and which the env maps internally).

Both wrap a single `UAVCommandBundle` step: `SimulatorManager.map_actions_to_uavs()` merges
external (gym/pettingzoo-supplied) actions with the internally-generated ones before dispatch to
`DynamicsEngine`. Observation space definitions live in `rl/common/obs_space_definitions.py`
(`OBS_SPACE` registry). `rl/surrogate/` is currently an empty stub package (just `__init__.py`).

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
  existing vertiports — the primary, actively-developed path. Smoke-tested via `deployment.py`.
- **Vertiport design simulation** (`uam_simulator_vp_design.py`, `simulator_manager_vp_design.py`,
  `deployment_vp_design.py` in `src/urbannav/`; `rl/vertiport_design/vertiport_design_env.py` for
  the Gym wrapper): a separate, parallel pipeline for the vertiport *placement* problem (where to
  put vertiports given OD demand). OD-matrix input is passed at runtime via `--od-matrix` (see
  `deployment_vp_design.py`'s docstring) rather than baked into a repo data directory. These are
  independent entry points, not alternate code paths through the same classes — don't assume
  changes to `simulator_manager.py` propagate to `simulator_manager_vp_design.py` or vice versa.
  `rl/vertiport_design/vertiport_design_env.py` is unfinished (incomplete `spaces.` assignments) —
  check before assuming it runs.

## Conventions specific to this codebase

- `urbannav` and `rl` are real installed packages (`pip install -e .`, layout in `pyproject.toml`)
  — import with `from urbannav.atc import ATC` / `from rl.common.obs_space_definitions import
  OBS_SPACE`, not bare module names. There is no `sys.path` manipulation anymore; if you find code
  still doing that, it predates the `src/` restructure and should be fixed rather than emulated.
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
- Code style is enforced by `pre-commit` (black/isort/flake8, 100-char lines) — match that
  formatting in new code even if you don't run the hooks locally.

- All functions should contain docstring, for simulator related functions docstring should explain the purpose and usage. For learning (ML, DL, RL) related functions docstrings should define purpose of function, how its used in context of models, their input and outputs, the transformation performed by the function. If function is implementing a model then should include the papers that were sources/inspiration for the model, with link link to paper. For models specifically, this should be right after class declaration. For mathematical functions invoked in model, each function should have explanation of the math and its operation, its need and how it helps the model. 