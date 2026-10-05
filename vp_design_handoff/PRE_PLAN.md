# UrbanNav: Vertiport Design (VP design) Pre-Plan

**What this document is.** A *pre-plan*. Each section (S00 to S20) describes one
self-contained change to UrbanNav: why it is needed, which files it touches,
a code sketch, the risks we already know about, and the tests. The agent turns each
section into a *detailed plan* by doing its own research first, then implements
it. This catches gaps the pre-plan overlooked before any code changes.

**Where it lives.** Copy this file to `docs/vp_design/PRE_PLAN.md`. Detailed plans go to
`docs/vp_design/detailed_plans/SXX_<short_name>.md`.

---

## 0. How the agent uses this document

### 0.1 Workflow per section (strictly sequential: one section at a time)

| Phase | What the agent does | Ends with |
|---|---|---|
| **A: Research and detailed plan** | Read the section (primary context). Then search the repository yourself (secondary context): every caller of every function you will touch, every config key, every test that exercises the code, every subclass (testbed!). List gaps or conflicts the section missed, especially small ones that would break code (renamed attributes, changed tuple shapes, units, import cycles, fixtures). Read the existing tests for the touched code and decide whether they are sufficient. Write `docs/vp_design/detailed_plans/SXX_<name>.md` with exact edits (file, function, resolved line numbers, before/after), the full test list (existing and new), and open questions. | **STOP.** The user approves or edits the detailed plan. |
| **B: Implement** | Apply exactly the approved detailed plan. No scope creep. If something unexpected appears, stop and ask. | Code changed. |
| **C: Test** | Run the section's existing tests and the new tests (see the test type). Report pass/fail with a short log. | Test report. |
| **D: Commit** | One commit with the section's prescribed message. Report what changed (files, resolved line numbers). | **STOP.** Wait for the user to say "next". |

### 0.2 Rules for every section

* **Comments.** Every metric or variable line gets a one-line comment: *what it is* plus *source*,
  and its **unit** in brackets, e.g. `# passenger wait, enqueue->depart (trip log) [min]`.
  Values that are assumed because data is missing carry `ASSUMPTION:` in the comment and live
  in `urbannav/vp_design/assumptions.py`.
* **Units.** Simulator internals use seconds [s] and metres [m]. The demand side uses
  minutes [min]. λ (lambda) is in trips per minute [trips/min]. Convert only at documented
  boundary functions, e.g. `air_legs_from_trip_log(dt_s=...)`.
* **Abbreviations.** Use the glossary (Section 2). The first use of an abbreviation in a new
  docstring is spelled out.
* **Repo conventions** (`CLAUDE.md`): conda env `AAM_AMOD`, `pip install -e .`,
  `black` (line length 100), `isort` (black profile), `flake8`. Run
  `pre-commit run --all-files` before committing. The `tests/` layering
  (`test_init` → `test_step` → collision → integration) and the module-scoped `sim` fixture
  stay as they are.
* **Back-compatibility.** Renamed public methods keep a thin deprecated alias
  (`warnings.warn(..., DeprecationWarning)`) unless Phase A shows no caller remains.
  The testbed (`testbed/`) must keep working without edits unless a section says otherwise.
* **Test type** for every section: **SIM** (simulator tests under `tests/`, plus the RL tests
  that run the simulator standalone), **VP-ENV** (vertiport-design tests, new folder
  `tests/vp_design/` or `rl/vertiport_design/tests/`; Phase A decides), or both.

### 0.3 Commit message format

`<type>(<scope>/SXX): <summary>`
* type: `feat` | `fix` | `refactor` | `docs` | `test`
* scope: `sim` | `vp_env` | `vp_design` | `config` | `docs`

Example: `fix(sim/S03): count NMAC encounters in SensorEngine`

### 0.4 Data gate

Sections S00 to S05 need no GMNS data. **Before S06 Phase C, STOP** until the user has
placed the GMNS Plus data in `data/gmns_plus/<city>/` (Austin: `node.csv`, `link.csv`,
`demand.csv`). From S06 on, tests use the real Austin data (199 zones, 10,717 nodes),
plus the small synthetic fixture where a test must be fast or deterministic.

### 0.5 Hand-off bundle (files delivered with this plan)

| Delivered file | Destination in repo | Section |
|---|---|---|
| `PRE_PLAN.md` | `docs/vp_design/PRE_PLAN.md` | S00 |
| `src/urbannav/vp_design/__init__.py` | same path | S06 |
| `src/urbannav/vp_design/assumptions.py` | same path | S06 |
| `src/urbannav/vp_design/gmns_io.py` | same path | S06 |
| `src/urbannav/vp_design/zones.py` | same path | S07, S08 |
| `src/urbannav/vp_design/regions.py` | same path | S09 |
| `src/urbannav/vp_design/door_to_door.py` | same path | S13 |
| `src/urbannav/vp_design/mode_choice.py` | same path | S14 |
| `src/urbannav/vp_design/metrics.py` | same path | S13 |
| `src/urbannav/vp_design/normalization.py` | same path | S15 |
| `src/urbannav/vp_design/phase2_structures.py` | same path (parked) | S19 |
| `scripts/vp_design/make_vp_design_config.py` | same path | S11 |
| `scripts/vp_design/plot_zone_map.py` | same path | S11 |
| `examples/regions_example.txt` | `docs/vp_design/regions_example.txt` | S09 |

These files were validated on a synthetic GMNS city (36 zones, 900 nodes). Austin
validation happens in the sections. They **supersede** the earlier chat scripts
`gmns_metrics.py`, `example_pipeline.py` and `zone_vertiport_design_env.py`; do not add those.
The user's `Austin_data_view.ipynb` and `Austin_Node_Map.py` are reference material only.
Their Voronoi logic is re-implemented in `zones.py`, their building-per-cell logic in
`phase2_structures.py`, and their plot in `plot_zone_map.py`.

---

## 1. Problem definition (settled; overrides anything earlier)

### 1.1 Objects

| Object | Meaning in UrbanNav | Source |
|---|---|---|
| **Airspace** | Where UAVs fly: OSMnx place polygon (`location_name`, e.g. "Austin, Texas, USA") projected to UTM [m], plus restricted areas (OSM features buffered by `buffer_radius`). In VP design with GMNS, the boundary is **extended** to cover all zones (S08). | OSMnx |
| **Zone** | A GMNS Plus traffic analysis zone (TAZ). Austin has **199 zones**; their centroid nodes are `node_id` 1 to 199. | GMNS Plus `node.csv` |
| **Zone cell** | The Voronoi cell around a zone centroid: all points closer to it than to any other centroid. Geometry only (regions, plots, Phase 2 building candidates). | computed (`zones.py`) |
| **Region** | A group of zone cells. Built by **k-means** on zone centroids (user picks k, e.g. 5 or 50) **or** read from a **region file** written by a person. Regions are numbered 0..R-1. | `regions.py` |
| **Vertiport (Phase 1)** | **Zone = vertiport candidate.** The vertiport sits at the zone centroid. The agent picks **one zone per region**; the picked zones are the active vertiports. | design decision |
| **Vertiport (Phase 2)** | A structure (building, office, ...) inside a zone cell (S19). | future |
| **Demand** | `demand.csv`: zone-to-zone trip volume. Summed to regions it becomes λ[r,s], the trip arrival rate per region pair [trips/min], which the simulator uses. The zone-level table is kept for access/egress weighting and mode choice. | GMNS Plus |

**Correction of earlier chat content.** The first approach (OSM structures, e.g.
`('building','commercial')`, as vertiport candidates grouped into k-means regions) was a
**prototype without data**. It is not deleted. It stays in `airspace.py` as Family B2 (S01)
because it documents what data VP design needs. "Zone" in this plan always means the GMNS zone.

### 1.2 Three families of vertiport creation (S01)

| Family | Used for | Regions? | ID scheme |
|---|---|---|---|
| **A: Standalone simulator: random vertiports** | Running the simulator by itself: `deployment.py`, single-agent and multi-agent RL controller training | no | `0..N-1` in creation order |
| **B: VP design without a data source** | Developing and testing the VP design environment with no city data: **B1 synthetic** geometric patterns (no network I/O); **B2 OSM structures + k-means** (prototype) | yes | `region*10_000 + index_in_region` |
| **C: VP design with a data source: GMNS Plus zones** | The real VP design problem | yes (k-means or region file) | `vp_id = zone_id` |

### 1.3 Door-to-door time (what the design is judged on)

For a trip from zone i (region r) to zone j (region s), with vertiports at the selected zones kᵣ and kₛ:

```
T_ij = t(i,kᵣ) + τ_o + W_rs + F_rs + H_rs + τ_d + t(kₛ,j)     [min]
       access    terminal  wait  flight  hold  terminal  egress
```

* `t(·,·)`: GMNS zone-to-zone car travel time [min]
* `τ_o`, `τ_d`: terminal times (ASSUMPTION)
* `W`, `F`, `H`: from the simulator's trip log

Averaged over all trips from r to s this splits exactly into
`ACC[kᵣ,s] + air part + EGR[kₛ,r]`. The ACC/EGR tables are precomputed once per city
(`door_to_door.py`). The zone-level OD table is what makes access/egress depend on *which*
zone is chosen; the region-level table cannot.

---

## 2. Glossary (S00 copies this to `docs/vp_design/GLOSSARY.md`)

| Term | Meaning |
|---|---|
| AAM / UAM | Advanced / Urban Air Mobility: passenger or cargo flights inside cities. |
| UAV | Unmanned aerial vehicle; in UrbanNav, every simulated aircraft. |
| eVTOL | Electric vertical take-off and landing aircraft (air taxi); the kind of vehicle UAM assumes. |
| Vertiport / pad | Take-off and landing site / one landing slot at it (`number_of_landing_pad`). |
| GMNS / GMNS Plus | General Modeling Network Specification: a standard CSV format for road networks plus zones and demand. |
| OSM / OSMnx | OpenStreetMap / the Python library used to download its boundaries, roads and buildings. |
| TAZ (zone) | Traffic analysis zone: the unit transport planners use to count trips. |
| Zone centroid | The representative node of a zone (Austin: node_id 1..199). |
| Centroid connector | A virtual link joining a centroid to the road network (Austin: length 0, capacity 99,999). |
| Voronoi tessellation / zone cell | Partition of the map into cells, each containing points closest to one centroid. |
| Region | A group of zone cells; the agent picks one vertiport per region. |
| k-means, k | Clustering algorithm grouping points into k clusters; k = number of regions. |
| OD | Origin-destination: a trip's start and end zone (or region). |
| OD matrix / table | Trips per origin-destination pair. |
| volume (demand.csv) | Trips per OD pair per demand period (period length is an ASSUMPTION, default 60 min). |
| λ (lambda) | Trip arrival rate per region pair [trips/min], the rate of the Poisson process the demand model draws from. |
| Poisson process / Poisson draw | Standard model for independent random requests; the number of arrivals in Δt is Poisson(λ·Δt). |
| Bernoulli draw (coin flip) | Old demand draw: at most one trip per step with probability λ·Δt; wrong when λ·Δt is near or above 1. |
| dt, step | Simulator time step [s] and one tick of the simulator. |
| Design step / inner sim | One RL/MCTS decision (a vertiport selection) / the simulator run that evaluates it. |
| Skim / travel-time table | Zone × zone table of shortest-path time [min], distance [m], toll [USD]. |
| Intrazonal | Within one zone; intrazonal time = factor × time to the nearest other zone (the table's diagonal is 0). |
| Dijkstra | Shortest-path algorithm used for the tables. |
| BPR | Bureau of Public Roads volume-delay function: congested time = t0·(1+α(v/c)^β). |
| v/c | Volume-to-capacity ratio of a road link; 1 = at capacity. |
| PLF | Peak load factor: converts period volume to hourly flow. |
| FFTT | Free-flow travel time [min]. |
| IVT / OVT | In-vehicle / out-of-vehicle time. Out-of-vehicle (walk, wait, access) minutes are weighted more heavily. |
| Access / egress | Ground leg to the origin vertiport / from the destination vertiport. |
| Terminal time | Check-in/boarding (origin) or exit (destination) time at a vertiport. |
| Passenger wait | Time from request (enqueue) to departure. |
| Landing hold | Time from arriving at the destination airspace to landing (pad queue). |
| Door-to-door time | Total trip time from origin zone to destination zone. |
| GT / GC | Generalized time / cost: travel time with weights plus money converted to time (or the reverse). |
| VOT | Value of time [USD/h]: converts money to minutes. |
| VMT | Vehicle miles traveled [vehicle-miles]. |
| Logit (binary) | Choice model: P(UAM) = 1/(1+exp(β·(GT_uam + bias − GT_car))). |
| β (beta) | Logit scale [1/min]: how sharply travellers react to time differences. |
| ASC / UAM bias | Alternative-specific constant: fixed preference against UAM, expressed here in minutes. |
| Logsum / consumer surplus | Standard appraisal measure of traveller benefit from adding an option [min/trip or USD]. |
| Captured share / mode share | Fraction of eligible trips choosing UAM. |
| SP survey | Stated-preference survey: hypothetical-choice questionnaire used to estimate β, ASC, weights. |
| Coverage share | Activity-weighted share of zones within a threshold access time of their vertiport. |
| Gini | Inequality index (0 = equal, 1 = maximally unequal); here of access times. |
| p90 | 90th percentile. |
| p-median / p-center / MCLP | Facility-location objectives: minimize average distance / worst distance / maximize demand covered. |
| TCQSM / TCRP / HCM | Transit Capacity and Quality of Service Manual / Transit Cooperative Research Program / Highway Capacity Manual (US standards). |
| NMAC | Near mid-air collision: two UAVs closer than `nmac_radius`. |
| NMAC pair-step vs encounter | Pair-step: one pair in NMAC for one step (current count). Encounter: one NMAC *event*, counted once when the pair first enters range. |
| Flight hour | One UAV flying for one hour; rate denominator for safety metrics. |
| RA | Restricted area: OSM features UAVs must avoid, buffered by `buffer_radius` [m]. |
| Utilization | Busy time / available time (pads or fleet) [-]. |
| Little's law | Average number busy = arrival rate × time each is busy; used for fleet sizing. |
| CRS / UTM / EPSG | Coordinate reference system / Universal Transverse Mercator (metric projection) / registry codes (EPSG:4326 = lon/lat). |
| GNN / PPO / SB3 | Graph neural network / Proximal Policy Optimization / Stable-Baselines3. |
| MCTS / UCT / rollout | Monte Carlo tree search / Upper Confidence bounds for Trees (selection rule) / cheap simulated continuation used to value a node. |
| Surrogate | Learned model that predicts episode outcomes instead of running the simulator. |
| ATC / AerBus / ORCA / PID | Air traffic control module / controller bus / Optimal Reciprocal Collision Avoidance / proportional-integral-derivative controller. |

---

## 3. Repository findings (referenced by sections as F#)

| ID | Finding | Location |
|---|---|---|
| F1 | VP design environment never enables the demand model: it constructs `UAMSimulator(config_path=...)` without `od_matrix_path`/`zone_region_map_path`, so missions stay random. | `rl/vertiport_design/vertiport_design_env.py` `__init__` |
| F2 | `get_episode_metrics()`: `pair_avg_wait_time` is computed as `land_step − arrive_airspace_step` (that is the **landing hold**). Passenger wait (`depart_step − enqueue_step`) is not reported. Values are in seconds. | `src/urbannav/demand_model.py` |
| F3 | Pad capacity has three sources that disagree: `Vertiport.landing_takeoff_capacity = 4` (hard-coded), config `vertiport.number_of_landing_pad: 3`, and utilization reads `config.airspace.pad_capacity` (absent, so the default 1 is used). | `vertiport.py`, `sample_config.yaml`, `demand_model.py` |
| F4 | Demand draw is a Bernoulli coin flip, p = λ·dt/60: at most one trip per region pair per step; silently loses trips when λ·dt/60 ≥ 1. | `demand_model.py` |
| F5 | GMNS demand far exceeds the fleet (synthetic test: Little's law suggests thousands of UAVs vs 5 to 11 configured). Fix: config pre-script + `demand_scale` + mode choice. | config |
| F6 | NMAC gaps G1 to G4 and findings A5 to A8 (S03). | sensor / metrics |
| F7 | `uam_sound.py` is a stub (noise model `NotImplementedError`, imports a missing template). | `src/urbannav/uam_sound.py` |
| F8 | Vertiport height: random family sets `z = random.randint(1500, 3500)` (comment calls it flight altitude, but it is the vertiport height [m]); synthetic family creates 2D points; testbed has `altitude_range = (1500, 3500)`. | `airspace.py` `add_n_random_vps_to_vplist`; `testbed/config_schema.py` |
| F9 | `Vertiport.id = id(self)`: changes every run. `metrics_collector.py` keys edges by `id(vp)`. TODO "create a dict that maps vp_id to vp". | `vertiport.py`, `metrics_collector.py`, `airspace.py` |
| F10 | VP design environment bootstrap does a full reset (random vertiports via Family A), then adds region candidates to the same `vertiport_list`, which is capped by `max_num_vps_airspace = config.airspace.number_of_vertiports`. Families are mixed in one list. | `vertiport_design_env.py`, `airspace.make_regions_dict` |
| F11 | `make_regions_dict` / `add_vps_from_regions_to_vplist` guard `try: assert hasattr(...) except: AttributeError(...)` creates the exception but never raises it; `polygon_dict` always exists anyway. | `airspace.py` |
| F12 | Existing k-means clusters OSM structure polygons, not zones. | `airspace.assign_region_to_vertiports` |
| F13 | `create_vertiport_from_lat_long` raises `NotImplementedError`; `remove_vp` is a stub. | `airspace.py` |
| F14 | `sample_points(num, rng=self.seed)` TODO warning; `add_n_random_vps_to_vplist` reseeds the **global** `random` and `np.random` (side effects on other modules). | `airspace.py` |
| F15 | GraphBuilder node features are `[is_selected, x, y]` only. | `rl/vertiport_design/graph_builder.py` |
| F16 | No MCTS code; `rl/surrogate` has a `predict_episode_outcome` stub described as the value estimator for an MCTS/RL outer loop. | `rl/surrogate/` |
| F17 | Testbed subclasses `SimulatorManager` (`_init_airspace`, `_build_vertiports_random`) and mirrors the Airspace attribute surface. | `testbed/` |
| F18 | `rl/conftest.py` builds test configs from `sample_config.yaml` sections; schema changes must keep these fixtures valid. | `rl/conftest.py` |
| F19 | `src/urbannav/deployment_vp_design.py` is a VP-design runner; must follow environment changes. | as named |
| F20 | UAV registry: STANDARD `max_speed` 10 m/s (≈36 km/h), far below eVTOL cruise. | `component_schema.py` `UAV_TYPE_REGISTRY` |
| F21 | Austin GMNS: some centroids outside the OSMnx city boundary or far from roads; connectors have length 0; `obs_volume` all 0; some zones have no OSM buildings (Phase 2). | `Austin_data_view.ipynb` |
| F22 | Training config uses `simulator_step=2`: a smoke-test value only (user). Realistic values are set in S18. | training script |

---

## 4. Section index

| ID | Title | Test type | Depends on | Data |
|---|---|---|---|---|
| S00 | Conventions, glossary, pre-plan protocol (docs only) | none | none | no |
| S01 | Airspace cleanup: three vertiport families, stable IDs, vertiport height | SIM | S00 | no |
| S02 | Config restructure (standalone vs VP design, YES/NO) and build dispatch | SIM + VP-ENV | S01 | no |
| S03 | NMAC in the simulator (sensor-local fix) | SIM | S01 | no |
| S04 | Simulator demand and operator-metric fixes (Poisson, trip-log legs, pad capacity, λ update) | SIM | S02 | no |
| S05 | UAV speeds for all registry types | SIM | S03 | no |
| **gate** | **STOP: user adds `data/gmns_plus/<city>/`** | | | |
| S06 | `urbannav.vp_design` package, GMNS loader, road network tables | VP-ENV | S05 | yes |
| S07 | Zones: centroids → airspace CRS → Voronoi cells | VP-ENV | S06 | yes |
| S08 | Zone–airspace reconciliation: extended airspace, restricted-area report | SIM + VP-ENV | S07 | yes |
| S09 | Regions: k-means or region file; zone→region map; region λ | VP-ENV | S08 | yes |
| S10 | Family C (GMNS zones) in Airspace and VertiportDesignEnv `__init__` wiring | SIM + VP-ENV | S09 | yes |
| S11 | Config pre-script and zone map plot | VP-ENV | S10 | yes |
| S12 | NMAC in the VP design environment | VP-ENV | S03, S10 | yes |
| S13 | Door-to-door time and metrics by stakeholder (placement table) | VP-ENV | S04, S12 | yes |
| S14 | Demand responds to placement (mode choice + λ loop) | VP-ENV | S13 | yes |
| S15 | Raw metrics → generalization block → reward (GNN-RL; shared with MCTS) | VP-ENV | S14 | yes |
| S16 | VP design environment integration and graph features | VP-ENV | S15 | yes |
| S17 | MCTS (deferred; started by the user) | VP-ENV | S15 | yes |
| S18 | End-to-end campaign on real data | SIM + VP-ENV | S16 | yes |
| S19 | Phase 2 hooks: structures inside cells (docs + parked code) | none | S07 | no |
| S20 | Future: cruise-altitude band | SIM | S01 | no |

---

# Sections

Each section uses the same headings: **Goal · Background · Files · Proposed changes ·
Verify in Phase A · Tests · Acceptance · Commit · STOP.**

---

## S00: Conventions, glossary, pre-plan protocol (docs only)

**Test type:** none · **Depends on:** none

**Goal.** Put the shared rules in the repo so every later section can link to them.

**Background.** You (the user) work in applied RL and control, not transportation, so every
transport term needs a plain explanation next to the code. The pre-plan workflow (Section 0)
must be discoverable by any agent working in the repo.

**Files.**
* Add: `docs/vp_design/PRE_PLAN.md` (this file), `docs/vp_design/GLOSSARY.md` (Section 2),
  `docs/vp_design/CONVENTIONS.md` (Section 0.2 and 0.3), `docs/vp_design/detailed_plans/.gitkeep`.
* Modify: `CLAUDE.md`: add a short "Vertiport design work" paragraph pointing to these docs
  and stating the one-section-at-a-time protocol.

**Proposed changes.** Copy text; no code.

**Verify in Phase A.** Existing `docs/` layout (create the folder if absent). Avoid
duplicating anything already in `CLAUDE.md`.

**Tests.** None. Check that markdown renders.

**Acceptance.** Files exist; `CLAUDE.md` links to them.

**Commit.** `docs(docs/S00): VP design pre-plan, glossary, conventions`

**STOP.**

---

## S01: Airspace cleanup: three vertiport families, stable IDs, vertiport height

**Test type:** SIM · **Depends on:** S00

### Goal
Reorganize `airspace.py` so a reader immediately sees **which method is for what**:
1. **Family A, standalone simulator:** random vertiports, no regions.
2. **Family B, VP design without a data source:** B1 synthetic patterns; B2 OSM structures + k-means.
3. **Family C, VP design with GMNS Plus:** placeholder here, implemented in S10.

At the same time: give vertiports **stable IDs** chosen per family, and set vertiport
**height** to a seeded random value in **20 to 100 m**. The standalone simulator, single-agent
training and multi-agent environments must keep working exactly as before.

### Background
* F8, F9, F10, F11, F12, F13, F14, F17.
* Today Family A (`add_n_random_vps_to_vplist`), B1 (`make_regions_dict_synthetic`) and B2
  (`make_regions_dict`, `add_vps_from_regions_to_vplist`, `assign_region_to_vertiports`,
  `assign_vertiports_to_regions`, `_sample_vertiport_from_region`) are interleaved with
  utilities, so it is unclear which methods belong to the standalone simulator and which
  to VP design.
* Callers to keep working: `SimulatorManager._build_vertiports_random`
  (standalone), `VertiportDesignEnv.__init__` (B1/B2), `testbed` (own airspace),
  `deployment.py`, `src/urbannav/deployment_vp_design.py`,
  `rl/single_agent/*`, `rl/multi_agent/*`, `tests/*`, `rl/**/tests/*`, `benchmarks/*`.
* Vertiport z [m] is used by the 3D planners as **start and end altitude**. The
  1500 to 3500 m values were vertiport heights. New range 20 to 100 m (rooftop scale).
  **Consequence:** UAVs now fly between 20 and 100 m (no high band), so 3D NMAC rates will rise
  compared with before. S20 restores a band with cruise altitudes. Building-height data is not
  in OSMnx; a building-height database is future work, so heights stay random for now.

### Files
* Modify: `src/urbannav/airspace.py` (structure, renames + aliases, IDs, heights),
  `src/urbannav/vertiport.py` (`vp_id`, `source`, `zone_id` fields).
* Read for context: `simulator_manager.py`, `uam_simulator.py`, `atc.py`,
  `deployment.py`, `src/urbannav/deployment_vp_design.py`,
  `rl/single_agent/single_agent_gym_env.py`, `rl/single_agent/single_agent_training.py`
  (and the RL training script name Phase A finds), `rl/multi_agent/multi_agent_gym_env.py`,
  `rl/vertiport_design/*`, `testbed/testbed_airspace.py`,
  `testbed/testbed_simulator_manager.py`, `metrics_collector.py`, `tests/conftest.py`.

### Proposed changes

**(a) Layout of `airspace.py`.** Banner comments, in this order:

```python
# =============================================================================
# 0. CONSTRUCTION - boundary + restricted areas (shared by ALL families)
# =============================================================================
# 1. SHARED VERTIPORT UTILITIES - add/get/lookup, id assignment, height sampling
# =============================================================================
# 2. FAMILY A - STANDALONE SIMULATOR: random vertiports, no regions
#    used by SimulatorManager._build_vertiports_random -> deployment.py,
#    single-agent / multi-agent RL controller training
# =============================================================================
# 3. FAMILY B - VP DESIGN WITHOUT A DATA SOURCE (regions from geometry)
#    B1 synthetic patterns  - no network I/O, CI and quick experiments
#    B2 OSM structures + k-means - prototype that documents what data VP design needs
# =============================================================================
# 4. FAMILY C - VP DESIGN WITH A DATA SOURCE: GMNS Plus zones (implemented in S10)
# =============================================================================
# 5. VP DESIGN SELECTION API - regions_dict access, set the selected vertiports
# =============================================================================
# 6. NOT IMPLEMENTED / STUBS - kept with explicit NotImplementedError + reason
# =============================================================================
```

**(b) Renames.** Old names stay as deprecated aliases.

| Old | New | Family |
|---|---|---|
| `add_n_random_vps_to_vplist(n)` | `build_random_vertiports(n)` | A |
| `make_regions_dict_synthetic(...)` | `build_candidate_regions_synthetic(...)` | B1 |
| `make_regions_dict(tag_str, num_regions)` | `build_candidate_regions_osm_structures(tag_str, num_regions)` | B2 |
| `add_vps_from_regions_to_vplist(...)` | `sample_vertiports_from_osm_structure_regions(...)` | B2 |
| `assign_region_to_vertiports` | `_kmeans_assign_regions` | B2 internal |
| `assign_vertiports_to_regions` | `_group_vertiports_by_region` | shared internal |
| `set_vertiport_list_vp_design(list)` | `set_selected_vertiports(list)` | selection API |
| `max_num_vps_airspace` | `max_vertiports` (property alias kept) | A |

Fix F11 (raise properly or delete the meaningless guard). Fix F14: use a local
`np.random.default_rng(seed)`; do not reseed global `random` / `np.random`. If Phase A finds
another module relies on that global reseeding, report it rather than silently changing
behaviour. The meaning of `seed` must stay the same so Family A positions remain
reproducible with the same seed. Phase A decides whether exact same positions are required;
if so, keep `sample_points(..., rng=seed)` and change only z.

**(c) Stable IDs (`vertiport.py`).**

```python
class Vertiport:
    def __init__(self, location: Point, vp_id: int | None = None,
                 source: str = "unspecified", zone_id: int | None = None,
                 region: int | None = None) -> None:
        # stable vertiport id [-]: assigned by the Airspace family that creates it
        #   Family A (random):        0..N-1 in creation order
        #   Family B (synthetic/OSM): region * 10_000 + index_in_region
        #   Family C (GMNS zones):    zone_id
        # None -> legacy fallback id(self) (tests/conftest.py, testbed create Vertiport directly)
        self.id = vp_id if vp_id is not None else id(self)
        self.source = source            # 'random' | 'synthetic' | 'osm_structures' | 'gmns_zones'
        self.zone_id = zone_id          # GMNS zone id (Family C only) [-]
        self.region = region            # region id 0..R-1 (Families B, C) [-]
        ...
```

In `Airspace`:
* `self.vertiport_source: str | None`: set by the first family that builds vertiports.
  A second, different family raises `ValueError`. This prevents mixed ID namespaces (F10).
* `self.vertiport_by_id: dict[int, Vertiport]`: resolves the TODO. Filled on every add.
  Duplicate IDs raise.
* `get_vertiport_by_id(vp_id)`.

**(d) Height.**

```python
# vertiport height range (z of take-off/landing point) [m]; ASSUMPTION until a
# building-height database is linked to OSM polygons (future work)
self.vertiport_height_range_m = vertiport_height_range_m      # e.g. (20, 100); None in 2D
self._height_rng = np.random.default_rng(seed)                # local, seeded

def _sample_vertiport_height_m(self) -> float:
    lo, hi = self.vertiport_height_range_m
    z_m = float(self._height_rng.integers(lo, hi + 1))         # vertiport height [m]
    return z_m

def _make_vertiport_point(self, x: float, y: float) -> Point:
    if self.vertiport_height_range_m is None:                  # 2D mode
        return Point(x, y)
    z_m = self._sample_vertiport_height_m()
    return Point(x, y, z_m)
```

Every family creates its points via `_make_vertiport_point`, including B1 synthetic,
which today makes 2D points (Phase A: check 3D planners with 2D points). The Airspace gets
`vertiport_height_range_m` from the simulator: `(20, 100)` in 3D, `None` in 2D. Before S02 it
comes from a constructor default; S02 moves it into config.

**(e) Family C placeholder.**

```python
def build_candidate_regions_gmns_zones(self, *args, **kwargs):
    """Family C - GMNS Plus zones as vertiport candidates (zone == vertiport, Phase 1)."""
    raise NotImplementedError("Implemented in PRE_PLAN S10")
```

**(f) Metrics keyed by stable id.** In `metrics_collector.py`, build
`vp_id_to_idx` from `vp.id` instead of `id(vp)`. Fallback: `vp.id` equals `id(vp)` for legacy
vertiports, so the behaviour is identical there.

### Verify in Phase A
* Every caller of every renamed method and attribute (grep the whole repo, including
  notebooks under the repo and `benchmarks/`).
* Where 3D planners read vertiport z; whether a 2D `Point` from B1 breaks 3D mode today.
* `ATC.assign_vertiports` and `get_vp_id_list()` users: does anything assume `id` is a memory address?
* Does GraphBuilder key vertiports by `id(vp)` or `vp.id`?
* Testbed: `TestbedAirspace` builds `Vertiport(...)` directly. Confirm the legacy fallback keeps
  it working. Leave testbed `altitude_range` unchanged (separate synthetic testbed) unless the
  user asks.
* Whether `max_vertiports` still matters once VP design stops putting candidates into
  `vertiport_list` (S02).

### Tests
* **Existing (must pass unchanged):** `pytest tests/` (all layers), `pytest rl/` (single-agent,
  multi-agent, vertiport design, surrogate), the testbed tests, `python deployment.py` (smoke).
* **New (SIM):** `tests/test_airspace_families.py`:
  * Family A: N vertiports, ids `0..N-1`, z in [20, 100], same ids and z for the same seed
    across two Airspace builds (use the module-scoped `sim` fixture to avoid repeated OSM fetches).
  * B1: ids `region*10_000+i`, every vertiport has `region`, z in [20, 100] in 3D.
  * Mixing families raises `ValueError`; duplicate id raises; `get_vertiport_by_id` round-trip.
  * Deprecated aliases emit `DeprecationWarning` and give identical results.
  * 2D mode: points are 2D.

### Acceptance
All existing tests green; new tests green; `deployment.py` runs; one short single-agent
training run (Phase A picks the smallest existing smoke command) starts and steps.

### Commit
`refactor(sim/S01): airspace vertiport families, stable ids, 20-100 m heights`

**STOP.**

---

## S02: Config restructure (standalone vs VP design, YES/NO) and build dispatch

**Test type:** SIM + VP-ENV · **Depends on:** S01

### Goal
Make the config say explicitly which mode it is for, and keep only the relevant keys
in each block. `SimulatorManager` builds vertiports from the right family. The VP design
environment no longer bootstraps random vertiports (F10).

### Background
`airspace` currently mixes keys needed by every mode (`location_name`, restricted-area
tags) with standalone-only keys (`number_of_vertiports`) and B2-only keys
(`vertiport_tag_list`). Decision: `vp_design.enabled` is the explicit YES/NO, and the
pre-script (S11) asks the question.

### Files
* Modify: `src/urbannav/component_schema.py` (pydantic models), `sample_config.yaml`,
  `simulator_manager.py` (`_init_airspace`, `_build_assets` dispatch),
  `rl/vertiport_design/vertiport_design_env.py` (`__init__` bootstrap),
  `src/urbannav/deployment_vp_design.py`, `rl/conftest.py` (only if fixtures break).
* Read: `testbed/config_schema.py` (reuses `UAMSimulatorConfig` etc.), `rl/conftest.py`,
  `tests/conftest.py`.

### Proposed changes

**Config schema** (YAML):

```yaml
simulator: {dt: 1.0, total_timestep: 5000, mode: '3D', seed: 123}
vertiport:
  number_of_landing_pad: 3          # pads per vertiport [-] (single source, S04)
  height_range_m: [20, 100]         # vertiport z range [m]; ignored in 2D
airspace:                           # needed by EVERY mode
  location_name: 'Austin, Texas, USA'
  airspace_restricted_area_tag_list: [['building', 'office']]
  buffer_radius_m: 500              # restricted-area buffer [m]
standalone:                         # used only when vp_design.enabled is false
  number_of_vertiports: 20          # Family A count [-]
vp_design:
  enabled: false                    # YES/NO: is this config for vertiport design?
  vertiport_source: gmns_zones      # gmns_zones (C) | synthetic (B1) | osm_structures (B2)
  synthetic: {num_regions: 4, num_vertiports_per_region: 5}
  osm_structures: {vertiport_tag_list: [['building', 'commercial']], num_regions: 4}
  gmns: {}                          # filled by the S11 pre-script (paths to artifacts)
```

Pydantic: `StandaloneConfig`, `VPDesignConfig` (+ `SyntheticRegionsConfig`,
`OSMStructuresConfig`, `GMNSConfig` with all fields Optional until S10/S11). Root validator
for **back-compat**: old `airspace.number_of_vertiports` maps to
`standalone.number_of_vertiports`, and old `airspace.vertiport_tag_list` maps to
`vp_design.osm_structures.vertiport_tag_list`. Both emit a `DeprecationWarning`.
`vp_design` absent means `enabled: false`.

**Build dispatch** in `SimulatorManager`:

```python
def _build_vertiports(self):
    """Pick the vertiport family from config (PRE_PLAN S01/S02)."""
    vpd = self.config.vp_design
    if not vpd.enabled:                                   # Family A - standalone simulator
        self.airspace.build_random_vertiports(self.config.standalone.number_of_vertiports)
        return
    if vpd.vertiport_source == "synthetic":               # Family B1
        self.airspace.build_candidate_regions_synthetic(**vpd.synthetic.model_dump())
    elif vpd.vertiport_source == "osm_structures":        # Family B2
        self.airspace.build_candidate_regions_osm_structures(**vpd.osm_structures.kwargs())
    elif vpd.vertiport_source == "gmns_zones":            # Family C (S10)
        self.airspace.build_candidate_regions_gmns_zones(**vpd.gmns.kwargs())
    # initial selection: first candidate of every region (VP design env overrides per step)
    self.airspace.set_selected_vertiports(
        [self.airspace.regions_dict[r][0] for r in sorted(self.airspace.regions_dict)])
```

Keep `_build_vertiports_random` as the name the testbed overrides (F17), and have it call
the dispatch. Phase A must confirm the testbed override still bypasses everything.
`VertiportDesignEnv.__init__`: require `vp_design.enabled == true`. If it is false, raise:
"This config is not a VP design config (vp_design.enabled: false). Run
scripts/vp_design/make_vp_design_config.py and answer YES." Remove the region-building calls
from the environment (the simulator now builds candidates). Keep `region_mode` /
`region_kwargs` as deprecated overrides mapped onto `vp_design.vertiport_source` for
back-compat.

### Verify in Phase A
* Every reader of `config.airspace.*` and `config.vertiport.*` (simulator, ATC,
  renderer, logger's `full_config` dump, testbed, RL fixtures).
* `deployment_vp_design.py` flow.
* Whether the logger serializes the full config (new blocks must serialize).

### Tests
* Existing: all of `tests/`, `rl/`, testbed; `deployment.py`.
* New (SIM): old-style config loads with deprecation warnings and runs standalone.
  New-style standalone config is identical in behaviour (same vertiport ids and positions for
  the same seed).
* New (VP-ENV): VP design env with `vertiport_source: synthetic` builds without any random
  vertiports in `vertiport_list` (only the selection). A standalone config passed to the
  environment raises the clear error.

### Acceptance
Green tests; standalone and VP design (synthetic) both start; `sample_config.yaml` uses the
new schema.

### Commit
`refactor(config/S02): standalone vs vp_design config blocks and family dispatch`

**STOP.**

---

## S03: NMAC in the simulator (sensor-local fix)

**Test type:** SIM · **Depends on:** S01

### Goal
Count **NMAC encounters** correctly inside the simulator, in the sensor code that already
detects NMACs, so that every consumer (logger, environments, VP design) gets correct numbers,
with or without logging. The VP design side is S12.

### Background: what works
The detection → NMAC → collision chain is correct:
* `sensor_partial.py`: `PartialSensor.get_uav_detection` → `get_nmac` → `get_uav_collision`
  (each a subset of the previous; spatial-hash broad phase, 3D distance narrow phase).
* `sensor_engine.py`: `SensorEngine.get_nmac()` builds `{uav_id: set(nmac partner ids)}`
  for the whole fleet.
* `SimulatorManager._step_uavS()` calls it every step and returns the 5-tuple
  `(ra_detect, uav_detect, nmac_dict, ra_collision_dict, uav_collision_dict)`.
* `UAMSimulator.step` → `Logger.log_step(collisions=...)` → `MetricsCollector.record` →
  per-step `nmac_pairs`, per-edge `nmac_count`; `_calculate_metrics` gives `total_nmac_events`.

### Background: gaps
* **G1. `uav.nmac_count` is never incremented.** Initialized to 0 in `uav_template.py`;
  `metrics_collector.py` has `#TODO: fix logic for incrementing nmac_count` and
  `#! sensor does not increment NMAC count`.
* **G2. Pair-steps, not encounters.** `total_nmac_events = sum(len(s['collisions']['nmac_pairs']) for s in self._steps)`.
  A pair inside range for 30 steps counts 30, so the value depends on `dt`. Per-edge
  `nmac_count` has the same issue.
* **G3. Logger-only.** NMAC numbers exist only in Logger/MetricsCollector. With
  `logging.enabled: false` they disappear, and they never reach `get_episode_metrics()`.
* **G4. Per-edge counts keyed by `id(vp)`.** Unstable across runs (fixed via S01 stable ids;
  region-pair attribution is S12).

### Background: further findings
* **A5. Sensor-off near vertiports (by design).** Detection, and therefore NMAC, returns
  nothing when `uav.get_sensor_operational()` is False. The user switches sensors off near
  vertiports to abstract take-off/landing crowding (many UAVs, multiple pads). **Decision:**
  keep this. Document that the NMAC metric means *in-flight NMACs among UAVs with operational
  sensors*. Phase A must confirm where sensors are switched off and on, so that all in-flight
  NMACs are counted.
* **A6. Asymmetric radii.** Each UAV uses its own `nmac_radius` (STANDARD 200 m, HEAVY larger).
  The sorted-pair de-duplication counts a pair if *either* side reports it. Document this
  rule; do not change it.
* **A7. Surrogate compatibility.** `rl/surrogate/rollout_metrics.py` and its tests reproduce
  `total_nmac_events` as pair-step sums. **Keep `total_nmac_events` unchanged** and add new
  fields alongside it.
* **A8. Frozen collided UAVs.** With `persist_collided_uavs: true`, frozen UAVs may stay in
  `uav_dict` and keep forming NMAC pairs with passers-by. Phase A checks and documents this.

### Files
* Modify: `src/urbannav/sensor_engine.py` (encounter tracking inside `SensorEngine.get_nmac`),
  `src/urbannav/simulator_manager.py` (flight-time accumulator, `get_nmac_summary()`),
  `src/urbannav/uam_simulator.py` (pass onsets to the logger),
  `src/urbannav/logger.py`, `src/urbannav/metrics_collector.py` (new fields, remove the TODO).
* Read: `sensor_partial.py`, `sensor_template.py`, `uav_template.py`,
  `rl/surrogate/rollout_metrics.py`, `tests/conftest.py`, `tests/test_collision_scenario.py`.

### Proposed changes (no new module; local to the sensor script)

```python
# sensor_engine.py - SensorEngine
def __init__(...):
    ...
    self._prev_nmac_pairs: set = set()        # NMAC pairs at previous step [set of (id_i, id_j)]
    self.nmac_onsets_this_step: list = []     # NMAC encounters starting this step [list of pairs]
    self.total_nmac_encounters: int = 0       # encounters since reset [count]

def get_nmac(self) -> Dict[int, set]:
    nmac_dict = {...}                                         # unchanged per-UAV query
    pairs = {tuple(sorted((u, p))) for u, ps in nmac_dict.items() for p in ps}
    # NMAC encounter onset: pair in NMAC now but not at previous step [count]
    self.nmac_onsets_this_step = sorted(pairs - self._prev_nmac_pairs)
    for i, j in self.nmac_onsets_this_step:
        for uid in (i, j):
            self.uav_dict[uid].nmac_count += 1                # G1: encounters per UAV [count]
    self.total_nmac_encounters += len(self.nmac_onsets_this_step)
    self._prev_nmac_pairs = pairs
    return nmac_dict
```

`SensorEngine` is rebuilt on every `SimulatorManager.reset()`, so encounter state resets with
the episode. Phase A confirms this.

```python
# simulator_manager.py
self._uav_flight_s = 0.0      # accumulated UAV flight time [s] (UAVs with uav_in_flight)
# in step(): self._uav_flight_s += n_in_flight * self.dt
def get_nmac_summary(self) -> dict:
    flight_h = self._uav_flight_s / 3600.0                              # [h]
    enc = self.sensor_module.total_nmac_encounters                      # [count]
    return {"nmac_encounters": enc, "uav_flight_hours": flight_h,
            "nmac_per_flight_hour": enc / flight_h if flight_h > 0 else float("nan")}
```

`MetricsCollector.record(..., nmac_onsets=None)`: optional keyword passed from
`UAMSimulator.step`. **The 5-tuple returned by `_step_uavS` keeps its shape.** It adds
`total_nmac_encounters` and per-edge `nmac_onsets` (keyed by stable `vp.id`). Remove the TODO
and `#!` comments once `nmac_count` is meaningful.

### Verify in Phase A
Every caller of `get_nmac`, `_step_uavS`, `log_step`, `record`; A5 to A8; whether RL
environments (single-agent, multi-agent) read `nmac_count` in rewards (agent_logic). If they
do, the reward semantics change from 0 to real counts, so report this to the user before
implementing.

### Tests
* Existing (unchanged): `tests/test_collision_scenario.py`, `tests/test_collision_performance.py`,
  `rl/surrogate/tests/test_rollout_episode_metrics.py`, `rl/**/tests`.
* New (SIM) `tests/test_nmac_encounters.py` using `build_three_uav_rig` /
  `set_scripted_positions` (A–B close head-on; C grazes A and B around t = 18):
  * A–B gives exactly 1 encounter while in range for several steps.
  * C–A and C–B each give 1 encounter.
  * `uav.nmac_count` equals the per-UAV encounters.
  * The old pair-step total is unchanged.
  * A pair leaving and re-entering range counts 2.
  * Sensor off produces no NMAC (documents A5).
  * With logging disabled, `get_nmac_summary()` is still correct.

### Acceptance
Green; a printed pair-step vs encounter comparison for one `deployment.py` episode.

### Commit
`fix(sim/S03): count NMAC encounters in SensorEngine, per-UAV and per-flight-hour`

**STOP.**

---

## S04: Simulator demand and operator-metric fixes

**Test type:** SIM · **Depends on:** S02

### Goal
Fix F2, F3, F4; let λ change between design steps; add the operator metrics the simulator
must provide (S13 table).

### Files
* Modify: `src/urbannav/demand_model.py` (`DemandModelMixin`), `src/urbannav/vertiport.py`
  (capacity from config), `src/urbannav/airspace.py` (pass pad count to every family),
  `src/urbannav/simulator_manager.py` (`update_lambda_matrix`).
* Read: `uam_simulator.py` (OD loading), `rl/vertiport_design/graph_builder.py`
  (does it read `get_episode_metrics` keys?), `rl/surrogate/*` (same).

### Proposed changes

**(a) Poisson draw (F4).** In the per-step demand generation of `DemandModelMixin`
(Phase A: find the Bernoulli draw on `self._demand_rng`):

```python
# Trip requests are independent and random, so arrivals per region pair follow a
# Poisson process with rate lambda[r, s] [trips/min] (standard arrival model in
# queueing and transit). The number of requests in one step of length dt is
#     n ~ Poisson(lambda[r, s] * dt_min),   dt_min = dt / 60 [min]
# The old coin flip (rng.random() < lambda*dt_min) is only valid when lambda*dt_min << 1
# and can never create more than one request per step, so it silently lost demand.
dt_min = self.dt / 60.0                                          # step length [min]
n_requests = self._demand_rng.poisson(self.lambda_matrix * dt_min)   # (R, R) [requests/step]
for o, d in zip(*np.nonzero(n_requests)):
    for _ in range(int(n_requests[o, d])):
        self._enqueue_request(o, d)                              # existing enqueue logic
```

**(b) Trip-log legs (F2).** `get_episode_metrics()` adds, from `_trip_log`:
* `pair_avg_passenger_wait_min = depart − enqueue`
* `pair_avg_flight_min = arrive_airspace − depart`
* `pair_avg_landing_hold_min = land − arrive_airspace`
* `pair_p95_trip_min`: reliability, 95th percentile of `land − enqueue`.

All are in [min] (× dt/60), discard trips enqueued before a `warmup_steps` argument, and are
NaN where a pair has no completed trip. Keep the old keys (`pair_avg_wait_time`,
`pair_avg_trip_time`) for back-compat, with a comment saying what they really measure.
Phase A lists their readers.

**(c) Pad capacity, single source (F3).** `config.vertiport.number_of_landing_pad` →
`Vertiport.landing_takeoff_capacity` (passed by every Airspace family) →
utilization uses `vp.landing_takeoff_capacity`. Remove the read of `config.airspace.pad_capacity`.

**(d) `SimulatorManager.update_lambda_matrix(lam)`.** Validates shape (R, R), non-negative and
finite; replaces `self.lambda_matrix` for the next episode. Used by S14.

**(e) Operator metrics** (sim-side, added to `get_episode_metrics()`, units in key names):
* `departure_queue_mean`, `departure_queue_max` [requests]
* `unserved_requests_end` [requests]
* `peak_hour_landings_per_vertiport` [landings/h] (rolling 3600 s window)
* `flow_imbalance_per_vertiport` [landings − departures]
* `idle_uavs_mean_per_vertiport` [UAVs]
* `uav_distance_flown_km` [km] (energy proxy)
* `pad_utilization_mean` [-]
* `fleet_utilization` [-] (existing)
* `demand_served_ratio` [-] (existing)

### Verify in Phase A
Exact location of the draw; what else reads `lambda_matrix` / `n_regions`; units of existing
metrics; does anything depend on at most one request per step?

### Tests
* Existing: demand-model tests (Phase A finds them), `tests/test_integration.py`, `rl/`.
* New (SIM):
  * Poisson mean and variance over many steps ≈ λ·Δt (statistical tolerance).
  * λ·Δt > 1 now yields more than one request per step.
  * Trip-log legs on a hand-made trip log equal hand-computed minutes.
  * Utilization uses the configured pad count.
  * `update_lambda_matrix` rejects a bad shape.

### Commit
`fix(sim/S04): poisson demand, trip-log air legs, single pad-capacity source`

**STOP.**

---

## S05: UAV speeds for all registry types

**Test type:** SIM · **Depends on:** S03

### Goal
Realistic air-taxi speeds for every type in `UAV_TYPE_REGISTRY` (F20), within logical limits,
without breaking control, sensing or arrival logic.

### Proposed values (Phase A refines; all [m/s])

| Type | `max_speed` (cruise) | `max_velocity` (cap) | Rationale |
|---|---|---|---|
| STANDARD | 55 (≈198 km/h) | 67 (≈241 km/h) | Conservative for current eVTOL designs (cruise roughly 200 to 300 km/h) |
| HEAVY | 45 | 55 | Heavier, slower |
| SINGLE_AGENT_LEARNING, MULTI_AGENT_LEARNING | as their base type (STANDARD) | as base | Comparable with non-learning UAVs |
| ORCA | as STANDARD | as STANDARD | Same airframe, different controller |
| any other type found | by analogy, document | | |

Also set `assumptions.prior_cruise_speed_mps` equal to the STANDARD `max_speed`.

### Dependent parameters (Phase A must check each; propose changes for user approval)
* `dt` = 1 s → 55 m per step. Arrival/goal threshold, landing radius and PID gains must
  tolerate this (no overshoot loops).
* `detection_radius` (STANDARD 500 m): at 110 m/s closing speed that is about 4.5 s of
  warning. Propose a value from closing speed × desired warning time and **ask the user
  before changing it**.
* `nmac_radius` (200 m): keep. It is comparable to the 500 ft (~150 m) NMAC definition
  used for manned aircraft.
* ORCA time horizon, acceleration limits (time to reach cruise), spatial-hash spacing
  (`PartialSensor` spacing ≥ detection radius).
* Tests with hard-coded speeds or positions.

### Tests
* Existing: all `tests/`, `rl/`.
* New (SIM):
  * A STANDARD UAV reaches cruise and arrives at a vertiport 10 km away without overshoot
    oscillation.
  * Collision scenario still detects → NMAC → collision in order.

### Commit
`feat(sim/S05): air-taxi speeds for all UAV types`

**STOP.**

---

## Data gate: STOP

Before **S06 Phase C**: the user adds `data/gmns_plus/<city>/` (Austin first). The agent
confirms the files load, reports node, zone and OD-row counts, and checks `.gitignore`. The
user decides whether raw GMNS data and generated artifacts (`data/vp_design/`) are committed.

---

## S06: `urbannav.vp_design` package, GMNS loader, road network tables

**Test type:** VP-ENV · **Depends on:** S05 + data gate

### Goal
Add the package and the GMNS layer: load a city, build the road graph, compute zone × zone
car time [min], distance [m] and toll [USD] tables ("skims").

### Files
* Add (from the hand-off bundle): `src/urbannav/vp_design/__init__.py`, `assumptions.py`,
  `gmns_io.py`. Copy the other bundle modules too; later sections wire them in.
* Modify: `pyproject.toml` / `environment_ubuntu.yml` only if needed (`scipy`, `pyproj`,
  `shapely>=2`, `scikit-learn`, `geopandas`, optional `tabulate` for `Assumptions.export(.md)`).
* Read: `pyproject.toml` package discovery (is `urbannav.vp_design` picked up?).

### Content (already in `gmns_io.py`)
* `load_gmns(city_dir)`: validates required columns; drops zero-volume OD rows.
* Link time: `vdf_fftt` (fallbacks: miles/mph, km/(km/h)); connectors of length 0 are clipped
  to a tiny positive time. BPR congested time with `ref_volume` (Austin `obs_volume` is all 0).
* `RoadNetwork`: CSR graph; `dir_flag` 1/−1/0; parallel links keep the fastest (a CSR matrix
  would sum them); multi-source Dijkstra; path sums of length/toll by vectorised pointer
  jumping (verified against brute-force path reconstruction).
* `zone_nodes()`: one node per zone; prefers `node_id == zone_id` (Austin 1..199).
* `zone_skims()`, `save_skims`/`load_skims`, `link_midpoints` + `ground_congestion_near`.

### Verify in Phase A
Austin units (length metres? `vdf_fftt` minutes?) against a few links; the v/c sanity warning;
zones whose centroid is not connected to the main network (notebook flagged some). Report them;
the intrazonal and unreachable rules handle them, but the user should see the list.

### Tests (VP-ENV) `tests/vp_design/test_gmns_io.py`
* Austin: 199 zones, 10,717 nodes; `zone_nodes()` returns node_id 1..199.
* Path-sum check: pick 3 zones, reconstruct paths from predecessors, compare length and toll.
* Table symmetry sanity (not exact; one-way streets).
* Synthetic fixture (small grid under `tests/vp_design/fixtures/`) for fast deterministic
  checks: parallel link handling, `dir_flag` handling, toll accumulation.

### Commit
`feat(vp_design/S06): vp_design package, GMNS loader, zone travel-time tables`

**STOP.**

---

## S07: Zones: centroids → airspace CRS → Voronoi cells

**Test type:** VP-ENV · **Depends on:** S06

### Goal
Turn the 199 GMNS zone centroids into vertiport candidates in the airspace's coordinate system,
and build one Voronoi **zone cell** per zone (re-implementing the lost code / notebook).

### Content (already in `zones.py`)
* `project_zones(zone_nodes, crs)`: adds `x`, `y` [m] in the airspace CRS (the UTM CRS of
  `Airspace.location_utm_gdf`).
* `build_voronoi_cells(zones, extent, crs)`: `shapely.voronoi_polygons` in metres (not degrees,
  as in the notebook), each cell matched to its own centroid, clipped to `extent`;
  `area_km2` column.
* `save_zone_artifacts`: `zones.csv`, `zone_cells.geojson` (EPSG:4326).

### Verify in Phase A
* The CRS used must equal `Airspace.location_utm_gdf.crs` (OSMnx `project_gdf`). The pre-script
  uses the same call. `--offline` uses `estimate_utm_crs()`; check they match for Austin.
* Compare cell count and shapes with the notebook's `zone_cells.geojson` if the user provides it.

### Tests (VP-ENV) `tests/vp_design/test_zones.py`
* 199 cells; every centroid covered by its own cell; cells do not overlap (pairwise
  intersection area ≈ 0); union area equals the extent area (tolerance).
* Projection round-trip (UTM → lon/lat) within 1e-6 deg.

### Commit
`feat(vp_design/S07): zone centroids in airspace CRS and Voronoi zone cells`

**STOP.**

---

## S08: Zone–airspace reconciliation: extended airspace, restricted-area report

**Test type:** SIM + VP-ENV · **Depends on:** S07

### Goal
Decision **D2 = extend**: some Austin centroids lie outside the OSMnx city boundary. Instead of
dropping them (and their demand), the airspace boundary becomes
`union(city boundary, convex hull of zone centroids buffered by hull_buffer_m)`.
Also report zones whose centroid sits inside a restricted-area (RA) buffer.

### Files
* Modify: `src/urbannav/airspace.py` `__init__`: optional
  `boundary_extension_lonlat: shapely geometry | None`, unioned with the geocoded place polygon
  **before** projection and **before** the restricted-area fetch (so RAs are fetched over the
  extended area).
* `zones.py` (bundle): `extended_airspace_boundary`, `reconcile_with_airspace`.
* Read: everything that reads `location_utm_gdf` (ATC mid-point, Renderer, sample-space for
  Family A).

### Proposed changes
```python
# airspace.py __init__ (sketch)
location_gdf = geocode_to_gdf(self.location_name)                 # OSMnx place polygon [EPSG:4326]
if boundary_extension_lonlat is not None:                         # VP design Family C only (S08)
    geom = shapely.union(location_gdf.geometry.iloc[0], boundary_extension_lonlat)
    location_gdf = location_gdf.set_geometry([geom], crs="EPSG:4326")
```
The extension comes from `airspace_boundary.geojson` written by the pre-script (S11) and is
passed by the build dispatch only for `vertiport_source: gmns_zones`.

### RA conflicts (open decision; default = keep + report)
Zones inside an RA buffer stay candidates; the count and ids go to `report.json` and the log.
Alternatives for the user: exclude them from the candidate set, or move the vertiport to the
nearest point outside the buffer within its cell.

### Verify in Phase A
Larger polygon → longer OSM fetch time and more RAs (measure); Renderer extent; Family A is
unaffected (extension only in Family C).

### Tests
* SIM: Airspace with an extension polygon builds; `location_utm_gdf` covers all zone centroids.
* VP-ENV: reconciliation report on Austin lists the outside-boundary zones; RA report present.

### Commit
`feat(sim+vp_design/S08): extended airspace for GMNS zones, RA conflict report`

**STOP.**

---

## S09: Regions: k-means or region file; zone→region map; region λ

**Test type:** VP-ENV · **Depends on:** S08

### Goal
Group zones into regions in two ways, and produce the simulator's region OD rate λ.

### Content (already in `regions.py`)
* `kmeans_regions(zones, k, seed)`: k-means on centroid x, y [m]; regions relabelled by
  cluster-centre position (west→east, then south→north) so numbering is stable. Re-implements
  the lost Band 1 code for `band1_output_{5,50}_region`; the old outputs are not available,
  so there is no regression target.
* Region file: `region <r> == [z1, z2, ...]`, `#` comments, **regions numbered 0..R-1 by the
  user** (decision). `parse_region_file`, `write_region_file` (k-means output written as an
  editable file), `validate_zone_region` (every zone exactly once, known ids, no gaps).
* `write_zone_region_map` / `read_zone_region_map`: CSV `zone_id, region_id`. This is the format
  `UAMSimulator` already reads (first column zone, second region).
* `region_polygons(cells, zone_region)`: dissolved cells per region.
* `region_od_rate(demand, zone_region, A)`: λ[r,s] [trips/min] = Σ volume × occupancy ×
  `demand_scale` / `demand_period_min`, with diagonal 0 (intra-region trips would have the same
  origin and destination vertiport).

### Verify in Phase A
How `UAMSimulator` / `DemandModelMixin` use `zone_region_map` today (any assumption on the zone
id type, int vs str); whether a zero diagonal is acceptable for the demand model.

### Tests (VP-ENV) `tests/vp_design/test_regions.py`
* k = 5 and k = 50 on Austin: all 199 zones assigned; deterministic across runs.
* Region file: valid example parses; duplicate zone, unknown zone, missing zone and
  numbering gap each raise a clear error; write → parse round-trip.
* λ: sum equals inter-region demand × factors; diagonal zero; shape (R, R).

### Commit
`feat(vp_design/S09): regions from k-means or region file, region OD rate`

**STOP.**

---

## S10: Family C (GMNS zones) in Airspace and VertiportDesignEnv `__init__` wiring

**Test type:** SIM + VP-ENV · **Depends on:** S09

### Goal
Implement `Airspace.build_candidate_regions_gmns_zones` and make the VP design environment run
with GMNS zones end to end, demand model on (fixes F1).

### Files
* Modify: `airspace.py` (Family C), `simulator_manager.py` (dispatch passes paths +
  extension), `uam_simulator.py` (read `od_matrix_path` / `zone_region_map_path` from
  `config.vp_design.gmns` when not given explicitly),
  `rl/vertiport_design/vertiport_design_env.py` (`__init__`, `_apply_selection_and_inner_reset`),
  `src/urbannav/deployment_vp_design.py`, `component_schema.py` (`GMNSConfig` fields).
* Read: `graph_builder.py`, `vp_action_space_definitions.py`, `demand_model.py`
  (`vertiport_region_map` usage).

### Proposed changes
```python
# airspace.py - Family C
def build_candidate_regions_gmns_zones(self, zones_csv: str, zone_region_map: str, **_) -> None:
    """Family C: every GMNS zone is a vertiport candidate at its centroid (Phase 1).

    regions_dict[r] = [Vertiport per zone of region r], sorted by zone_id
    (= action index order). vp_id = zone_id (stable across runs).
    """
    zones = pd.read_csv(zones_csv)                         # zone_id, x [m], y [m], ...
    zr = read_zone_region_map(zone_region_map)             # zone_id -> region_id
    self._claim_vertiport_source("gmns_zones")
    self.regions_dict = {}
    for r in sorted(set(zr.values)):
        rows = zones[zones.zone_id.isin(zr.index[zr == r])].sort_values("zone_id")
        self.regions_dict[r] = [
            self._register(Vertiport(self._make_vertiport_point(row.x, row.y),
                                     vp_id=int(row.zone_id), source="gmns_zones",
                                     zone_id=int(row.zone_id), region=int(r)))
            for row in rows.itertuples()]
    self.num_regions = len(self.regions_dict)
    # candidates are NOT put into vertiport_list; only the selection is (set_selected_vertiports)
```

VP design environment per design step:
```python
selected = self.graph_builder.region_idx_to_vertiport(action)          # one Vertiport per region
sm.airspace.set_selected_vertiports(selected)
sm.update_vertiport_region_map({vp.id: vp.region for vp in selected})   # demand routing
self.uam_simulator.reset(rebuild_airspace=False)
```

### Verify in Phase A
* GraphBuilder candidate order equals `regions_dict` order (sorted zone_id).
* `vertiport_region_map` keys are `vp.id` (now `zone_id`).
* Heights: one sample per zone, deterministic per seed (sample in zone order).
* Episode length vs `simulator_step` (F22).

### Tests
* SIM: Family C builds 199 candidates, ids = zone ids, regions 0..R-1, z in [20, 100].
* VP-ENV: env reset + 3 design steps with k = 5 on Austin, demand model on (requests are
  generated), selection survives soft reset, `info` contains episode metrics.

### Commit
`feat(sim+vp_env/S10): GMNS zones as vertiport candidates, env wiring with demand model`

**STOP.**

---

## S11: Config pre-script and zone map plot

**Test type:** VP-ENV · **Depends on:** S10

### Goal
One command creates a ready-to-run config (standalone **or** VP design) and all GMNS
artifacts; it asks **"Is this config for vertiport design? (YES/NO)"**.

### Files
* Add: `scripts/vp_design/make_vp_design_config.py`, `scripts/vp_design/plot_zone_map.py`
  (bundle), `docs/vp_design/regions_example.txt`.
* Modify: align the YAML keys the script writes with the S02 schema exactly (`vp_design.gmns.*`
  names); `sample_config.yaml` comment pointing to the script.

### Behaviour (already in the script)
* **NO** → standalone config: `vp_design.enabled: false`, `standalone.number_of_vertiports`.
* **YES** + `gmns_zones`: runs S06 to S09 and writes `zones.csv`, `zone_cells.geojson`,
  `airspace_boundary.geojson`, `zone_region_map.csv`, `regions.txt`, `regions.geojson`,
  `region_lambda.npy`, `skims.npz`, `report.json` into `data/vp_design/<city>/`, and fills
  `vp_design.gmns` with their paths.
* User-defined values: city, GMNS folder, k **or** region file, demand period [min],
  `demand_scale` [-], pads, vertiport height range [m], dt [s], steps, 2D/3D, seed,
  STANDARD fleet count.
* Prints a **fleet recommendation** by Little's law: UAVs ≈ total λ [trips/min] × mean cycle
  time [min] / target utilization. The cycle time is flight estimate + turnaround (ASSUMPTION
  `--turnaround-min`). This shows how far demand exceeds the fleet (F5); the user picks the
  fleet and/or `demand_scale`.
* `--offline` skips OSMnx geocoding (boundary = buffered hull) for CI.

### Tests (VP-ENV)
* Script with `--vp-design yes --offline --k 5` on Austin writes all artifacts; the produced
  YAML loads with `UAMConfig`.
* `--vp-design no` produces a config that runs `deployment.py`.
* Interactive prompt (monkeypatch `input`) accepts YES/NO variants and rejects others.
* `plot_zone_map.py` writes a PNG.

### Commit
`feat(vp_design/S11): config pre-script with YES/NO mode and GMNS artifacts`

**STOP.**

---

## S12: NMAC in the VP design environment

**Test type:** VP-ENV · **Depends on:** S03, S10

### Goal
Use the simulator's NMAC encounters (S03) inside the VP design environment, attributed to
**region pairs**, so safety can enter metrics and reward (G3, G4 environment side).

### Proposed changes
```python
# demand_model.py - DemandModelMixin (called each step after sensing)
def _accumulate_nmac(self, onset_pairs):
    for pair in onset_pairs:
        for uid in pair:                                   # each in-flight UAV's mission
            uav = self.atc.uav_dict.get(uid)
            if uav is None or not getattr(uav, "uav_in_flight", False):
                continue
            o = self.vertiport_region_map.get(uav.start_vertiport.id, -1)   # origin region [-]
            d = self.vertiport_region_map.get(uav.end_vertiport.id, -1)     # destination region [-]
            if 0 <= o < self.n_regions and 0 <= d < self.n_regions:
                self._nmac_events_od[o, d] += 1           # NMAC encounters per region pair [count]
```

`get_episode_metrics()` adds `nmac_events_od` (R, R) [count], `nmac_encounters` [count],
`uav_flight_hours` [h], `nmac_per_flight_hour` [encounters/h] (from
`SimulatorManager.get_nmac_summary()`), `uav_collisions` [count], `ra_collisions` [count].
The VP design `info` dict carries them. Works with `logging.enabled: false`.

### Verify in Phase A
Where onsets are available to the demand model each step
(`self.sensor_module.nmac_onsets_this_step`); attribution of an encounter between two UAVs on
different region pairs (each UAV's pair counts once, documented).

### Tests (VP-ENV)
`nmac_events_od.sum()` equals the in-flight encounter attributions; finite rate; logging
disabled still yields metrics; synthetic forced-encounter scenario.

### Commit
`feat(vp_env/S12): NMAC encounters per region pair in VP design metrics`

**STOP.**

---

## S13: Door-to-door time and metrics by stakeholder

**Test type:** VP-ENV · **Depends on:** S04, S12

### Goal
Wire `door_to_door.py` and `metrics.py`, and make every metric's status explicit:
present, partial (what to fix) or absent (what to add, and where).

### Background
Three stakeholder perspectives (TCRP Report 88): **passenger** (door-to-door quality),
**operator** (capacity, efficiency), **community** (safety, exposure, equity). Door-to-door
decomposition: Section 1.3.

### Metric placement table

Legend: **P** = present, **Pa** = partial, **A** = absent.

| Group | Metric [unit] | Status | Where it lives (after the plan) | Action |
|---|---|---|---|---|
| Passenger | Access time [min] | A | `vp_design/door_to_door.py` `ZoneDoorToDoor.ACC` | wire (S13) |
| Passenger | Egress time [min] | A | `door_to_door.py` `EGR` | wire |
| Passenger | Passenger wait (enqueue→depart) [min] | A | `demand_model.py` `get_episode_metrics` `pair_avg_passenger_wait_min`; `door_to_door.air_legs_from_trip_log` | S04 |
| Passenger | Flight time [min] | Pa (`pair_avg_trip_time` = land − depart includes hold) | `pair_avg_flight_min` | S04 |
| Passenger | Landing hold [min] | Pa (mislabelled as wait) | `pair_avg_landing_hold_min` | S04 |
| Passenger | Terminal times [min] | A (ASSUMPTION) | `assumptions.py` | done |
| Passenger | Door-to-door time [min] | A | `ZoneDoorToDoor.evaluate` | wire |
| Passenger | UAM/car time ratio [-] | A | `evaluate()['uam_car_time_ratio']` | wire |
| Passenger | Share of trips where UAM is faster [-] | A | `evaluate()` | wire |
| Passenger | Reliability, p95 trip time [min] | A | `demand_model.py` `pair_p95_trip_min` | S04 |
| Passenger | Demand served ratio [-] | P | `demand_model.py` | keep |
| Passenger | Captured share (mode choice) [-] | A | `vp_design/mode_choice.py` | S14 |
| Passenger | Consumer surplus [USD/period, min/trip] | A | `mode_choice.py` | S14 |
| Operator | Pad utilization [-] | Pa (wrong capacity) | `demand_model.py` `pad_utilization_mean` | S04 |
| Operator | Departure queue mean/max [requests] | A | `demand_model.py` | S04 |
| Operator | Unserved requests at end [requests] | A | `demand_model.py` | S04 |
| Operator | Peak-hour landings per vertiport [landings/h] | A | `demand_model.py` | S04 |
| Operator | Flow imbalance / idle UAVs per vertiport | A | `demand_model.py` | S04 |
| Operator | Fleet utilization [-] | P | `demand_model.py` | keep |
| Operator | Distance flown (energy proxy) [km] | A | `demand_model.py` `uav_distance_flown_km` | S04 |
| Operator | Ground congestion near vertiport (v/c) [-] | A | `gmns_io.ground_congestion_near` | wire (static) |
| Community | NMAC encounters, per flight hour [count, /h] | Pa (pair-steps, logger-only) | `sensor_engine.py` (S03), `demand_model.py` (S12) | S03, S12 |
| Community | UAV collisions, RA collisions [count] | Pa (logger-only) | `get_episode_metrics()` | S12 |
| Community | Exposure proxy: activity near vertiports [-] | A (no noise model, F7) | `metrics.activity_near_vertiports` | wire |
| Community | Coverage share [-] | A | `metrics.coverage_and_equity` | wire |
| Community | Access Gini, p90, max [-, min, min] | A | `metrics.coverage_and_equity` | wire |
| Community | Car VMT removed / access VMT added [vehicle-miles] | A | `mode_choice.py` | S14 |

### Proposed changes
`metrics.collect_raw_metrics(d2d, mcm, selection, zone_weight, air, sim_metrics)` returns one
flat dict of **raw (un-normalized)** values, units in key names. The environment builds `d2d`
(`ZoneDoorToDoor`) and `mcm` (`ModeChoiceModel`) **once** in `__init__` from `skims.npz`,
`zones.csv`, `zone_region_map.csv` and `demand.csv`. Air legs come from
`air_legs_from_trip_log(sm._trip_log, R, dt_s=sm.dt, warmup_steps=...)`.

### Verify in Phase A
Key names from S04/S12 match those `collect_raw_metrics` expects (`sim_metrics` docstring); the
intrazonal rule; zones whose region has no trips (NaN handling).

### Tests (VP-ENV)
* Table decomposition equals brute force per OD pair (Austin, random selection) within 1e-9.
* `evaluate()` falls back to priors where the simulator gives NaN.
* Coverage/Gini on a hand-made 3-zone case.
* All metric values finite on Austin for k = 5 and k = 50.

### Commit
`feat(vp_env/S13): door-to-door and stakeholder metrics for VP design`

**STOP.**

---

## S14: Demand responds to placement (mode choice + λ loop)

**Test type:** VP-ENV · **Depends on:** S13

### Goal
Make the number of UAM trips depend on the vertiport selection, and feed the resulting λ back
into the simulator.

### Explanation (plain language)
With a fixed λ, a badly placed vertiport still gets the same trips, so demand cannot reward
good placement. In transportation practice, and in UAM placement studies (e.g. Rath & Chow 2022;
Wu & Zhang 2021), people choose UAM only when it beats driving for *their* trip. A **binary
logit** model turns "how much better or worse UAM is" into a probability:

```
GT_uam = 1.5*(access+egress) + 2.0*wait + terminals + flight + hold
         + 2*transfer_penalty + (fare + access driving cost)/VOT          [min]
GT_car = car time + car terminal time + (operating cost + toll + parking)/VOT   [min]
P(UAM) = 1 / (1 + exp(beta*(GT_uam + uam_bias - GT_car)))                  [-]
captured trips(i, j) = trips(i, j) * P(UAM)
```

The weights, β, `uam_bias`, VOT and fares are ASSUMPTIONS with ranges (`assumptions.py`), so
they can be swept for sensitivity. The logit is non-linear, so it is evaluated **per zone pair**
(~0.7 ms per selection), then summed into λ[r,s].

### Loop with the simulator (GNN-RL environment)
```
design step t:
    selection_t
    air_{t-1}  = air_legs_from_trip_log(previous inner sim)      # None at t = 0 -> priors
    m          = mcm.evaluate(selection_t, air_{t-1})
    sm.update_lambda_matrix(m["captured_lambda"])                # S04
    reset(rebuild_airspace=False); run inner sim                 # produces air_t
```
Lagged by one design step (no inner fixed-point loop). The MCTS variant is an open question (S17, D3).

### Files
* Add: `src/urbannav/vp_design/mode_choice.py` (bundle).
* Modify: `vertiport_design_env.py` (call the loop), `demand_model.py` (accept updated λ each
  episode).

### Verify in Phase A
λ update timing relative to `_init_demand_state()` in `reset`; `demand_scale` interplay.

### Tests (VP-ENV)
* `captured_lambda.sum() × period / demand_scale == captured_trips`.
* Captured share rises when `uam_bias_min` falls (monotonic check over `A.sweep`).
* The environment's λ changes between design steps when the selection changes.

### Commit
`feat(vp_env/S14): mode choice makes demand respond to vertiport placement`

**STOP.**

---

## S15: Raw metrics → generalization block → reward (GNN-RL; shared with MCTS)

**Test type:** VP-ENV · **Depends on:** S14

### Goal
One scoring function used by GNN-RL and MCTS that transfers across cities.

### Why normalize
Raw minutes, trip counts and NMAC counts scale with city size, demand and fleet, so the same
"good" design looks different in each city. Each raw metric becomes a value in [0, 1]
(1 = better) using ratios to a **city reference** (demand-weighted car time), shares, and rates
per flight hour.

### Code shape (already in `normalization.py`)
```python
raw = collect_raw_metrics(d2d, mcm, selection, zone_weight, air, sim_metrics)   # un-normalized
ref = CityReference.from_d2d(d2d)                                               # once per city
norm = normalize_metrics(raw, ref)      # contains:  #### generalization block (normalizing metrics) ####
s = score(norm, weights)                # weighted mean in [0, 1]; NaN terms skipped; penalties subtracted
# GNN-RL
reward = gnn_rl_reward(s, s_prev, mode="improvement")   # s_t - s_{t-1}, or "absolute"
# MCTS
value = mcts_value(s)                                   # bounded in [0, 1] for UCT
```

Normalized terms:
* `n_door_to_door = 1/(1 + T/car)`
* `n_access`
* `n_captured`
* `n_served`
* `n_coverage`
* `n_equity = 1 − Gini`
* `n_safety = 1/(1 + nmac_rate/ref_rate)`
* `n_community`
* penalty `p_pad_overload` (utilization above 0.85)

Weights in `DEFAULT_WEIGHTS` are design choices; the user tunes them.

### Files
* Add: `src/urbannav/vp_design/normalization.py` (bundle).
* Modify: `vertiport_design_env.py`: new `reward_type` values `score_improvement` (default for
  GNN-RL) and `score_absolute`; keep `distance_improvement`.

### Tests (VP-ENV)
* All normalized terms in [0, 1] on Austin.
* Score bounded.
* Improvement reward sums telescopically to `s_T − s_0`.
* NaN terms skipped (before the simulator provides values).

### Commit
`feat(vp_env/S15): normalized score and rewards for GNN-RL and MCTS`

**STOP.**

---

## S16: VP design environment integration and graph features

**Test type:** VP-ENV · **Depends on:** S15

### Goal
Replace coordinate-only node features (F15) with demand-aware, normalized features that
transfer across cities, and finish environment wiring.

### Proposed features (all normalized, inside a
`#### generalization block (normalizing metrics) ####` comment block in `graph_builder.py`)

| Node feature | Meaning | Unit |
|---|---|---|
| `is_selected` | candidate currently selected | 0/1 |
| `x_norm`, `y_norm` | position scaled to the airspace bounding box | [-] |
| `production_share` | zone's produced trips / its region's produced trips | [-] |
| `attraction_share` | zone's attracted trips / its region's attracted trips | [-] |
| `access_norm` | Q-weighted mean of `ACC[k, s]` over destinations / city mean car time | [-] |
| `egress_norm` | Q-weighted mean of `EGR[k, r]` over origins / city mean car time | [-] |
| `cell_area_share` | zone cell area / region area | [-] |
| `in_ra_buffer`, `outside_city_boundary` | reconciliation flags (S08) | 0/1 |
| dynamic (selected nodes, previous step) | pad utilization, departure queue / served requests, captured share of its region | [-] |

| Edge feature | Meaning | Unit |
|---|---|---|
| `dist_norm` | straight-line distance / airspace diagonal | [-] |
| `od_share` | Q[r,s] / ΣQ between the endpoint regions (0 within a region) | [-] |
| `car_ratio` | CAR[r,s] / city mean car time | [-] |
| `both_selected` | both endpoints selected | 0/1 |

Update `NODE_FEAT_DIM` / `EDGE_FEAT_DIM` in `vp_obs_space_definitions.py` and the GNN encoder
input sizes (Phase A finds the policy file). With 199 zones and full connectivity there are
about 39,400 directed edges. Rollout-buffer memory (~1 MB per observation, about 2 GB for
2048 steps) is acceptable on the user's server (64 GB RAM, 24 GB GPU).

### Verify in Phase A
Feature order consumers (policy, any saved checkpoints: old checkpoints become incompatible,
so tell the user); `GRAPH-METRICS` obs type stub.

### Tests (VP-ENV)
* Observation matches the space (shape, dtype, bounds).
* Features finite and within [0, 1] where documented.
* PPO + GNN runs 2 design steps (smoke).

### Commit
`feat(vp_env/S16): demand-aware normalized graph features, env integration`

**STOP.**

---

## S17: MCTS (deferred; started by the user)

**Test type:** VP-ENV · **Depends on:** S15

**Trigger.** Do nothing in this section until the user has added their MCTS scripts to the repo
and prompts: *"Read the MCTS files and use PRE_PLAN S17."* Then run Phase A on their code.

### Agreed design (to adapt the user's implementation to)
* **Decision structure:** one zone per region. A tree level = one region, in a fixed order
  (Phase A: the user's code may differ; adapt rather than rewrite).
* **Evaluation:**
  * During search (rollouts), use the cheap static score:
    `collect_raw_metrics` (tables + mode choice, ~1 ms) → `normalize_metrics` → `score` →
    `mcts_value` ∈ [0, 1] (bounded for UCT).
  * Run the simulator only on leaves / top-k candidates. Simulated metrics (wait, hold,
    utilization, NMAC rate) then refine the value.
* **Reuse:** the same reward/score module as GNN-RL (S15), so results are comparable.
* **Transposition table:** keyed by the selection tuple (zone ids in region order).
* **Future:** `rl/surrogate` `predict_episode_outcome` as a learned value function to replace
  some simulator calls.

### Open question D3 (the user answers here before implementation)
How the demand loop (S14) runs inside MCTS:
* (a) lagged air legs from the previous simulated leaf;
* (b) fixed-point iterations per simulated leaf (λ → sim → air legs → λ, max ~3);
* (c) priors only during search, simulate only the final best few selections.

### Wiring points
`ZoneDoorToDoor`, `ModeChoiceModel`, `collect_raw_metrics`, `CityReference`,
`normalize_metrics`, `score`, `mcts_value`; simulator access via `VertiportDesignEnv` (soft
reset) or a thin evaluator around `UAMSimulator`.

### Tests (VP-ENV)
* On the synthetic fixture, MCTS finds the brute-force best selection for small R.
* Deterministic with a seed.
* Respects the simulator budget.

### Commit
`feat(vp_env/S17): wire MCTS to shared VP design scoring`

**STOP.**

---

## S18: End-to-end campaign on real data

**Test type:** SIM + VP-ENV · **Depends on:** S16 (S17 optional)

**Goal.** Prove the whole system on Austin, and record performance.

**Steps.**
1. Standalone: `deployment.py`; short single-agent and multi-agent training smoke runs.
2. Pre-script: k = 5 and k = 50 configs.
3. VP design with GNN-RL: 2 episodes × N design steps. Use realistic `simulator_step` and
   warm-up: replace the smoke value 2 (F22) with a value long enough for trips to complete;
   Phase A proposes one from flight times.
4. MCTS, if S17 is done.
5. Report:
   * step time and memory (reuse `benchmarks/deployment_regression_analysis.py`)
   * share of region pairs with simulated air legs
   * metric table for the best selection
   * Little's-law fleet vs configured fleet

**Commit.** `test(sim+vp_env/S18): end-to-end Austin campaign and report`

**STOP.**

---

## S19: Phase 2 hooks: structures inside cells (docs + parked code)

**Test type:** none · **Depends on:** S07

**Content.**
* `vp_design/phase2_structures.py` (bundle, parked): OSM building candidates per zone cell
  (from the notebook), snapping to the GMNS road network, candidate access/egress tables.
* **Cell system independence:** cells are Voronoi today. Any other tessellation only changes
  the cell polygons, and therefore which buildings belong to each zone. The design (one structure
  per cell as the vertiport) is unchanged.
* Known issues to resolve then:
  * zones with zero candidate buildings (notebook)
  * building heights from a database linked to OSM polygons (replaces random 20 to 100 m)
  * `create_vertiport_from_lat_long` (F13)

**Commit.** `docs(vp_design/S19): phase 2 structure-candidate plan and parked code`

**STOP.**

---

## S20: Future: cruise-altitude band

**Test type:** SIM · **Depends on:** S01

**Goal (future).** With vertiports at 20 to 100 m and planners flying start z → end z, every
UAV flies low and vertical separation is lost. Add a per-UAV **cruise altitude** sampled from a
configurable band (e.g. several hundred metres; user decides). Each flight then has three
parts: climb from the vertiport height to cruise altitude, cruise, and descent to the
destination vertiport height. This restores the multi-altitude band the old 1500 to 3500 m
vertiport heights produced.

**Files (expected).** 3D planners, `component_schema.py` (band config), mission assignment in
ATC.

**Commit.** `feat(sim/S20): per-UAV cruise altitude band`

**STOP.**

---

## Appendix A: Decisions recorded

| ID | Decision |
|---|---|
| D1 | UAV speeds: realistic air-taxi values for all types (S05). |
| D2 | Zones outside the city boundary: **extend the airspace** (S08). |
| D3 | Demand loop inside MCTS: **open**, answered in S17. GNN-RL uses a one-step lag (S14). |
| D4 | Vertiport height: seeded random integer in **[20, 100] m**; OSMnx has no building heights. |
| D5 | Config: `vp_design.enabled` YES/NO; the pre-script asks the question. |
| D6 | Region file: users number regions 0..R-1. |
| D7 | Band 1 outputs are lost: k-means is re-implemented with no regression target. |
| D8 | NMAC: simulator fix in the sensor engine (S03); VP design consumes it (S12). Sensor-off near vertiports is by design. |
| D9 | Graph size: full connectivity is fine with 199 zones; memory is acceptable on the server. |
| D10 | Package name `urbannav.vp_design`; data in `data/gmns_plus/<city>/`. |

## Appendix B: Open items for the user (agent raises them in the relevant Phase A)

* Zones inside restricted-area buffers: keep + report (default), exclude, or move (S08).
* `detection_radius` change after the speed update (S05).
* Whether to commit raw GMNS data / generated artifacts (data gate).
* Reward weights `DEFAULT_WEIGHTS` and `ref_nmac_per_flight_h` (S15).
* Whether single/multi-agent RL rewards read `nmac_count` (S03): semantics change from always 0
  to real counts.
