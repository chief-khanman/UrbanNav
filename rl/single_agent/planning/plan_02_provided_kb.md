# Plan 02 — Provided knowledge base (variants 4, 7)

See [README.md](README.md) for the overall chain and conventions. Load
`knowledge_01_baseline_variants_summary.md` before starting.

## Context

Variants 4 and 7 add a static-obstacle "knowledge base" term to the reward/obs. The KB must be
built **behind a swappable abstraction**, not hard-coded as a vector: the user's stated future
direction is a graph-based KB (a complete graph of static-obstacle locations handed in as
"provided", or an empty graph progressively filled in during training as "learned" — see plan_03
and the future `plan_0N_graph_kb.md`). Today's distance/bearing-vector implementation must not
block that swap.

## Knowledge-base abstraction

Mirrors the repo's existing pluggable-component convention (`*_template.py` ABC + `*_CLASS_MAP`
registry, e.g. `dynamics_template.py` / `DYNAMICS_CLASS_MAP`) rather than inventing a new pattern:

- `knowledge_base_template.py` (location: alongside `sensor_template.py` in `src/urbannav/`, since
  it's sensor-data-adjacent) defines `KnowledgeBase(ABC)` with exactly two representation-agnostic
  lifecycle methods:
  - `reset()` — called every episode.
  - `observe(detection_event)` — called every step, fed from the sensor's existing RA-detection
    output (`sensor_partial.py`'s detection/collision flags).

  These two methods are all `agent_logic.py` and the gym env wrapper ever call directly — they
  don't know or care whether the underlying representation is a feature vector, a graph, or
  anything else.
- Every concrete `KnowledgeBase` also takes `mode: Literal['provided', 'learned']` at construction:
  `provided` seeds full knowledge up front (ground-truth RA polygons today; a complete
  static-obstacle graph later), `learned` starts empty and grows only via `observe()` calls. This
  generalizes unchanged to a future graph KB — "empty graph filled during training" is just
  `GraphKB(mode='learned')` reusing the exact lifecycle plan_03 builds for the vector case.
- Concrete classes register in a `KB_CLASS_MAP` (e.g. `{'distance_bearing': DistanceBearingKB}`),
  selected by the `kb_type` string already present in plan_00's registry entries — exactly like
  `dynamics`/`controller` strings resolve through `DYNAMICS_CLASS_MAP`. Adding `GraphKB` later
  means: implement the ABC, register `'graph': GraphKB`, change one config string.
- **What this does not paper over:** the obs vector an MLP policy consumes and a graph an eventual
  GNN policy would consume are genuinely different shapes — swapping representations will still
  need a new `OBS_SPACE` entry and (for the graph case) a custom SB3 feature extractor alongside
  the new `KnowledgeBase` subclass. That's an inherent consequence of changing representations, not
  something the abstraction can or should hide. What it guarantees reusable: registry/config-driven
  selection, the provided-vs-learned lifecycle, and the wiring into the sensor's detection events.

## Phase A — implement

1. `knowledge_base_template.py`: `KnowledgeBase(ABC)` as above.
2. `KB_CLASS_MAP` registry (same module or a sibling, following the `*_engine.py`-side `CLASS_MAP`
   convention).
3. `DistanceBearingKB(KnowledgeBase)`: in `mode='provided'`, `reset()` seeds itself from
   `Airspace`'s ground-truth RA polygons (`airspace.py:109-143`, `restricted_airspace_geo_series`)
   and exposes a distance/bearing-to-nearest-RA feature; `observe()` is a no-op in this mode since
   everything is known up front.
4. New `OBS_SPACE` entries in `obs_space_definitions.py`: `AGENT-RA-KB`, `AGENT-N-INTRUDER-RA-KB`
   — same shape family as the existing `AGENT-RA`/`AGENT-N-INTRUDER-RA` entries plus the KB
   feature, reading from a `DistanceBearingKB` instance instead of the raw RA-flag lookup used by
   `r4` today.
5. Update `variant_registry.py`: flip `v4_goal_avoid_static_provided_kb` and
   `v7_goal_avoid_agents_avoid_static_provided_kb` from `status="pending_kb_infra"` to `"ready"`.

## Phase B — smoke test

`smoke_test.py --variant v4_goal_avoid_static_provided_kb` and
`--variant v7_goal_avoid_agents_avoid_static_provided_kb`. Confirm `pytest rl/single_agent/tests -v`
still passes (extend tests for the new obs entries rather than replacing existing ones).

## Phase C — knowledge capture

`knowledge_02_provided_kb.md` — **explicitly record the final `KnowledgeBase` ABC method
signatures and the `KB_CLASS_MAP` registration pattern here**, since plan_03 (and any future graph
KB) implements against them. Then `knowledge_02_provided_kb_summary.md`.

## Next

Open [plan_03_learned_kb.md](plan_03_learned_kb.md) in a new chat.
