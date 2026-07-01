# Plan 01 — Policy architectures (MLP / RNN / GNN / GNN+RNN) + baseline variants

See [README.md](README.md) for the overall chain and conventions. Load
`knowledge_00_shared_foundation_summary.md` before starting (full
`knowledge_00_shared_foundation.md` if more detail is needed).

This plan **replaced** plan_01's original narrow scope ("validate the 4 ready variants, no new
code") with a larger one, per explicit decision: no renumbering of plan_02/03/04 — they stay as
provided-KB / learned-KB / continual-curriculum. The original validation goal became Phase B of
this plan, exercised across every architecture rather than MLP-only.

**Status: done.** See [knowledge_01_baseline_variants.md](knowledge_01_baseline_variants.md) /
[summary](knowledge_01_baseline_variants_summary.md) for what was actually built, the smoke-test
sweep results, and deferred extension points.

## Context

Every variant trained with a hardcoded `PPO("MlpPolicy", ...)` over a flat observation vector
(`rl/common/agent_logic.py:extract_observation`, `rl/common/obs_space_definitions.py`).
`POLICY_ARCH = ['MLP', 'GNN', 'RNN']` existed as a stub in `obs_space_definitions.py` with
`NotImplementedError` for the non-MLP cases — this plan implemented them for real, plus a fourth,
combined architecture.

This was built now (ahead of the provided/learned knowledge-base work in plan_02/03) because the
node/edge graph schema and the per-sample graph-construction code built here for ego+intruders is
the same seam plan_02/03's future graph-based `KnowledgeBase` will plug into — building it now
means that swap later touches the obs encoding and feature extractor only, not the
registry/lifecycle/sensor-wiring layer (see plan_02's "Knowledge-base abstraction" section).

Four other-agent representations, selectable per training run via CLI (analogous to the existing
`--n-intruder` flag — **not** a new `VariantConfig` field, since architecture is orthogonal to
the reward/obs variant and a registry field would force an 8×4 combinatorial blow-up):

1. **MLP** (existing) — flat vector through SB3's default `FlattenExtractor`.
2. **RNN** — GRU or LSTM (CLI-selectable) consuming the `N` intruder slots as a sequence within a
   single timestep, CADRL/GA3C-CADRL style.
3. **GNN** — ego + intruders as nodes of a star graph, `GATConv`-based, following the pattern
   already validated in `rl/vertiport_design/policies/custom_gnn.py` (which itself already
   depends on `torch-geometric==2.5.3`, already pinned in `environment_ubuntu.yml` — **no new
   dependency**).
4. **GNN+RNN** — spatial GNN per timestep, RNN over a **fixed-length frame-stack window** of past
   timesteps' graph embeddings (decided over true cross-episode recurrence via `sb3_contrib`
   `RecurrentPPO`, to avoid a new dependency and BPTT/masking complexity — bounded context `T` is
   an accepted tradeoff, consistent with the existing fixed-`N` padding philosophy).

Architecture choice only makes sense for obs_types with an intruder block (`AGENT-N-INTRUDER`,
`AGENT-N-INTRUDER-RA`) — `v1_goal_only`/`v3_goal_avoid_static` have none, so non-`MLP` archs against
them is a hard validation error, not a silent fallback.

Decided trade-offs (do not re-derive — these were explicit user decisions):
- Other-agent count stays the existing fixed-`N` zero-padded representation (no obs-space change
  for plain RNN/GNN). A true variable-size graph observation was **not** built now, but the
  graph-construction code is factored into a single seam (`graph_utils.py`) precisely so that swap
  later doesn't require touching the GNN/RNN forward passes.
- RA/KB info stays a separate flat scalar, concatenated post-hoc onto whatever the other-agent
  encoder produces — it is **not** folded into the graph as a node. This keeps this plan decoupled
  from plan_02/03's not-yet-built `KnowledgeBase`.

## Phase A — implement policy-architecture infrastructure

New package `rl/single_agent/policies/` (mirrors `rl/vertiport_design/policies/` layout):

1. **`graph_utils.py`** — single source of truth for slicing the existing flat obs layout
   (`self_block = obs[:9]`, `intruder_block = obs[9:9+6N].reshape(N,6)`,
   `ra_scalar = obs[9+6N]` if present), reused by all three new extractors instead of each
   re-deriving offsets. `real_intruder_mask()` recovers real-vs-padded slots from a zero-row
   heuristic (no explicit mask channel). `build_star_graph()` returns raw, per-sample node/edge
   tensors for one ego-centric star graph (ego node = self-obs, intruder node = relative position,
   edge = relative velocity).
2. **`gat_encoder.py`** — `EgoIntruderGATEncoder`, the shared per-frame GATConv spatial encoder
   used by both `gnn_extractor.py` and `gnn_rnn_extractor.py` (weight-tied across frames).
3. **`rnn_extractor.py`** — `IntruderRNNExtractor`. GRU/LSTM over real intruder rows, fed
   **farthest-first → nearest-last** (the GA3C-CADRL recency-bias trick), final hidden state
   concatenated with self-feat/RA.
4. **`gnn_extractor.py`** — `IntruderGNNExtractor`. Single-timestep star graph through
   `EgoIntruderGATEncoder`, ego's pooled embedding concatenated with RA.
5. **`gnn_rnn_extractor.py`** — `IntruderGNNRNNExtractor`. Expects a frame-stacked obs (T·D,) from
   `VecFrameStack`; runs the weight-tied spatial encoder per frame, then a temporal GRU/LSTM
   (chronological order) over the T frame embeddings.
6. **CLI wiring** (`single_agent_training.py`, `smoke_test.py`): `--policy-arch
   {MLP,RNN,GNN,GNN_RNN}` (default `MLP`), `--rnn-type {gru,lstm}` (default `gru`),
   `--temporal-window` (default `4`). `build_policy_kwargs()` validates obs_type compatibility;
   `wrap_env_for_arch()` applies `VecFrameStack` only for `GNN_RNN`.
7. **Simplified `obs_space_definitions.py`**: removed the dead `policy_arch` param and
   `NotImplementedError` branch from `get_obs_space()` — obs shape is policy_arch-agnostic.

## Phase B — smoke test sweep (merged baseline-variant validation)

Ran `smoke_test.py` for:
- `v1_goal_only`, `v3_goal_avoid_static` — `MLP` only (no intruder block).
- `v2_goal_avoid_agents`, `v6_goal_avoid_agents_avoid_static` — each × `{MLP, RNN-gru, RNN-lstm,
  GNN, GNN_RNN-gru, GNN_RNN-lstm}`.

14 runs total, all passed. Added `rl/single_agent/tests/test_policy_extractors.py` (37 cases,
forward-pass shape checks). `pytest rl/single_agent/tests -v` — 52 passed.

## Phase C — knowledge capture

See [knowledge_01_baseline_variants.md](knowledge_01_baseline_variants.md) for the full 14-combo
comparison table, extractor interfaces, the zero-row mask heuristic and its caveat, the
`VecFrameStack` wiring point, and the deferred extension points (variable-size graph, full-mesh
edges, RA-as-node, true cross-episode recurrence).

## Next

Open [plan_02_provided_kb.md](plan_02_provided_kb.md) in a new chat — it can optionally reuse
`graph_utils.build_star_graph`'s seam for a future graph-based KB, but is not required to; its
existing distance/bearing-vector scope is unaffected by this plan.
