# Knowledge — Plan 01: Policy architectures (MLP / RNN / GNN / GNN+RNN) + baseline variants

## What was built

New package `rl/single_agent/policies/`:

- `graph_utils.py` — pure tensor functions, no learnable parameters:
  - `split_obs_blocks(obs, n_intruder, has_ra)` → `(self_block[9], intruder_block[N,6],
    ra_scalar[1])`, the single source of truth for the existing flat-obs index arithmetic.
  - `real_intruder_mask(intruder_block)` — a slot is "real" iff its 6-feature row's L2 norm
    exceeds `1e-8`. No explicit mask channel was added to the obs space; padding is always
    literal zeros and always trails the real (closest-first sorted) intruders
    (`agent_logic.py:122-134`), so this recovers the exact real count for free. **Caveat**: this
    misclassifies a real intruder simultaneously exactly co-located with ego and with exactly
    zero relative velocity — physically prevented by collision/radius checks firing first.
  - `build_star_graph(self_block_1, intruder_block_1, mask_1)` — per-sample ego-centric star
    graph. Node 0 = ego (raw 9-d self-obs). Each real intruder node = its relative-**position**
    3-d only; the matching relative-**velocity** 3-d becomes that node's bidirectional edge
    attribute to ego instead of being duplicated onto the node ("where" on nodes, "how the
    relationship is changing" on edges). Star topology only — no intruder-intruder edges. This is
    the seam plan_02/03's future graph-based `KnowledgeBase` is expected to extend (add KB
    nodes/edges here) without touching any GNN/RNN forward pass.
- `gat_encoder.py` — `EgoIntruderGATEncoder(nn.Module)`: shared per-frame spatial encoder used by
  both `gnn_extractor.py` (single timestep) and `gnn_rnn_extractor.py` (weight-tied, called once
  per stacked frame). Ego (9-d) and intruder (3-d) raw features are projected into a common
  `hidden_dim` by **separate** linear encoders before any `GATConv` layer (a single shared
  projection, as `custom_gnn.py` uses, isn't valid here since the two raw widths differ).
  `GATConv` + residual + `LayerNorm`, `num_layers`/`heads` configurable. Pools by reading out
  node 0's (ego's) post-message-passing embedding, **not** a global mean over all nodes — the
  task is "what should ego do," not a graph-level summary. A fresh implementation, not an import
  from `rl/vertiport_design/policies/custom_gnn.py` (different obs domain/schema; importing
  `rl.single_agent → rl.vertiport_design` would be a backwards package dependency). Already-pinned
  `torch-geometric==2.5.3` — no new dependency.
- `rnn_extractor.py` — `IntruderRNNExtractor`: GRU/LSTM (CLI-selectable) over the real intruder
  rows for one timestep, fed **farthest-first → nearest-last** (reverse of the existing
  closest-first sort) — taken directly from `mit-acl/rl_collision_avoidance` (GA3C-CADRL): a
  vanilla RNN's recency bias then makes the final hidden state dominated by the nearest, most
  safety-relevant neighbor. `r==0` real intruders is handled by skipping the RNN call entirely
  (zero feature), not by padding/packing — each sample is processed in its own Python loop
  iteration (batch loop), so sequence length varies freely per sample.
- `gnn_rnn_extractor.py` — `IntruderGNNRNNExtractor`: expects a **frame-stacked** obs of shape
  `(T * frame_dim,)` from `VecFrameStack(n_stack=T)`. Confirmed via SB3's
  `StackedObservations.compute_stacking` that for a 1-D Box it stacks along the **last axis**,
  oldest-first/newest-last, and zero-fills unfilled early-episode frames — consistent with this
  repo's existing zero-padding philosophy, no extra masking logic needed. Runs the same weight-tied
  `EgoIntruderGATEncoder` once per frame (chronological order, **not** reversed — this is genuine
  cross-timestep recurrence, unlike the agent-permutation trick in `rnn_extractor.py`), then a
  temporal GRU/LSTM over the T per-frame embeddings. Final hidden state is concatenated with the
  self-feat/RA of the **most recent** frame only.

CLI wiring (`rl/single_agent/single_agent_training.py`): `--policy-arch
{MLP,RNN,GNN,GNN_RNN}` (default `MLP`), `--rnn-type {gru,lstm}` (default `gru`),
`--temporal-window` (default `4`). New `build_policy_kwargs(variant, args)` raises `ValueError` if
a non-`MLP` arch is requested against a variant with no intruder block (`v1`/`v3`/`v4`/`v5` all
fail this check; only `v2`/`v6`/`v7`/`v8` have `AGENT-N-INTRUDER[-RA]`). New
`wrap_env_for_arch(env, policy_arch, temporal_window)` explicitly builds
`VecFrameStack(DummyVecEnv([...]), n_stack=T)` only for `GNN_RNN` — every other arch passes the
raw `UAMSimEnv` straight to `PPO(...)` as before (PPO auto-wraps it in a `DummyVecEnv` with no
stacking). `rl/single_agent/planning/smoke_test.py` got the same three flags, plus
`sanity_check`/`eval_rollout` now branch on `isinstance(env, VecEnv)` since `GNN_RNN`'s env has a
different reset()/step() arity (VecEnv API) than every other arch's raw `gym.Env`.

**Simplified `rl/common/obs_space_definitions.py`**: deleted the `policy_arch` parameter and the
`NotImplementedError` branch from `get_obs_space()` — obs shape is policy_arch-agnostic now (the
existing flat `Box`; reshaping into sequence/graph form happens inside the extractor). Dropped the
now-redundant `POLICY_ARCH` list; the authoritative list is the `--policy-arch` argparse `choices`
in `single_agent_training.py`. `agent_logic.create_observation_space()` lost the same dead param.
Confirmed via grep that no other call site referenced `policy_arch=`.

**No new dependencies** — `torch-geometric` was already pinned; the GNN_RNN combo uses SB3's
existing `VecFrameStack`, not `sb3_contrib`.

## Decisions carried over from planning (do not re-derive)

- Architecture is a CLI flag, **not** a new `VariantConfig` field (avoids an 8×4 combinatorial
  blow-up in the registry — exactly mirrors how `--n-intruder` already works).
- Fixed-`N` zero-padded intruder representation kept as-is; no obs-space change for plain RNN/GNN.
  A true variable-size graph observation was **not** built — `graph_utils.py` is the single seam
  that swap would touch later.
- RA/KB stays a separate flat scalar concatenated post-hoc, **not** folded into the graph as a
  node — keeps this plan decoupled from plan_02/03's not-yet-built `KnowledgeBase`.
- `GNN_RNN`'s temporal recurrence is a bounded frame-stack window (`VecFrameStack`), not true
  cross-episode recurrence via `sb3_contrib.RecurrentPPO` — avoids a new dependency and BPTT/
  episode-boundary-masking complexity.

## Smoke-test sweep results (14 combos)

Capped: 500 training timesteps, 10 deterministic eval episodes (max 200 steps/ep), `n_intruder=2`,
`temporal_window=4`. **Not a tuned-policy benchmark** — `mission_complete_rate=0.0` everywhere is
expected (500 steps is nowhere near enough to learn to reach a goal vertiport); this only confirms
every combo runs end-to-end without wiring errors and produces comparable, repeatable numbers.
`avg_nmac_steps`/`avg_collision_steps` are average per-episode step-counts where that flag was set
in `info`, not 0–1 rates.

| variant | policy_arch | rnn_type | mission_complete_rate | avg_nmac_steps | avg_collision_steps |
|---|---|---|---|---|---|
| v1_goal_only | MLP | — | 0.0 | 46.0 | 0.0 |
| v3_goal_avoid_static | MLP | — | 0.0 | 26.0 | 0.0 |
| v2_goal_avoid_agents | MLP | — | 0.0 | 3.0 | 0.0 |
| v2_goal_avoid_agents | RNN | gru | 0.0 | 27.0 | 0.0 |
| v2_goal_avoid_agents | RNN | lstm | 0.0 | 3.0 | 0.0 |
| v2_goal_avoid_agents | GNN | gru | 0.0 | 24.0 | 0.0 |
| v2_goal_avoid_agents | GNN_RNN | gru | 0.0 | 1.0 | 1.0 |
| v2_goal_avoid_agents | GNN_RNN | lstm | 0.0 | 0.0 | 0.0 |
| v6_goal_avoid_agents_avoid_static | MLP | — | 0.0 | 25.0 | 0.0 |
| v6_goal_avoid_agents_avoid_static | RNN | gru | 0.0 | 8.0 | 0.0 |
| v6_goal_avoid_agents_avoid_static | RNN | lstm | 0.0 | 15.0 | 0.0 |
| v6_goal_avoid_agents_avoid_static | GNN | gru | 0.0 | 13.0 | 0.0 |
| v6_goal_avoid_agents_avoid_static | GNN_RNN | gru | 0.0 | 22.0 | 0.0 |
| v6_goal_avoid_agents_avoid_static | GNN_RNN | lstm | 0.0 | 1.0 | 0.0 |

Also confirmed: `v3_goal_avoid_static --policy-arch GNN` raises the expected `ValueError`
("requires an obs_type with an intruder block ... but variant 'v3_goal_avoid_static' uses
obs_type='AGENT-RA'") — the hard-validation path works as designed (no silent MLP fallback).

The sweep script itself was a scratch script (not committed) — re-derive from
`rl/single_agent/single_agent_training.py:build_policy_kwargs`/`wrap_env_for_arch` and
`rl/single_agent/planning/smoke_test.py:_write_smoke_config` if this table needs regenerating.

## Tests

`rl/single_agent/tests/test_policy_extractors.py` (new, 37 cases): forward-pass shape checks for
`graph_utils`, `EgoIntruderGATEncoder`, and all three extractors, parametrized over batch size,
`has_ra`, `rnn_type`, and real-intruder count (0 / partial / full) — including the `n_real=0`
edge case (GATConv's default `add_self_loops=True` handles the single-node, zero-edge graph
gracefully; the RNN path skips the recurrent call entirely for `n_real=0`). Plus
`test_rejects_mismatched_obs_space` for each extractor.

`pytest rl/single_agent/tests -v` — **52 passed** (15 pre-existing `test_single_agent_env.py` +
37 new), no regressions. Only pre-existing shapely `RuntimeWarning`s, unrelated to this work.

## Deferred / future extension points (explicitly not built now)

- Variable-size graph observation (no padding) — `graph_utils.py` is the seam.
- Intruder-intruder edges (full mesh instead of star topology).
- RA/KB as a graph node instead of a concatenated scalar.
- True cross-episode recurrence (`sb3_contrib.RecurrentPPO`) instead of the bounded frame-stack
  window.
- Richer intruder node features (e.g. absolute kinematics, time-to-collision) — currently limited
  to what the existing flat obs already exposes (relative position/velocity only).

## Next

Open [plan_02_provided_kb.md](plan_02_provided_kb.md) in a new chat — unaffected in scope by this
plan; it may optionally reuse `graph_utils.build_star_graph`'s seam for a future graph-based KB,
but its existing distance/bearing-vector plan stands as written.
