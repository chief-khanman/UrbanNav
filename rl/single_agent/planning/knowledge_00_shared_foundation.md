# Knowledge — Plan 00: Shared foundation

## What was built

- `rl/single_agent/variant_registry.py`: `VariantConfig` dataclass + `VARIANT_CONFIG_MAP` with all 8
  variants, plus `get_variant(name)` (raises `ValueError` for unknown names, `NotImplementedError`
  for `status != "ready"`).
- `rl/common/agent_logic.py`: added the `r1r2r4` key to `compute_reward`'s dispatch dict
  (`r1 + r2 + r4`) and updated the docstring. No other dispatch keys were added — see "Deviation
  from the original plan" below.
- `rl/single_agent/single_agent_training.py`: replaced the old demo skeleton with a real CLI
  (`--variant`, `--config-path`, `--total-timesteps`, `--n-intruder`, `--seed`), a `build_env()`
  helper (variant name -> `UAMSimEnv`), separate train/eval `UAMSimulator` instances (avoids
  sharing simulator state between training rollouts and `EvalCallback`'s eval rollouts), and SB3
  `CheckpointCallback` + `EvalCallback` wiring. Checkpoints land in
  `rl/single_agent/checkpoints/<variant_name>/`.
- `rl/single_agent/planning/smoke_test.py`: generic `--variant` runner — env sanity check (reset +
  5 steps, asserts obs shape and finite reward), a capped `PPO.learn()`, then a short deterministic
  eval rollout. Imports `build_env` from `single_agent_training.py` rather than duplicating it.
  Writes its own temp config (1 STANDARD + 1 SINGLE_AGENT_LEARNING, logging/rendering off,
  `total_timestep=200`) mirroring `rl/conftest.py`'s `single_agent_config_path` fixture.
- `.gitignore`: added `/rl/single_agent/checkpoints/`.
- `rl/single_agent/planning/README.md` + `plan_00`–`plan_04` markdown files (the full chain).

## Deviation from the original plan (important — read this)

The approved plan initially called for three new no-speed reward keys (`r1r3`, `r1r4`, `r1r3r4`).
**The user corrected this mid-implementation**: "goal" in their conception includes reaching it in
the shortest time, i.e. the speed term `r2` is part of every goal-reaching variant, not a separate
axis. This was fixed by reverting those three keys and adding exactly one new key instead:
`r1r2r4` (goal + speed + avoid-static, no avoid-agents term). The other three combinations needed
by the 8 variants (`r1r2`, `r1r2r3`, `r1r2r3r4`) already existed in `agent_logic.py` before this
plan started.

Final reward_type mapping used by `variant_registry.py` (all include `r2`):

| variant | reward_type |
|---|---|
| v1_goal_only | `r1r2` |
| v2_goal_avoid_agents | `r1r2r3` |
| v3_goal_avoid_static | `r1r2r4` (new) |
| v4/v5 (static + KB) | `r1r2r4` (new) |
| v6_goal_avoid_agents_avoid_static | `r1r2r3r4` |
| v7/v8 (agents+static + KB) | `r1r2r3r4` |

**If you are reading this in a later plan/session and see references to `r1r3`/`r1r4`/`r1r3r4`
anywhere (e.g. stale comments), they are wrong — the live source of truth is `agent_logic.py`'s
dispatch dict and `variant_registry.py`'s `reward_type` fields.**

## Gotchas hit

- None beyond the reward-term correction above. SB3's `EvalCallback` auto-wraps a raw `gym.Env` in
  a `DummyVecEnv` internally, so passing the eval env un-vectorized is fine — but it's a genuinely
  separate `UAMSimulator`/`UAMSimEnv` instance from the training env, not the same object, to avoid
  cross-contaminating simulator state between training and eval rollouts.
- `rl/conftest.py`'s existing fixtures rebuild a fresh `UAMSimulator` (and re-fetch/re-cache OSM
  data via osmnx) per test rather than reusing a module-scoped one — `smoke_test.py` follows the
  same pattern for consistency; first run is slower until osmnx's local cache is warm, subsequent
  runs are fast.

## Smoke-test results

- `python rl/single_agent/planning/smoke_test.py --variant v1_goal_only --total-timesteps 500` —
  PASSED (env sanity check, capped learn, eval rollout all completed without exception).
- `python rl/single_agent/planning/smoke_test.py --variant v6_goal_avoid_agents_avoid_static
  --total-timesteps 500` — PASSED.
- `get_variant()` error paths verified directly: `v4_goal_avoid_static_provided_kb` (status
  `pending_kb_infra`) raises `NotImplementedError`; an unknown name raises `ValueError` listing all
  8 valid options; `v1_goal_only` resolves to the expected `VariantConfig`.
- `pytest rl/single_agent/tests -v` — 15 passed (no regressions; pre-existing shapely
  `RuntimeWarning`s are unrelated to this change).

## Deferred / open items

- Variants 4/5/7/8 remain `status="pending_kb_infra"` until plan_02/plan_03 build the
  `KnowledgeBase` ABC and `DistanceBearingKB`.
- Variants 1/2/3/6 have only been smoke-tested (capped ~500-step runs), not actually trained to
  competence — that's plan_01's job (baseline comparison stats).

## Files touched

- `rl/single_agent/variant_registry.py` (new)
- `rl/common/agent_logic.py` (reward dispatch dict + docstring)
- `rl/single_agent/single_agent_training.py` (rewritten)
- `rl/single_agent/planning/smoke_test.py` (new)
- `.gitignore` (added checkpoint path)
- `rl/single_agent/planning/README.md`, `plan_00..04_*.md` (new)
