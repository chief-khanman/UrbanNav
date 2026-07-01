# Plan 00 — Shared foundation

See [README.md](README.md) for the overall chain and conventions.

## Context

`rl/single_agent/single_agent_training.py` is currently a skeleton (PPO via stable-baselines3,
mostly commented-out wiring). `rl/common/agent_logic.py` already implements four reward
combinations (`r1`, `r1r2`, `r1r2r3`, `r1r2r3r4` — goal, +speed, +avoid-agents, +avoid-static-RA;
`agent_logic.py:257-339`, dispatch at `:328-332`) over six obs-space variants in
`rl/common/obs_space_definitions.py:38-45`. Before building any of the 8 reward variants, the
training script and reward dispatch need to be config-driven (one registry, not 8 forked copies of
training code), and a reusable smoke-test runner needs to exist since every later plan in this
chain depends on it.

## Phase A — implement

1. **`rl/single_agent/variant_registry.py`** — new file. A `VARIANT_CONFIG_MAP` dict (mirrors the
   repo's existing `*_CLASS_MAP` pattern, e.g. `DYNAMICS_CLASS_MAP`), with all 8 variants
   pre-declared by name:

   | name | obs_type | reward_type | kb_type | kb_mode | status |
   |---|---|---|---|---|---|
   | `v1_goal_only` | `AGENT` | `r1r2` | `None` | `None` | `ready` |
   | `v2_goal_avoid_agents` | `AGENT-N-INTRUDER` | `r1r2r3` | `None` | `None` | `ready` |
   | `v3_goal_avoid_static` | `AGENT-RA` | `r1r2r4` | `None` | `None` | `ready` |
   | `v4_goal_avoid_static_provided_kb` | `AGENT-RA-KB` | `r1r2r4` | `distance_bearing` | `provided` | `pending_kb_infra` |
   | `v5_goal_avoid_static_learned_kb` | `AGENT-RA-KB` | `r1r2r4` | `distance_bearing` | `learned` | `pending_kb_infra` |
   | `v6_goal_avoid_agents_avoid_static` | `AGENT-N-INTRUDER-RA` | `r1r2r3r4` | `None` | `None` | `ready` |
   | `v7_goal_avoid_agents_avoid_static_provided_kb` | `AGENT-N-INTRUDER-RA-KB` | `r1r2r3r4` | `distance_bearing` | `provided` | `pending_kb_infra` |
   | `v8_goal_avoid_agents_avoid_static_learned_kb` | `AGENT-N-INTRUDER-RA-KB` | `r1r2r3r4` | `distance_bearing` | `learned` | `pending_kb_infra` |

   Note "goal" always pairs with `r2` (speed): reaching the goal in the shortest time is part of
   the goal objective, not a separate axis — so every reward_type above includes `r2`.

   `obs_type` for the KB variants (`AGENT-RA-KB`, `AGENT-N-INTRUDER-RA-KB`) does not exist in
   `OBS_SPACE` yet — that's fine, those rows stay `pending_kb_infra` until plan_02. A lookup
   function `get_variant(name)` should raise a clear error if asked to actually train a
   `pending_kb_infra` variant.

2. **Extend `agent_logic.compute_reward`'s dispatch dict** (`agent_logic.py:328-332`) with one new
   key: `r1r2r4` (= `r1 + r2 + r4`, goal + avoid-static, no avoid-agents term). The other three
   needed combinations (`r1r2`, `r1r2r3`, `r1r2r3r4`) already existed.

3. **Harden `rl/single_agent/single_agent_training.py`**:
   - CLI args (`argparse` or similar): `--variant` (looked up in `VARIANT_CONFIG_MAP`),
     `--total-timesteps`, `--n-intruder`, `--seed`.
   - Checkpoint dir: `rl/single_agent/checkpoints/<variant_name>/` — add this path to `.gitignore`
     alongside the existing `/tensorboard_logs/` entry.
   - Wire SB3 `CheckpointCallback` and `EvalCallback` so runs are resumable/evaluable, not just a
     bare `.learn()` call.

4. **`rl/single_agent/planning/smoke_test.py`** — generic `--variant <name>` runner, reused
   unmodified by plan_01/02/03:
   - Step 1: build the env for the variant, call `reset()`/`step()` a handful of times, assert obs
     shape matches `OBS_SPACE[obs_type]` and reward/termination types are sane.
   - Step 2: a capped `PPO(...).learn(total_timesteps=<small>)` call (a few hundred to a couple
     thousand steps — see README's smoke-test convention) — must complete without exception.
   - Step 3: a short evaluation rollout, print mission-complete/collision flags so a human can
     eyeball sanity.

## Phase B — smoke test

Run `python rl/single_agent/planning/smoke_test.py --variant v1_goal_only` and
`--variant v6_goal_avoid_agents_avoid_static` (representative: simplest variant, and the most
reward-term-dense `ready` variant). Confirm `pytest rl/single_agent/tests -v` still passes.

## Phase C — knowledge capture

Write `knowledge_00_shared_foundation.md` covering: final registry shape (any fields added/changed
from the table above), the exact new dispatch keys, any SB3/Gymnasium API gotchas hit while wiring
callbacks, and smoke-test output. Then `knowledge_00_shared_foundation_summary.md` (<30 lines).

## Next

Continue directly into [plan_01_baseline_variants.md](plan_01_baseline_variants.md) — no new infra
needed there, low risk, fine to do in the same chat. Open in a fresh chat instead if you'd rather
split here.
