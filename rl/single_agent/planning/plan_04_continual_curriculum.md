# Plan 04 — Continual-learning curriculum (PackNet)

See [README.md](README.md) for the overall chain and conventions. Load
`knowledge_03_learned_kb_summary.md` before starting.

## Context

Separate from the 8 standalone variants (plans 00–03), this plan builds a **sequential**
curriculum: train goal-reaching first, continue training from those weights to add
avoid-other-agents, then continue again to add avoid-static-obstacles — using a continual-learning
method so later stages don't catastrophically forget earlier skills.

Decisions already made (see the top-level plan in `~/.claude/plans/for-single-agent-we-bubbly-lynx.md`
for full rationale):
- **PackNet first** (simplest mechanism: iterative magnitude pruning + frozen per-task masks).
  CompoNet and CKA-RL-proper are deferred to later, optional plan files reusing this same curriculum
  runner — see "Next" below.
- Continual learning uses a **separate, custom PyTorch PPO loop** adapted from `~/Dev/CKA-RL`,
  kept apart from the stable-baselines3 stack used for variants 1–8 — avoids hacking SB3 internals.
  `~/Dev/CKA-RL` is GPLv3; UrbanNav currently has no LICENSE file, so porting its masking logic is
  fine for this internal research repo, but worth remembering if the repo is ever published.
- Task sequence: Task A = `v1_goal_only` → Task B = `v2_goal_avoid_agents` → Task C =
  `v6_goal_avoid_agents_avoid_static` (all already defined/validated in plans 00–01's registry).
- PackNet reference implementation to port from: `~/Dev/CKA-RL/experiments/meta-world/models/
  packnet.py` (lines ~77-179) — the Meta-World variant targets continuous-action MLP
  actor-critics, the closer architectural match to UrbanNav's continuous `Box` action space, versus
  the Atari variant which is CNN/discrete-action.

## Phase A — implement

New `rl/single_agent/continual/` package:
1. `packnet_policy.py` — mask/freeze wrapper around an MLP actor-critic, ported from CKA-RL's
   Meta-World `packnet.py` masking mechanism: each task gets an allocated fraction of network
   weights via iterative magnitude-based pruning; a binary mask per task zeroes out
   weights not assigned to it; weights assigned to earlier tasks are frozen (no gradient updates)
   when training later tasks.
2. `custom_ppo.py` — minimal PPO loop, adapted/simplified from CKA-RL's Atari `run_ppo.py`
   structure (kept on PPO rather than switching to SAC, for consistency with the rest of this
   project's RL stack).
3. `curriculum_runner.py` — sequential task driver:
   - Train Task A fresh with `custom_ppo.py` + an unmasked `packnet_policy.py`.
   - Prune + freeze Task A's mask.
   - Train Task B from Task A's unpruned remainder.
   - Prune + freeze Task B's mask.
   - Train Task C from Task B's unpruned remainder.
   - Reuses the Task A/B/C envs already defined via `variant_registry.py` (plan_00/01) — no new
     env code, just a new training loop driving them.

## Phase B — smoke test

Run the full 3-stage sequence at capped step counts per stage (per the README's smoke-test
convention — a few hundred to a couple thousand steps per stage, not full training). Verify
directly (printed/logged state, not inferred): mask allocation per stage, weight freezing across
stage transitions, and that no stage corrupts the previous stage's frozen weights (e.g. compare
frozen-weight values before/after the next stage's training).

## Phase C — knowledge capture

`knowledge_04_continual_curriculum.md` — record the exact PackNet hyperparameters used (pruning
fraction per task, retraining schedule), any deviations from CKA-RL's reference implementation, and
smoke-test results. Then `knowledge_04_continual_curriculum_summary.md`.

## Next (future, optional, not built now)

CompoNet and CKA-RL-proper can be added later as `plan_05_continual_componet.md` /
`plan_06_continual_cka_rl.md`, swapping in a `componet_policy.py` / `cka_rl_policy.py` against the
same `curriculum_runner.py`. A future graph-based `KnowledgeBase` (see plan_02) could also combine
with this curriculum as a further task-sequence extension once it exists.
