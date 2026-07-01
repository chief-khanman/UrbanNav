# Single-agent RL planning index

Tracks the multi-session build-out of 8 single-agent reward variants plus a continual-learning
curriculum on top of them. Each step is its own plan file, implemented and smoke-tested before the
next is opened — open the next plan file in a **new chat** (fresh context window) once a step's
knowledge files exist.

## Naming convention

- `plan_NN_<name>.md` — implementation plan for that step (Context, Phase A/B/C, Next).
- `knowledge_NN_<name>.md` — detailed findings written after implementation + smoke test: decisions
  made (and any deviations from the plan), gotchas hit, exact files/configs touched, smoke-test
  results, deferred items.
- `knowledge_NN_<name>_summary.md` — condensed version (<30 lines). Load this first at the start of
  the next plan file's session; fall back to the full `knowledge_NN_<name>.md` only if more detail
  is needed.

## Chain

| # | Plan | Status | Knowledge |
|---|------|--------|-----------|
| 00 | [plan_00_shared_foundation.md](plan_00_shared_foundation.md) — variant registry, reward dispatch, training script, smoke-test runner | done | [knowledge_00_shared_foundation.md](knowledge_00_shared_foundation.md) / [summary](knowledge_00_shared_foundation_summary.md) |
| 01 | [plan_01_baseline_variants.md](plan_01_baseline_variants.md) — policy architectures (MLP/RNN/GNN/GNN+RNN feature extractors for modeling other agents) + variants 1/2/3/6 validated across all applicable architectures | done | [knowledge_01_baseline_variants.md](knowledge_01_baseline_variants.md) / [summary](knowledge_01_baseline_variants_summary.md) |
| 02 | [plan_02_provided_kb.md](plan_02_provided_kb.md) — `KnowledgeBase` ABC + `DistanceBearingKB(mode='provided')`, variants 4/7 | not started | — |
| 03 | [plan_03_learned_kb.md](plan_03_learned_kb.md) — `DistanceBearingKB(mode='learned')`, variants 5/8 | not started | — |
| 04 | [plan_04_continual_curriculum.md](plan_04_continual_curriculum.md) — PackNet 3-stage curriculum (goal → avoid-agents → avoid-static) | not started | — |

Future, not yet scheduled: `plan_05_continual_componet.md`, `plan_06_continual_cka_rl.md` (same
curriculum runner from plan_04, swap the policy-wrapping method), and a future graph-based
`KnowledgeBase` implementation (see "Knowledge-base abstraction" in plan_02) reusing the same ABC.

## Design decisions that apply across the whole chain

- Other-agent policy architecture (`MLP`/`RNN`/`GNN`/`GNN_RNN`, plan_01) is a CLI flag on
  `single_agent_training.py`/`smoke_test.py`, **not** a `VariantConfig` field — orthogonal to the
  reward/obs variant axis, same precedent as `--n-intruder`. `rl/single_agent/policies/
  graph_utils.py`'s `build_star_graph()` is the seam any future graph-based `KnowledgeBase`
  (plan_02/03) would extend with KB nodes/edges.
- Static-obstacle knowledge base is built behind a `KnowledgeBase(ABC)` (introduced in plan_02) so
  the initial distance/bearing-vector representation can later be swapped for a graph-based one
  without rewriting the registry/lifecycle/sensor-wiring layer — only the obs encoding and feature
  extractor are representation-specific. See plan_02 for the full rationale.
- Continual learning (plan_04+) uses a separate custom PyTorch PPO loop adapted from
  `~/Dev/CKA-RL`, kept apart from the stable-baselines3 stack used for variants 1–8 — avoids hacking
  SB3 internals. PackNet first; CompoNet/CKA-RL-proper deferred.
- Every plan's smoke test is a capped short run (env sanity check, then a few hundred–few thousand
  timesteps), never a full training session — the goal is to catch wiring bugs, not produce a
  usable policy.
- `pytest rl/single_agent/tests -v` must keep passing after each plan's Phase A.
