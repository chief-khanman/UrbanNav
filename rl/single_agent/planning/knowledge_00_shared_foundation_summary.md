# Summary — Plan 00: Shared foundation

- Built: `rl/single_agent/variant_registry.py` (`VARIANT_CONFIG_MAP`, `get_variant()`), one new
  reward-dispatch key in `agent_logic.py`, a real CLI in `single_agent_training.py`
  (`build_env()` + SB3 checkpoint/eval callbacks), and `rl/single_agent/planning/smoke_test.py`.
- **Correction made during implementation:** "goal" always includes the speed term `r2` — only
  `r1r2r4` needed adding (not `r1r3`/`r1r4`/`r1r3r4` as the original plan said). Full reward-type
  mapping for all 8 variants is in `knowledge_00_shared_foundation.md`'s table — read that table,
  don't assume from variant names.
- Variants 1/2/3/6 = `status="ready"`. Variants 4/5/7/8 = `status="pending_kb_infra"` (built in
  plan_02/plan_03).
- Smoke tests passed for `v1_goal_only` and `v6_goal_avoid_agents_avoid_static`;
  `pytest rl/single_agent/tests -v` — 15 passed, no regressions.
- Checkpoints write to `rl/single_agent/checkpoints/<variant>/` (gitignored).
- No other gotchas. Next: plan_01_baseline_variants.md (pure validation, no new code).
