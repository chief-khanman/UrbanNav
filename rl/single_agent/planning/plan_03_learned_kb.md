# Plan 03 — Learned knowledge base (variants 5, 8)

See [README.md](README.md) for the overall chain and conventions. Load
`knowledge_02_provided_kb_summary.md` before starting — it records the `KnowledgeBase` ABC method
signatures this plan implements against; read the full `knowledge_02_provided_kb.md` if any detail
is missing from the summary.

## Context

Variants 5 and 8 use the same `KnowledgeBase` ABC and `DistanceBearingKB` class from plan_02, but
in `mode='learned'`: the agent starts each episode with no static-obstacle knowledge and builds it
up only from what it actually detects, persisting that knowledge for the rest of the episode even
if it moves back out of detection range.

## Phase A — implement

1. `DistanceBearingKB(mode='learned')`: `reset()` clears to empty (no known RA zones); `observe
   (detection_event)` is what actually populates it — each time the sensor's RA-detection flag
   fires for a zone, record that zone's distance/bearing-relevant info into the KB's internal
   state. Once observed, a zone's knowledge persists (doesn't get cleared just because the UAV
   moved away) — only `reset()` (new episode) clears it.
2. Wire `UAMSimEnv` to own one `KnowledgeBase` instance per learning UAV, constructed via
   `KB_CLASS_MAP[kb_type](mode=kb_mode)` from the variant's registry entry:
   - Call `.reset()` in `UAMSimEnv.reset()`.
   - Call `.observe()` in `.step()`, fed from the existing RA-detection flag already computed each
     step (no new sensor work — this just routes existing detection output to the KB).
   - Thread the KB instance into `agent_logic.extract_observation()` in place of the provided-mode
     ground-truth read plan_02 used.
3. No new `OBS_SPACE` entries needed — reuse `AGENT-RA-KB`/`AGENT-N-INTRUDER-RA-KB` from plan_02,
   just backed by a `mode='learned'` KB instance instead of `mode='provided'`.
4. Update `variant_registry.py`: flip `v5_goal_avoid_static_learned_kb` and
   `v8_goal_avoid_agents_avoid_static_learned_kb` from `status="pending_kb_infra"` to `"ready"`.

## Phase B — smoke test

`smoke_test.py --variant v5_goal_avoid_static_learned_kb` and
`--variant v8_goal_avoid_agents_avoid_static_learned_kb`. Specifically verify the "unknown until
first detection" behavior directly (e.g. print/log the KB's feature output at episode start vs.
after the first RA detection event, don't just infer it from reward numbers). Confirm
`pytest rl/single_agent/tests -v` still passes.

## Phase C — knowledge capture

`knowledge_03_learned_kb.md` — this completes all 8 standalone variants; include a short
recap/index of all 8 (`v1`–`v8`) with their final `status` and where each is implemented, since
plan_04 (and anyone resuming this project later) will want that map without re-deriving it. Then
`knowledge_03_learned_kb_summary.md`.

## Next

Open [plan_04_continual_curriculum.md](plan_04_continual_curriculum.md) in a new chat.
