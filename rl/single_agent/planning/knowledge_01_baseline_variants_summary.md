# Summary — Plan 01: Policy architectures + baseline variants

- Built `rl/single_agent/policies/`: `graph_utils.py` (obs slicing, zero-padding-based real-
  intruder mask, ego-centric star-graph construction — no learnable params), `gat_encoder.py`
  (shared `EgoIntruderGATEncoder`, GATConv+residual+LayerNorm, ego-readout not mean-pool),
  `rnn_extractor.py` (GRU/LSTM, farthest-first→nearest-last per GA3C-CADRL), `gnn_extractor.py`,
  `gnn_rnn_extractor.py` (per-frame GNN + temporal RNN over a `VecFrameStack` window).
- `--policy-arch {MLP,RNN,GNN,GNN_RNN}` / `--rnn-type {gru,lstm}` / `--temporal-window` are CLI
  flags on `single_agent_training.py` + `smoke_test.py`, **not** new `VariantConfig` fields.
  `build_policy_kwargs()` hard-errors (no silent fallback) if a non-MLP arch targets a variant
  with no intruder block.
- No new dependency: `torch-geometric` was already pinned; temporal recurrence uses SB3's
  `VecFrameStack`, not `sb3_contrib`.
- Simplified `obs_space_definitions.get_obs_space()` — dropped the dead `policy_arch` param and
  `NotImplementedError` branch; obs shape is now arch-agnostic.
- All 14 planned (variant × arch) smoke-test combos passed; `pytest rl/single_agent/tests -v` —
  52 passed (15 existing + 37 new extractor shape tests), no regressions.
- Full comparison table and design-decision rationale in `knowledge_01_baseline_variants.md` —
  read that before touching `rl/single_agent/policies/`.
- Next: `plan_02_provided_kb.md`, unaffected in scope; may reuse `graph_utils.build_star_graph`'s
  seam later but doesn't have to.
