"""Registry of the 8 single-agent reward/obs variants described in
rl/single_agent/planning/. Mirrors the repo's existing pluggable-component
pattern (e.g. DYNAMICS_CLASS_MAP in dynamics_engine.py): a config string
resolves to a concrete (obs_type, reward_type, ...) configuration instead of
each variant getting its own forked training script.

obs_type values come from rl.common.obs_space_definitions.OBS_SPACE.
reward_type values come from rl.common.agent_logic.compute_reward's dispatch.
"Goal-reaching" always includes the speed term (r2) — reaching the goal in
the shortest time is part of the goal objective, not a separate variant axis
— so every entry below pairs r1 with r2.
kb_type/kb_mode are placeholders for the knowledge-base variants (4/5/7/8),
wired up in rl/single_agent/planning/plan_02_provided_kb.md and
plan_03_learned_kb.md — until then those entries carry
status='pending_kb_infra' and cannot be trained.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class VariantConfig:
    name: str
    obs_type: str
    reward_type: str
    kb_type: Optional[str] = None
    kb_mode: Optional[str] = None
    status: str = "ready"


VARIANT_CONFIG_MAP: Dict[str, VariantConfig] = {
    "v1_goal_only": VariantConfig(
        name="v1_goal_only",
        obs_type="AGENT",
        reward_type="r1r2",
    ),
    "v2_goal_avoid_agents": VariantConfig(
        name="v2_goal_avoid_agents",
        obs_type="AGENT-N-INTRUDER",
        reward_type="r1r2r3",
    ),
    "v3_goal_avoid_static": VariantConfig(
        name="v3_goal_avoid_static",
        obs_type="AGENT-RA",
        reward_type="r1r2r4",
    ),
    "v4_goal_avoid_static_provided_kb": VariantConfig(
        name="v4_goal_avoid_static_provided_kb",
        obs_type="AGENT-RA-KB",
        reward_type="r1r2r4",
        kb_type="distance_bearing",
        kb_mode="provided",
        status="pending_kb_infra",
    ),
    "v5_goal_avoid_static_learned_kb": VariantConfig(
        name="v5_goal_avoid_static_learned_kb",
        obs_type="AGENT-RA-KB",
        reward_type="r1r2r4",
        kb_type="distance_bearing",
        kb_mode="learned",
        status="pending_kb_infra",
    ),
    "v6_goal_avoid_agents_avoid_static": VariantConfig(
        name="v6_goal_avoid_agents_avoid_static",
        obs_type="AGENT-N-INTRUDER-RA",
        reward_type="r1r2r3r4",
    ),
    "v7_goal_avoid_agents_avoid_static_provided_kb": VariantConfig(
        name="v7_goal_avoid_agents_avoid_static_provided_kb",
        obs_type="AGENT-N-INTRUDER-RA-KB",
        reward_type="r1r2r3r4",
        kb_type="distance_bearing",
        kb_mode="provided",
        status="pending_kb_infra",
    ),
    "v8_goal_avoid_agents_avoid_static_learned_kb": VariantConfig(
        name="v8_goal_avoid_agents_avoid_static_learned_kb",
        obs_type="AGENT-N-INTRUDER-RA-KB",
        reward_type="r1r2r3r4",
        kb_type="distance_bearing",
        kb_mode="learned",
        status="pending_kb_infra",
    ),
}


def get_variant(name: str) -> VariantConfig:
    """Look up a variant by name, raising if it's unknown or not yet trainable."""
    if name not in VARIANT_CONFIG_MAP:
        raise ValueError(
            f"Unknown variant '{name}'. Valid options: {sorted(VARIANT_CONFIG_MAP)}"
        )
    variant = VARIANT_CONFIG_MAP[name]
    if variant.status != "ready":
        raise NotImplementedError(
            f"Variant '{name}' has status='{variant.status}' and cannot be trained yet "
            "(see rl/single_agent/planning/ for the plan that implements it)."
        )
    return variant
