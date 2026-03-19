"""
obs_space_definitions.py
========================
Observation space definitions for UAMSimEnv.

All obs spaces use normalised, bounded features so that a plain MLP policy
can train without feature-scale issues.

Self-obs layout (9 features — always the first block):
    [dx_goal/dr, dy_goal/dr, dz_goal/dr,   # normalised 3D goal delta
     vx/ms, vy/ms, vz/ms,                   # normalised velocity components
     cos(heading), sin(heading),             # heading as unit vector
     speed/ms]                               # normalised scalar speed
    where dr = detection_radius, ms = max_speed

Per-intruder obs (6 features each, zero-padded when fewer detected):
    [rel_x/dr, rel_y/dr, rel_z/dr,          # relative 3D position
     dvx/ms,  dvy/ms,  dvz/ms]              # relative velocity

RA obs (1 feature, appended last when present):
    1.0 = no RA detected (safe), 0.0 = RA within detection range (danger)
"""

from __future__ import annotations

from typing import List

import numpy as np
from gymnasium import spaces


# ---------------------------------------------------------------------------
# Global constants
# ---------------------------------------------------------------------------

# All valid observation space type strings.
# Pass one of these as obs_type to UAMSimEnv and get_obs_space().
OBS_SPACE: List[str] = [
    'AGENT',               # self only                   — shape (9,)
    'AGENT-INTRUDER',      # self + 1 closest intruder   — shape (15,)
    'AGENT-N-INTRUDER',    # self + N closest intruders  — shape (9 + 6·N,)
    'AGENT-INTRUDER-RA',   # self + 1 intruder + RA flag — shape (16,)
    'AGENT-RA',            # self + RA flag only         — shape (10,)
    'AGENT-N-INTRUDER-RA', # self + N intruders + RA     — shape (9 + 6·N + 1,)
]

# Supported policy feature extractor architectures.
# MLP is fully implemented.  GNN and RNN are stubs for future work.
POLICY_ARCH: List[str] = ['MLP', 'GNN', 'RNN']


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def _box(n: int) -> spaces.Box:
    """Return an unbounded float32 Box of shape (n,)."""
    return spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(n,),
        dtype=np.float32,
    )


# ---------------------------------------------------------------------------
# Per-type builder functions
# ---------------------------------------------------------------------------

def obs_space_agent() -> spaces.Box:
    """Self obs only — shape (9,)."""
    return _box(9)


def obs_space_agent_intruder() -> spaces.Box:
    """Self obs + 1 closest intruder — shape (15,)."""
    return _box(9 + 6)


def obs_space_agent_n_intruder(n: int) -> spaces.Box:
    """Self obs + N closest intruders — shape (9 + 6·N,)."""
    if n < 1:
        raise ValueError(f"n_intruder must be >= 1, got {n}")
    return _box(9 + 6 * n)


def obs_space_agent_intruder_ra() -> spaces.Box:
    """Self obs + 1 closest intruder + RA flag — shape (16,)."""
    return _box(9 + 6 + 1)


def obs_space_agent_ra() -> spaces.Box:
    """Self obs + RA flag only (no intruder block) — shape (10,)."""
    return _box(9 + 1)


def obs_space_agent_n_intruder_ra(n: int) -> spaces.Box:
    """Self obs + N closest intruders + RA flag — shape (9 + 6·N + 1,)."""
    if n < 1:
        raise ValueError(f"n_intruder must be >= 1, got {n}")
    return _box(9 + 6 * n + 1)


# ---------------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------------

def get_obs_space(
    obs_type: str,
    n_intruder: int = 3,
    policy_arch: str = 'MLP',
) -> spaces.Box:
    """
    Return the Gymnasium observation space for the given obs_type.

    Args:
        obs_type:    One of the strings in OBS_SPACE.
        n_intruder:  Number of intruder slots for 'AGENT-N-INTRUDER*' types.
                     Ignored for non-N types.
        policy_arch: 'MLP' returns a flat Box (default).
                     'GNN' and 'RNN' are stubs — raise NotImplementedError
                     until graph/sequence feature extractors are implemented.

    Returns:
        gymnasium.spaces.Box matching the selected obs_type + policy_arch.

    Raises:
        ValueError          — unknown obs_type.
        NotImplementedError — policy_arch is 'GNN' or 'RNN' (not yet implemented).
    """
    if policy_arch not in POLICY_ARCH:
        raise ValueError(
            f"Unknown policy_arch '{policy_arch}'. Valid options: {POLICY_ARCH}"
        )

    if policy_arch in ('GNN', 'RNN'):
        # TODO: GNN — build node/edge feature tensors from intruder + RA data
        # TODO: RNN — build a sequence of intruder observations ordered by distance
        raise NotImplementedError(
            f"policy_arch='{policy_arch}' is not yet implemented. "
            "Use policy_arch='MLP' for now."
        )

    # MLP path: flat Box for all obs types
    dispatch = {
        'AGENT':               obs_space_agent,
        'AGENT-INTRUDER':      obs_space_agent_intruder,
        'AGENT-N-INTRUDER':    lambda: obs_space_agent_n_intruder(n_intruder),
        'AGENT-INTRUDER-RA':   obs_space_agent_intruder_ra,
        'AGENT-RA':            obs_space_agent_ra,
        'AGENT-N-INTRUDER-RA': lambda: obs_space_agent_n_intruder_ra(n_intruder),
    }

    if obs_type not in dispatch:
        raise ValueError(
            f"Unknown obs_type '{obs_type}'. Valid options: {OBS_SPACE}"
        )

    return dispatch[obs_type]()
