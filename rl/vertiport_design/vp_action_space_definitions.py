"""
vp_action_space_definitions.py
===============================
Action space definitions for VertiportDesignEnv.

Mirrors the OBS_SPACE / get_obs_space pattern in
rl/common/obs_space_definitions.py.
"""

from __future__ import annotations

from typing import Dict, List

from gymnasium import spaces

from urbannav.airspace import Airspace

# All valid action space type strings.
VP_ACTION_SPACE: List[str] = [
    'PER_REGION_DISCRETE',  # one discrete pick per region — implemented
    'SUBSET_SELECTION',     # combinatorial subset pick — NotImplementedError stub
]


def action_space_per_region_discrete(airspace: Airspace) -> spaces.MultiDiscrete:
    """
    One discrete choice per region: index into that region's candidate
    vertiport list. Shape (n_regions,).

    airspace.regions_dict must already be populated (call
    airspace.make_regions_dict(...) first).
    """
    if not hasattr(airspace, 'regions_dict'):
        raise RuntimeError(
            'Cannot build PER_REGION_DISCRETE action space: airspace.regions_dict '
            'is not set. Call airspace.make_regions_dict(...) first.'
        )
    return spaces.MultiDiscrete(
        [len(vp_list) for vp_list in airspace.regions_dict.values()]
    )


def get_vp_action_space(action_type: str, airspace: Airspace) -> spaces.Space:
    """
    Return the Gymnasium action space for the given action_type.

    Args:
        action_type: One of the strings in VP_ACTION_SPACE.
        airspace:    Airspace instance with regions_dict already populated.

    Raises:
        ValueError          — unknown action_type.
        NotImplementedError — action_type='SUBSET_SELECTION' (not yet implemented).
    """
    if action_type == 'PER_REGION_DISCRETE':
        return action_space_per_region_discrete(airspace)

    if action_type == 'SUBSET_SELECTION':
        raise NotImplementedError(
            "action_type='SUBSET_SELECTION' is not yet implemented. "
            "Use action_type='PER_REGION_DISCRETE' for now."
        )

    raise ValueError(
        f"Unknown action_type '{action_type}'. Valid options: {VP_ACTION_SPACE}"
    )
