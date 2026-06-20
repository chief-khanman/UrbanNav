"""Shared per-agent RL glue used by both single-agent (Gymnasium) and
multi-agent (PettingZoo) UAM environments.

Each function takes the acting UAV's id explicitly rather than reading it off
`self`, so the same logic can be called once per agent from a multi-agent env's
step()/reset() loop, or once for the sole learning UAV in the single-agent env.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from gymnasium import spaces

from urbannav.component_schema import ActionType, UAVCommand, SimulatorState
from urbannav.utils import euclidean_distance

from rl.common.obs_space_definitions import get_obs_space

# PointMass -> (2,) [accel_norm, yaw_rate_norm]
# TwoDVector-Holonomic -> (2,) [ax_norm, ay_norm]
# SixDOF -> (4,) [ax_norm, ay_norm, az_norm, yaw_rate_norm]
_ACTION_SHAPE_MAP: Dict[str, int] = {
    'PointMass':             2,
    'TwoDVector-Holonomic':  2,
    'SixDOF':                4,
}


def create_action_space(dynamics_name: str) -> spaces.Box:
    """Build a normalized [-1, 1]^n action Box for the given dynamics model."""
    if dynamics_name not in _ACTION_SHAPE_MAP:
        raise ValueError(
            f"Unknown dynamics_name '{dynamics_name}'. "
            f"Valid options: {list(_ACTION_SHAPE_MAP.keys())}"
        )
    n = _ACTION_SHAPE_MAP[dynamics_name]
    return spaces.Box(
        low=np.full(n, -1.0, dtype=np.float32),
        high=np.full(n, 1.0, dtype=np.float32),
        dtype=np.float32,
    )


def create_observation_space(obs_type: str, n_intruder: int = 3) -> spaces.Box:
    """Thin re-export of get_obs_space, kept here so callers only need this module."""
    return get_obs_space(obs_type, n_intruder)


def extract_observation(
    state: SimulatorState,
    collisions: Optional[Tuple],
    uav_id: int,
    obs_type: str,
    n_intruder: int = 3,
) -> np.ndarray:
    """
    Build a flat float32 observation vector for the given UAV.
    Returns a zero-vector matching the obs_type's space shape if the UAV is no
    longer in atc_state.
    """
    uav = state.atc_state.get(uav_id, None)
    obs_space = get_obs_space(obs_type, n_intruder)
    if uav is None:
        return np.zeros(obs_space.shape, dtype=np.float32)

    dr = euclidean_distance(uav.mission_start_point, uav.mission_end_point)
    ms = uav.max_speed

    # ---- self obs (9 features) ----
    goal_x = uav.mission_end_point.x
    goal_y = uav.mission_end_point.y
    goal_z = uav.mission_end_point.z if uav.mission_end_point.has_z else 0.0

    self_obs: List[float] = [
        (goal_x - uav.current_position.x) / dr,
        (goal_y - uav.current_position.y) / dr,
        (goal_z - uav.pz) / dr,
        uav.vx / ms,
        uav.vy / ms,
        uav.vz / ms,
        math.cos(uav.current_heading),  # TODO: placeholder for final_heading - current_heading
        math.sin(uav.current_heading),
        uav.current_speed / ms,
    ]

    # ---- determine which blocks are needed ----
    needs_intruders = obs_type in (
        'AGENT-INTRUDER', 'AGENT-N-INTRUDER',
        'AGENT-INTRUDER-RA', 'AGENT-N-INTRUDER-RA',
    )
    needs_ra = obs_type in (
        'AGENT-INTRUDER-RA', 'AGENT-RA', 'AGENT-N-INTRUDER-RA',
    )
    n_slots = (
        1 if obs_type in ('AGENT-INTRUDER', 'AGENT-INTRUDER-RA')
        else n_intruder if needs_intruders
        else 0
    )

    # ---- intruder obs ----
    intruder_obs: List[float] = []

    if needs_intruders:
        detected_ids: set = (
            collisions[1].get(uav_id, set())
            if collisions is not None else set()
        )
        valid_ids = [i for i in detected_ids if i in state.atc_state]

        def _dist3d(iid: int) -> float:
            iv = state.atc_state[iid]
            return math.sqrt(
                (iv.current_position.x - uav.current_position.x) ** 2
                + (iv.current_position.y - uav.current_position.y) ** 2
                + (iv.pz - uav.pz) ** 2
            )

        sorted_ids = sorted(valid_ids, key=_dist3d)

        for intruder_id in sorted_ids[:n_slots]:
            iv = state.atc_state[intruder_id]
            intruder_obs += [
                (iv.current_position.x - uav.current_position.x) / dr,
                (iv.current_position.y - uav.current_position.y) / dr,
                (iv.pz - uav.pz) / dr,
                (iv.vx - uav.vx) / ms,
                (iv.vy - uav.vy) / ms,
                (iv.vz - uav.vz) / ms,
            ]

        filled = min(len(sorted_ids), n_slots)
        intruder_obs += [0.0] * (6 * (n_slots - filled))

    # ---- RA obs (1 feature) ----
    ra_obs: List[float] = []
    if needs_ra:
        ra_safe = (
            0.0
            if (collisions is not None and collisions[0].get(uav_id))
            else 1.0
        )
        ra_obs = [ra_safe]

    full_obs = self_obs + intruder_obs + ra_obs
    return np.array(full_obs, dtype=np.float32)


def format_action(
    action: np.ndarray,
    uav_id: int,
    dynamics_name: str,
    uav_dict: Dict[int, Any],
) -> Dict[int, List[UAVCommand]]:
    """
    Denormalize the [-1,1]^n policy action to physical units and wrap in a
    UAVCommandBundle entry for simulator_manager.map_actions_to_uavs().
    """
    uav = uav_dict[uav_id]
    max_a = uav.max_acceleration
    max_yr = uav.max_heading_change

    if dynamics_name == 'PointMass':
        payload = np.array([action[0] * max_a, action[1] * max_yr], dtype=np.float32)
    elif dynamics_name == 'TwoDVector-Holonomic':
        payload = np.array([action[0] * max_a, action[1] * max_a], dtype=np.float32)
    elif dynamics_name == 'SixDOF':
        payload = np.array(
            [action[0] * max_a, action[1] * max_a, action[2] * max_a, action[3] * max_yr],
            dtype=np.float32,
        )
    else:
        raise ValueError(f"Unknown dynamics_name '{dynamics_name}' in format_action.")

    return {uav_id: [UAVCommand(ActionType.CONTROL, payload)]}


def build_info(
    state: SimulatorState,
    collisions: Optional[Tuple],
    uav_id: Optional[int],
) -> Dict[str, Any]:
    """
    Build the info dict returned by reset() and step() for one agent.

    Keys:
        ra_detected      : RA sensor triggered for this UAV
        uav_detected     : another UAV detected within detection radius
        nmac             : NMAC threshold breached
        collision_ra     : actual collision with restricted airspace
        collision_uav    : actual collision with another UAV
        mission_complete : UAV reached goal vertiport
        dist_to_goal     : 3-D Euclidean distance to mission end point (metres)
        current_step     : simulator time step
    """
    uav = state.atc_state.get(uav_id) if uav_id is not None else None

    def _has(col_idx: int) -> bool:
        return bool(
            collisions is not None and uav_id is not None
            and collisions[col_idx].get(uav_id)
        )

    if uav is None:
        dist_3d = 0.0
    else:
        ex = uav.mission_end_point.x
        ey = uav.mission_end_point.y
        ez = uav.mission_end_point.z if uav.mission_end_point.has_z else 0.0
        dist_3d = math.sqrt(
            (uav.current_position.x - ex) ** 2
            + (uav.current_position.y - ey) ** 2
            + (uav.pz - ez) ** 2
        )

    return {
        'ra_detected':      _has(0),
        'uav_detected':     _has(1),
        'nmac':             _has(2),
        'collision_ra':     _has(3),
        'collision_uav':    _has(4),
        'mission_complete': bool(uav and uav.current_mission_complete_status),
        'dist_to_goal':     dist_3d,
        'current_step':     state.currentstep,
    }


def check_terminated(
    state: SimulatorState,
    collisions: Optional[Tuple],
    uav_id: Optional[int],
) -> bool:
    """
    Episode is done (terminated) for this agent when:
      - the UAV was removed from atc_state (collision system culled it)
      - UAV-UAV physical collision
      - RA physical collision
      - mission waypoint reached
    """
    if uav_id is None or uav_id not in state.atc_state:
        return True  # never started or culled

    if collisions is not None:
        if collisions[4].get(uav_id):   # uav-uav collision
            return True
        if collisions[3].get(uav_id):   # ra collision
            return True

    uav = state.atc_state[uav_id]
    if uav.current_mission_complete_status:  # reached goal
        return True

    return False


def compute_reward(
    state: SimulatorState,
    info: Dict[str, Any],
    uav_id: Optional[int],
    reward_type: str = 'r1',
) -> float:
    """
    Composable reward with four additive components.

    r1 — goal reaching:
        -normalised_dist_to_goal - 0.1*heading_err_normalised
        +10 bonus on mission complete
    r2 — speed incentive:
        +current_speed / max_speed
    r3 — intruder avoidance:
        -5 on NMAC; -0.5 when any UAV detected
    r4 — RA avoidance:
        -10 on RA collision; -0.5 when RA detected

    reward_type composition:
        'r1'       -> r1
        'r1r2'     -> r1 + r2
        'r1r2r3'   -> r1 + r2 + r3
        'r1r2r3r4' -> r1 + r2 + r3 + r4

    Terminal collision penalties (-100) are always applied on top, regardless
    of reward_type.
    """
    uav = state.atc_state.get(uav_id) if uav_id is not None else None

    # ---- r1: goal reaching ----
    if uav is not None:
        dist_norm = info['dist_to_goal'] / euclidean_distance(
            uav.mission_start_point, uav.mission_end_point
        )
        dx = uav.mission_end_point.x - uav.current_position.x
        dy = uav.mission_end_point.y - uav.current_position.y
        heading_to_goal = math.atan2(dy, dx)
        heading_err = abs(
            math.atan2(
                math.sin(uav.current_heading - heading_to_goal),
                math.cos(uav.current_heading - heading_to_goal),
            )
        ) / math.pi

        r1 = -dist_norm - 0.1 * heading_err
        if info['mission_complete']:
            r1 += 10.0
    else:
        r1 = 0.0

    # ---- r2: speed incentive ----
    r2 = (uav.current_speed / uav.max_speed) if uav is not None else 0.0

    # ---- r3: intruder avoidance ----
    if info['nmac']:
        r3 = -5.0
    elif info['uav_detected']:
        r3 = -0.5
    else:
        r3 = 0.0

    # ---- r4: RA avoidance ----
    if info['collision_ra']:
        r4 = -10.0
    elif info['ra_detected']:
        r4 = -0.5
    else:
        r4 = 0.0

    reward = {
        'r1':       r1,
        'r1r2':     r1 + r2,
        'r1r2r3':   r1 + r2 + r3,
        'r1r2r3r4': r1 + r2 + r3 + r4,
    }.get(reward_type, r1)

    if info['collision_uav']:
        reward -= 100.0
    if info['collision_ra']:
        reward -= 100.0

    return float(reward)
