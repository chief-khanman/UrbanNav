"""
normalization.py
================
Raw metrics -> generalization block (normalizing metrics) -> score / reward.

Why normalize: a policy trained on Austin must work on another city. Raw values
(minutes, trip counts, NMAC counts) scale with city size, demand level and
fleet size, so the same "good" design gives very different numbers in
different cities. Each raw metric is turned into a dimensionless value in
[0, 1] where 1 = better, using ratios to a city reference (car travel time),
shares, and rates. The score is a weighted mean of these.

Used identically by
    GNN-RL  reward_t = score_t - score_{t-1}   ('improvement', dense, matches the
                                                current distance-improvement reward)
              or     = score_t                 ('absolute')
    MCTS    leaf value = score in [0, 1]       (UCT needs bounded values)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np

DEFAULT_WEIGHTS: Dict[str, float] = {
    # higher-is-better normalized terms (weights are design choices; tune per study)
    "n_door_to_door": 1.0,
    "n_access": 0.5,
    "n_captured": 1.0,
    "n_served": 0.5,
    "n_coverage": 0.5,
    "n_equity": 0.25,
    "n_safety": 0.5,
    "n_community": 0.25,
    # penalty term (subtracted)
    "p_pad_overload": 0.5,
}


@dataclass
class CityReference:
    """City-specific reference values computed once per city (no selection needed).

    mean_car_min           demand-weighted mean car time of inter-region trips [min]
                           (ZoneDoorToDoor: sum(Q*CAR)/sum(Q))
    ref_nmac_per_flight_h  NMAC rate treated as 'bad' (score 0.5) [encounters/h]  ASSUMPTION
    pad_util_ok            pad utilization above which overload penalty starts [-]  ASSUMPTION
    """
    mean_car_min: float
    ref_nmac_per_flight_h: float = 1.0
    pad_util_ok: float = 0.85

    @staticmethod
    def from_d2d(d2d) -> "CityReference":
        Q = d2d.Q
        ok = (Q > 0) & np.isfinite(d2d.CAR)
        return CityReference(mean_car_min=float(np.sum(Q[ok] * d2d.CAR[ok]) / Q[ok].sum()))


def _get(raw, key, default=np.nan):
    v = raw.get(key, default)
    return default if v is None else float(v)


def normalize_metrics(raw: Dict[str, float], ref: CityReference) -> Dict[str, float]:
    """Raw metrics (units in names) -> dimensionless terms in [0, 1]."""
    #### generalization block (normalizing metrics) ####
    n: Dict[str, float] = {}
    # passenger: door-to-door time relative to driving; 1/(1+ratio): ratio 0.5 -> 0.67, 1 -> 0.5
    n["n_door_to_door"] = 1.0 / (1.0 + _get(raw, "mean_door_to_door_min") / ref.mean_car_min)
    # passenger: access+egress burden relative to driving
    n["n_access"] = 1.0 / (1.0 + (_get(raw, "mean_access_min") + _get(raw, "mean_egress_min"))
                           / ref.mean_car_min)
    # demand: share of eligible trips that choose UAM (already a share)
    n["n_captured"] = _get(raw, "captured_share", 0.0)
    # operator: share of generated requests that were served (already a share)
    n["n_served"] = _get(raw, "demand_served_ratio", np.nan)
    # equity / coverage (already shares; Gini 0 = equal)
    n["n_coverage"] = _get(raw, "coverage_share", 0.0)
    n["n_equity"] = 1.0 - _get(raw, "access_gini", 1.0)
    # safety: NMAC encounter rate relative to a reference rate
    n["n_safety"] = 1.0 / (1.0 + _get(raw, "nmac_per_flight_hour", np.nan)
                           / ref.ref_nmac_per_flight_h)
    # community exposure proxy: less activity next to vertiports is better
    n["n_community"] = 1.0 - _get(raw, "activity_near_vertiports_share", 0.0)
    # penalty: pad utilization above the comfortable level (queues grow without bound near 1)
    util = _get(raw, "pad_utilization_mean", np.nan)
    n["p_pad_overload"] = (float(np.clip((util - ref.pad_util_ok) / (1 - ref.pad_util_ok), 0, 1))
                           if np.isfinite(util) else np.nan)
    #### end generalization block ####
    return n


def score(normalized: Dict[str, float], weights: Optional[Dict[str, float]] = None) -> float:
    """Weighted mean of available terms in [0, 1]; NaN terms (not yet measured) are skipped."""
    w = weights or DEFAULT_WEIGHTS
    pos = [(w[k], v) for k, v in normalized.items() if not k.startswith("p_")
           and k in w and np.isfinite(v)]
    pen = [(w[k], v) for k, v in normalized.items() if k.startswith("p_")
           and k in w and np.isfinite(v)]
    s = sum(wk * v for wk, v in pos) / max(sum(wk for wk, _ in pos), 1e-12)
    s -= sum(wk * v for wk, v in pen) / max(sum(wk for wk, _ in pos), 1e-12)
    return float(np.clip(s, 0.0, 1.0))


def gnn_rl_reward(score_t: float, score_prev: Optional[float], mode: str = "improvement") -> float:
    """GNN-RL reward per design step."""
    if mode == "absolute" or score_prev is None:
        return score_t
    return score_t - score_prev


def mcts_value(score_t: float) -> float:
    """MCTS leaf value in [0, 1]."""
    return float(np.clip(score_t, 0.0, 1.0))
