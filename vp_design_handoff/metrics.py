"""
metrics.py
==========
Static (map-only) passenger / community / equity metrics, and the collector
that gathers ALL raw (un-normalized) metrics for one vertiport selection.

Stakeholder groups (TCRP Report 88 performance-measurement perspectives):
    passenger  door-to-door quality      -> door_to_door.py, mode_choice.py, here
    operator   capacity / efficiency     -> simulator (DemandModelMixin.get_episode_metrics)
    community  safety, exposure, equity  -> simulator (NMAC) + here

"Static" = computable from the map and GMNS data alone (no simulation), so it
is cheap enough for every MCTS rollout.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np
import pandas as pd

from urbannav.vp_design.door_to_door import ZoneDoorToDoor
from urbannav.vp_design.mode_choice import ModeChoiceModel


def weighted_gini(x: np.ndarray, w: np.ndarray) -> float:
    """Gini coefficient [0..1] of x with weights w. 0 = everyone equal, 1 = maximal inequality.

    Used for equity of access time across zones (transit equity literature,
    e.g. Delbosc & Currie 2011 Lorenz/Gini).
    """
    o = np.argsort(x)
    x, w = x[o], w[o]
    cw, cxw = np.cumsum(w), np.cumsum(x * w)
    if cxw[-1] <= 0:
        return 0.0
    P = np.r_[0, cw / cw[-1]]
    L = np.r_[0, cxw / cxw[-1]]
    return float(1 - np.sum((P[1:] - P[:-1]) * (L[1:] + L[:-1])))


def weighted_quantile(x: np.ndarray, w: np.ndarray, q: float) -> float:
    o = np.argsort(x)
    cw = np.cumsum(w[o])
    return float(x[o][np.searchsorted(cw, q * cw[-1])])


def coverage_and_equity(d2d: ZoneDoorToDoor, selection: Dict[int, int],
                        zone_weight: pd.Series) -> Dict[str, float]:
    """Zone-level access to the region's vertiport, weighted by zone activity.

    coverage_share  share of activity within coverage_threshold_min of its vertiport [-]
                    (maximal covering location idea, Church & ReVelle 1974; TCQSM catchments)
    access_gini     inequality of access time across zones [-]
    access_p90_min  90th percentile access time [min]
    access_max_min  worst-zone access time [min] (p-center objective)
    """
    a = d2d.per_zone_access(selection).to_numpy()                       # [min]
    w = zone_weight.reindex(d2d.zone_ids).fillna(0.0).to_numpy()        # [trips]
    ok = np.isfinite(a) & (w > 0)
    a, w = a[ok], w[ok]
    return {
        "coverage_share": float(w[a <= d2d.A.coverage_threshold_min].sum() / w.sum()),
        "access_gini": weighted_gini(a, w),
        "access_p90_min": weighted_quantile(a, w, 0.9),
        "access_max_min": float(a.max()),
    }


def activity_near_vertiports(d2d: ZoneDoorToDoor, selection: Dict[int, int],
                             zone_weight: pd.Series) -> float:
    """Community exposure proxy [-]: share of zone activity whose centroid lies
    within noise_proxy_radius_m [m] of a selected vertiport.

    ASSUMPTION: stands in for a noise model (uam_sound.py is a stub) and for
    population (GMNS has none; trip ends are the activity proxy).
    """
    k = d2d.selected_rows(selection)
    vp = d2d.zone_xy[k]                                                  # [m]
    d = np.hypot(d2d.zone_xy[:, None, 0] - vp[None, :, 0],
                 d2d.zone_xy[:, None, 1] - vp[None, :, 1]).min(axis=1)  # nearest vertiport [m]
    w = zone_weight.reindex(d2d.zone_ids).fillna(0.0).to_numpy()
    return float(w[d <= d2d.A.noise_proxy_radius_m].sum() / max(w.sum(), 1e-12))


def collect_raw_metrics(d2d: ZoneDoorToDoor, mcm: ModeChoiceModel,
                        selection: Dict[int, int], zone_weight: pd.Series,
                        air: Optional[Dict[str, np.ndarray]] = None,
                        sim_metrics: Optional[Dict[str, float]] = None) -> Dict[str, float]:
    """All raw (un-normalized) metrics for one selection, flat dict with units in key names.

    air         region-pair air legs [min] from air_legs_from_trip_log (None before sim)
    sim_metrics scalar operator/safety metrics from the simulator, expected keys
                (after PRE_PLAN S03/S04): demand_served_ratio [-], pad_utilization_mean [-],
                fleet_utilization [-], nmac_per_flight_hour [encounters/h],
                uav_collisions [count], ra_collisions [count]
    """
    a = air or {}
    raw: Dict[str, float] = {}
    d = d2d.evaluate(selection, a.get("wait_min"), a.get("flight_min"), a.get("holding_min"))
    raw.update({k: v for k, v in d.items() if not isinstance(v, np.ndarray)})
    m = mcm.evaluate(selection, air)
    raw.update({k: v for k, v in m.items() if not isinstance(v, np.ndarray)})
    raw.update(coverage_and_equity(d2d, selection, zone_weight))
    raw["activity_near_vertiports_share"] = activity_near_vertiports(d2d, selection, zone_weight)
    raw.update(sim_metrics or {})
    raw["_captured_lambda"] = m["captured_lambda"]          # array, for the simulator (not a metric)
    return raw
