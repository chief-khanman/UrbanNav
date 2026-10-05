r"""
door_to_door.py
===============
Door-to-door UAM travel time for a selection {region_id: zone_id}.

Phase 1: the vertiport of region r sits at the centroid of the selected zone
k_r, so ground access/egress times are entries of the GMNS zone-to-zone car
travel-time table t(i, j) [min].

A trip from zone i (region r) to zone j (region s) takes

    T_ij = t(i, k_r) + tau_o + W_rs + F_rs + H_rs + tau_d + t(k_s, j)      [min]
           \_access_/          \__ simulator __/          \_egress_/

    tau_o, tau_d  origin / destination terminal time (ASSUMPTION, assumptions.py)
    W_rs  passenger wait   = depart_step - enqueue_step         (simulator)
    F_rs  flight time      = arrive_airspace_step - depart_step (simulator)
    H_rs  landing hold     = land_step - arrive_airspace_step   (simulator)

Why the zone-level OD table is needed: the region OD table only says how many
trips go r -> s, not which zones they start in, so it cannot weight t(i, k_r).
Because all trips of region r share one vertiport, the demand-weighted mean
over all zone pairs r -> s splits exactly into three terms:

    mean T_rs = ACC[k_r, s] + (tau_o + W + F + H + tau_d) + EGR[k_s, r]

    ACC[k, s] = sum_{i in r} P[i, s] t(i, k) / Q[r, s],  P[i, s] = trips zone i -> region s
    EGR[m, r] = sum_{j in s} A[j, r] t(m, j) / Q[r, s],  A[j, r] = trips region r -> zone j

ACC/EGR are (n_zones x n_regions) tables built once per city; evaluating a
selection is a table lookup (~0.1 ms). Verified against brute force per OD pair.
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np
import pandas as pd

from urbannav.vp_design.assumptions import Assumptions, estimate_flight_time_min


class ZoneDoorToDoor:
    """Precomputed access / egress tables for zone-as-vertiport design.

    car_time_min : (Z, Z) car travel time [min], zones in `zone_ids` order
                   (gmns_io.RoadNetwork.zone_skims()['time_min']).
    zone_ids     : (Z,) zone ids (sorted).
    zone_region  : Series zone_id -> region_id (0..R-1).
    demand       : GMNS demand.csv rows (o_zone_id, d_zone_id, volume [trips/period]).
    zone_xy      : (Z, 2) zone centroids in airspace CRS [m] (pre-sim flight estimate).
    """

    def __init__(self, car_time_min: np.ndarray, zone_ids: np.ndarray,
                 zone_region: pd.Series, demand: pd.DataFrame, zone_xy: np.ndarray,
                 A: Optional[Assumptions] = None):
        self.A = A or Assumptions()
        self.zone_ids = np.asarray(zone_ids)
        Z = len(self.zone_ids)
        self.zpos = pd.Series(np.arange(Z), index=self.zone_ids)          # zone_id -> row index
        reg = zone_region.reindex(self.zone_ids)
        if reg.isna().any():
            raise ValueError(f"{int(reg.isna().sum())} zones have no region")
        self.region = reg.to_numpy().astype(int)                           # region of each zone
        self.R = int(self.region.max()) + 1                                # number of regions
        self.zone_xy = np.asarray(zone_xy, float)                          # [m]

        # ---- car travel time with intrazonal rule [min] ----
        T = np.array(car_time_min, dtype=float)
        T[~np.isfinite(T)] = self.A.unreachable_min
        off = T + np.diag(np.full(Z, np.inf))
        np.fill_diagonal(T, self.A.intrazonal_factor * off.min(axis=1))
        self.T = T

        # ---- zone OD matrix q[i, j] [trips/period] ----
        D = demand[["o_zone_id", "d_zone_id", "volume"]].copy()
        D["i"] = D["o_zone_id"].map(self.zpos)
        D["j"] = D["d_zone_id"].map(self.zpos)
        dropped = D["i"].isna() | D["j"].isna()
        self.dropped_volume = float(D.loc[dropped, "volume"].sum())       # trips with unknown zones
        D = D[~dropped]
        q = np.zeros((Z, Z))
        np.add.at(q, (D["i"].astype(int), D["j"].astype(int)), D["volume"].to_numpy(float))
        self.q = q

        M = np.zeros((Z, self.R))
        M[np.arange(Z), self.region] = 1.0           # zone -> region membership [0/1]
        P = q @ M                                    # P[i, s] trips zone i -> region s
        Aj = q.T @ M                                 # Aj[j, r] trips region r -> zone j
        self.Q = M.T @ q @ M                         # Q[r, s] region OD [trips/period]
        np.fill_diagonal(self.Q, 0.0)                # intra-region trips excluded

        same = self.region[:, None] == self.region[None, :]
        Tin = np.where(same, T, 0.0)                 # only zones of the same region
        with np.errstate(invalid="ignore", divide="ignore"):
            self.ACC = (Tin.T @ P) / self.Q[self.region, :]        # (Z, R) [min]
            self.EGR = (Tin @ Aj) / self.Q[:, self.region].T       # (Z, R) [min]
            self.CAR = (M.T @ (q * T) @ M) / self.Q                # (R, R) mean car time [min]

    # ------------------------------------------------------------------
    def candidates(self, region: int) -> np.ndarray:
        """Zone ids of a region, sorted = action index order."""
        return np.sort(self.zone_ids[self.region == region])

    def action_to_selection(self, action) -> Dict[int, int]:
        """MultiDiscrete action (one index per region) -> {region_id: zone_id}."""
        return {r: int(self.candidates(r)[a]) for r, a in enumerate(action)}

    def selected_rows(self, selection: Dict[int, int]) -> np.ndarray:
        return np.array([self.zpos[selection[r]] for r in range(self.R)])

    def estimate_flight_min(self, k: np.ndarray) -> np.ndarray:
        """Pre-simulation flight time between selected zones (R x R) [min]."""
        xy = self.zone_xy[k]
        d = np.hypot(xy[:, None, 0] - xy[None, :, 0], xy[:, None, 1] - xy[None, :, 1])   # [m]
        return estimate_flight_time_min(d, self.A)

    # ------------------------------------------------------------------
    def evaluate(self, selection: Dict[int, int],
                 wait_min: Optional[np.ndarray] = None,
                 flight_min: Optional[np.ndarray] = None,
                 holding_min: Optional[np.ndarray] = None) -> Dict[str, object]:
        """Door-to-door metrics for {region_id: zone_id}.

        wait/flight/holding: (R, R) [min] from the simulator (air_legs_from_trip_log);
        NaN entries (no completed trip) fall back to assumptions / estimate.
        """
        R, A = self.R, self.A
        k = self.selected_rows(selection)
        acc = self.ACC[k, :]                   # acc[r, s] access time [min]
        egr = self.EGR[k, :].T                 # egr[r, s] = EGR[k_s, r] egress time [min]

        def fill(x, fallback):
            if x is None:
                return fallback
            x = np.asarray(x, float).copy()
            m = ~np.isfinite(x)
            x[m] = fallback[m]
            return x

        W = fill(wait_min, np.full((R, R), A.prior_wait_min))       # passenger wait [min]
        F = fill(flight_min, self.estimate_flight_min(k))            # flight [min]
        H = fill(holding_min, np.zeros((R, R)))                      # landing hold [min]
        air = A.terminal_time_origin_min + W + F + H + A.terminal_time_dest_min
        T_rs = acc + air + egr                                       # door-to-door [min]
        valid = (self.Q > 0) & np.isfinite(T_rs)
        Qv = np.where(valid, self.Q, 0.0)
        tot = max(Qv.sum(), 1e-12)

        def wm(x):
            return float(np.sum(np.where(valid, x, 0.0) * Qv) / tot)

        return {
            "T_rs_min": np.where(valid, T_rs, np.nan),              # [min] per region pair
            "mean_door_to_door_min": wm(T_rs),                      # [min]
            "mean_access_min": wm(acc),                             # [min]
            "mean_egress_min": wm(egr),                             # [min]
            "mean_wait_min": wm(W),                                 # [min]
            "mean_flight_min": wm(F),                               # [min]
            "mean_holding_min": wm(H),                              # [min]
            "mean_air_part_min": wm(air),                           # [min]
            "mean_car_min": wm(self.CAR),                           # [min]
            "uam_car_time_ratio": wm(T_rs) / max(wm(self.CAR), 1e-12),   # [-]
            "share_trips_uam_faster": float(np.sum(Qv * (T_rs < self.CAR)) / tot),  # [-]
            "access_egress_share": wm(acc + egr) / max(wm(T_rs), 1e-12),            # [-]
        }

    def per_zone_access(self, selection: Dict[int, int]) -> pd.Series:
        """Car access time [min] from every zone to its own region's vertiport."""
        k = self.selected_rows(selection)
        return pd.Series(self.T[np.arange(len(self.zone_ids)), k[self.region]],
                         index=self.zone_ids, name="access_min")


# ----------------------------------------------------------------------
def air_legs_from_trip_log(trip_log, n_regions: int, dt_s: float, warmup_steps: int = 0):
    """(R, R) mean passenger wait / flight / landing hold [min] from DemandModelMixin._trip_log.

    wait    = depart_step - enqueue_step
    flight  = arrive_airspace_step - depart_step
    holding = land_step - arrive_airspace_step
    dt_s    simulator time step [s]; warmup_steps: trips enqueued earlier are ignored.
    Only completed trips are logged -> waits are biased low when queues saturate;
    always report demand_served_ratio alongside.
    """
    s = {k: np.zeros((n_regions, n_regions)) for k in ("wait", "flight", "holding")}
    n = np.zeros((n_regions, n_regions))
    f = dt_s / 60.0                                    # steps -> minutes
    for e in trip_log:
        if e.get("land_step") is None or e["enqueue_step"] < warmup_steps:
            continue
        o, d = e["o_region"], e["d_region"]
        s["wait"][o, d] += (e["depart_step"] - e["enqueue_step"]) * f
        s["flight"][o, d] += (e["arrive_airspace_step"] - e["depart_step"]) * f
        s["holding"][o, d] += (e["land_step"] - e["arrive_airspace_step"]) * f
        n[o, d] += 1
    with np.errstate(invalid="ignore", divide="ignore"):
        return {f"{k}_min": np.where(n > 0, v / n, np.nan) for k, v in s.items()}
