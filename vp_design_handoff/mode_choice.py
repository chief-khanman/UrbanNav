"""
mode_choice.py
==============
"Demand responds to placement": how many travellers choose UAM over car for a
given vertiport selection, and the resulting region OD rate for the simulator.

Why
    With a fixed OD rate, a badly placed vertiport still receives the same
    trips, so the simulator cannot tell good from bad placements by demand.
    In transportation practice (and UAM placement studies such as Rath & Chow
    2022, Wu & Zhang 2021) the number of UAM users depends on how UAM compares
    with driving for each origin-destination (OD) pair.

How (binary logit, the standard discrete-choice model)
    GT  generalized time [min] = weighted travel time + money converted to
        minutes with VOT (value of time).
        GT_uam = w_a*(access+egress) + w_w*wait + terminals + flight + hold
                 + 2*transfer_penalty + (fare + access driving cost)/VOT
        GT_car = car time + car terminal time + (operating cost + toll + parking)/VOT
    P(UAM) = 1 / (1 + exp(beta * (GT_uam + uam_bias - GT_car)))
        beta      logit scale [1/min]          (ASSUMPTION)
        uam_bias  UAM penalty [min] (= negative ASC, alternative-specific constant)
    Captured trips  = OD trips x P(UAM)
    Consumer surplus (logsum) = (1/beta) * ln(1 + exp(-beta*delta)) [min/trip]:
        the standard appraisal measure of the benefit of adding UAM as an option.

The logit is non-linear, so it is evaluated per zone pair (not on region
averages), then summed to regions.

Loop with the simulator (RL environment, see PRE_PLAN S14):
    design step t:  selection_t -> evaluate(selection_t, air legs from step t-1)
                    -> captured_lambda -> simulator.update_lambda_matrix() -> run
                    -> new air legs (trip log) for step t+1
"""
from __future__ import annotations

from typing import Dict, Optional

import numpy as np

from urbannav.vp_design.assumptions import (M_PER_MILE, car_money_usd, money_to_minutes,
                                            uam_fare_usd)
from urbannav.vp_design.door_to_door import ZoneDoorToDoor


def _diag_fix(D: np.ndarray, factor: float, fill: float) -> np.ndarray:
    D = np.array(D, float)
    D[~np.isfinite(D)] = fill
    off = D + np.diag(np.full(len(D), np.inf))
    np.fill_diagonal(D, factor * off.min(axis=1))
    return D


class ModeChoiceModel:
    """Per-zone-pair UAM vs car choice for a vertiport selection.

    d2d       : ZoneDoorToDoor (zone OD q, car time T [min], regions, assumptions)
    car_dist_m: (Z, Z) car path length [m]  (zone_skims()['dist_m'])
    car_toll_usd: (Z, Z) toll along the car path [USD] (zone_skims()['toll_usd'])
    """

    def __init__(self, d2d: ZoneDoorToDoor, car_dist_m: np.ndarray, car_toll_usd: np.ndarray):
        self.d2d, self.A = d2d, d2d.A
        self.dist = _diag_fix(car_dist_m, self.A.intrazonal_factor, 0.0)    # [m]
        self.toll = np.nan_to_num(np.asarray(car_toll_usd, float))          # [USD]
        i, j = np.nonzero(d2d.q)
        keep = d2d.region[i] != d2d.region[j]                               # inter-region only
        self.i, self.j = i[keep], j[keep]
        self.ro, self.rd = d2d.region[self.i], d2d.region[self.j]
        self.q_veh = d2d.q[self.i, self.j]                                  # [trips/period]
        self.q = self.q_veh * self.A.vehicle_occupancy                      # [person trips/period]

    def evaluate(self, selection: Dict[int, int],
                 air: Optional[Dict[str, np.ndarray]] = None) -> Dict[str, object]:
        """air: {'wait_min','flight_min','holding_min'} (R, R) [min] or None (prior/estimate)."""
        d2d, A = self.d2d, self.A
        R = d2d.R
        k = d2d.selected_rows(selection)
        ko, kd = k[self.ro], k[self.rd]                       # vertiport rows per OD pair
        t_acc, t_egr = d2d.T[self.i, ko], d2d.T[kd, self.j]   # access / egress [min]
        d_acc, d_egr = self.dist[self.i, ko], self.dist[kd, self.j]   # [m]

        F_rs = d2d.estimate_flight_min(k)
        W_rs = np.full((R, R), A.prior_wait_min)
        H_rs = np.zeros((R, R))
        if air is not None:
            for key, arr in (("wait_min", W_rs), ("flight_min", F_rs), ("holding_min", H_rs)):
                if key in air:
                    m = np.isfinite(air[key])
                    arr[m] = air[key][m]
        W, F, H = W_rs[self.ro, self.rd], F_rs[self.ro, self.rd], H_rs[self.ro, self.rd]

        straight = np.hypot(*(d2d.zone_xy[ko] - d2d.zone_xy[kd]).T)          # [m]
        eligible = straight / 1000.0 >= A.min_uam_trip_km

        t_uam = (t_acc + A.terminal_time_origin_min + W + F + H
                 + A.terminal_time_dest_min + t_egr)                         # door-to-door [min]
        t_car = d2d.T[self.i, self.j] + A.car_terminal_time_min              # [min]
        access_cost = A.car_cost_per_mile_usd * (d_acc + d_egr) / M_PER_MILE # [USD]
        gt_uam = (A.access_time_weight * (t_acc + t_egr) + A.wait_time_weight * W
                  + A.terminal_time_origin_min + A.terminal_time_dest_min + F + H
                  + 2 * A.transfer_penalty_min
                  + money_to_minutes(uam_fare_usd(straight, A) + access_cost, A))   # [min]
        gt_car = t_car + money_to_minutes(
            car_money_usd(self.dist[self.i, self.j], self.toll[self.i, self.j], A), A)  # [min]

        b = A.logit_beta_per_min
        delta = gt_uam + A.uam_bias_min - gt_car                             # [min]
        p = np.where(eligible, 1.0 / (1.0 + np.exp(np.clip(b * delta, -50, 50))), 0.0)
        cs = np.where(eligible, np.logaddexp(0.0, -b * delta) / b, 0.0)      # [min/trip]

        qc = self.q * p                                                      # captured [trips/period]
        lam = np.zeros((R, R))
        np.add.at(lam, (self.ro, self.rd), qc)
        lam *= A.demand_scale / A.demand_period_min                          # [trips/min]

        tot = max(self.q.sum(), 1e-12)
        return {
            "captured_lambda": lam,                                          # [trips/min] (R, R)
            "eligible_trips": float(self.q[eligible].sum()),                 # [trips/period]
            "captured_trips": float(qc.sum()),                               # [trips/period]
            "captured_share": float(qc.sum() / tot),                         # [-]
            "consumer_surplus_usd": float(np.sum(self.q * cs) * A.vot_usd_per_hr / 60),  # [USD/period]
            "consumer_surplus_min_per_trip": float(np.sum(self.q * cs) / tot),            # [min/trip]
            "time_saved_person_h": float(np.sum(qc * (t_car - t_uam)) / 60),            # [h/period]
            "car_vmt_removed": float(np.sum(self.q_veh * p * self.dist[self.i, self.j])
                                     / M_PER_MILE),                          # VMT [vehicle-miles/period]
            "access_vmt_added": float(np.sum(qc * (d_acc + d_egr)) / M_PER_MILE),       # [vehicle-miles/period]
        }
