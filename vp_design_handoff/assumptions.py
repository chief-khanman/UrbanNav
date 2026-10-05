"""
assumptions.py
==============
Every parameter the GMNS Plus dataset (node.csv, link.csv, demand.csv) CANNOT
supply lives here, in ONE registry. Nothing else in urbannav.vp_design
hard-codes an assumed value.

Each entry records
    value         number used now
    unit          unit of `value` (see docs/vp_design/GLOSSARY.md)
    low / high    defensible range -> one-at-a-time sensitivity analysis
    rationale     why this value / source to cite
    replace_with  data that would replace the assumption later

Paper workflow
    A = Assumptions()
    A.export("assumptions_table.md")       # appendix table
    for A_i in A.sweep("uam_bias_min"):    # sensitivity runs
        ...

Entries marked PLACEHOLDER are order-of-magnitude values; verify against the
cited source for the study year/country before publishing.

Phase 1 note: a zone IS a vertiport candidate (vertiport at the zone centroid
node), so ground access/egress time comes straight from the GMNS zone-to-zone
car travel-time table; no building snapping is needed (that is Phase 2).
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, Iterator, Optional

import numpy as np
import pandas as pd


@dataclass(frozen=True)
class Assumption:
    value: object            # value used now
    unit: str                # unit of value
    low: Optional[float]     # sensitivity range lower bound (None = not numeric)
    high: Optional[float]    # sensitivity range upper bound
    rationale: str           # why / source
    replace_with: str        # data that would replace this assumption


DEFAULTS: Dict[str, Assumption] = {
    # ------------------------------------------------------------------ demand
    "demand_period_min": Assumption(
        60.0, "min", 60, 180,
        "ASSUMPTION. demand.csv stores a trip volume but not the time period it "
        "covers. Assumed one peak hour: lambda [trips/min] = volume / 60.",
        "Demand period from the GMNS Plus city metadata (DTALite settings)."),
    "vehicle_occupancy": Assumption(
        1.0, "persons/vehicle", 1.0, 1.7,
        "ASSUMPTION. Treats demand.csv volume as person trips. If volume is "
        "vehicle trips, person trips = volume x occupancy.",
        "Occupancy by trip purpose from a household travel survey (NHTS, US)."),
    "demand_scale": Assumption(
        1.0, "-", 0.001, 1.0,
        "ASSUMPTION. Global multiplier applied to the region OD rate fed to the "
        "simulator so demand fits the simulated fleet. 1.0 = full GMNS demand.",
        "Fleet size / market-share scenario from an operator or SP survey."),
    "min_uam_trip_km": Assumption(
        0.0, "km", 0.0, 15.0,
        "ASSUMPTION. Optional eligibility filter on straight-line vertiport "
        "distance; 0 = rely only on the inter-region rule.",
        "Distance distribution of observed air-taxi / helicopter trips."),

    # ----------------------------------------------------------- value of time
    "vot_usd_per_hr": Assumption(
        18.0, "USD/h", 12.0, 30.0,
        "PLACEHOLDER. VOT (value of time). USDOT guidance values personal local "
        "travel at ~50% of median hourly household income; outside the US use "
        "0.5 x local hourly wage.",
        "USDOT Departmental Guidance on Valuation of Travel Time (latest "
        "revision) or the national appraisal guidance of the study country."),
    "access_time_weight": Assumption(
        1.5, "x in-vehicle time", 1.0, 2.0,
        "Out-of-vehicle (access/egress) minutes feel longer than in-vehicle "
        "minutes; transit meta-analyses (e.g. Wardman 2004) find ~1.5-2.",
        "Calibrated weight from a UAM stated-preference (SP) survey."),
    "wait_time_weight": Assumption(
        2.0, "x in-vehicle time", 1.5, 2.5,
        "Waiting is weighted ~1.5-2.5x in-vehicle time in transit demand models.",
        "Calibrated weight from a UAM SP survey."),
    "transfer_penalty_min": Assumption(
        5.0, "min per transfer", 0.0, 15.0,
        "Fixed penalty per mode change (car->air, air->car); standard in transit "
        "assignment. A UAM trip always has two transfers.",
        "Calibrated transfer penalty from SP / revealed-preference data."),

    # ---------------------------------------------------------- UAM operations
    "terminal_time_origin_min": Assumption(
        5.0, "min", 3.0, 10.0,
        "ASSUMPTION. Check-in / boarding at the origin vertiport. Not modelled "
        "by GMNS or the simulator.",
        "Vertiport operator procedures, observed helicopter-shuttle times."),
    "terminal_time_dest_min": Assumption(
        3.0, "min", 1.0, 5.0,
        "ASSUMPTION. Deplaning / exiting the destination vertiport.",
        "Vertiport operator procedures."),
    "prior_wait_min": Assumption(
        5.0, "min", 0.0, 15.0,
        "ASSUMPTION. Pre-simulation guess of passenger wait (enqueue -> depart). "
        "Replaced by simulator output wherever a region pair has completed trips.",
        "Simulator trip log (depart_step - enqueue_step)."),
    "prior_cruise_speed_mps": Assumption(
        55.0, "m/s", 20.0, 70.0,
        "ASSUMPTION. Pre-simulation flight-time estimate only. Keep equal to "
        "UAV_TYPE_REGISTRY['STANDARD'].max_speed after the UAV-speed section. "
        "Replaced by simulated flight time when available.",
        "Simulator trip log (arrive_airspace_step - depart_step)."),
    "air_detour_factor": Assumption(
        1.15, "-", 1.0, 1.4,
        "ASSUMPTION. Air paths are longer than straight lines (corridors, "
        "restricted areas).",
        "Simulated path length / straight-line distance."),
    "vertical_phase_min": Assumption(
        2.0, "min", 1.0, 4.0,
        "ASSUMPTION. Take-off + climb + descent + landing time added to cruise.",
        "Aircraft performance model / simulator."),

    # ------------------------------------------------------------------- money
    "uam_fare_base_usd": Assumption(
        5.0, "USD/trip", 0.0, 20.0,
        "PLACEHOLDER base fare.", "Operator fare data."),
    "uam_fare_per_mile_usd": Assumption(
        3.0, "USD/passenger-mile", 0.44, 5.73,
        "Uber Elevate (2016) estimated ~5.7 USD/passenger-mile near term and "
        "<0.5 long term; default is a mid scenario.", "Operator fare data."),
    "car_cost_per_mile_usd": Assumption(
        0.25, "USD/mile", 0.15, 0.40,
        "PLACEHOLDER variable vehicle operating cost (fuel, maintenance, tyres); "
        "ownership cost excluded (short-run choice). AAA 'Your Driving Costs' "
        "order of magnitude.", "AAA or national vehicle operating cost data."),
    "parking_cost_usd": Assumption(
        0.0, "USD/trip", 0.0, 20.0,
        "ASSUMPTION. No parking data in GMNS.", "Parking price by zone."),
    "car_terminal_time_min": Assumption(
        3.0, "min", 0.0, 10.0,
        "ASSUMPTION. Walk to/from parked car + parking search; not in GMNS.",
        "Parking search studies / surveys by zone."),

    # ------------------------------------------------------------ mode choice
    "logit_beta_per_min": Assumption(
        0.05, "1/min", 0.02, 0.10,
        "ASSUMPTION. Scale of generalized-time utility in the binary logit "
        "(UAM vs car); order of magnitude of urban mode-choice models.",
        "Estimated from a UAM SP survey (e.g. Munich UAM SP studies)."),
    "uam_bias_min": Assumption(
        15.0, "min", 0.0, 30.0,
        "ASSUMPTION. UAM penalty in equivalent minutes (novelty, safety "
        "perception, weather risk) = negative ASC (alternative-specific constant).",
        "ASC from a UAM SP survey."),

    # ------------------------------------------------------- access / policy
    "intrazonal_factor": Assumption(
        0.5, "-", 0.3, 1.0,
        "Access time when the trip starts in the vertiport's own zone = factor x "
        "car time to the nearest other zone (standard intrazonal rule; the "
        "travel-time table diagonal is 0 because centroid connectors have length 0).",
        "Zone area / mean intra-zone trip length."),
    "coverage_threshold_min": Assumption(
        15.0, "min", 10.0, 20.0,
        "Policy threshold for a 'covered' zone; analogous to TCQSM walk "
        "catchments (0.25 mi bus, 0.5 mi rail) but for drive access.",
        "Planning-agency standard."),
    "noise_proxy_radius_m": Assumption(
        1000.0, "m", 500.0, 2000.0,
        "ASSUMPTION. Radius around a selected vertiport used for the community "
        "exposure proxy (no noise model yet: uam_sound.py is a stub).",
        "Noise contours from a UAM noise model."),
    "unreachable_min": Assumption(
        180.0, "min", 60.0, 300.0,
        "Travel time substituted for zone pairs with no road path.",
        "Fix network connectivity."),
    "population_proxy": Assumption(
        "trip_ends", "-", None, None,
        "ASSUMPTION. GMNS has no population column. Zone weight = trips produced "
        "+ trips attracted (activity proxy).",
        "Census / WorldPop gridded population joined to zone cells."),
    "bpr_volume_field": Assumption(
        "ref_volume", "-", None, None,
        "Link volume used for congested times (BPR). obs_volume is all 0 in the "
        "Austin GMNS Plus data, so ref_volume is used.",
        "Observed counts or a traffic assignment run."),
}


class Assumptions:
    """Registry wrapper: A.vot_usd_per_hr -> 18.0."""

    def __init__(self, overrides: Optional[Dict[str, object]] = None):
        self._a: Dict[str, Assumption] = dict(DEFAULTS)
        for k, v in (overrides or {}).items():
            if k not in self._a:
                raise KeyError(f"Unknown assumption '{k}'")
            self._a[k] = replace(self._a[k], value=v)

    def __getattr__(self, name):
        a = self.__dict__.get("_a")
        if a is not None and name in a:
            return a[name].value
        raise AttributeError(name)

    def with_(self, **overrides) -> "Assumptions":
        cur = {k: v.value for k, v in self._a.items()}
        cur.update(overrides)
        return Assumptions(cur)

    def to_frame(self) -> pd.DataFrame:
        fields = ("value", "unit", "low", "high", "rationale", "replace_with")
        return pd.DataFrame([{"name": k, **{f: getattr(v, f) for f in fields}}
                             for k, v in self._a.items()])

    def export(self, path: str) -> None:
        df = self.to_frame()
        if path.endswith(".md"):
            with open(path, "w") as f:
                f.write(df.to_markdown(index=False))
        else:
            df.to_csv(path, index=False)

    def sweep(self, name: str, n: int = 5) -> Iterator["Assumptions"]:
        """One-at-a-time sensitivity: copies with `name` swept low -> high."""
        a = self._a[name]
        if a.low is None or a.high is None:
            raise ValueError(f"'{name}' has no numeric range")
        for v in np.linspace(a.low, a.high, n):
            yield self.with_(**{name: float(v)})


# =============================================================================
# Approximation helpers (use registry values only)
# =============================================================================
M_PER_MILE = 1609.344   # metres per mile [m/mile]


def estimate_flight_time_min(straight_m, A: Assumptions):
    """Pre-simulation flight time [min] from straight-line vertiport distance [m]."""
    path_m = np.asarray(straight_m, float) * A.air_detour_factor      # air path length [m]
    return A.vertical_phase_min + path_m / A.prior_cruise_speed_mps / 60.0


def uam_fare_usd(straight_m, A: Assumptions):
    """UAM fare [USD/trip] = base + per-mile x straight-line miles."""
    return A.uam_fare_base_usd + A.uam_fare_per_mile_usd * np.asarray(straight_m) / M_PER_MILE


def car_money_usd(dist_m, toll_usd, A: Assumptions):
    """Car out-of-pocket cost [USD/trip] = operating cost + toll + parking."""
    return (A.car_cost_per_mile_usd * np.asarray(dist_m) / M_PER_MILE
            + np.asarray(toll_usd) + A.parking_cost_usd)


def money_to_minutes(usd, A: Assumptions):
    """Convert money [USD] to equivalent time [min] using VOT."""
    return np.asarray(usd) / A.vot_usd_per_hr * 60.0


def population_proxy_trip_ends(demand: pd.DataFrame) -> pd.Series:
    """Zone weight [trips] = productions + attractions (activity proxy)."""
    p = demand.groupby("o_zone_id")["volume"].sum()      # trips produced per zone [trips/period]
    a = demand.groupby("d_zone_id")["volume"].sum()      # trips attracted per zone [trips/period]
    return p.add(a, fill_value=0.0).rename("weight")
