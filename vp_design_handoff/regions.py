"""
regions.py
==========
Zones -> regions, and the region-level OD rate matrix the simulator uses.

A region is a group of zone cells. The agent (GNN-RL or MCTS) picks ONE zone
per region; the vertiport is placed at that zone's centroid.

Two ways to define regions
    1. k-means on zone centroids (x, y in airspace CRS [m]); the user chooses k
       (e.g. 5 or 50). Re-implements the lost "Band 1" code that produced
       band1_output_{5,50}_region.
    2. Region file written by a person (e.g. a transportation planner):

           # comment lines start with '#'
           region 0 == [1, 2, 3, 17]
           region 1 == [4, 5, 6]

       Region numbers must be 0..R-1 (the simulator indexes regions this way).
       Every zone must appear in exactly one region.

Outputs
    zone_region_map.csv   zone_id, region_id     (read by UAMSimulator)
    regions.geojson       union of zone cells per region (plots, features)
    region_lambda.npy     lambda[r, s]: OD rate region r -> region s [trips/min]

lambda (Greek letter) = arrival rate of trip requests, here per region pair,
in trips per minute; it is the rate of the Poisson process the demand model draws from.
"""
from __future__ import annotations

import re
from typing import Iterable, Optional

import geopandas as gpd
import numpy as np
import pandas as pd

from urbannav.vp_design.assumptions import Assumptions

_REGION_LINE = re.compile(r"^\s*region\s+(\d+)\s*==\s*\[([^\]]*)\]\s*$")


# =============================================================================
# 1. k-means regions
# =============================================================================
def kmeans_regions(zones: pd.DataFrame, k: int, seed: int = 0) -> pd.Series:
    """Cluster zone centroids (x, y [m]) into k regions.

    Region ids are relabelled by cluster-centre position (west->east, then
    south->north) so the same k and data give the same numbering every run.
    Returns Series zone_id -> region_id (0..k-1).
    """
    from sklearn.cluster import KMeans
    xy = zones[["x", "y"]].to_numpy()
    km = KMeans(n_clusters=k, random_state=seed, n_init=10).fit(xy)
    order = np.lexsort((km.cluster_centers_[:, 1], km.cluster_centers_[:, 0]))
    relabel = np.empty(k, dtype=int)
    relabel[order] = np.arange(k)
    return pd.Series(relabel[km.labels_], index=zones["zone_id"].to_numpy(),
                     name="region_id").rename_axis("zone_id")


# =============================================================================
# 2. Region file
# =============================================================================
def parse_region_file(path: str) -> pd.Series:
    """Read 'region <r> == [<zone>, <zone>, ...]' lines -> Series zone_id -> region_id."""
    rows = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            s = line.split("#", 1)[0].strip()
            if not s:
                continue
            m = _REGION_LINE.match(s)
            if not m:
                raise ValueError(f"{path}:{n}: expected 'region <r> == [z1, z2, ...]', got {line!r}")
            r = int(m.group(1))
            zones = [int(t) for t in m.group(2).replace(" ", "").split(",") if t]
            rows += [(z, r) for z in zones]
    df = pd.DataFrame(rows, columns=["zone_id", "region_id"])
    dup = df["zone_id"][df["zone_id"].duplicated()].unique()
    if len(dup):
        raise ValueError(f"zones listed in more than one region: {sorted(dup.tolist())}")
    return df.set_index("zone_id")["region_id"]


def write_region_file(zone_region: pd.Series, path: str) -> None:
    """Write a Series zone_id -> region_id in region-file format (editable starting point)."""
    with open(path, "w") as f:
        f.write("# region <r> == [zone ids]; regions numbered 0..R-1\n")
        for r, grp in zone_region.groupby(zone_region):
            f.write(f"region {int(r)} == [{', '.join(str(int(z)) for z in sorted(grp.index))}]\n")


def validate_zone_region(zone_region: pd.Series, zone_ids: Iterable[int]) -> int:
    """Check every zone is in exactly one region and regions are 0..R-1. Returns R."""
    zone_ids = set(int(z) for z in zone_ids)
    listed = set(int(z) for z in zone_region.index)
    if listed - zone_ids:
        raise ValueError(f"unknown zone ids in region definition: {sorted(listed - zone_ids)}")
    if zone_ids - listed:
        raise ValueError(f"zones missing from region definition: {sorted(zone_ids - listed)}")
    regs = sorted(set(int(r) for r in zone_region.values))
    if regs != list(range(len(regs))):
        raise ValueError(f"regions must be numbered 0..R-1 without gaps, got {regs}")
    return len(regs)


def write_zone_region_map(zone_region: pd.Series, path: str) -> None:
    """CSV with columns zone_id, region_id (format read by UAMSimulator)."""
    zone_region.rename("region_id").rename_axis("zone_id").reset_index().to_csv(path, index=False)


def read_zone_region_map(path: str) -> pd.Series:
    df = pd.read_csv(path)
    return pd.Series(df.iloc[:, 1].astype(int).to_numpy(),
                     index=df.iloc[:, 0].astype(int).to_numpy(), name="region_id")


# =============================================================================
# 3. Region geometry and region OD rate
# =============================================================================
def region_polygons(cells: gpd.GeoDataFrame, zone_region: pd.Series) -> gpd.GeoDataFrame:
    """Dissolve zone cells into one polygon per region."""
    c = cells.copy()
    c["region_id"] = c["zone_id"].map(zone_region)
    return c.dissolve(by="region_id", as_index=False)[["region_id", "geometry"]]


def region_od_rate(demand: pd.DataFrame, zone_region: pd.Series, A: Assumptions,
                   n_regions: Optional[int] = None) -> np.ndarray:
    """lambda[r, s]: region OD rate [trips/min] for the simulator's demand model.

    lambda = sum of zone volumes r->s x vehicle_occupancy x demand_scale / demand_period_min
    The diagonal (trips inside one region) is set to 0: origin and destination
    vertiport would be the same, so these trips are not served by UAM.
    """
    R = n_regions or int(zone_region.max()) + 1
    ro = demand["o_zone_id"].map(zone_region)
    rd = demand["d_zone_id"].map(zone_region)
    ok = ro.notna() & rd.notna()
    lam = np.zeros((R, R))
    np.add.at(lam, (ro[ok].astype(int).to_numpy(), rd[ok].astype(int).to_numpy()),
              demand.loc[ok, "volume"].to_numpy(float))
    lam *= A.vehicle_occupancy * A.demand_scale / A.demand_period_min   # [trips/min]
    np.fill_diagonal(lam, 0.0)
    return lam
