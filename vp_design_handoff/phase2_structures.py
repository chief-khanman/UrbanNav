"""
phase2_structures.py  (PARKED - not used in Phase 1)
=====================================================
Phase 2: vertiport candidates become OSM structures (buildings, offices, ...)
INSIDE each zone cell; the agent picks one structure per zone/region.

Independent of how cells are made: cells could be Voronoi (zones.py) or any
other tessellation. Changing the tessellation only changes which buildings
fall in each cell; the "one structure per cell" design is unchanged.

Moved here from the earlier gmns_metrics.py (snap / vertiport_skims) and from
Austin_data_view.ipynb (building centroids per zone cell). Phase 1 never
imports this module.
"""
from __future__ import annotations

from typing import List, Tuple

import geopandas as gpd
import numpy as np
import pandas as pd


def building_candidates_per_cell(place: str, cells: gpd.GeoDataFrame,
                                 tag_value_pairs: List[Tuple[str, str]]) -> gpd.GeoDataFrame:
    """OSM building centroids assigned to the zone cell they fall in.

    tag_value_pairs e.g. [('building','commercial'), ('building','office')].
    Returns GeoDataFrame(osmid, tag, value, zone_id, geometry) in cells.crs.
    Note (notebook finding): some zones have zero candidate buildings; Phase 2
    must decide how to treat them (fallback to the zone centroid, or exclude).
    """
    import osmnx as ox
    frames = []
    for tag, value in tag_value_pairs:
        b = ox.features_from_place(place, tags={tag: [value]}).reset_index()
        cent = b.geometry.to_crs(cells.crs).centroid
        frames.append(gpd.GeoDataFrame({"osmid": b["id"], "tag": tag, "value": value},
                                       geometry=cent, crs=cells.crs))
    allb = pd.concat(frames, ignore_index=True)
    return gpd.sjoin(allb, cells[["zone_id", "geometry"]], how="inner",
                     predicate="within").drop(columns="index_right")


def snap_points_to_network(net, xy: np.ndarray, crs) -> pd.DataFrame:
    """Snap points [airspace CRS, m] to the nearest connected GMNS road node.

    Returns node_idx and snap_dist_m [m] (straight line from point to node).
    """
    from pyproj import Transformer
    from scipy.spatial import cKDTree
    tf = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    x, y = tf.transform(net.nodes["x_coord"].to_numpy(), net.nodes["y_coord"].to_numpy())
    ok = np.flatnonzero(net.connected_mask)
    d, i = cKDTree(np.c_[x[ok], y[ok]]).query(xy)
    return pd.DataFrame({"node_idx": ok[i], "snap_dist_m": d})


def candidate_access_egress(net, cand_node_idx: np.ndarray, zone_node_idx: np.ndarray,
                            snap_dist_m: np.ndarray, walk_speed_mps: float = 1.3):
    """Access (zone -> candidate) and egress (candidate -> zone) car time [min] and
    distance [m] for structure candidates; snap distance added as walking time."""
    egr_t, egr_d, _ = net.sssp(cand_node_idx, zone_node_idx)
    acc_t, acc_d, _ = net.sssp(cand_node_idx, zone_node_idx, reverse=True)
    snap_min = (snap_dist_m / walk_speed_mps / 60)[:, None]
    return {"access_min": acc_t + snap_min, "access_m": acc_d,
            "egress_min": egr_t + snap_min, "egress_m": egr_d}
