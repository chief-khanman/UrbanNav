"""
zones.py
========
GMNS Plus zones -> airspace coordinates -> Voronoi zone cells -> reconciliation
with the OSMnx airspace boundary.

Definitions (Phase 1)
    zone          a GMNS Plus traffic analysis zone (TAZ). Austin: 199 zones,
                  centroid nodes node_id 1..199.
    zone centroid the zone's GMNS node (lon/lat). Phase 1: the vertiport
                  candidate sits exactly here (zone == vertiport candidate).
    zone cell     Voronoi cell of the zone centroid: every point of the map that
                  is closer to this centroid than to any other centroid. Cells
                  are only geometry for regions / plots / Phase 2 building
                  candidates; travel times still come from the GMNS network.
    airspace CRS  the UTM projection of Airspace.location_utm_gdf [m].

Re-implements the Voronoi part of Austin_data_view.ipynb with two changes:
    * cells are computed in the projected airspace CRS [m], not in degrees,
      so cell shapes are not distorted;
    * shapely.voronoi_polygons replaces geovoronoi (one dependency fewer).

Airspace extension (decision D2 = extend): some Austin zone centroids lie
outside the OSMnx city boundary. The airspace boundary becomes
    union(city boundary, convex hull of zone centroids buffered by hull_buffer_m)
so every zone and all its demand stay in the problem.
"""
from __future__ import annotations

from typing import Dict, Optional

import geopandas as gpd
import numpy as np
import pandas as pd
import shapely
from shapely.geometry import MultiPoint, Point
from shapely.geometry.base import BaseGeometry


def project_zones(zone_nodes: pd.DataFrame, crs) -> pd.DataFrame:
    """Add projected coordinates x, y [m] in the airspace CRS to zone centroids.

    zone_nodes: output of RoadNetwork.zone_nodes() (zone_id, node_id, node_idx, lon, lat).
    """
    from pyproj import Transformer
    tf = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    x, y = tf.transform(zone_nodes["lon"].to_numpy(), zone_nodes["lat"].to_numpy())
    out = zone_nodes.copy()
    out["x"] = x       # easting in airspace CRS [m]
    out["y"] = y       # northing in airspace CRS [m]
    return out


def extended_airspace_boundary(city_boundary: BaseGeometry, zones: pd.DataFrame,
                               hull_buffer_m: float = 2000.0) -> BaseGeometry:
    """Airspace boundary [airspace CRS] = city boundary U buffered hull of zone centroids.

    city_boundary: Airspace.location_utm_gdf.geometry.iloc[0] (projected) [m]
    hull_buffer_m: margin around the outermost zone centroids [m]
    """
    hull = MultiPoint(list(zip(zones["x"], zones["y"]))).convex_hull.buffer(hull_buffer_m)
    return shapely.union(city_boundary, hull)


def build_voronoi_cells(zones: pd.DataFrame, extent: BaseGeometry, crs) -> gpd.GeoDataFrame:
    """One Voronoi cell per zone, clipped to `extent` [airspace CRS, m].

    Returns GeoDataFrame(zone_id, area_km2, geometry) in `crs`.
    """
    pts = [Point(x, y) for x, y in zip(zones["x"], zones["y"])]
    cells = shapely.voronoi_polygons(MultiPoint(pts), extend_to=extent)
    parts = list(cells.geoms)
    tree = shapely.STRtree(parts)
    geoms = []
    for p in pts:                                   # match each centroid to its own cell
        cand = tree.query(p, predicate="intersects")
        cell = next(parts[i] for i in cand if parts[i].covers(p))
        geoms.append(shapely.intersection(cell, extent))
    gdf = gpd.GeoDataFrame({"zone_id": zones["zone_id"].to_numpy()}, geometry=geoms, crs=crs)
    gdf["area_km2"] = gdf.geometry.area / 1e6       # zone cell area [km^2]
    return gdf


def reconcile_with_airspace(zones: pd.DataFrame, city_boundary: BaseGeometry,
                            ra_buffer_union: Optional[BaseGeometry] = None) -> Dict:
    """Flag zones outside the OSMnx city boundary and inside restricted-area buffers.

    Adds boolean columns to a copy of `zones`:
        inside_city_boundary   centroid within the OSMnx place polygon
        inside_ra_buffer       centroid within a restricted-area buffer (RA = OSM
                               features listed in airspace_restricted_area_tag_list,
                               buffered by Airspace.buffer_radius [m])
    Policy (D2): zones outside the city are KEPT (airspace is extended).
    Zones inside RA buffers are KEPT and REPORTED (open decision, see PRE_PLAN S08).
    """
    z = zones.copy()
    pts = shapely.points(z["x"].to_numpy(), z["y"].to_numpy())
    z["inside_city_boundary"] = shapely.covers(city_boundary, pts)        # on-boundary counts as inside
    z["inside_ra_buffer"] = (shapely.covers(ra_buffer_union, pts)
                             if ra_buffer_union is not None else False)
    report = {
        "n_zones": int(len(z)),
        "n_outside_city_boundary": int((~z["inside_city_boundary"]).sum()),
        "zones_outside_city_boundary": z.loc[~z["inside_city_boundary"], "zone_id"].tolist(),
        "n_inside_ra_buffer": int(z["inside_ra_buffer"].sum()),
        "zones_inside_ra_buffer": z.loc[z["inside_ra_buffer"], "zone_id"].tolist(),
    }
    return {"zones": z, "report": report}


def save_zone_artifacts(out_dir: str, zones: pd.DataFrame, cells: gpd.GeoDataFrame) -> None:
    """Write zones.csv (ids, lon/lat, x/y [m], flags) and zone_cells.geojson (EPSG:4326)."""
    import os
    os.makedirs(out_dir, exist_ok=True)
    zones.to_csv(os.path.join(out_dir, "zones.csv"), index=False)
    cells.to_crs("EPSG:4326").to_file(os.path.join(out_dir, "zone_cells.geojson"),
                                      driver="GeoJSON")
