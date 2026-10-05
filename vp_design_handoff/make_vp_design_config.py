"""
make_vp_design_config.py
========================
Pre-script: build the simulator config (YAML) and, in vertiport-design (VP
design) mode, all GMNS-derived artifacts the VertiportDesignEnv needs.

First question (always asked unless --vp-design is given):
    "Is this config for vertiport design? (YES/NO)"
        NO  -> standalone simulator config (random vertiports; single/multi-agent RL,
               deployment). Writes vp_design.enabled: false.
        YES -> VP design config. With --vertiport-source gmns_zones (default) it also
               runs the zone/region pipeline and writes artifacts.

Artifacts (YES + gmns_zones), written to --artifacts-dir (default data/vp_design/<city>/):
    zones.csv              zone_id, node_id, lon, lat, x [m], y [m], flags
    zone_cells.geojson     Voronoi cell per zone (EPSG:4326)
    airspace_boundary.geojson  extended airspace boundary (EPSG:4326), decision D2
    zone_region_map.csv    zone_id, region_id
    regions.txt            region file (editable; regions 0..R-1)
    regions.geojson        union of cells per region
    region_lambda.npy      lambda[r, s] region OD rate [trips/min]
    skims.npz              zone x zone car time [min], distance [m], toll [USD]
    report.json            reconciliation + fleet recommendation

Fleet recommendation (Little's law: average number busy = arrival rate x time
each one is busy):  UAVs ~ total lambda [trips/min] x cycle time [min] / target
utilization. Printed only; the user chooses the fleet size.

Example
    python scripts/vp_design/make_vp_design_config.py --vp-design yes \
        --city "Austin, Texas, USA" --gmns-dir data/gmns_plus/austin --k 5 \
        --template sample_config.yaml --out configs/vp_design/austin_k5.yaml
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os

import numpy as np
import yaml


def ask_yes_no(prompt: str) -> bool:
    while True:
        a = input(f"{prompt} (YES/NO): ").strip().lower()
        if a in ("yes", "y"):
            return True
        if a in ("no", "n"):
            return False


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--vp-design", choices=["yes", "no"], help="vertiport-design mode (asked if omitted)")
    p.add_argument("--template", default="sample_config.yaml", help="base YAML to copy")
    p.add_argument("--out", required=True, help="output YAML path")
    p.add_argument("--city", default="Austin, Texas, USA", help="OSMnx place name")
    # standalone
    p.add_argument("--num-vertiports", type=int, default=20, help="standalone: random vertiports")
    # simulator
    p.add_argument("--dt", type=float, default=1.0, help="time step [s]")
    p.add_argument("--total-timestep", type=int, default=5000, help="steps per episode")
    p.add_argument("--mode", choices=["2D", "3D"], default="3D")
    p.add_argument("--seed", type=int, default=123)
    p.add_argument("--pads", type=int, default=3, help="landing pads per vertiport")
    p.add_argument("--vp-height-range-m", type=float, nargs=2, default=[20.0, 100.0],
                   help="vertiport z range [m] (3D only)")
    p.add_argument("--fleet-standard", type=int, default=None, help="STANDARD UAV count")
    # vp design
    p.add_argument("--vertiport-source", choices=["gmns_zones", "synthetic", "osm_structures"],
                   default="gmns_zones")
    p.add_argument("--gmns-dir", help="data/gmns_plus/<city>")
    p.add_argument("--artifacts-dir", help="default data/vp_design/<city folder>")
    p.add_argument("--k", type=int, help="k-means regions (mutually exclusive with --region-file)")
    p.add_argument("--region-file", help="user region file (region r == [zones])")
    p.add_argument("--demand-period-min", type=float, default=60.0, help="[min]")
    p.add_argument("--demand-scale", type=float, default=1.0, help="[-]")
    p.add_argument("--hull-buffer-m", type=float, default=2000.0, help="[m]")
    p.add_argument("--turnaround-min", type=float, default=10.0,
                   help="ASSUMPTION per-trip turnaround for fleet recommendation [min]")
    p.add_argument("--target-utilization", type=float, default=0.8, help="[-]")
    p.add_argument("--offline", action="store_true",
                   help="skip OSMnx geocoding (boundary = buffered zone hull only)")
    return p.parse_args()


def run_gmns_pipeline(args) -> dict:
    """GMNS -> zones -> cells -> regions -> lambda + skims. Returns paths + report."""
    import geopandas as gpd
    import shapely

    from urbannav.vp_design.assumptions import Assumptions
    from urbannav.vp_design.gmns_io import RoadNetwork, load_gmns, save_skims
    from urbannav.vp_design.regions import (kmeans_regions, parse_region_file, region_od_rate,
                                            region_polygons, validate_zone_region,
                                            write_region_file, write_zone_region_map)
    from urbannav.vp_design.zones import (build_voronoi_cells, extended_airspace_boundary,
                                          project_zones, reconcile_with_airspace,
                                          save_zone_artifacts)

    if (args.k is None) == (args.region_file is None):
        raise SystemExit("give exactly one of --k or --region-file")
    city_folder = os.path.basename(os.path.normpath(args.gmns_dir))
    out = args.artifacts_dir or os.path.join("data", "vp_design", city_folder)
    os.makedirs(out, exist_ok=True)
    A = Assumptions({"demand_period_min": args.demand_period_min,
                     "demand_scale": args.demand_scale})

    nodes, links, demand = load_gmns(args.gmns_dir)
    net = RoadNetwork(nodes, links, time="congested", volume_field=A.bpr_volume_field)
    zn = net.zone_nodes()

    if args.offline:
        crs = gpd.GeoSeries(gpd.points_from_xy(zn.lon, zn.lat), crs="EPSG:4326").estimate_utm_crs()
        zones = project_zones(zn, crs)
        city = shapely.MultiPoint(list(zip(zones.x, zones.y))).convex_hull
    else:
        import osmnx as ox
        city_gdf = ox.projection.project_gdf(ox.geocode_to_gdf(args.city))   # same as Airspace
        crs = city_gdf.crs
        zones = project_zones(zn, crs)
        city = city_gdf.geometry.iloc[0]

    boundary = extended_airspace_boundary(city, zones, args.hull_buffer_m)
    rec = reconcile_with_airspace(zones, city)          # RA check happens in Airspace (S08)
    zones = rec["zones"]
    cells = build_voronoi_cells(zones, boundary, crs)
    save_zone_artifacts(out, zones, cells)
    gpd.GeoSeries([boundary], crs=crs).to_crs("EPSG:4326").to_file(
        os.path.join(out, "airspace_boundary.geojson"), driver="GeoJSON")

    zr = kmeans_regions(zones, args.k, seed=args.seed) if args.k else parse_region_file(args.region_file)
    R = validate_zone_region(zr, zones.zone_id)
    write_zone_region_map(zr, os.path.join(out, "zone_region_map.csv"))
    write_region_file(zr, os.path.join(out, "regions.txt"))
    region_polygons(cells, zr).to_crs("EPSG:4326").to_file(os.path.join(out, "regions.geojson"),
                                                         driver="GeoJSON")
    lam = region_od_rate(demand, zr, A, R)                              # [trips/min]
    np.save(os.path.join(out, "region_lambda.npy"), lam)
    sk = net.zone_skims(zones)
    save_skims(os.path.join(out, "skims.npz"), **sk)

    # ---- fleet recommendation (Little's law) ----
    cx = zones.groupby(zones.zone_id.map(zr))[["x", "y"]].mean().to_numpy()   # region centres [m]
    d = np.hypot(cx[:, None, 0] - cx[None, :, 0], cx[:, None, 1] - cx[None, :, 1])  # [m]
    flight_min = A.vertical_phase_min + d * A.air_detour_factor / A.prior_cruise_speed_mps / 60
    lam_tot = lam.sum()                                                  # [trips/min]
    cycle = float((lam * flight_min).sum() / max(lam_tot, 1e-12)) + args.turnaround_min  # [min]
    fleet = math.ceil(lam_tot * cycle / args.target_utilization)
    report = {**rec["report"], "n_regions": R, "crs": str(crs),
              "total_lambda_trips_per_min": float(lam_tot), "mean_cycle_min": cycle,
              "recommended_fleet_littles_law": fleet}
    with open(os.path.join(out, "report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
    return {"artifacts_dir": out, "n_regions": R, "recommended_fleet": fleet}


def main():
    args = parse_args()
    vp = (args.vp_design == "yes") if args.vp_design else ask_yes_no(
        "Is this config for vertiport design?")
    with open(args.template) as f:
        cfg = yaml.safe_load(f)
    cfg = copy.deepcopy(cfg)

    cfg.setdefault("simulator", {}).update(
        {"dt": args.dt, "total_timestep": args.total_timestep, "mode": args.mode, "seed": args.seed})
    cfg.setdefault("vertiport", {}).update(
        {"number_of_landing_pad": args.pads, "height_range_m": list(args.vp_height_range_m)})
    air = cfg.setdefault("airspace", {})
    air["location_name"] = args.city
    # PROPOSED SCHEMA (PRE_PLAN S02): standalone-only and vp-design-only keys move out of `airspace`
    air.pop("number_of_vertiports", None)
    vertiport_tag_list = air.pop("vertiport_tag_list", [["building", "commercial"]])
    cfg["standalone"] = {"number_of_vertiports": args.num_vertiports}

    vpd = {"enabled": bool(vp), "vertiport_source": args.vertiport_source}
    if vp and args.vertiport_source == "gmns_zones":
        res = run_gmns_pipeline(args)
        a = res["artifacts_dir"]
        vpd["gmns"] = {
            "city_dir": args.gmns_dir, "artifacts_dir": a,
            "zones_csv": os.path.join(a, "zones.csv"),
            "zone_cells": os.path.join(a, "zone_cells.geojson"),
            "airspace_boundary": os.path.join(a, "airspace_boundary.geojson"),
            "zone_region_map": os.path.join(a, "zone_region_map.csv"),
            "region_lambda": os.path.join(a, "region_lambda.npy"),
            "skims": os.path.join(a, "skims.npz"),
            "demand_period_min": args.demand_period_min, "demand_scale": args.demand_scale,
        }
        print(f"Recommended STANDARD fleet (Little's law): {res['recommended_fleet']}")
    elif vp and args.vertiport_source == "synthetic":
        vpd["synthetic"] = {"num_regions": args.k or 4, "num_vertiports_per_region": 5}
    elif vp:
        vpd["osm_structures"] = {"vertiport_tag_list": vertiport_tag_list,
                                 "num_regions": args.k or 4}
    cfg["vp_design"] = vpd

    if args.fleet_standard is not None:
        for e in cfg.get("fleet_composition", []):
            if e.get("type_name") == "STANDARD":
                e["count"] = args.fleet_standard

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
