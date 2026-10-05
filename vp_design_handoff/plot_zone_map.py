"""
plot_zone_map.py
================
Visual QA of the VP-design artifacts (adapted from Austin_Node_Map.py):
zone cells coloured by region, zone centroids labelled with zone_id, extended
airspace boundary, and optionally the OSMnx drive network as a basemap.

    python scripts/vp_design/plot_zone_map.py --artifacts-dir data/vp_design/austin \
        --city "Austin, Texas, USA" --out renders/vp_design/austin_regions.png [--basemap]
"""
import argparse
import os

import geopandas as gpd
import matplotlib.pyplot as plt
import pandas as pd


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--artifacts-dir", required=True)
    p.add_argument("--city", default="Austin, Texas, USA")
    p.add_argument("--out", required=True)
    p.add_argument("--basemap", action="store_true", help="download OSMnx drive network")
    a = p.parse_args()

    cells = gpd.read_file(os.path.join(a.artifacts_dir, "zone_cells.geojson"))
    zones = pd.read_csv(os.path.join(a.artifacts_dir, "zones.csv"))
    zr = pd.read_csv(os.path.join(a.artifacts_dir, "zone_region_map.csv"))
    boundary = gpd.read_file(os.path.join(a.artifacts_dir, "airspace_boundary.geojson"))
    cells = cells.merge(zr, on="zone_id")

    fig, ax = plt.subplots(figsize=(14, 14), facecolor="#1a1a2e")
    ax.set_facecolor("#1a1a2e")
    if a.basemap:
        import osmnx as ox
        G = ox.graph_from_place(a.city, network_type="drive")
        ox.plot_graph(G, ax=ax, show=False, close=False, node_size=0,
                      edge_color="#4a4a6a", edge_linewidth=0.4, bgcolor="#1a1a2e")
    cells.plot(ax=ax, column="region_id", cmap="tab20", alpha=0.35, edgecolor="white",
               linewidth=0.4, zorder=2)
    boundary.boundary.plot(ax=ax, color="#ffe066", linewidth=1.0, zorder=3)
    ax.scatter(zones.lon, zones.lat, s=8, c="#ff6b35", edgecolors="white", linewidths=0.5, zorder=4)
    for r in zones.itertuples():
        ax.annotate(str(int(r.zone_id)), (r.lon, r.lat), fontsize=5, color="#ffe066",
                    xytext=(0, 4), textcoords="offset points", ha="center", zorder=5)
    n_reg = cells.region_id.nunique()
    ax.set_title(f"{a.city} - {len(zones)} zones, {n_reg} regions (cells coloured by region)",
                 color="white")
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out, dpi=300, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"saved {a.out}")


if __name__ == "__main__":
    main()
