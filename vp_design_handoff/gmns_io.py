"""
gmns_io.py
==========
GMNS Plus loader, road network, and zone-to-zone car travel-time tables
("skims": a planner's term for a zone x zone table of travel time / distance /
cost on the shortest path).

Data location (one folder per city):
    data/gmns_plus/<city>/node.csv     node_id, zone_id, x_coord, y_coord, geometry
    data/gmns_plus/<city>/link.csv     link_id, from_node_id, to_node_id, dir_flag, length, ...
    data/gmns_plus/<city>/demand.csv   o_zone_id, d_zone_id, volume

GMNS Plus unit conventions (verify per city):
    x_coord, y_coord  longitude/latitude [deg, EPSG:4326]
    length            [m]               vdf_length_mi     [mile]
    vdf_fftt          free-flow time [min]
    free_speed        [km/h]            vdf_free_speed_mph [mile/h]
    capacity          [vehicles/h/lane] lanes             [-]
    vdf_toll          [USD]             vdf_alpha, vdf_beta: BPR parameters [-]
    vdf_plf           peak load factor  [-] (period volume -> hourly flow)
    dir_flag          1 = from->to, -1 = to->from, 0 = both directions
    ref_volume        reference (modelled) link volume [vehicles/period]
    obs_volume        observed counts (all 0 in Austin)

Austin observations (Austin_data_view.ipynb):
    * nodes 1..199 are the 199 zone centroids (one node per zone).
    * zone centroid connectors: virtual links with length 0 and capacity 99_999
      that stand in for local streets. Their time is ~0, so access time from a
      centroid ignores intra-zone travel -> intrazonal rule in door_to_door.py.
"""
from __future__ import annotations

import os
from typing import Dict, Optional, Sequence

import numpy as np
import pandas as pd
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

M_PER_MILE = 1609.344   # [m/mile]

REQUIRED_COLUMNS = {
    "node.csv": {"node_id", "zone_id", "x_coord", "y_coord"},
    "link.csv": {"from_node_id", "to_node_id"},
    "demand.csv": {"o_zone_id", "d_zone_id", "volume"},
}


# =============================================================================
# 1. Loading
# =============================================================================
def load_gmns(city_dir: str):
    """Load node, link, demand tables of one GMNS Plus city; validates columns."""
    out = []
    for name in ("node.csv", "link.csv", "demand.csv"):
        path = os.path.join(city_dir, name)
        df = pd.read_csv(path)
        missing = REQUIRED_COLUMNS[name] - set(df.columns)
        if missing:
            raise ValueError(f"{path} is missing columns {sorted(missing)}")
        out.append(df)
    nodes, links, demand = out
    demand = demand[demand["volume"] > 0].copy()        # OD rows with trips [trips/period]
    return nodes, links, demand


def _col(df: pd.DataFrame, name: str, default=np.nan) -> pd.Series:
    if name in df.columns:
        return pd.to_numeric(df[name], errors="coerce")
    return pd.Series(default, index=df.index, dtype=float)


# =============================================================================
# 2. Link-level quantities
# =============================================================================
def link_free_flow_min(links: pd.DataFrame) -> pd.Series:
    """Free-flow link time [min]: vdf_fftt, else miles/mph, else km/(km/h)."""
    t = _col(links, "vdf_fftt")
    t = t.where(t > 0, _col(links, "vdf_length_mi") / _col(links, "vdf_free_speed_mph") * 60)
    t = t.where(np.isfinite(t) & (t > 0),
                _col(links, "length") / 1000 / _col(links, "free_speed") * 60)
    # connectors (length 0) end up ~0; clip keeps Dijkstra weights strictly positive
    return t.fillna(t.median()).clip(lower=1e-3)


def link_length_m(links: pd.DataFrame) -> pd.Series:
    """Link length [m]."""
    L = _col(links, "length")
    return L.where(L > 0, _col(links, "vdf_length_mi") * M_PER_MILE).fillna(0.0)


def link_bpr(links: pd.DataFrame, fftt_min: pd.Series, volume_field: str):
    """BPR (Bureau of Public Roads) congested time [min] and v/c [-].

    t = t0 * (1 + alpha * (v/c)^beta)
        v  hourly flow = period volume x vdf_plf          [vehicles/h]
        c  capacity x lanes                               [vehicles/h]
    v/c = volume-to-capacity ratio (1.0 = road at capacity).
    """
    vol = _col(links, volume_field).fillna(0.0)                 # [vehicles/period]
    plf = _col(links, "vdf_plf").fillna(1.0).replace(0, 1.0)    # peak load factor [-]
    lanes = _col(links, "lanes").fillna(1.0).clip(lower=1.0)    # [-]
    cap = _col(links, "capacity") * lanes                       # [vehicles/h]
    alpha = _col(links, "vdf_alpha").fillna(0.15)               # BPR alpha [-]
    beta = _col(links, "vdf_beta").fillna(4.0)                  # BPR beta [-]
    vc = (vol * plf / cap).where(cap > 0, 0.0).fillna(0.0)      # v/c [-]
    if vc.quantile(0.95) > 3:
        print(f"[warn] 95th pct v/c = {vc.quantile(0.95):.1f}; check volume/capacity/plf "
              f"units for '{volume_field}'.")
    return fftt_min * (1 + alpha * vc ** beta), vc


# =============================================================================
# 3. Road network with fast multi-source shortest paths
# =============================================================================
class RoadNetwork:
    """Directed road graph (CSR sparse matrix) with Dijkstra + path attribute sums.

    time: 'free' (free-flow) | 'congested' (BPR with `volume_field`)
    """

    def __init__(self, nodes: pd.DataFrame, links: pd.DataFrame,
                 time: str = "congested", volume_field: str = "ref_volume",
                 allowed_link_types: Optional[Sequence] = None):
        self.nodes = nodes.reset_index(drop=True)
        self.N = len(self.nodes)                                      # number of nodes [-]
        self.id2idx = pd.Series(np.arange(self.N), index=self.nodes["node_id"].to_numpy())

        L = links.copy()
        if allowed_link_types is not None:
            L = L[L["link_type"].isin(allowed_link_types)]
        L["t_free"] = link_free_flow_min(L)                           # [min]
        L["t_cong"], L["vc"] = link_bpr(L, L["t_free"], volume_field)  # [min], [-]
        L["len_m"] = link_length_m(L)                                 # [m]
        L["toll"] = _col(L, "vdf_toll").fillna(0.0)                   # [USD]
        if time not in ("free", "congested"):
            raise ValueError(time)
        L["w"] = L["t_free"] if time == "free" else L["t_cong"]       # Dijkstra weight [min]
        L["u"] = L["from_node_id"].map(self.id2idx)
        L["v"] = L["to_node_id"].map(self.id2idx)
        L = L.dropna(subset=["u", "v"])
        L[["u", "v"]] = L[["u", "v"]].astype(np.int64)
        self.links = L

        dflag = _col(L, "dir_flag").fillna(1)
        fwd = L.loc[dflag >= 0, ["u", "v", "w", "len_m", "toll"]]
        bwd = L.loc[dflag <= 0, ["v", "u", "w", "len_m", "toll"]]
        bwd.columns = ["u", "v", "w", "len_m", "toll"]
        E = pd.concat([fwd, bwd], ignore_index=True)
        # parallel links: keep the fastest (csr_matrix would SUM duplicates)
        E = E.loc[E.groupby(["u", "v"])["w"].idxmin()]
        E["key"] = E["u"] * self.N + E["v"]
        E = E.sort_values("key")
        self.e_key = E["key"].to_numpy()
        self.e_len = E["len_m"].to_numpy()
        self.e_toll = E["toll"].to_numpy()

        self.G = csr_matrix((E["w"].to_numpy(), (E["u"].to_numpy(), E["v"].to_numpy())),
                            shape=(self.N, self.N))
        self.Gt = self.G.T.tocsr()
        self.connected_mask = (np.diff(self.G.indptr) > 0) & (np.diff(self.Gt.indptr) > 0)

    def _path_sums(self, pred: np.ndarray, attr: np.ndarray, reverse: bool) -> np.ndarray:
        """Sum an edge attribute along every shortest-path tree branch.

        Vectorised pointer jumping, O(k * N * log(depth)). Verified against
        brute-force path reconstruction.
        """
        k, N = pred.shape
        cols = np.broadcast_to(np.arange(N, dtype=np.int64), (k, N))
        valid = pred >= 0
        p = pred.astype(np.int64)
        keys = np.where(reverse, cols * N + p, p * N + cols)[valid]
        w = np.zeros((k, N))
        w[valid] = attr[np.searchsorted(self.e_key, keys)]
        ptr = np.where(valid, p, cols)
        acc = w
        while True:
            nxt = np.take_along_axis(ptr, ptr, 1)
            if np.array_equal(nxt, ptr):
                return acc
            acc = acc + np.take_along_axis(acc, ptr, 1)
            ptr = nxt

    def sssp(self, sources: np.ndarray, targets: np.ndarray,
             reverse: bool = False, chunk: int = 32):
        """Shortest-path time [min], length [m], toll [USD], shape (len(sources), len(targets))."""
        G = self.Gt if reverse else self.G
        T, D, C = [], [], []
        for s in range(0, len(sources), chunk):
            src = sources[s:s + chunk]
            dist, pred = dijkstra(G, directed=True, indices=src, return_predecessors=True)
            T.append(dist[:, targets])
            D.append(self._path_sums(pred, self.e_len, reverse)[:, targets])
            C.append(self._path_sums(pred, self.e_toll, reverse)[:, targets])
        T, D, C = np.vstack(T), np.vstack(D), np.vstack(C)
        D[~np.isfinite(T)] = np.inf
        return T, D, C

    def zone_nodes(self) -> pd.DataFrame:
        """One centroid node per zone: zone_id, node_idx, lon [deg], lat [deg].

        Prefers the node whose node_id equals the zone_id (Austin: nodes 1..199);
        otherwise the zone node nearest the mean of that zone's nodes.
        """
        z = self.nodes[self.nodes["zone_id"].notna()].copy()
        z = z[z["zone_id"].astype(str).str.strip() != ""]
        z["zone_id"] = z["zone_id"].astype(float).astype(int)
        z["idx"] = z.index.to_numpy()
        m = z.groupby("zone_id")[["x_coord", "y_coord"]].transform("mean")
        z["d2"] = (z["x_coord"] - m["x_coord"]) ** 2 + (z["y_coord"] - m["y_coord"]) ** 2
        z.loc[z["node_id"] == z["zone_id"], "d2"] = -1.0          # exact centroid wins
        z = z.loc[z.groupby("zone_id")["d2"].idxmin()]
        out = pd.DataFrame({"zone_id": z["zone_id"].to_numpy(),
                            "node_id": z["node_id"].to_numpy(),
                            "node_idx": z["idx"].to_numpy(),
                            "lon": z["x_coord"].to_numpy(), "lat": z["y_coord"].to_numpy()})
        return out.sort_values("zone_id").reset_index(drop=True)

    def zone_skims(self, zones: pd.DataFrame) -> Dict[str, np.ndarray]:
        """Zone x zone car tables, zones ordered as `zones` (sorted zone_id):
        time_min [min], dist_m [m], toll_usd [USD]."""
        n = zones["node_idx"].to_numpy()
        T, D, C = self.sssp(n, n)
        return {"zone_ids": zones["zone_id"].to_numpy(), "time_min": T,
                "dist_m": D, "toll_usd": C}

    def link_midpoints(self, crs) -> pd.DataFrame:
        """Link midpoints in `crs` [m] with v/c [-] and length [m] (ground congestion)."""
        from pyproj import Transformer
        tf = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
        x, y = tf.transform(self.nodes["x_coord"].to_numpy(), self.nodes["y_coord"].to_numpy())
        L = self.links
        return pd.DataFrame({"mx": (x[L["u"]] + x[L["v"]]) / 2, "my": (y[L["u"]] + y[L["v"]]) / 2,
                             "vc": L["vc"].to_numpy(), "len_m": L["len_m"].to_numpy()})


def ground_congestion_near(points_xy: np.ndarray, mids: pd.DataFrame,
                           radius_m: float = 1000.0) -> pd.DataFrame:
    """Length-weighted mean and max v/c [-] of road links within radius_m [m] of each point.

    Operator/community metric: vertiports add access traffic to these links.
    """
    from scipy.spatial import cKDTree
    tree = cKDTree(mids[["mx", "my"]].to_numpy())
    vc, ln = mids["vc"].to_numpy(), mids["len_m"].to_numpy()
    rows = []
    for xy in points_xy:
        ii = tree.query_ball_point(xy, radius_m)
        rows.append((np.average(vc[ii], weights=ln[ii] + 1e-9), vc[ii].max()) if ii
                    else (np.nan, np.nan))
    return pd.DataFrame(rows, columns=["vc_mean_near", "vc_max_near"])


def save_skims(path: str, **arrays) -> None:
    np.savez_compressed(path, **arrays)


def load_skims(path: str) -> Dict[str, np.ndarray]:
    with np.load(path, allow_pickle=True) as f:
        return {k: f[k] for k in f.files}
