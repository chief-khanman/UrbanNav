"""urbannav.vp_design — vertiport-design (VP design) data layer and metrics.

Phase 1 model (see docs/vp_design/PRE_PLAN.md, "Problem definition"):
    zone      = one GMNS Plus traffic analysis zone (TAZ); its centroid node is
                the vertiport candidate. zone == vertiport candidate in Phase 1.
    zone cell = Voronoi cell around the zone centroid (zones.build_voronoi_cells).
    region    = a group of zone cells (k-means or a user region file, regions.py).
    action    = one zone per region.

Modules
    assumptions   every value GMNS cannot supply, with unit/range/rationale
    gmns_io       GMNS Plus loader, road network, zone-to-zone car travel-time tables
    zones         zone centroids -> airspace coordinates, Voronoi cells, airspace reconciliation
    regions       zones -> regions (k-means or region file), region OD rate matrix
    door_to_door  access / egress tables and door-to-door travel time per selection
    mode_choice   demand responds to placement (binary logit UAM vs car)
    metrics       static passenger / community / equity metrics
    normalization raw metrics -> generalization block -> reward (GNN-RL and MCTS)
    phase2_structures  parked Phase-2 code (OSM buildings inside a zone cell)
"""
