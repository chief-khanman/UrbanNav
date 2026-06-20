"""
graph_builder.py
=================
Builds graph representations (node_feat / edge_index / edge_attr) of vertiport
configurations for the 'GRAPH' obs_type consumed by VertiportDesignEnv and
CustomGNN.

Ported from the user's previously-validated VertiportGraphBuilder, simplified
per that code's own TODOs: node features are [is_selected, normalized_x,
normalized_y] and edge features are [normalized_distance,
both_endpoints_selected] — dropping the edge-type one-hot and placeholder
flow-metric features used in the original exploratory version.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np
import torch

from urbannav.airspace import Airspace
from urbannav.vertiport import Vertiport


class GraphBuilder:
    """Builds and manages graph representations of vertiport configurations."""

    def __init__(self, airspace: Airspace, connectivity_type: str = 'full'):
        """
        Args:
            airspace:          Airspace instance with regions_dict already
                                populated (call airspace.make_regions_dict(...) first).
            connectivity_type: 'full' (every vertiport pair connected) — the
                                only mode implemented; 'inter_intra' from the
                                original exploratory version is not ported
                                (full connectivity is what the validated
                                training run used).
        """
        if not hasattr(airspace, 'regions_dict'):
            raise RuntimeError(
                'Cannot build GraphBuilder: airspace.regions_dict is not set. '
                'Call airspace.make_regions_dict(...) first.'
            )
        if connectivity_type != 'full':
            raise NotImplementedError(
                f"connectivity_type='{connectivity_type}' is not implemented. "
                "Use connectivity_type='full'."
            )

        self.airspace = airspace
        self.connectivity_type = connectivity_type

        self.vertiport_to_idx: Dict[Vertiport, int] = {}
        self.idx_to_vertiport: Dict[int, Vertiport] = {}
        self._build_vertiport_mapping()

    def _build_vertiport_mapping(self):
        """Create bidirectional mapping between vertiports and indices."""
        idx = 0
        for vertiport_list in self.airspace.regions_dict.values():
            for vertiport in vertiport_list:
                self.vertiport_to_idx[vertiport] = idx
                self.idx_to_vertiport[idx] = vertiport
                idx += 1

    def compute_distance(self, vp1: Vertiport, vp2: Vertiport) -> float:
        """Euclidean distance between two vertiports."""
        return vp1.location.distance(vp2.location)

    def build_graph(self, selected_vertiports: List[Vertiport]):
        """
        Build graph with node features, edge index, and edge attributes.

        Args:
            selected_vertiports: list of currently selected vertiports (one per region).

        Returns:
            x: Node features [num_nodes, 3]
            edge_index: Edge connectivity [2, num_edges]
            edge_attr: Edge features [num_edges, 2]
        """
        x = self._build_node_features(selected_vertiports)
        edge_index, edge_attr = self._build_fully_connected_graph(selected_vertiports)
        return x, edge_index, edge_attr

    def _build_node_features(self, selected_vertiports: List[Vertiport]) -> torch.Tensor:
        """Node features: [is_selected, normalized_x, normalized_y]."""
        num_vertiports = len(self.vertiport_to_idx)
        x = torch.zeros(num_vertiports, 3)

        all_x = [vp.location.x for vp in self.idx_to_vertiport.values()]
        all_y = [vp.location.y for vp in self.idx_to_vertiport.values()]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        range_x = max_x - min_x if max_x != min_x else 1.0
        range_y = max_y - min_y if max_y != min_y else 1.0

        for idx, vertiport in self.idx_to_vertiport.items():
            is_selected = 1.0 if vertiport in selected_vertiports else 0.0
            normalized_x = (vertiport.location.x - min_x) / range_x
            normalized_y = (vertiport.location.y - min_y) / range_y
            x[idx] = torch.tensor([is_selected, normalized_x, normalized_y])

        return x

    def _build_fully_connected_graph(self, selected_vertiports: List[Vertiport]):
        """Fully connected graph over all candidate vertiports."""
        num_vertiports = len(self.vertiport_to_idx)
        all_x = [vp.location.x for vp in self.idx_to_vertiport.values()]
        all_y = [vp.location.y for vp in self.idx_to_vertiport.values()]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        diagonal = float(np.sqrt((max_x - min_x) ** 2 + (max_y - min_y) ** 2)) or 1.0

        edges = []
        edge_attrs = []
        for i in range(num_vertiports):
            for j in range(num_vertiports):
                if i == j:
                    continue
                vp_i = self.idx_to_vertiport[i]
                vp_j = self.idx_to_vertiport[j]
                edges.append([i, j])

                distance = self.compute_distance(vp_i, vp_j)
                both_selected = 1.0 if (vp_i in selected_vertiports and vp_j in selected_vertiports) else 0.0
                edge_attrs.append([distance / diagonal, both_selected])

        edge_index = torch.tensor(edges, dtype=torch.long).t()
        edge_attr = torch.tensor(edge_attrs, dtype=torch.float32)
        return edge_index, edge_attr

    def indices_to_vertiports(self, indices) -> List[Vertiport]:
        """Convert iterable of indices to list of vertiports."""
        return [self.idx_to_vertiport[int(idx)] for idx in indices]

    def vertiports_to_indices(self, vertiports: List[Vertiport]) -> torch.Tensor:
        """Convert list of vertiports to tensor of indices."""
        return torch.tensor([self.vertiport_to_idx[vp] for vp in vertiports])

    def region_idx_to_vertiport(self, action) -> List[Vertiport]:
        """
        Use a per-region action array to build a new vertiport list.

        Args:
            action: array-like where action[region] is the index into that
                    region's candidate vertiport list (airspace.regions_dict[region]).

        Returns:
            new_vp_list: one vertiport per region, in region order.
        """
        new_vp_list = []
        for region, vp_list in self.airspace.regions_dict.items():
            new_vp_list.append(vp_list[action[region]])
        return new_vp_list
