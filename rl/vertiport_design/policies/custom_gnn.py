"""
custom_gnn.py
==============
GNN feature extractor for the 'GRAPH' obs_type produced by GraphBuilder /
VP_OBS_SPACE. Ported from the user's previously-validated CustomGNN
(GATConv-based, residual + LayerNorm, global mean pool).
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from gymnasium import spaces
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from torch_geometric.nn import GATConv


class CustomGNN(BaseFeaturesExtractor):
    """
    Custom GNN feature extractor for graph-structured observations.

    Processes graph observations (node_feat, edge_index, edge_attr) through
    GAT layers and concatenates selected_actions to produce final features.

    :param observation_space: gymnasium.spaces.Dict containing:
        - node_feat: [num_nodes, node_feat_dim]
        - edge_index: [2, num_edges]
        - edge_attr: [num_edges, edge_feat_dim]
        - selected_actions: [num_regions]
    :param features_dim: Hidden dimension for GNN layers
    :param final_dim: Output dimension of GNN before concatenating selected_actions
    :param num_layers: Number of GAT layers
    """

    def __init__(
        self,
        observation_space: spaces.Dict,
        features_dim: int = 16,
        final_dim: int = 8,
        num_layers: int = 1,
    ):
        num_regions = observation_space['selected_actions'].shape[0]
        total_output_dim = final_dim + num_regions

        # Must call super().__init__() with the actual output dimension —
        # this sets self._features_dim which the policy network reads.
        super().__init__(observation_space, total_output_dim)

        self.node_feat_dim = observation_space['node_feat'].shape[1]
        self.edge_feat_dim = observation_space['edge_attr'].shape[1]

        self.node_proj = nn.Linear(self.node_feat_dim, features_dim)
        self.edge_proj = nn.Linear(self.edge_feat_dim, features_dim)

        self.conv_layers = nn.ModuleList([
            GATConv(
                in_channels=features_dim,
                out_channels=features_dim,
                edge_dim=features_dim,
                heads=4,
                concat=False,  # average heads, output stays [num_nodes, features_dim]
            )
            for _ in range(num_layers)
        ])

        self.layer_norms = nn.ModuleList([
            nn.LayerNorm(features_dim) for _ in range(num_layers)
        ])

        self.output_proj = nn.Linear(features_dim, final_dim)

    def forward(self, observations) -> torch.Tensor:
        """
        SB3 passes batched observations even with a single env. Shapes:
            node_feat: [batch_size, num_nodes, node_feat_dim]
            edge_index: [batch_size, 2, num_edges]
            edge_attr: [batch_size, num_edges, edge_feat_dim]
            selected_actions: [batch_size, num_regions]

        Returns:
            torch.Tensor: [batch_size, final_dim + num_regions]
        """
        node_feat = observations['node_feat']
        edge_index = observations['edge_index'].long()
        edge_attr = observations['edge_attr']
        selected_actions = observations['selected_actions']

        batch_size = node_feat.shape[0]
        graph_embeddings = []

        for i in range(batch_size):
            x = node_feat[i]
            ei = edge_index[i]
            ea = edge_attr[i]

            h = self.node_proj(x)
            ea_proj = self.edge_proj(ea)

            for conv, norm in zip(self.conv_layers, self.layer_norms):
                h_new = conv(h, ei, ea_proj)
                h_new = F.relu(h_new)
                h = norm(h + h_new)  # residual connection

            graph_emb = torch.mean(h, dim=0)  # global mean pool
            graph_emb = self.output_proj(graph_emb)
            graph_embeddings.append(graph_emb)

        graph_embeddings = torch.stack(graph_embeddings, dim=0)
        output = torch.cat([graph_embeddings, selected_actions.float()], dim=1)
        return output
