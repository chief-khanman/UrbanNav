"""
gat_encoder.py
===============
Shared per-frame ego+intruder graph encoder, used standalone by
gnn_extractor.py (single timestep) and weight-tied across the temporal
window by gnn_rnn_extractor.py (one call per stacked frame) — factored out
so the two extractors don't duplicate the GATConv wiring.

**Papers and inspirations:**
- **CADRL** (Chen, Liu, Kreiss, Pavone, ICRA 2017, https://arxiv.org/abs/1609.07845):
  decentralized, ego-centric multi-agent collision avoidance — the star
  topology here (ego attends to each other agent independently, no
  agent-agent edges) follows CADRL's ego-centric framing rather than a full
  social graph.
- **GAT** (Velickovic et al., ICLR 2018, https://arxiv.org/abs/1710.10903):
  attention-weighted neighbor aggregation — `GATConv` lets ego's updated
  embedding weight each intruder by learned relevance rather than treating
  all detected intruders equally.

Architecturally mirrors `rl/vertiport_design/policies/custom_gnn.py`'s
GATConv + residual + LayerNorm pattern, but is a fresh implementation rather
than an import: the obs domain/schema differs (ego+intruder UAV graph vs.
vertiport-region graph) and importing across `rl.single_agent` ->
`rl.vertiport_design` would be a backwards package dependency.
"""
from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GATConv

from rl.single_agent.policies.graph_utils import build_star_graph


class EgoIntruderGATEncoder(nn.Module):
    """
    Encode one ego-centric star graph (ego + real intruders) into a single
    fixed-size embedding for ego, via stacked GATConv layers with residual
    connections and LayerNorm.

    Ego and intruder raw features have different widths (9 vs 3), so each
    node type is projected into the shared `hidden_dim` by its own linear
    encoder before any graph convolution runs — a single shared projection
    (as used in custom_gnn.py, where all nodes already share one raw width)
    would not be valid here.

    Pools by reading out node 0's (ego's) post-message-passing embedding,
    not a global mean over all nodes — the task is "what should ego do",
    not a graph-level summary, so ego's own attention-aggregated embedding
    is the more faithful readout for this ego-centric setting.

    Args:
        hidden_dim: width nodes/edges are projected into and GATConv operates on.
        num_layers: number of stacked GATConv (+residual+LayerNorm) rounds.
        heads: number of attention heads per GATConv layer (averaged, not concatenated).
    """

    def __init__(self, hidden_dim: int = 64, num_layers: int = 2, heads: int = 4):
        super().__init__()
        self.hidden_dim = hidden_dim

        self.ego_node_encoder = nn.Linear(9, hidden_dim)
        self.intruder_node_encoder = nn.Linear(3, hidden_dim)
        self.edge_encoder = nn.Linear(3, hidden_dim)

        self.conv_layers = nn.ModuleList([
            GATConv(
                in_channels=hidden_dim,
                out_channels=hidden_dim,
                edge_dim=hidden_dim,
                heads=heads,
                concat=False,  # average heads, output stays [num_nodes, hidden_dim]
            )
            for _ in range(num_layers)
        ])
        self.layer_norms = nn.ModuleList([nn.LayerNorm(hidden_dim) for _ in range(num_layers)])

    def forward(
        self, self_block: torch.Tensor, intruder_block: torch.Tensor, mask: torch.Tensor,
    ) -> torch.Tensor:
        """
        Args:
            self_block:     [batch, 9]
            intruder_block: [batch, n_intruder, 6]
            mask:           [batch, n_intruder] bool, True where real.

        Returns:
            ego_embedding: [batch, hidden_dim] — ego's embedding after message
            passing with its real intruders for that sample.

        Processes one sample (one graph) at a time in a Python loop — follows
        the same per-batch-item convention already established in
        custom_gnn.py, since each sample's graph has a different number of
        real nodes (PyG's GATConv operates on one graph's node/edge tensors
        at a time, not a padded batch tensor).
        """
        batch_size = self_block.shape[0]
        embeddings = []

        for i in range(batch_size):
            ego_raw, intruder_pos_raw, edge_index, edge_attr_raw = build_star_graph(
                self_block[i], intruder_block[i], mask[i],
            )

            ego_h = self.ego_node_encoder(ego_raw).unsqueeze(0)         # [1, hidden_dim]
            intruder_h = self.intruder_node_encoder(intruder_pos_raw)    # [n_real, hidden_dim]
            h = torch.cat([ego_h, intruder_h], dim=0)                    # [1+n_real, hidden_dim]
            edge_attr = self.edge_encoder(edge_attr_raw)                 # [2*n_real, hidden_dim]

            for conv, norm in zip(self.conv_layers, self.layer_norms):
                h_new = conv(h, edge_index, edge_attr)
                h_new = F.relu(h_new)
                h = norm(h + h_new)  # residual connection

            embeddings.append(h[0])  # ego's own updated embedding (node 0)

        return torch.stack(embeddings, dim=0)
