"""
graph_flow_gnn.py
==================
Surrogate Model 1: discrete-time graph network for vertiport-edge UAV flow.

Nodes = vertiports, edges = flight paths. Predicts next-step node and edge
attributes (UAV counts) with a hard conservation constraint ensuring total
UAV count is preserved across timesteps.

Architecture follows the encode-process-decode pattern (GNS / MeshGraphNets)
with a conservation projection layer (Beucler et al., 2019).

Two variants:
  - GraphFlowGNN: stateless (no temporal memory)
  - GraphFlowRecurrentGNN: per-node GRU hidden state across timesteps
"""

from __future__ import annotations

from typing import Dict, Optional, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.data import Data
from torch_geometric.utils import degree

from rl.surrogate.backbones.surrogate_template import SurrogateModel
from rl.surrogate.datasets.graph_flow_dataset import EDGE_ATTR_DIM, NODE_ATTR_DIM


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------


class _MLP(nn.Module):
    def __init__(self, in_dim: int, hidden_dim: int, out_dim: int, layers: int = 2):
        super().__init__()
        mods = [nn.Linear(in_dim, hidden_dim), nn.ReLU()]
        for _ in range(layers - 2):
            mods += [nn.Linear(hidden_dim, hidden_dim), nn.ReLU()]
        mods.append(nn.Linear(hidden_dim, out_dim))
        self.net = nn.Sequential(*mods)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class _MessagePassingBlock(nn.Module):
    """One round of message passing: edge update → node update → global update."""

    def __init__(self, hidden_dim: int):
        super().__init__()
        self.edge_mlp = _MLP(hidden_dim * 3 + hidden_dim, # IN
                             hidden_dim,                  # HIDDEN
                             hidden_dim)                  # OUT
        
        self.node_mlp = _MLP(hidden_dim * 2,              # IN
                             hidden_dim,                  # HIDDEN
                             hidden_dim)                  # OUT
        
        self.global_mlp = _MLP(hidden_dim * 3,            # IN
                               hidden_dim,                # HIDDEN
                               hidden_dim)                # OUT
        #TODO: change LayerNorm -> GraphNorm and compare results by keeping everything else the same 
        self.edge_norm = nn.LayerNorm(hidden_dim)
        self.node_norm = nn.LayerNorm(hidden_dim)

    def forward(
        self,
        h_node: torch.Tensor,
        h_edge: torch.Tensor,
        h_global: torch.Tensor,
        # Directed edge_index, but for full_mesh/distance_threshold both (i,j) and
        # (j,i) are always present as separate rows, so every node still receives
        # messages from both directions -- effectively undirected without literal
        # symmetrization. demand_driven is the one topology that can be genuinely
        # one-directional (symmetrized in _build_edge_index_demand_driven instead).
        edge_index: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:

        src, dst = edge_index[0], edge_index[1]
        num_nodes = h_node.shape[0]

        # Edge update: f_e([h_src, h_dst, h_edge, h_global]). Computed from both
        # endpoint nodes (current supply/demand at each side of this OD pair), the
        # edge's own previous embedding (a residual memory of this specific
        # corridor), and the global context (network-wide UAV load) -- this is the
        # "message" that then gets aggregated into dst below. Node state alone
        # can't represent this pairwise/relational information.
        h_global_exp = h_global.expand(h_edge.shape[0],
                                       -1)
        edge_input = torch.cat([h_node[src],
                                h_node[dst],
                                h_edge,
                                h_global_exp],
                                dim=-1)
        h_edge_new = self.edge_norm(h_edge + self.edge_mlp(edge_input))

        # Node update: f_v([h_node, agg_incoming_edges]). Mean (not sum) over
        # incoming edges: full_mesh in-degree = num_nodes-1, which varies across
        # the sweep dataset's varied vertiport counts per config -- an
        # unnormalized sum would bias aggregated message magnitude by topology
        # size alone, hurting generalization across configs.
        agg = torch.zeros_like(h_node)
        agg.scatter_add_(0,
                         dst.unsqueeze(-1).expand(-1, h_edge_new.shape[-1]),
                         h_edge_new
                         )
        in_degree = degree(dst, num_nodes=num_nodes, dtype=h_edge_new.dtype).clamp(min=1).unsqueeze(-1)
        agg = agg / in_degree
        node_input = torch.cat([h_node, agg],
                               dim=-1)
        h_node_new = self.node_norm(h_node + self.node_mlp(node_input))

        # Global update: f_u([h_global, mean(h_node'), mean(h_edge')])
        global_input = torch.cat(
                                [h_global, 
                                 h_node_new.mean(dim=0, keepdim=True), 
                                 h_edge_new.mean(dim=0, keepdim=True)
                                ],
                                dim=-1,
                                )
        h_global_new = h_global + self.global_mlp(global_input)

        return h_node_new, h_edge_new, h_global_new


# ---------------------------------------------------------------------------
# Conservation projection
# ---------------------------------------------------------------------------

#TODO: add new function for local conservation projection
# Global conservation projection
def conservation_projection(
    node_uavs: torch.Tensor,
    edge_uavs: torch.Tensor,
    total_uavs: torch.Tensor,
) -> Tuple[torch.Tensor, torch.Tensor]:
    """Differentiable projection enforcing sum(node_uavs) + sum(edge_uavs) = total_uavs.

    Clamps negatives to zero, then redistributes the residual proportionally.

    The "projection" is the `scale = total_uavs / current_sum` rescale below: a
    proportional rescale of every predicted count onto the constraint surface
    {v : sum(v) = total_uavs} -- the simplest projection onto that linear surface.

    "Differentiable" doesn't mean "has learnable parameters" (it has none) -- it
    means gradients can flow *through* this op during backprop into the upstream
    decoder weights, since it's composed entirely of differentiable ops (ReLU,
    scalar division/multiplication). This is Beucler et al. (2019)'s term, used to
    contrast with non-differentiable hard-conservation techniques (e.g. exact
    simplex projection via sorting, or post-hoc numpy clipping outside the
    autograd graph) that would block gradient flow entirely.
    """
    node_uavs = F.relu(node_uavs)
    edge_uavs = F.relu(edge_uavs)

    current_sum = node_uavs.sum() + edge_uavs.sum()
    # Guards a divide-by-zero, not a real physical scenario: total_uavs (the
    # target) is always > 0 for a nonempty fleet, but current_sum (the predicted
    # post-ReLU total) can collapse toward 0 early in training when the decoder's
    # near-random weights produce strongly negative deltas across every
    # node/edge. Frequent hits here are a training-health signal (model hasn't
    # learned sensible residuals yet / is diverging), not an expected steady state.
    if current_sum < 1e-8:
        return node_uavs, edge_uavs

    scale = total_uavs.squeeze() / current_sum
    return node_uavs * scale, edge_uavs * scale


# ---------------------------------------------------------------------------
# GraphFlowGNN (stateless)
# ---------------------------------------------------------------------------


class GraphFlowGNN(SurrogateModel):
    """Encode-process-decode GNN for vertiport-edge UAV flow prediction.

    Nodes = vertiports, edges = flight paths.  Predicts next-step node and
    edge attributes (UAV counts) with a hard conservation constraint
    ensuring total UAV count is preserved across timesteps.

    **Papers and inspirations:**

    Architecture:
    - **GNS / Learning to Simulate** (Sanchez-Gonzalez et al., ICML 2020,
      https://arxiv.org/abs/2002.09405):
      Encode-process-decode pattern for learned physics simulation.  Our
      three-stage pipeline (encoder MLPs → message-passing blocks → decoder
      MLPs) follows this template.  Noise injection during training (adding
      Gaussian noise to input node/edge features) is borrowed from GNS to
      stabilize autoregressive rollout at inference time.

    - **MeshGraphNets** (Pfaff et al., ICLR 2021,
      https://arxiv.org/abs/2010.03409):
      Edge→node→global message-passing structure with residual connections
      and LayerNorm.  Our _MessagePassingBlock directly implements this
      three-level update pattern.  Residual decoding (predicting deltas
      rather than absolute values) also follows MeshGraphNets.

    Conservation:
    - **Beucler et al.** (ICML 2019, https://arxiv.org/abs/1906.06622):
      Hard architectural constraint for conservation in neural network
      emulators (climate physics).  Demonstrated that post-network
      projection achieves conservation to machine precision, outperforming
      soft loss penalties.  Our conservation_projection() implements this
      approach: clamp negatives, then proportionally rescale so
      sum(node_uavs) + sum(edge_uavs) = total_uavs.

    - **KCLNet** (Xu et al., AAAI 2026, https://arxiv.org/abs/2603.24101):
      Conservation enforced within message-passing architecture via
      current-embedding constraints at each depth.  Validates our design
      choice of node-level conservation at vertiport junctions (analogous
      to Kirchhoff's Current Law at electrical nodes).

    - **GNN-ODFill** (Zhang et al., 2025,
      https://doi.org/10.1016/j.patcog.2025.111470):
      GNN with hard flow conservation constraints for transportation
      origin-destination matrix completion.  Closest domain match to our
      vertiport network flow prediction — confirms hard constraints work
      in hub-and-spoke transit networks.

    Domain relevance:
    - **T-GCN** (Zhao et al., 2019, https://arxiv.org/abs/1811.05320):
      GCN + GRU for traffic prediction on road networks.  Validates the
      spatial-GNN approach for transportation flow forecasting.

    - **DCRNN** (Li et al., ICLR 2018, https://arxiv.org/abs/1707.01926):
      Scheduled sampling during autoregressive training to reduce exposure
      bias.  Applicable to future multi-step training improvements.

    - **STGCN** (Yu et al., IJCAI 2018, https://arxiv.org/abs/1709.04875):
      Pure convolutional spatiotemporal approach (no RNN), demonstrating
      faster training than recurrent alternatives for traffic forecasting.
    """

    # Indices into node/edge feature vectors for the UAV-count channels
    NODE_UAV_INDICES = [0, 1]  # n_grounded, n_landing_queue
    EDGE_UAV_INDEX = 0  # n_in_transit
    # Remaining trailing columns that are physically non-negative but not part of
    # the conserved-total constraint above -- clamped post-decode instead, since
    # nothing else constrains them. Indices match NODE_SPEED_KEYS/EDGE_SPEED_KEYS
    # + EDGE_EVENT_KEYS order in graph_flow_dataset.py.
    NODE_NONNEG_INDICES = [3, 4, 5]  # avg_speed, speed_min, speed_max
    EDGE_NONNEG_INDICES = [3, 4, 5, 6]  # avg_speed, speed_min, speed_max, nmac_count

    def __init__(
        self,
        hidden_dim: int = 64,
        num_mp_rounds: int = 3,
        noise_std: float = 3e-4,
    ):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_mp_rounds = num_mp_rounds
        self.noise_std = noise_std

        self.node_encoder = _MLP(NODE_ATTR_DIM, hidden_dim, hidden_dim)
        self.edge_encoder = _MLP(EDGE_ATTR_DIM, hidden_dim, hidden_dim)
        # [total_uavs, step, bias, num_removed, new_mission_completions,
        # total_collision_events, total_ra_collision_events] -- ground-truth
        # global scalars read off the input graph (see _build_global_input).
        self.global_encoder = _MLP(7, hidden_dim, hidden_dim)

        self.mp_blocks = nn.ModuleList(
            [_MessagePassingBlock(hidden_dim) for _ in range(num_mp_rounds)]
        )

        self.node_decoder = _MLP(hidden_dim, hidden_dim, NODE_ATTR_DIM)
        self.edge_decoder = _MLP(hidden_dim, hidden_dim, EDGE_ATTR_DIM)
        # Extra global-scalar heads, off h_global post-message-passing -- each
        # predicts one of the global scalars _build_global_input reads back off
        # the graph on the NEXT call, so an autoregressive rollout can thread
        # them forward without ground truth. removed_count_decoder also drives
        # the total_uavs decrement below (otherwise total_uavs is just passed
        # through unchanged -- correct for single-step teacher-forced training,
        # wrong for rollout, where the fleet actually shrinks on collisions).
        # new_arrivals/collision_event/ra_collision_event are accumulated/summed
        # externally across a rollout by
        # rollout_metrics.graph_flow_rollout_to_episode_metrics to approximate
        # avg_missions_completed/total_uav_collision_events/total_ra_collision_events.
        self.removed_count_decoder = _MLP(hidden_dim, hidden_dim, 1)
        self.new_arrivals_decoder = _MLP(hidden_dim, hidden_dim, 1)
        self.collision_event_decoder = _MLP(hidden_dim, hidden_dim, 1)
        self.ra_collision_event_decoder = _MLP(hidden_dim, hidden_dim, 1)

    def _build_global_input(self, graph: Data, total_uavs: torch.Tensor) -> torch.Tensor:
        """Assemble the global-scalar input vector, tolerating graphs that lack
        the newer optional attributes (e.g. hand-built graphs in older tests)."""
        device = total_uavs.device
        step_val = graph.step.float() if hasattr(graph, "step") else torch.zeros(1, device=device)

        def _scalar(attr_name: str) -> torch.Tensor:
            if hasattr(graph, attr_name):
                return getattr(graph, attr_name).float().view(1, 1)
            return torch.zeros(1, 1, device=device)

        return torch.cat(
            [
                total_uavs.view(1, 1),
                step_val.view(1, 1),
                torch.ones(1, 1, device=device),
                _scalar("num_removed"),
                _scalar("new_mission_completions"),
                _scalar("total_collision_events"),
                _scalar("total_ra_collision_events"),
            ],
            dim=-1,
        )

    def _finalize(
        self,
        h_node: torch.Tensor,
        h_edge: torch.Tensor,
        h_global: torch.Tensor,
        x: torch.Tensor,
        edge_attr: torch.Tensor,
        edge_index: torch.Tensor,
        total_uavs: torch.Tensor,
        graph: Data,
    ) -> Data:
        """Decode, apply conservation/non-negativity clamps, and build the
        next-step graph. Shared by GraphFlowGNN and
        GraphFlowRecurrentGNN.predict_graph_next_state, which differ only in how
        h_node/h_edge/h_global are produced (the recurrent variant injects a GRU
        step beforehand) -- everything from decode onward is identical.
        """
        # Decode (residual). Everything from here on is parameter-free tensor
        # ops -- gradient flows through them back into node_decoder/edge_decoder,
        # but no learnable weights live below this point.
        delta_node = self.node_decoder(h_node)
        delta_edge = self.edge_decoder(h_edge)
        next_x = x + delta_node
        next_edge_attr = edge_attr + delta_edge

        # Global scalar heads, computed before the conservation projection below
        # so next_total_uavs (this step's fleet size, after accounting for this
        # step's predicted removals) -- not the stale pre-removal `total_uavs` --
        # is what the projection targets. This keeps next_x/next_edge_attr's UAV
        # count channels self-consistent with the total_uavs stored on this same
        # returned graph (both reflect the same, already-decremented total),
        # instead of drifting one step out of sync across a rollout.
        predicted_removed = F.relu(self.removed_count_decoder(h_global)).view(1)
        predicted_new_arrivals = F.relu(self.new_arrivals_decoder(h_global)).view(1)
        predicted_collision_events = F.relu(self.collision_event_decoder(h_global)).view(1)
        predicted_ra_collision_events = F.relu(self.ra_collision_event_decoder(h_global)).view(1)
        next_total_uavs = (total_uavs - predicted_removed).clamp(min=0)

        # Conservation projection on UAV-count channels only.
        # Clamp sub-channels non-negative before computing fractions: needed so
        # node_fracs (redistribution weights across grounded/queue) stay in
        # [0,1]; conservation_projection's own internal ReLU (on the summed
        # scalar) is technically redundant for this node branch given this
        # clamp, but not for edge_uavs below, which reaches it unclamped.
        node_uavs = F.relu(next_x[:, self.NODE_UAV_INDICES].clone())
        edge_uavs = next_edge_attr[:, self.EDGE_UAV_INDEX].clone()

        proj_node, proj_edge = conservation_projection(
            node_uavs.sum(dim=-1),  # total UAVs at each node
            edge_uavs,
            next_total_uavs,
        )

        # Redistribute projected node UAVs back to grounded/queue proportionally
        node_total_raw = node_uavs.sum(dim=-1, keepdim=True).clamp(min=1e-8)
        node_fracs = node_uavs / node_total_raw
        next_x[:, self.NODE_UAV_INDICES] = node_fracs * proj_node.unsqueeze(-1)
        next_edge_attr[:, self.EDGE_UAV_INDEX] = proj_edge

        # Non-negativity clamp on the remaining physically-non-negative trailing
        # columns (speed stats + nmac_count), which the conservation projection
        # above doesn't touch.
        next_x[:, self.NODE_NONNEG_INDICES] = F.relu(next_x[:, self.NODE_NONNEG_INDICES])
        next_edge_attr[:, self.EDGE_NONNEG_INDICES] = F.relu(
            next_edge_attr[:, self.EDGE_NONNEG_INDICES]
        )

        return Data(
            x=next_x,
            edge_index=edge_index,
            edge_attr=next_edge_attr,
            total_uavs=next_total_uavs,
            # step["step"] = SimulatorState.currentstep, a 1:1 simulator timestep index.
            step=graph.step + 1 if hasattr(graph, "step") else torch.tensor([1]),
            num_removed=predicted_removed,
            new_mission_completions=predicted_new_arrivals,
            total_collision_events=predicted_collision_events,
            total_ra_collision_events=predicted_ra_collision_events,
        )

    def predict_graph_next_state(self, graph: Data) -> Data:
        x = graph.x  # [N, NODE_ATTR_DIM]
        edge_attr = graph.edge_attr  # [E, EDGE_ATTR_DIM]
        edge_index = graph.edge_index  # [2, E]
        total_uavs = graph.total_uavs  # [1]

        # GNS-style noise injection during training
        if self.training and self.noise_std > 0:
            x = x + torch.randn_like(x) * self.noise_std
            edge_attr = edge_attr + torch.randn_like(edge_attr) * self.noise_std

        # Encode
        h_node = self.node_encoder(x)
        h_edge = self.edge_encoder(edge_attr)
        h_global = self.global_encoder(self._build_global_input(graph, total_uavs))  # [1, hidden_dim]

        # Process
        for mp in self.mp_blocks:
            h_node, h_edge, h_global = mp(h_node, h_edge, h_global, edge_index)

        return self._finalize(h_node, h_edge, h_global, x, edge_attr, edge_index, total_uavs, graph)

    def predict_next_state(self, state: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError("GraphFlowGNN operates on graph Data, not per-UAV tensors")

    def predict_episode_outcome(self, batch: Dict[str, torch.Tensor]) -> torch.Tensor:
        raise NotImplementedError("GraphFlowGNN does not support episode outcome prediction")


# ---------------------------------------------------------------------------
# GraphFlowRecurrentGNN (with per-node GRU temporal memory)
# ---------------------------------------------------------------------------


class GraphFlowRecurrentGNN(GraphFlowGNN):
    """GraphFlowGNN extended with per-node GRU for temporal memory.

    Maintains hidden states across timesteps to capture multi-step transit
    delays (a UAV takes several steps to traverse an edge).

    **Papers and inspirations (in addition to GraphFlowGNN citations):**

    Temporal extensions:
    - **T-GCN** (Zhao et al., 2019, https://arxiv.org/abs/1811.05320):
      GCN + GRU hybrid — spatial graph convolution captures topology,
      GRU captures temporal dynamics.  Our per-node GRU cell follows
      this pattern: after spatial message passing, each node's hidden
      state is updated via GRUCell(h_spatial, h_prev_temporal).

    - **DCRNN** (Li et al., ICLR 2018, https://arxiv.org/abs/1707.01926):
      Diffusion convolution + GRU encoder-decoder with scheduled sampling.
      Demonstrated 12-15% improvement over non-temporal baselines on
      traffic forecasting.  Motivates our temporal extension for capturing
      edge transit delays (UAVs take multiple steps to fly between
      vertiports, creating temporal dependencies the stateless variant
      cannot model).

    - **TMS-GNN** (Baghbani et al., 2025,
      https://doi.org/10.1016/j.trc.2025.105111):
      Multistep GNN with scheduled sampling for bus network passenger
      flow.  Addresses autoregressive exposure bias — relevant for
      future improvements to our multi-step rollout training.
    """

    def __init__(
        self,
        hidden_dim: int = 64,
        num_mp_rounds: int = 3,
        noise_std: float = 3e-4,
    ):
        super().__init__(hidden_dim=hidden_dim, num_mp_rounds=num_mp_rounds, noise_std=noise_std)
        self.node_gru = nn.GRUCell(hidden_dim, hidden_dim)
        self._h_node_prev: Optional[torch.Tensor] = None

    def reset_hidden(self) -> None:
        self._h_node_prev = None

    def predict_graph_next_state(self, graph: Data) -> Data:
        x = graph.x
        edge_attr = graph.edge_attr
        edge_index = graph.edge_index
        total_uavs = graph.total_uavs

        if self.training and self.noise_std > 0:
            x = x + torch.randn_like(x) * self.noise_std
            edge_attr = edge_attr + torch.randn_like(edge_attr) * self.noise_std

        h_node = self.node_encoder(x)
        h_edge = self.edge_encoder(edge_attr)
        h_global = self.global_encoder(self._build_global_input(graph, total_uavs))

        # Inject temporal memory via GRU before message passing
        if self._h_node_prev is not None and self._h_node_prev.shape[0] == h_node.shape[0]:
            h_node = self.node_gru(h_node, self._h_node_prev)
        else:
            h_node = self.node_gru(h_node, torch.zeros_like(h_node))

        for mp in self.mp_blocks:
            h_node, h_edge, h_global = mp(h_node, h_edge, h_global, edge_index)

        # Store for next timestep (detach to prevent BPTT across episodes)
        self._h_node_prev = h_node.detach()

        return self._finalize(h_node, h_edge, h_global, x, edge_attr, edge_index, total_uavs, graph)
