"""
train.py
=========
Generic training loop for any backbone in BACKBONE_REGISTRY against either
TrajectoryDataset (next-state prediction) or EpisodeOutcomeDataset
(episode-outcome prediction).

CLI:
    python -m rl.surrogate.train \\
        --logs-root logs \\
        --task next_state \\
        --backbone GNN \\
        --epochs 10 \\
        --batch-size 64

Stdout-only logging by default — no W&B/MLflow integration (plan item
intentionally deferred).
"""

from __future__ import annotations

import argparse
from typing import Dict, List, Tuple

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split

from rl.surrogate.backbones import BACKBONE_REGISTRY, SurrogateModel
from rl.surrogate.datasets.dual_graph_dataset import DualGraphDataset
from rl.surrogate.datasets.episode_outcome_dataset import EpisodeOutcomeDataset
from rl.surrogate.datasets.graph_flow_dataset import GraphFlowDataset
from rl.surrogate.datasets.trajectory_dataset import TrajectoryDataset

VALID_TASKS = ('next_state', 'episode_outcome', 'graph_flow', 'dual_graph')


def _split(dataset, val_fraction: float, seed: int) -> Tuple:
    n = len(dataset)
    n_val = max(int(round(n * val_fraction)), 1) if n >= 2 else 0
    n_train = n - n_val
    if n_val == 0:
        return dataset, None
    generator = torch.Generator().manual_seed(seed)
    return random_split(dataset, [n_train, n_val], generator=generator)


def train_next_state(
    model: SurrogateModel,
    dataset: TrajectoryDataset,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    val_fraction: float,
    seed: int,
    device: torch.device,
    verbose: int = 1,
) -> Dict[str, List[float]]:
    """Train predict_next_state via supervised MSE."""
    train_ds, val_ds = _split(dataset, val_fraction, seed)
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size) if val_ds is not None else None

    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    loss_fn = nn.MSELoss()
    history: Dict[str, List[float]] = {'train_loss': [], 'val_loss': []}
    model.to(device)

    for epoch in range(1, epochs + 1):
        model.train()
        train_losses: List[float] = []
        for state, action, next_state in train_loader:
            state = state.to(device)
            action = action.to(device)
            next_state = next_state.to(device)
            pred = model.predict_next_state(state, action)
            loss = loss_fn(pred, next_state)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_losses.append(loss.item())
        avg_train = float(sum(train_losses) / max(len(train_losses), 1))
        history['train_loss'].append(avg_train)

        val_msg = ''
        if val_loader is not None:
            model.eval()
            val_losses: List[float] = []
            with torch.no_grad():
                for state, action, next_state in val_loader:
                    state = state.to(device)
                    action = action.to(device)
                    next_state = next_state.to(device)
                    pred = model.predict_next_state(state, action)
                    val_losses.append(loss_fn(pred, next_state).item())
            avg_val = float(sum(val_losses) / max(len(val_losses), 1))
            history['val_loss'].append(avg_val)
            val_msg = f' | val_loss={avg_val:.6f}'

        if verbose > 0:
            print(f'[next_state] epoch {epoch:3d}/{epochs} | train_loss={avg_train:.6f}{val_msg}')

    return history


def train_episode_outcome(
    model: SurrogateModel,
    dataset: EpisodeOutcomeDataset,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    val_fraction: float,
    seed: int,
    device: torch.device,
    verbose: int = 1,
) -> Dict[str, List[float]]:
    """Train predict_episode_outcome via supervised MSE."""
    train_ds, val_ds = _split(dataset, val_fraction, seed)
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size) if val_ds is not None else None

    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    loss_fn = nn.MSELoss()
    history: Dict[str, List[float]] = {'train_loss': [], 'val_loss': []}
    model.to(device)

    for epoch in range(1, epochs + 1):
        model.train()
        train_losses: List[float] = []
        for batch in train_loader:
            batch = {k: v.to(device) for k, v in batch.items()}
            pred = model.predict_episode_outcome(batch)
            loss = loss_fn(pred, batch['target'])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_losses.append(loss.item())
        avg_train = float(sum(train_losses) / max(len(train_losses), 1))
        history['train_loss'].append(avg_train)

        val_msg = ''
        if val_loader is not None:
            model.eval()
            val_losses: List[float] = []
            with torch.no_grad():
                for batch in val_loader:
                    batch = {k: v.to(device) for k, v in batch.items()}
                    pred = model.predict_episode_outcome(batch)
                    val_losses.append(loss_fn(pred, batch['target']).item())
            avg_val = float(sum(val_losses) / max(len(val_losses), 1))
            history['val_loss'].append(avg_val)
            val_msg = f' | val_loss={avg_val:.6f}'

        if verbose > 0:
            print(f'[episode_outcome] epoch {epoch:3d}/{epochs} | train_loss={avg_train:.6f}{val_msg}')

    return history


def train_graph_flow(
    model: SurrogateModel,
    dataset: GraphFlowDataset,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    val_fraction: float,
    seed: int,
    device: torch.device,
    conservation_weight: float = 10.0,
    verbose: int = 1,
) -> Dict[str, List[float]]:
    """Train predict_graph_next_state on (graph_t, graph_t+1) pairs."""
    train_ds, val_ds = _split(dataset, val_fraction, seed)
    train_loader = DataLoader(train_ds, batch_size=1, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=1) if val_ds is not None else None

    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    loss_fn = nn.MSELoss()
    history: Dict[str, List[float]] = {
        'train_loss': [], 'val_loss': [], 'conservation_violation': [],
    }
    model.to(device)

    for epoch in range(1, epochs + 1):
        model.train()
        train_losses: List[float] = []
        cons_violations: List[float] = []

        for (g_t, g_tp1) in train_loader:
            g_t = g_t.to(device)
            g_tp1 = g_tp1.to(device)

            pred = model.predict_graph_next_state(g_t)

            node_loss = loss_fn(pred.x, g_tp1.x)
            edge_loss = loss_fn(pred.edge_attr, g_tp1.edge_attr)
            recon_loss = node_loss + edge_loss

            pred_total = pred.x[:, 0].sum() + pred.x[:, 1].sum() + pred.edge_attr[:, 0].sum()
            target_total = g_t.total_uavs.squeeze()
            cons_loss = (pred_total - target_total).pow(2)

            loss = recon_loss + conservation_weight * cons_loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_losses.append(recon_loss.item())
            cons_violations.append(cons_loss.item())

        scheduler.step()

        avg_train = float(sum(train_losses) / max(len(train_losses), 1))
        avg_cons = float(sum(cons_violations) / max(len(cons_violations), 1))
        history['train_loss'].append(avg_train)
        history['conservation_violation'].append(avg_cons)

        val_msg = ''
        if val_loader is not None:
            model.eval()
            val_losses: List[float] = []
            with torch.no_grad():
                for (g_t, g_tp1) in val_loader:
                    g_t = g_t.to(device)
                    g_tp1 = g_tp1.to(device)
                    pred = model.predict_graph_next_state(g_t)
                    val_losses.append(
                        (loss_fn(pred.x, g_tp1.x) + loss_fn(pred.edge_attr, g_tp1.edge_attr)).item()
                    )
            avg_val = float(sum(val_losses) / max(len(val_losses), 1))
            history['val_loss'].append(avg_val)
            val_msg = f' | val_loss={avg_val:.6f}'

        if verbose > 0:
            print(
                f'[graph_flow] epoch {epoch:3d}/{epochs} | '
                f'train_loss={avg_train:.6f} | cons_viol={avg_cons:.8f}{val_msg}'
            )

    return history


def train_dual_graph(
    model: SurrogateModel,
    dataset: DualGraphDataset,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    val_fraction: float,
    seed: int,
    device: torch.device,
    conservation_weight: float = 10.0,
    verbose: int = 1,
) -> Dict[str, List[float]]:
    """Train dual-graph model on (hetero_t, hetero_t+1) pairs."""
    train_ds, val_ds = _split(dataset, val_fraction, seed)
    train_loader = DataLoader(train_ds, batch_size=1, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=1) if val_ds is not None else None

    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    loss_fn = nn.MSELoss()
    history: Dict[str, List[float]] = {
        'train_loss': [], 'val_loss': [], 'conservation_violation': [],
    }
    model.to(device)

    for epoch in range(1, epochs + 1):
        model.train()
        train_losses: List[float] = []
        cons_violations: List[float] = []

        for (g_t, g_tp1) in train_loader:
            g_t = g_t.to(device)
            g_tp1 = g_tp1.to(device)

            preds = model.predict_dual_graph_next_state(g_t)

            uav_loss = loss_fn(preds['uav_x'], g_tp1['uav'].x)
            vp_loss = loss_fn(preds['vp_x'], g_tp1['vertiport'].x)
            recon_loss = uav_loss + vp_loss

            # Conservation violation on collision_status channel
            pred_active = preds['uav_x'][:, 8].sum()
            target_active = g_t.total_uavs.squeeze()
            cons_loss = (pred_active - target_active).pow(2)

            loss = recon_loss + conservation_weight * cons_loss
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            train_losses.append(recon_loss.item())
            cons_violations.append(cons_loss.item())

        scheduler.step()

        avg_train = float(sum(train_losses) / max(len(train_losses), 1))
        avg_cons = float(sum(cons_violations) / max(len(cons_violations), 1))
        history['train_loss'].append(avg_train)
        history['conservation_violation'].append(avg_cons)

        val_msg = ''
        if val_loader is not None:
            model.eval()
            val_losses: List[float] = []
            with torch.no_grad():
                for (g_t, g_tp1) in val_loader:
                    g_t = g_t.to(device)
                    g_tp1 = g_tp1.to(device)
                    preds = model.predict_dual_graph_next_state(g_t)
                    val_losses.append(
                        (loss_fn(preds['uav_x'], g_tp1['uav'].x)
                         + loss_fn(preds['vp_x'], g_tp1['vertiport'].x)).item()
                    )
            avg_val = float(sum(val_losses) / max(len(val_losses), 1))
            history['val_loss'].append(avg_val)
            val_msg = f' | val_loss={avg_val:.6f}'

        if verbose > 0:
            print(
                f'[dual_graph] epoch {epoch:3d}/{epochs} | '
                f'train_loss={avg_train:.6f} | cons_viol={avg_cons:.8f}{val_msg}'
            )

    return history


def train(
    task: str,
    backbone: str,
    logs_root: str,
    epochs: int = 10,
    batch_size: int = 32,
    learning_rate: float = 1e-3,
    val_fraction: float = 0.2,
    seed: int = 0,
    device: str = 'cpu',
    backbone_kwargs: dict = None,
    edge_type: str = 'full_mesh',
    distance_threshold: float = 0.0,
    conservation_weight: float = 10.0,
    verbose: int = 1,
) -> Tuple[SurrogateModel, Dict[str, List[float]]]:
    """Entry point: build dataset + backbone, run the matching trainer.

    Returns the trained model and its loss history.
    """
    if task not in VALID_TASKS:
        raise ValueError(f"Unknown task '{task}'. Valid: {VALID_TASKS}")
    if backbone not in BACKBONE_REGISTRY:
        raise ValueError(
            f"Unknown backbone '{backbone}'. Registered: {list(BACKBONE_REGISTRY.keys())}"
        )

    kwargs = dict(backbone_kwargs or {})
    dev = torch.device(device)

    if task == 'next_state':
        dataset = TrajectoryDataset.from_logs_root(logs_root)
        if len(dataset) == 0:
            raise RuntimeError(f"No transitions found under {logs_root!r} for next_state task.")
        model = BACKBONE_REGISTRY[backbone](**kwargs)
        history = train_next_state(
            model, dataset, epochs, batch_size, learning_rate,
            val_fraction, seed, dev, verbose,
        )
    elif task == 'graph_flow':
        dataset = GraphFlowDataset.from_logs_root(
            logs_root, edge_type=edge_type, distance_threshold=distance_threshold,
        )
        if len(dataset) == 0:
            raise RuntimeError(
                f"No graph-flow transitions found under {logs_root!r}. "
                "Ensure MetricsCollector records vertiport/edge snapshots."
            )
        model = BACKBONE_REGISTRY[backbone](**kwargs)
        history = train_graph_flow(
            model, dataset, epochs, batch_size, learning_rate,
            val_fraction, seed, dev, conservation_weight, verbose,
        )
    elif task == 'dual_graph':
        dataset = DualGraphDataset.from_logs_root(logs_root)
        if len(dataset) == 0:
            raise RuntimeError(
                f"No dual-graph transitions found under {logs_root!r}. "
                "Ensure step_history.json contains vertiport and UAV snapshots."
            )
        model = BACKBONE_REGISTRY[backbone](**kwargs)
        history = train_dual_graph(
            model, dataset, epochs, batch_size, learning_rate,
            val_fraction, seed, dev, conservation_weight, verbose,
        )
    else:
        dataset = EpisodeOutcomeDataset.from_logs_root(logs_root)
        if len(dataset) == 0:
            raise RuntimeError(
                f"No episodes with metadata.json + episode_metrics.json under {logs_root!r}."
            )
        kwargs.setdefault('num_outcome_targets', dataset._targets.shape[1])
        model = BACKBONE_REGISTRY[backbone](**kwargs)
        history = train_episode_outcome(
            model, dataset, epochs, batch_size, learning_rate,
            val_fraction, seed, dev, verbose,
        )

    return model, history


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description='Train a surrogate backbone on UrbanNav rollouts.')
    p.add_argument('--task', required=True, choices=list(VALID_TASKS))
    p.add_argument('--backbone', required=True, choices=list(BACKBONE_REGISTRY.keys()))
    p.add_argument('--logs-root', required=True)
    p.add_argument('--epochs', type=int, default=10)
    p.add_argument('--batch-size', type=int, default=32)
    p.add_argument('--learning-rate', type=float, default=1e-3)
    p.add_argument('--val-fraction', type=float, default=0.2)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--device', default='cpu')
    p.add_argument('--hidden-dim', type=int, default=32)
    p.add_argument('--edge-type', default='full_mesh', choices=['full_mesh', 'demand_driven', 'distance_threshold'])
    p.add_argument('--distance-threshold', type=float, default=0.0)
    p.add_argument('--conservation-weight', type=float, default=10.0)
    return p


def main() -> None:
    args = _build_arg_parser().parse_args()
    model, history = train(
        task=args.task,
        backbone=args.backbone,
        logs_root=args.logs_root,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        val_fraction=args.val_fraction,
        seed=args.seed,
        device=args.device,
        backbone_kwargs={'hidden_dim': args.hidden_dim},
        edge_type=args.edge_type,
        distance_threshold=args.distance_threshold,
        conservation_weight=args.conservation_weight,
    )
    print('Final losses:', history)


if __name__ == '__main__':
    main()
