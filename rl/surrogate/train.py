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
from rl.surrogate.datasets.episode_outcome_dataset import EpisodeOutcomeDataset
from rl.surrogate.datasets.trajectory_dataset import TrajectoryDataset

VALID_TASKS = ('next_state', 'episode_outcome')


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
    p.add_argument('--task', required=True, choices=VALID_TASKS)
    p.add_argument('--backbone', required=True, choices=list(BACKBONE_REGISTRY.keys()))
    p.add_argument('--logs-root', required=True)
    p.add_argument('--epochs', type=int, default=10)
    p.add_argument('--batch-size', type=int, default=32)
    p.add_argument('--learning-rate', type=float, default=1e-3)
    p.add_argument('--val-fraction', type=float, default=0.2)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--device', default='cpu')
    p.add_argument('--hidden-dim', type=int, default=32)
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
    )
    print('Final losses:', history)


if __name__ == '__main__':
    main()
