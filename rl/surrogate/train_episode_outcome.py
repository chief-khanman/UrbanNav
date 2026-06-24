"""
train_episode_outcome.py
===========================
First-pass standalone trainer for the one `value`-category model that
exists today: GNNSurrogateBackbone's predict_episode_outcome path (GAT over
initial UAV positions, pooled + concatenated with episode scalars).

Loads pre-compiled splits written by
rl.surrogate.datasets.prepare_episode_outcome_data (no raw-log parsing here).
Unlike the graph_flow/dual_graph dynamics models, this task uses real
mini-batching via a standard DataLoader.

CLI:
    python -m rl.surrogate.train_episode_outcome \\
        --data-dir logs/prepared/episode_outcome_sweep_Jun23 \\
        --epochs 20 --hidden-dim 32
"""

from __future__ import annotations

import argparse
import itertools
from typing import Dict, List, Optional

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from rl.surrogate.backbones.gnn_backbone import GNNSurrogateBackbone
from rl.surrogate.train_common import auto_device, check_metadata_task, load_flat_split

TASK = "episode_outcome"


def train(
    data_dir: str,
    epochs: int = 10,
    batch_size: int = 32,
    learning_rate: float = 1e-3,
    hidden_dim: int = 32,
    seed: int = 0,
    device: str = None,
    max_steps: Optional[int] = None,
    verbose: int = 1,
):
    check_metadata_task(data_dir, TASK)
    torch.manual_seed(seed)
    dev = torch.device(device or auto_device())

    train_ds = load_flat_split(data_dir, "train")
    val_ds = load_flat_split(data_dir, "val")
    test_ds = load_flat_split(data_dir, "test")
    if train_ds is None:
        raise RuntimeError(f"No train.pt found under {data_dir!r}. Run prepare_episode_outcome_data.py first.")

    num_outcome_targets = train_ds[0]["target"].shape[0]
    model = GNNSurrogateBackbone(hidden_dim=hidden_dim, num_outcome_targets=num_outcome_targets).to(dev)
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    loss_fn = nn.MSELoss()
    history: Dict[str, List[float]] = {"train_loss": [], "val_loss": []}

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=batch_size) if val_ds is not None else None

    for epoch in range(1, epochs + 1):
        model.train()
        train_losses: List[float] = []
        for batch in itertools.islice(train_loader, max_steps):
            batch = {k: v.to(dev) for k, v in batch.items()}
            pred = model.predict_episode_outcome(batch)
            loss = loss_fn(pred, batch["target"])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_losses.append(loss.item())
        avg_train = float(sum(train_losses) / max(len(train_losses), 1))
        history["train_loss"].append(avg_train)

        val_msg = ""
        if val_loader is not None:
            model.eval()
            val_losses: List[float] = []
            with torch.no_grad():
                for batch in itertools.islice(val_loader, max_steps):
                    batch = {k: v.to(dev) for k, v in batch.items()}
                    pred = model.predict_episode_outcome(batch)
                    val_losses.append(loss_fn(pred, batch["target"]).item())
            avg_val = float(sum(val_losses) / max(len(val_losses), 1))
            history["val_loss"].append(avg_val)
            val_msg = f" | val_loss={avg_val:.6f}"

        if verbose > 0:
            print(f"[episode_outcome] epoch {epoch:3d}/{epochs} | train_loss={avg_train:.6f}{val_msg}")

    if test_ds is not None:
        test_loader = DataLoader(test_ds, batch_size=batch_size)
        model.eval()
        test_losses: List[float] = []
        with torch.no_grad():
            for batch in itertools.islice(test_loader, max_steps):
                batch = {k: v.to(dev) for k, v in batch.items()}
                pred = model.predict_episode_outcome(batch)
                test_losses.append(loss_fn(pred, batch["target"]).item())
        history["test_loss"] = float(sum(test_losses) / max(len(test_losses), 1))
        if verbose > 0:
            print(f"[episode_outcome] test_loss={history['test_loss']:.6f}")

    return model, history


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Train the episode_outcome (GNNSurrogateBackbone) value model.")
    p.add_argument("--data-dir", required=True)
    p.add_argument("--epochs", type=int, default=10)
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--learning-rate", type=float, default=1e-3)
    p.add_argument("--hidden-dim", type=int, default=32)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default=None)
    p.add_argument("--max-steps", type=int, default=None, help="Cap iterations per train/val/test pass -- for smoke tests, not real training.")
    p.add_argument("--verbose", type=int, default=1)
    return p


def main() -> None:
    args = _build_arg_parser().parse_args()
    model, history = train(
        data_dir=args.data_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        hidden_dim=args.hidden_dim,
        seed=args.seed,
        device=args.device,
        max_steps=args.max_steps,
        verbose=args.verbose,
    )
    print("Final losses:", history)


if __name__ == "__main__":
    main()
