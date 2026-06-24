"""
train_graph_flow.py
=====================
Standalone trainer for the `graph_flow` dynamics model (GraphFlowGNN,
stateless encode-process-decode GNN over the vertiport/edge graph).

Loads pre-compiled splits written by
rl.surrogate.datasets.prepare_graph_flow_data (no raw-log parsing here).
GraphFlowGNN has no temporal memory, so the per-episode pair ordering
preserved in the .pt files doesn't matter -- pairs are flattened and
shuffled freely.

CLI:
    python -m rl.surrogate.train_graph_flow \\
        --data-dir logs/prepared/graph_flow_sweep_Jun23 \\
        --epochs 50 --hidden-dim 64
"""

from __future__ import annotations

import argparse
import itertools
from typing import Dict, List, Optional

import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from rl.surrogate.backbones.graph_flow_gnn import GraphFlowGNN
from rl.surrogate.train_common import auto_device, check_metadata_task, load_flat_split

TASK = "graph_flow"


def train(
    data_dir: str,
    epochs: int = 10,
    learning_rate: float = 1e-3,
    hidden_dim: int = 32,
    conservation_weight: float = 10.0,
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
        raise RuntimeError(f"No train.pt found under {data_dir!r}. Run prepare_graph_flow_data.py first.")

    model = GraphFlowGNN(hidden_dim=hidden_dim).to(dev)
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    loss_fn = nn.MSELoss()
    history: Dict[str, List[float]] = {"train_loss": [], "val_loss": [], "conservation_violation": []}

    train_loader = DataLoader(train_ds, batch_size=1, shuffle=True, collate_fn=lambda b: b[0])
    val_loader = DataLoader(val_ds, batch_size=1, collate_fn=lambda b: b[0]) if val_ds is not None else None

    for epoch in range(1, epochs + 1):
        model.train()
        train_losses: List[float] = []
        cons_violations: List[float] = []

        for (g_t, g_tp1) in itertools.islice(train_loader, max_steps):
            g_t, g_tp1 = g_t.to(dev), g_tp1.to(dev)

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
        history["train_loss"].append(avg_train)
        history["conservation_violation"].append(avg_cons)

        val_msg = ""
        if val_loader is not None:
            model.eval()
            val_losses: List[float] = []
            with torch.no_grad():
                for (g_t, g_tp1) in itertools.islice(val_loader, max_steps):
                    g_t, g_tp1 = g_t.to(dev), g_tp1.to(dev)
                    pred = model.predict_graph_next_state(g_t)
                    val_losses.append((loss_fn(pred.x, g_tp1.x) + loss_fn(pred.edge_attr, g_tp1.edge_attr)).item())
            avg_val = float(sum(val_losses) / max(len(val_losses), 1))
            history["val_loss"].append(avg_val)
            val_msg = f" | val_loss={avg_val:.6f}"

        if verbose > 0:
            print(f"[graph_flow] epoch {epoch:3d}/{epochs} | train_loss={avg_train:.6f} | cons_viol={avg_cons:.8f}{val_msg}")

    if test_ds is not None:
        test_loader = DataLoader(test_ds, batch_size=1, collate_fn=lambda b: b[0])
        model.eval()
        test_losses: List[float] = []
        with torch.no_grad():
            for (g_t, g_tp1) in itertools.islice(test_loader, max_steps):
                g_t, g_tp1 = g_t.to(dev), g_tp1.to(dev)
                pred = model.predict_graph_next_state(g_t)
                test_losses.append((loss_fn(pred.x, g_tp1.x) + loss_fn(pred.edge_attr, g_tp1.edge_attr)).item())
        history["test_loss"] = float(sum(test_losses) / max(len(test_losses), 1))
        if verbose > 0:
            print(f"[graph_flow] test_loss={history['test_loss']:.6f}")

    return model, history


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Train the graph_flow (GraphFlowGNN) dynamics model.")
    p.add_argument("--data-dir", required=True)
    p.add_argument("--epochs", type=int, default=10)
    p.add_argument("--learning-rate", type=float, default=1e-3)
    p.add_argument("--hidden-dim", type=int, default=32)
    p.add_argument("--conservation-weight", type=float, default=10.0)
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
        learning_rate=args.learning_rate,
        hidden_dim=args.hidden_dim,
        conservation_weight=args.conservation_weight,
        seed=args.seed,
        device=args.device,
        max_steps=args.max_steps,
        verbose=args.verbose,
    )
    print("Final losses:", history)


if __name__ == "__main__":
    main()
