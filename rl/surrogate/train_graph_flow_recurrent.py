"""
train_graph_flow_recurrent.py
================================
Standalone trainer for the `graph_flow_recurrent` dynamics model
(GraphFlowRecurrentGNN), which carries per-node GRU hidden state across
timesteps to capture multi-step transit delays.

Unlike train_graph_flow.py, this script CANNOT shuffle individual
(g_t, g_t+1) pairs -- the model's hidden state is only meaningful when pairs
within one episode are fed in their original temporal order, with
model.reset_hidden() called at each episode boundary. Loads pre-compiled
splits written by rl.surrogate.datasets.prepare_graph_flow_data via
load_episode_splits, which preserves the per-episode grouping; this script
shuffles EPISODE order each epoch but iterates pairs within an episode in
order.

CLI:
    python -m rl.surrogate.train_graph_flow_recurrent \\
        --data-dir logs/prepared/graph_flow_sweep_Jun23 \\
        --epochs 50 --hidden-dim 64
"""

from __future__ import annotations

import argparse
import random
from typing import Dict, List, Optional

import torch
import torch.nn as nn

from rl.surrogate.backbones.graph_flow_gnn import GraphFlowRecurrentGNN
from rl.surrogate.train_common import auto_device, check_metadata_task, load_episode_splits

TASK = "graph_flow"  # metadata.json records the dataset identity, not the model variant


def _run_episodes(model, episodes, dev, loss_fn, conservation_weight=None, optimizer=None, max_steps=None):
    """Iterate each episode's pairs in order, resetting hidden state at each
    episode boundary. If optimizer is given, trains (backprop per pair);
    otherwise evaluates under torch.no_grad(). Returns (recon_losses, cons_losses).
    max_steps caps the total number of pairs processed across all episodes --
    for smoke tests, not real training."""
    recon_losses: List[float] = []
    cons_losses: List[float] = []
    training = optimizer is not None
    steps_done = 0

    for episode_pairs in episodes:
        if max_steps is not None and steps_done >= max_steps:
            break
        model.reset_hidden()
        for (g_t, g_tp1) in episode_pairs:
            if max_steps is not None and steps_done >= max_steps:
                break
            steps_done += 1
            g_t, g_tp1 = g_t.to(dev), g_tp1.to(dev)

            if training:
                pred = model.predict_graph_next_state(g_t)
            else:
                with torch.no_grad():
                    pred = model.predict_graph_next_state(g_t)

            node_loss = loss_fn(pred.x, g_tp1.x)
            edge_loss = loss_fn(pred.edge_attr, g_tp1.edge_attr)
            recon_loss = node_loss + edge_loss

            if training:
                pred_total = pred.x[:, 0].sum() + pred.x[:, 1].sum() + pred.edge_attr[:, 0].sum()
                target_total = g_t.total_uavs.squeeze()
                cons_loss = (pred_total - target_total).pow(2)

                loss = recon_loss + conservation_weight * cons_loss
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                cons_losses.append(cons_loss.item())

            recon_losses.append(recon_loss.item())

    return recon_losses, cons_losses


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
    rng = random.Random(seed)
    dev = torch.device(device or auto_device())

    train_episodes = load_episode_splits(data_dir, "train")
    val_episodes = load_episode_splits(data_dir, "val")
    test_episodes = load_episode_splits(data_dir, "test")
    if train_episodes is None:
        raise RuntimeError(f"No train.pt found under {data_dir!r}. Run prepare_graph_flow_data.py first.")

    model = GraphFlowRecurrentGNN(hidden_dim=hidden_dim).to(dev)
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    loss_fn = nn.MSELoss()
    history: Dict[str, List[float]] = {"train_loss": [], "val_loss": [], "conservation_violation": []}

    for epoch in range(1, epochs + 1):
        model.train()
        shuffled = list(train_episodes)
        rng.shuffle(shuffled)
        recon_losses, cons_losses = _run_episodes(model, shuffled, dev, loss_fn, conservation_weight, optimizer, max_steps)

        avg_train = float(sum(recon_losses) / max(len(recon_losses), 1))
        avg_cons = float(sum(cons_losses) / max(len(cons_losses), 1))
        history["train_loss"].append(avg_train)
        history["conservation_violation"].append(avg_cons)

        val_msg = ""
        if val_episodes is not None:
            model.eval()
            val_recon, _ = _run_episodes(model, val_episodes, dev, loss_fn, max_steps=max_steps)
            avg_val = float(sum(val_recon) / max(len(val_recon), 1))
            history["val_loss"].append(avg_val)
            val_msg = f" | val_loss={avg_val:.6f}"

        scheduler.step()
        if verbose > 0:
            print(f"[graph_flow_recurrent] epoch {epoch:3d}/{epochs} | train_loss={avg_train:.6f} | cons_viol={avg_cons:.8f}{val_msg}")

    if test_episodes is not None:
        model.eval()
        test_recon, _ = _run_episodes(model, test_episodes, dev, loss_fn, max_steps=max_steps)
        history["test_loss"] = float(sum(test_recon) / max(len(test_recon), 1))
        if verbose > 0:
            print(f"[graph_flow_recurrent] test_loss={history['test_loss']:.6f}")

    return model, history


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Train the graph_flow_recurrent (GraphFlowRecurrentGNN) dynamics model.")
    p.add_argument("--data-dir", required=True)
    p.add_argument("--epochs", type=int, default=10)
    p.add_argument("--learning-rate", type=float, default=1e-3)
    p.add_argument("--hidden-dim", type=int, default=32)
    p.add_argument("--conservation-weight", type=float, default=10.0)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default=None)
    p.add_argument("--max-steps", type=int, default=None, help="Cap total pairs processed per train/val/test pass -- for smoke tests, not real training.")
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
