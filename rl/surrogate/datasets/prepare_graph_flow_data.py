"""
prepare_graph_flow_data.py
============================
Data processor for the `graph_flow` dataset (GraphFlowDataset), feeding both
the `graph_flow` and `graph_flow_recurrent` model variants -- they share the
identical (graph_t, graph_t+1) schema; only the model differs.

Discovers episode directories under a single raw-logs root (recursively, so
nested sweep layouts like logs/sweep_run_Jun23/run_<hash>/episode_*/ work),
splits them at the directory level into train/val/test, and -- critically --
preserves per-episode grouping in what it serializes: each split is saved as
a List[List[Tuple[Data, Data]]] (one inner list of consecutive pairs per
episode), not a flat merged list. This lets train_graph_flow.py flatten it
for shuffled stateless training while train_graph_flow_recurrent.py keeps
episodes intact and iterates them in order, resetting hidden state at each
episode boundary.

Usage::

    python -m rl.surrogate.datasets.prepare_graph_flow_data \\
        --logs-root logs/sweep_run_Jun23 \\
        --output-dir logs/prepared/graph_flow_sweep_Jun23 \\
        --edge-type full_mesh
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, Tuple

import torch
from torch_geometric.data import Data

from rl.surrogate.datasets.discovery import discover_episode_dirs, split_dirs
from rl.surrogate.datasets.graph_flow_dataset import GraphFlowDataset, VALID_EDGE_TYPES


def _build_episodes(
    dirs: List[str], edge_type: str, distance_threshold: float,
) -> List[List[Tuple[Data, Data]]]:
    return [
        GraphFlowDataset([d], edge_type=edge_type, distance_threshold=distance_threshold)._pairs
        for d in dirs
    ]


def prepare(
    logs_root: str,
    output_dir: str,
    val_fraction: float = 0.2,
    test_fraction: float = 0.1,
    seed: int = 0,
    allow_empty_splits: bool = False,
    edge_type: str = "full_mesh",
    distance_threshold: float = 0.0,
) -> None:
    dirs = discover_episode_dirs(logs_root)
    if not dirs:
        raise RuntimeError(f"No episode directories found under {logs_root!r}.")

    train_dirs, val_dirs, test_dirs = split_dirs(
        dirs, val_fraction, test_fraction, seed, allow_empty_splits,
    )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    counts = {}
    for split_name, split_dir_list in (
        ("train", train_dirs), ("val", val_dirs), ("test", test_dirs),
    ):
        episodes = _build_episodes(split_dir_list, edge_type, distance_threshold)
        episodes = [ep for ep in episodes if ep]  # drop episodes with <2 usable steps
        counts[split_name] = {
            "dirs": len(split_dir_list),
            "episodes": len(episodes),
            "pairs": sum(len(ep) for ep in episodes),
        }
        if episodes:
            torch.save(episodes, out / f"{split_name}.pt")

    metadata = {
        "task": "graph_flow",
        "dataset_kwargs": {"edge_type": edge_type, "distance_threshold": distance_threshold},
        "logs_root": logs_root,
        "val_fraction": val_fraction,
        "test_fraction": test_fraction,
        "seed": seed,
        "counts": counts,
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"[prepare_graph_flow_data] wrote {output_dir}: {counts}")


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--logs-root", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--val-fraction", type=float, default=0.2)
    p.add_argument("--test-fraction", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--allow-empty-splits", action="store_true")
    p.add_argument("--edge-type", default="full_mesh", choices=sorted(VALID_EDGE_TYPES))
    p.add_argument("--distance-threshold", type=float, default=0.0)
    return p


def main() -> None:
    args = _build_arg_parser().parse_args()
    prepare(
        logs_root=args.logs_root,
        output_dir=args.output_dir,
        val_fraction=args.val_fraction,
        test_fraction=args.test_fraction,
        seed=args.seed,
        allow_empty_splits=args.allow_empty_splits,
        edge_type=args.edge_type,
        distance_threshold=args.distance_threshold,
    )


if __name__ == "__main__":
    main()
