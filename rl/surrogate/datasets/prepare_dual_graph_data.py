"""
prepare_dual_graph_data.py
=============================
Data processor for the `dual_graph` dataset (DualGraphDataset), feeding the
`dual_graph` model variant (DualGraphGNN). No recurrent variant exists for
this model today, so no per-episode grouping is needed -- each split is
serialized as a flat List[Tuple[HeteroData, HeteroData]].

Discovers episode directories under a single raw-logs root (recursively),
splits them at the directory level into train/val/test, builds one
DualGraphDataset per split from its disjoint directory list, and saves it.

Usage::

    python -m rl.surrogate.datasets.prepare_dual_graph_data \\
        --logs-root logs/sweep_run_Jun23 \\
        --output-dir logs/prepared/dual_graph_sweep_Jun23
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from rl.surrogate.datasets.discovery import discover_episode_dirs, split_dirs
from rl.surrogate.datasets.dual_graph_dataset import DualGraphDataset, VALID_UAV_EDGE_TYPES

VALID_VP_EDGE_TYPES = ("full_mesh", "distance_threshold")


def prepare(
    logs_root: str,
    output_dir: str,
    val_fraction: float = 0.2,
    test_fraction: float = 0.1,
    seed: int = 0,
    allow_empty_splits: bool = False,
    uav_edge_type: str = "distance_threshold",
    uav_edge_distance: float = 200.0,
    vp_edge_type: str = "full_mesh",
    vp_edge_distance: float = 0.0,
) -> None:
    dirs = discover_episode_dirs(logs_root)
    if not dirs:
        raise RuntimeError(f"No episode directories found under {logs_root!r}.")

    train_dirs, val_dirs, test_dirs = split_dirs(
        dirs, val_fraction, test_fraction, seed, allow_empty_splits,
    )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    dataset_kwargs = dict(
        uav_edge_type=uav_edge_type,
        uav_edge_distance=uav_edge_distance,
        vp_edge_type=vp_edge_type,
        vp_edge_distance=vp_edge_distance,
    )

    counts = {}
    for split_name, split_dir_list in (
        ("train", train_dirs), ("val", val_dirs), ("test", test_dirs),
    ):
        ds = DualGraphDataset(split_dir_list, **dataset_kwargs)
        items = [ds[i] for i in range(len(ds))]
        counts[split_name] = {"dirs": len(split_dir_list), "pairs": len(items)}
        if items:
            torch.save(items, out / f"{split_name}.pt")

    metadata = {
        "task": "dual_graph",
        "dataset_kwargs": dataset_kwargs,
        "logs_root": logs_root,
        "val_fraction": val_fraction,
        "test_fraction": test_fraction,
        "seed": seed,
        "counts": counts,
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"[prepare_dual_graph_data] wrote {output_dir}: {counts}")


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--logs-root", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--val-fraction", type=float, default=0.2)
    p.add_argument("--test-fraction", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--allow-empty-splits", action="store_true")
    p.add_argument("--uav-edge-type", default="distance_threshold", choices=sorted(VALID_UAV_EDGE_TYPES))
    p.add_argument("--uav-edge-distance", type=float, default=200.0)
    p.add_argument("--vp-edge-type", default="full_mesh", choices=list(VALID_VP_EDGE_TYPES))
    p.add_argument("--vp-edge-distance", type=float, default=0.0)
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
        uav_edge_type=args.uav_edge_type,
        uav_edge_distance=args.uav_edge_distance,
        vp_edge_type=args.vp_edge_type,
        vp_edge_distance=args.vp_edge_distance,
    )


if __name__ == "__main__":
    main()
