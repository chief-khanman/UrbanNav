"""
prepare_episode_outcome_data.py
==================================
Data processor for the `episode_outcome` dataset (EpisodeOutcomeDataset),
feeding the one `value`-category model that exists today: GNNSurrogateBackbone
's predict_episode_outcome path.

First-pass scope: this processor does NOT support normalization yet
(TODO below) -- the target-metric set (currently just
avg_dist_to_goal_final) is expected to expand to transportation-relevant
metrics (completed-mission count, avg NMAC count, etc.) in a future round,
and normalization is better designed once that set stabilizes.

Discovers episode directories under a single raw-logs root (recursively),
splits them at the directory level into train/val/test, builds one
EpisodeOutcomeDataset per split from its disjoint directory list (with
max_uavs fixed from the train split so padded shapes match across splits),
and saves each as a flat list.

Usage::

    python -m rl.surrogate.datasets.prepare_episode_outcome_data \\
        --logs-root logs/sweep_run_Jun23 \\
        --output-dir logs/prepared/episode_outcome_sweep_Jun23
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from rl.surrogate.datasets.discovery import discover_episode_dirs, split_dirs
from rl.surrogate.datasets.episode_outcome_dataset import DEFAULT_TARGET_KEYS, EpisodeOutcomeDataset


def prepare(
    logs_root: str,
    output_dir: str,
    val_fraction: float = 0.2,
    test_fraction: float = 0.1,
    seed: int = 0,
    allow_empty_splits: bool = False,
    max_uavs: int = None,
    target_keys=None,
) -> None:
    target_keys = tuple(target_keys) if target_keys else DEFAULT_TARGET_KEYS

    dirs = discover_episode_dirs(logs_root)
    if not dirs:
        raise RuntimeError(f"No episode directories found under {logs_root!r}.")

    train_dirs, val_dirs, test_dirs = split_dirs(
        dirs, val_fraction, test_fraction, seed, allow_empty_splits,
    )

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # TODO: add train-only normalization (z-score positions/targets using
    # stats computed from the train split only, applied identically to
    # val/test) once the target-metric set stabilizes. Deliberately omitted
    # in this first pass -- see module docstring.
    train_ds = EpisodeOutcomeDataset(train_dirs, max_uavs=max_uavs, target_keys=target_keys, normalize=False)
    if len(train_ds) == 0:
        raise RuntimeError("episode_outcome: train split is empty after discovery/filtering.")
    fixed_max_uavs = train_ds.max_uavs  # pin so val/test get identical padded shape

    counts = {"train": {"dirs": len(train_dirs), "episodes": len(train_ds)}}
    torch.save([train_ds[i] for i in range(len(train_ds))], out / "train.pt")

    for split_name, split_dir_list in (("val", val_dirs), ("test", test_dirs)):
        ds = EpisodeOutcomeDataset(
            split_dir_list, max_uavs=fixed_max_uavs, target_keys=target_keys, normalize=False,
        )
        counts[split_name] = {"dirs": len(split_dir_list), "episodes": len(ds)}
        if len(ds) > 0:
            torch.save([ds[i] for i in range(len(ds))], out / f"{split_name}.pt")

    metadata = {
        "task": "episode_outcome",
        "max_uavs": fixed_max_uavs,
        "target_keys": list(target_keys),
        "logs_root": logs_root,
        "val_fraction": val_fraction,
        "test_fraction": test_fraction,
        "seed": seed,
        "counts": counts,
    }
    with open(out / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"[prepare_episode_outcome_data] wrote {output_dir}: {counts}")


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--logs-root", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--val-fraction", type=float, default=0.2)
    p.add_argument("--test-fraction", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--allow-empty-splits", action="store_true")
    p.add_argument("--max-uavs", type=int, default=None)
    p.add_argument("--target-keys", nargs="+", default=None)
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
        max_uavs=args.max_uavs,
        target_keys=args.target_keys,
    )


if __name__ == "__main__":
    main()
