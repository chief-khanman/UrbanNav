"""
discovery.py
=============
Shared, dataset-agnostic helpers used by the per-model `prepare_*_data.py`
scripts: recursive episode-directory discovery and seeded directory-level
train/val/test splitting.

Splitting happens at the *episode-directory* level (not at the level of
materialized transitions/pairs) so that no two splits ever share data from
the same episode.
"""

from __future__ import annotations

import random
from pathlib import Path
from typing import List, Tuple

#           FOR  |  dynamics_models  |                  | value-network models |                  
REQUIRED_FILES = ("step_history.json", "metadata.json", "episode_metrics.json")


def discover_episode_dirs(root: str) -> List[str]:
    """Recursively find every directory under root containing all of
    REQUIRED_FILES. Handles arbitrarily nested layouts, 
    e.g.
    logs/sweep_run_Jun23/run_<hash>/episode_<n>_<ts>/.
    """
    root_path = Path(root)
    if not root_path.exists():
        return []
    dirs = {
        p.parent
        for p in root_path.rglob(REQUIRED_FILES[0])
        if all((p.parent / f).exists() for f in REQUIRED_FILES)
    }
    return sorted(str(d) for d in dirs)


def split_dirs(
    dirs: List[str],
    val_fraction: float,
    test_fraction: float,
    seed: int,
    allow_empty_splits: bool = False,
) -> Tuple[List[str], List[str], List[str]]:
    """Seeded shuffle + slice of dirs into (train_dirs, val_dirs, test_dirs).

    Raises ValueError if val_dirs or test_dirs end up empty, unless
    allow_empty_splits=True (then prints a warning and proceeds).
    """
    if val_fraction + test_fraction >= 1.0:
        raise ValueError("val_fraction + test_fraction must be < 1.0")

    shuffled = list(dirs)
    random.Random(seed).shuffle(shuffled)
    n = len(shuffled)
    n_val = int(round(n * val_fraction))
    n_test = int(round(n * test_fraction))

    val_dirs = shuffled[:n_val]
    test_dirs = shuffled[n_val:n_val + n_test]
    train_dirs = shuffled[n_val + n_test:]

    for name, split in (("val", val_dirs), ("test", test_dirs)):
        if split:
            continue
        msg = (
            f"0 dirs assigned to {name} split (only {n} episode dir(s) discovered "
            f"total). Pass a larger --logs-root, or --allow-empty-splits to proceed "
            f"anyway (e.g. for a single-run smoke test)."
        )
        if allow_empty_splits:
            print(f"[discovery] WARNING: {msg}")
        else:
            raise ValueError(msg)

    return train_dirs, val_dirs, test_dirs
