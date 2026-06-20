"""
episode_outcome_dataset.py
============================
(start_condition, final_metric) PyTorch Dataset built from existing
logs/<episode>/metadata.json + episode_metrics.json files.

This is the AlphaZero-style value-target dataset: given an initial
configuration, predict an end-of-episode scalar (or vector) metric without
running the full simulator.

Start-condition features: a fixed-shape per-UAV padded tensor of initial
positions, plus a small set of episode-level scalars (UAV count, total
timestep budget).

Targets: configurable subset of the episode_metrics.json keys (default:
`avg_dist_to_goal_final` — a single regression scalar; pass `target_keys`
to select multiple scalars for multi-task value heads).
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch
from torch.utils.data import Dataset

START_POS_DIM: int = 3  # (x, y, z) per UAV
EPISODE_SCALAR_KEYS: Tuple[str, ...] = ('num_uavs_at_start', 'start_step')
DEFAULT_TARGET_KEYS: Tuple[str, ...] = ('avg_dist_to_goal_final',)


def discover_episode_dirs(logs_root: str) -> List[str]:
    """List immediate subdirectories of logs_root that contain BOTH
    metadata.json and episode_metrics.json."""
    root = Path(logs_root)
    if not root.exists():
        return []
    return sorted(
        str(p) for p in root.iterdir()
        if p.is_dir()
        and (p / 'metadata.json').exists()
        and (p / 'episode_metrics.json').exists()
    )


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, 'r') as f:
        return json.load(f)


class EpisodeOutcomeDataset(Dataset):
    """Predict end-of-episode metrics from initial conditions.

    Args:
        episode_dirs: directories each containing metadata.json AND
                      episode_metrics.json.
        max_uavs:     pad/truncate initial UAV positions to this size. Set
                      automatically from the data when None.
        target_keys:  episode_metrics.json keys to use as regression targets.
                      Default: ('avg_dist_to_goal_final',).
        normalize:    when True, z-score-normalize the start positions and
                      targets across the dataset. Stored statistics are
                      exposed via .stats for inverse-transforming predictions.
    """

    def __init__(
        self,
        episode_dirs: List[str],
        max_uavs: Optional[int] = None,
        target_keys: Tuple[str, ...] = DEFAULT_TARGET_KEYS,
        normalize: bool = False,
    ):
        self.episode_dirs = list(episode_dirs)
        self.target_keys = tuple(target_keys)
        self.normalize = bool(normalize)

        raw_starts: List[List[Tuple[float, float, float]]] = []
        raw_scalars: List[List[float]] = []
        raw_targets: List[List[float]] = []
        kept_dirs: List[str] = []

        for ep_dir in self.episode_dirs:
            md = _load_json(os.path.join(ep_dir, 'metadata.json'))
            em = _load_json(os.path.join(ep_dir, 'episode_metrics.json'))

            init_positions = md.get('initial_uav_positions') or {}
            if not init_positions:
                continue  # nothing to predict from

            pos_list = [
                (float(p.get('x', 0.0)), float(p.get('y', 0.0)), float(p.get('z', 0.0)))
                for p in init_positions.values()
            ]
            scalars = [float(md.get(k, 0) or 0) for k in EPISODE_SCALAR_KEYS]
            targets = [float(em.get(k, 0.0) or 0.0) for k in self.target_keys]

            raw_starts.append(pos_list)
            raw_scalars.append(scalars)
            raw_targets.append(targets)
            kept_dirs.append(ep_dir)

        if not raw_starts:
            self.max_uavs = max_uavs or 0
            self._start_positions = np.zeros((0, self.max_uavs, START_POS_DIM), dtype=np.float32)
            self._start_masks = np.zeros((0, self.max_uavs), dtype=np.float32)
            self._scalars = np.zeros((0, len(EPISODE_SCALAR_KEYS)), dtype=np.float32)
            self._targets = np.zeros((0, len(self.target_keys)), dtype=np.float32)
            self._source_dirs = []
            self.stats = {}
            return

        self.max_uavs = max_uavs if max_uavs is not None else max(len(p) for p in raw_starts)

        N = len(raw_starts)
        positions = np.zeros((N, self.max_uavs, START_POS_DIM), dtype=np.float32)
        masks = np.zeros((N, self.max_uavs), dtype=np.float32)
        for i, pos_list in enumerate(raw_starts):
            n = min(len(pos_list), self.max_uavs)
            for j in range(n):
                positions[i, j] = pos_list[j]
            masks[i, :n] = 1.0

        scalars = np.array(raw_scalars, dtype=np.float32)
        targets = np.array(raw_targets, dtype=np.float32)

        self.stats: Dict[str, np.ndarray] = {}
        if self.normalize:
            # Compute stats only over valid (masked) UAV positions
            mask_3d = np.repeat(masks[..., None], START_POS_DIM, axis=-1)
            n_valid = float(mask_3d.sum()) / START_POS_DIM
            if n_valid > 0:
                pos_mean = (positions * mask_3d).sum(axis=(0, 1)) / max(n_valid, 1.0)
                pos_var = ((positions - pos_mean) ** 2 * mask_3d).sum(axis=(0, 1)) / max(n_valid, 1.0)
                pos_std = np.sqrt(pos_var) + 1e-6
                positions = (positions - pos_mean) * mask_3d / pos_std
                self.stats['pos_mean'] = pos_mean
                self.stats['pos_std'] = pos_std

            tgt_mean = targets.mean(axis=0)
            tgt_std = targets.std(axis=0) + 1e-6
            targets = (targets - tgt_mean) / tgt_std
            self.stats['target_mean'] = tgt_mean
            self.stats['target_std'] = tgt_std

        self._start_positions = positions
        self._start_masks = masks
        self._scalars = scalars
        self._targets = targets
        self._source_dirs = kept_dirs

    def __len__(self) -> int:
        return len(self._start_positions)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        return {
            'positions': torch.from_numpy(self._start_positions[idx]),  # [max_uavs, 3]
            'mask': torch.from_numpy(self._start_masks[idx]),            # [max_uavs]
            'scalars': torch.from_numpy(self._scalars[idx]),             # [len(EPISODE_SCALAR_KEYS)]
            'target': torch.from_numpy(self._targets[idx]),              # [len(target_keys)]
        }

    def get_source(self, idx: int) -> str:
        return self._source_dirs[idx]

    @classmethod
    def from_logs_root(cls, logs_root: str, **kwargs) -> 'EpisodeOutcomeDataset':
        return cls(discover_episode_dirs(logs_root), **kwargs)
