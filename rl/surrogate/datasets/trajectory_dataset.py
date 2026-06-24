"""
trajectory_dataset.py
======================
(state, action, next_state) PyTorch Dataset built from existing
logs/<episode>/step_history.json files produced by UrbanNav's Logger.

UAV state per step is `[x, y, z, vx, vy, vz, speed, heading]` (8 features,
read directly from MetricsCollector.record()'s uav_snapshots — no new
logging format needed).

Action per UAV per step is `[ax, ay]` (or whatever 2-vec the controller
emits; absent UAVs get zeros). Step records where actions is empty are
skipped — they don't carry a supervisable transition.

Each dataset item is one (uav_id, step) pair flattened into a fixed-shape
tuple: (state[8], action[2], next_state[8]). UAVs that exit/enter the
fleet mid-episode are handled by only emitting transitions where the UAV
is present in BOTH step t and step t+1.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import torch
from torch.utils.data import Dataset

UAV_STATE_KEYS: Tuple[str, ...] = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'speed', 'heading')
UAV_STATE_DIM: int = len(UAV_STATE_KEYS)
UAV_ACTION_DIM: int = 2


def _extract_state(snap: Dict[str, Any]) -> np.ndarray:
    return np.array([float(snap.get(k, 0.0) or 0.0) for k in UAV_STATE_KEYS], dtype=np.float32)


def _extract_action(action_entry: Any) -> np.ndarray:
    """Coerce a logged action entry to a fixed-shape (2,) float32 array.
    Logger serializes actions as lists/tuples or scalars — fall back to zeros."""
    if action_entry is None:
        return np.zeros(UAV_ACTION_DIM, dtype=np.float32)
    if isinstance(action_entry, (list, tuple, np.ndarray)):
        arr = np.asarray(action_entry, dtype=np.float32).flatten()
        out = np.zeros(UAV_ACTION_DIM, dtype=np.float32)
        n = min(len(arr), UAV_ACTION_DIM)
        out[:n] = arr[:n]
        return out
    try:
        return np.array([float(action_entry), 0.0], dtype=np.float32)
    except (TypeError, ValueError):
        return np.zeros(UAV_ACTION_DIM, dtype=np.float32)


def discover_episode_dirs(logs_root: str) -> List[str]:
    """List immediate subdirectories of logs_root that contain a step_history.json."""
    root = Path(logs_root)
    if not root.exists():
        return []
    return sorted(
        str(p) for p in root.iterdir()
        if p.is_dir() and (p / 'step_history.json').exists()
    )


class TrajectoryDataset(Dataset):
    """(state, action, next_state) transitions from logged step histories.

    Args:
        episode_dirs: list of directories each containing step_history.json.
                      Pass either explicit paths or use discover_episode_dirs()
                      to scan a logs_root.
        include_actionless: when True (default), zero-fill the action vector
                            for transitions whose step record has an empty
                            `actions` dict. UAMSimulator currently invokes
                            Logger.log_step without an actions argument, so
                            most logged transitions are actionless — defaulting
                            to True keeps the dataset usable. Set False to
                            require non-empty action records (useful for
                            sub-datasets gathered with controllers that DO log
                            their actions through Logger).
    """

    def __init__(self, episode_dirs: List[str], include_actionless: bool = True):
        self.episode_dirs = list(episode_dirs)
        self.include_actionless = include_actionless

        # Materialize (state, action, next_state) triples eagerly. Episodes are
        # small JSON files; loading once at __init__ keeps __getitem__ fast and
        # decouples the dataset from disk I/O during training.
        self._states: List[np.ndarray] = []
        self._actions: List[np.ndarray] = []
        self._next_states: List[np.ndarray] = []
        self._source_keys: List[Tuple[str, int, int]] = []  # (episode_dir, step_t, uav_id) for debugging

        for ep_dir in self.episode_dirs:
            self._load_episode(ep_dir)

    def _load_episode(self, episode_dir: str) -> None:
        path = os.path.join(episode_dir, 'step_history.json')
        with open(path, 'r') as f:
            steps: List[Dict[str, Any]] = json.load(f)

        # Sort by step index — logger writes in order, but be defensive.
        steps.sort(key=lambda s: s['step'])

        for t in range(len(steps) - 1):
            s_t = steps[t]
            s_tp1 = steps[t + 1]
            actions_dict = s_t.get('actions') or {}
            if not actions_dict and not self.include_actionless:
                continue

            uavs_t = s_t.get('uavs') or {}
            uavs_tp1 = s_tp1.get('uavs') or {}
            common = set(uavs_t.keys()) & set(uavs_tp1.keys())

            for uid in common:
                state = _extract_state(uavs_t[uid])
                next_state = _extract_state(uavs_tp1[uid])
                action = _extract_action(actions_dict.get(uid) if actions_dict else None)
                self._states.append(state)
                self._actions.append(action)
                self._next_states.append(next_state)
                try:
                    uid_int = int(uid)
                except (TypeError, ValueError):
                    uid_int = -1
                self._source_keys.append((episode_dir, int(s_t['step']), uid_int))

    def __len__(self) -> int:
        return len(self._states)

    def __getitem__(self, idx: int) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        return (
            torch.from_numpy(self._states[idx]),
            torch.from_numpy(self._actions[idx]),
            torch.from_numpy(self._next_states[idx]),
        )

    def get_source(self, idx: int) -> Tuple[str, int, int]:
        """Return (episode_dir, step_t, uav_id) for the transition at idx."""
        return self._source_keys[idx]

    # from LOG directories root, example: ~/Dev/UrbanNav/logs/sweep<number>/run<hash>/episode_metric.json
    @classmethod
    def from_logs_root(
        cls, logs_root: Union[str, List[str]], **kwargs
    ) -> 'TrajectoryDataset':
        """Convenience constructor: build from every episode under logs_root.

        Args:
            logs_root: Single path or list of paths.  When a list is given,
                episodes from all directories are merged into one dataset.
        """
        if isinstance(logs_root, str):
            logs_root = [logs_root]

        all_dirs: List[str] = []
        for root_path in logs_root:
            all_dirs.extend(discover_episode_dirs(root_path))
        return cls(all_dirs, **kwargs)
