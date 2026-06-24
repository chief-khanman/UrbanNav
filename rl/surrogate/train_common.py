"""
train_common.py
=================
Generic plumbing shared by the per-model train_<variant>.py scripts: device
auto-detection, a thin Dataset wrapper around a pre-materialized list, loaders
for the {train,val,test}.pt files written by the prepare_*_data.py scripts,
and a metadata.json task-match safety check.

Model-specific training-loop logic does NOT live here -- each train_<variant>.py
owns its own loop.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional

import torch


def auto_device() -> str:
    """Pick the best available device: cuda > mps > cpu."""
    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


class ListDataset(torch.utils.data.Dataset):
    """Thin wrapper around a pre-materialized list of items, so DataLoader's
    __len__/__getitem__ contract is satisfied."""

    def __init__(self, items: list):
        self._items = items

    def __len__(self) -> int:
        return len(self._items)

    def __getitem__(self, idx: int):
        return self._items[idx]


def _load_pt(data_dir: str, split_name: str):
    path = Path(data_dir) / f"{split_name}.pt"
    if not path.exists():
        return None
    # weights_only=False: these files are produced by our own prepare_*_data.py
    # scripts from a trusted local source (not downloaded checkpoints), so the
    # security rationale behind PyTorch's weights_only=True default doesn't
    # apply here -- and it can't load arbitrary lists of Data/HeteroData anyway.
    return torch.load(path, weights_only=False)


def load_flat_split(data_dir: str, split_name: str) -> Optional[ListDataset]:
    """For dual_graph, episode_outcome, and graph_flow (non-recurrent): loads
    {split}.pt, flattening a nested List[List[item]] into one flat list if
    needed. Returns None if the file doesn't exist."""
    raw = _load_pt(data_dir, split_name)
    if raw is None:
        return None
    if raw and isinstance(raw[0], list):
        flat = [item for episode in raw for item in episode]
    else:
        flat = raw
    return ListDataset(flat)


def load_episode_splits(data_dir: str, split_name: str) -> Optional[List[list]]:
    """For graph_flow_recurrent: loads {split}.pt and returns the nested
    List[List[Tuple[Data,Data]]] as-is (one inner list per episode, in
    original temporal order). Returns None if the file doesn't exist."""
    return _load_pt(data_dir, split_name)


def check_metadata_task(data_dir: str, expected_task: str) -> None:
    """Raises ValueError if <data_dir>/metadata.json exists and its recorded
    'task' != expected_task."""
    path = Path(data_dir) / "metadata.json"
    if not path.exists():
        return
    with open(path) as f:
        meta = json.load(f)
    recorded_task = meta.get("task")
    if recorded_task is not None and recorded_task != expected_task:
        raise ValueError(
            f"data_dir {data_dir!r} was prepared for task '{recorded_task}' "
            f"(per metadata.json) but this script expects '{expected_task}'. "
            "Point --data-dir at output from the matching prepare_*_data.py script."
        )
