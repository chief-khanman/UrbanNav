"""
sweep_config.py
================
Generate a grid of simulator configurations for surrogate model data collection.

Takes a base YAML config (e.g. sample_config.yaml) and sweeps over user-specified
parameter ranges, producing one config dict per combination.

Usage (as library)::

    from rl.surrogate.data_collection.sweep_config import generate_sweep

    configs = generate_sweep(
        base_config_path="sample_config.yaml",
        sweep_params={
            "fleet_composition.0.count": [3, 5, 10],
            "airspace.number_of_vertiports": [10, 20],
            "simulator.seed": [42, 123, 456],
        },
    )
    # => 3 * 2 * 3 = 18 config dicts

Usage (CLI)::

    python -m rl.surrogate.data_collection.sweep_config \\
        --base-config sample_config.yaml \\
        --sweep "fleet_composition.0.count=[3,5,10]" \\
        --sweep "airspace.number_of_vertiports=[10,20]" \\
        --sweep "simulator.seed=[42,123]"
"""

from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import json
from itertools import product
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple, Union

import yaml


def _set_nested(d: Dict, dotted_key: str, value: Any) -> None:
    """Set a value in a nested dict using dot notation.

    List indices are supported via integer keys (e.g. ``fleet_composition.0.count``).
    """
    parts = dotted_key.split(".")
    obj: Any = d
    for part in parts[:-1]:
        if isinstance(obj, list):
            obj = obj[int(part)]
        else:
            obj = obj[part]
    last = parts[-1]
    if isinstance(obj, list):
        obj[int(last)] = value
    else:
        obj[last] = value


def _run_id(config: Dict) -> str:
    """Deterministic short hash of a config dict for unique directory naming."""
    blob = json.dumps(config, sort_keys=True, default=str).encode()
    return hashlib.sha256(blob).hexdigest()[:10]


def generate_sweep(
    base_config_path: Union[str, Path],
    sweep_params: Dict[str, Sequence[Any]],
) -> List[Tuple[str, Dict]]:
    """Generate (run_id, config_dict) pairs for every parameter combination.

    Args:
        base_config_path: Path to the template YAML config file.
        sweep_params: Mapping of dot-separated config keys to lists of values
            to sweep.  Example::

                {
                    "fleet_composition.0.count": [3, 5, 10],
                    "airspace.number_of_vertiports": [10, 20],
                    "simulator.seed": [42, 123],
                }

    Returns:
        List of (run_id, config_dict) tuples.  run_id is a short hash
        suitable for use as a directory name suffix.
    """
    with open(base_config_path, "r") as f:
        base = yaml.safe_load(f)

    keys = list(sweep_params.keys())
    value_lists = [sweep_params[k] for k in keys]

    configs: List[Tuple[str, Dict]] = []
    for combo in product(*value_lists):
        cfg = copy.deepcopy(base)
        for key, val in zip(keys, combo):
            _set_nested(cfg, key, val)
        rid = _run_id(cfg)
        configs.append((rid, cfg))

    return configs


def _parse_sweep_arg(arg: str) -> Tuple[str, List[Any]]:
    """Parse a CLI --sweep argument like ``key=[1,2,3]``."""
    key, raw = arg.split("=", 1)
    values = ast.literal_eval(raw)
    if not isinstance(values, list):
        values = [values]
    return key.strip(), values


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate sweep configs for data collection")
    parser.add_argument("--base-config", required=True, help="Path to base YAML config")
    parser.add_argument(
        "--sweep",
        action="append",
        required=True,
        help="Sweep param in format key=[v1,v2,...]. Repeat for multiple params.",
    )
    args = parser.parse_args()

    sweep_params: Dict[str, List[Any]] = {}
    for s in args.sweep:
        key, vals = _parse_sweep_arg(s)
        sweep_params[key] = vals

    configs = generate_sweep(args.base_config, sweep_params)
    print(f"Generated {len(configs)} configurations:")
    for rid, cfg in configs:
        uav_count = cfg.get("fleet_composition", [{}])[0].get("count", "?")
        n_vp = cfg.get("airspace", {}).get("number_of_vertiports", "?")
        seed = cfg.get("simulator", {}).get("seed", "?")
        print(f"  {rid}  uavs={uav_count}  vps={n_vp}  seed={seed}")


if __name__ == "__main__":
    main()
