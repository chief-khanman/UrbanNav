"""
run_surrogate_sweep.py
=======================
Launch the surrogate-model data collection sweep: 4 simulator instances,
one per UAV count (25/50/75/100), each run for 10,000 steps against
base_config_surrogate.yaml, with offline MP4-only rendering.

Usage::

    python -m rl.surrogate.data_collection.configs.run_surrogate_sweep \\
        --output-dir logs/surrogate_sweep_001
"""

from __future__ import annotations

import argparse
from pathlib import Path

from rl.surrogate.data_collection.parallel_runner import run_sweep

BASE_CONFIG = Path(__file__).parent / "base_config_surrogate.yaml"

SWEEP_PARAMS = {
    "fleet_composition.0.count": [25, 50, 75, 100],
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", required=True, help="Root output directory for run logs/renders"
    )
    args = parser.parse_args()

    run_sweep(
        base_config_path=str(BASE_CONFIG),
        sweep_params=SWEEP_PARAMS,
        output_dir=args.output_dir,
        num_workers=4,
        render=True,
    )


if __name__ == "__main__":
    main()
