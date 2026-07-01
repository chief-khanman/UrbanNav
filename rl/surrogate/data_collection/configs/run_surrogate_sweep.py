"""
run_surrogate_sweep.py
=======================
Launch the surrogate-model data collection sweep: simulator instances swept
over UAV count (25..100 step 5) crossed with REPEATS_PER_COUNT distinct
seeds each, run against base_config_surrogate.yaml, with offline MP4-only
rendering.

Varying `simulator.seed` per UAV count matters because it changes which
vertiports get sampled (see `Airspace.add_n_random_vps_to_vplist`), i.e. it
produces a distinct initial-condition sample for the same UAV count. The
surrogate model (`GNNSurrogateBackbone.predict_episode_outcome`) is trained
on (initial conditions -> episode metric) pairs, so multiple seeds per count
are needed for it to generalize across UAV-count buckets rather than
memorizing one vertiport layout per count.

Total runs = len(count values) * REPEATS_PER_COUNT.

Usage::

    python -m rl.surrogate.data_collection.configs.run_surrogate_sweep \\
        --output-dir logs/surrogate_sweep_001
"""
#! remember to assign appropriate name to surrogate_sweep

from __future__ import annotations

import argparse
from pathlib import Path

from rl.surrogate.data_collection.parallel_runner import run_sweep

BASE_CONFIG = Path(__file__).parent / "base_config_surrogate.yaml"

BASE_SEED = 123
REPEATS_PER_COUNT = 5  #! increase to 10/15/20...LATER (number of distinct vertiport layouts sampled per UAV count)
SEEDS = list(range(BASE_SEED, BASE_SEED + REPEATS_PER_COUNT))

SWEEP_PARAMS = {
    "fleet_composition.0.count": list(range(25, 101, 5)),
    "simulator.seed": SEEDS,
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
