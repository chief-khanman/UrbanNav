"""deployment_vp_design.py — Quick test runner for the UAM simulator (VP Design variant).

Run from the UrbanNav directory:
    python deployment_vp_design.py
    python deployment_vp_design.py --od-matrix path/to/od_lambda_matrix.npy

Rendering is controlled entirely by sample_config.yaml:
    rendering.enabled      — true/false
    rendering.mode         — 'offline', 'realtime', or 'both'
    rendering.frame_skip   — render every N+1 steps (reduces animation size)
    rendering.output_dir   — where to write GIF/MP4 files

OD matrix (--od-matrix):
    Optional path to the Band 1 OD lambda matrix output.
    Accepts .npy (numpy array) or .csv (comma-delimited) format.
    This is a one-time data entry for the vertiport design problem — the same
    matrix is used for all episodes within a single run.
    If omitted, the simulator falls back to random mission assignment
    (demand-unaware, same behaviour as the base UrbanNav simulator).
"""

import argparse
import os

from uam_simulator_vp_design import UAMSimulatorVPDesign

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'sample_config.yaml')


def main() -> None:
    parser = argparse.ArgumentParser(description='UAM VP Design deployment test runner')
    parser.add_argument(
        '--od-matrix',
        default=None,
        metavar='PATH',
        help='Path to Band 1 OD lambda matrix (.npy or .csv). '
             'If omitted, simulator uses random mission assignment.'
    )
    parser.add_argument(
        '--zone-region-map',
        default=None,
        metavar='PATH',
        help='Path to Band 1 zone-region mapping (zone_region_map.csv). '
             'Two-column CSV: zone_id, region_id. Static for the run — '
             'does not change between episodes. Used by the RL env to build '
             'vertiport_region_map per episode. Omit in standalone test mode.'
    )
    args = parser.parse_args()

    sim = UAMSimulatorVPDesign(
        config_path=CONFIG_FILE,
        od_matrix_path=args.od_matrix,
        zone_region_map_path=args.zone_region_map,
    )
    sim.reset()

    rendering_on = sim.renderer.enabled
    print(f'\nRunning {sim.total_timestep} steps '
          f'(logging: {sim.logger.enabled} | '
          f'rendering: {rendering_on}'
          + (f', mode={sim.renderer.mode}' if rendering_on else '')
          + ')\n')

    for step in range(sim.total_timestep):
        sim.step({})   # empty command bundle — all UAVs use internal controllers

        if step % 100 == 0:
            num_uavs = len(sim.simulator_manager.atc.uav_dict)
            print(f'  step {step:4d}/{sim.total_timestep} | active UAVs: {num_uavs}')

    # --- save standard logger metrics ---
    sim.logger.save()

    metrics = sim.logger.get_simulator_end_metrics()
    print('\n--- Simulator End Metrics ---')
    for key, value in metrics.items():
        print(f'  {key}: {value}')

    # --- VP design episode metrics (Group B observations) ---
    # These are the metrics consumed by the GNN-RL module for the VP design
    # problem. Printed here for test verification; in the RL pipeline this
    # dict is passed directly to the environment's observation builder.
    episode_metrics = sim.simulator_manager.get_episode_metrics()
    print('\n--- VP Design Episode Metrics (Group B) ---')
    for key, value in episode_metrics.items():
        print(f'  {key}: {value}')

    # --- save offline animation (no-op if rendering disabled or mode=realtime) ---
    sim.render()


if __name__ == '__main__':
    main()
