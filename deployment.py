"""deployment.py — Quick test runner for the UAM simulator, metrics logger, and renderer.

Run from the UrbanNav directory:
    python deployment.py

Rendering is controlled entirely by sample_config.yaml:
    rendering.enabled      — true/false
    rendering.mode         — 'offline', 'realtime', or 'both'
    rendering.frame_skip   — render every N+1 steps (reduces animation size)
    rendering.output_dir   — where to write GIF/MP4 files
"""
import os

from uam_simulator import UAMSimulator

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'sample_config.yaml')


def main() -> None:
    sim = UAMSimulator(config_path=CONFIG_FILE)
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

    # --- save metrics ---
    sim.logger.save()

    metrics = sim.logger.get_simulator_end_metrics()
    print('\n--- Episode Metrics ---')
    for key, value in metrics.items():
        print(f'  {key}: {value}')

    # --- save offline animation (no-op if rendering disabled or mode=realtime) ---
    sim.render()


if __name__ == '__main__':
    main()
