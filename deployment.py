"""deployment.py — Quick test runner for the UAM simulator and metrics logger.

Run from the UrbanNav directory:
    python deployment.py
"""
import os

from uam_simulator import UAMSimulator

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'sample_config.yaml')


def main() -> None:
    sim = UAMSimulator(config_path=CONFIG_FILE)
    sim.reset()

    print(f'\nRunning {sim.total_timestep} steps '
          f'(logging enabled: {sim.logger.enabled})\n')

    for step in range(sim.total_timestep):
        sim.step({})   # empty command bundle — all UAVs use internal controllers

        if step % 10 == 0:
            num_uavs = len(sim.simulator_manager.atc.uav_dict)
            print(f'  step {step:4d}/{sim.total_timestep} | active UAVs: {num_uavs}')

    sim.logger.save()

    metrics = sim.logger.get_simulator_end_metrics()
    print('\n--- Episode Metrics ---')
    for key, value in metrics.items():
        print(f'  {key}: {value}')


if __name__ == '__main__':
    main()
