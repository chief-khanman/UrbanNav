"""Shared fixtures: generate a small batch of UrbanNav episodes whose logger
output the surrogate datasets can read from. Cached for the whole module so
the slow OSM-driven simulator only runs once across tests."""

import os
from typing import List

import pytest
import yaml

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..', '..', '..')
SAMPLE_CONFIG_PATH = os.path.abspath(os.path.join(PROJECT_ROOT, 'sample_config.yaml'))


def _write_fast_config(tmp_path, log_dir: str) -> str:
    with open(SAMPLE_CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    cfg['logging']['enabled'] = True
    cfg['logging']['log_dir'] = log_dir
    cfg['rendering']['enabled'] = False
    cfg['simulator']['total_timestep'] = 8
    cfg['airspace']['number_of_vertiports'] = 6
    cfg['fleet_composition'] = [
        {'type_name': 'STANDARD', 'count': 3, 'dynamics': 'PointMass',
         'controller': 'PIDPointMassController', 'sensor': 'PartialSensor',
         'planner': 'PointMass-PID'},
    ]
    path = os.path.join(str(tmp_path), 'fast_config.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f)
    return path


@pytest.fixture(scope='module')
def episode_logs_root(tmp_path_factory) -> str:
    """Run two short UrbanNav episodes and return the logs/ root holding them."""
    from urbannav.uam_simulator import UAMSimulator

    base = tmp_path_factory.mktemp('surrogate_logs')
    logs_root = os.path.join(str(base), 'logs')
    config_path = _write_fast_config(base, logs_root)

    for _ in range(2):
        sim = UAMSimulator(config_path=config_path)
        sim.reset()
        for _ in range(sim.total_timestep):
            sim.step({})
        sim.logger.save()

    return logs_root


@pytest.fixture(scope='module')
def episode_dirs(episode_logs_root) -> List[str]:
    from rl.surrogate.datasets.trajectory_dataset import discover_episode_dirs
    dirs = discover_episode_dirs(episode_logs_root)
    assert dirs, f'No episode logs found under {episode_logs_root}'
    return dirs
