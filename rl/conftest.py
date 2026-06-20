"""Shared pytest fixtures for rl/**/tests/ — fast, disabled-logging/rendering
configs built from sample_config.yaml's simulator/vertiport/airspace sections,
with fleet_composition swapped per test scenario.
"""
import os

import pytest
import yaml

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..')
SAMPLE_CONFIG_PATH = os.path.join(PROJECT_ROOT, 'sample_config.yaml')


def _load_base_config() -> dict:
    with open(SAMPLE_CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    cfg['logging']['enabled'] = False
    cfg['rendering']['enabled'] = False
    cfg['simulator']['total_timestep'] = 30
    return cfg


def _write_config(tmp_path, cfg: dict) -> str:
    path = os.path.join(str(tmp_path), 'test_config.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f)
    return path


@pytest.fixture
def single_agent_config_path(tmp_path):
    """1 STANDARD + 1 SINGLE_AGENT_LEARNING (mode=TRAIN, count=1)."""
    cfg = _load_base_config()
    cfg['fleet_composition'] = [
        {'type_name': 'STANDARD', 'count': 1, 'dynamics': 'PointMass',
         'controller': 'PIDPointMassController', 'sensor': 'PartialSensor',
         'planner': 'PointMass-PID'},
        {'type_name': 'SINGLE_AGENT_LEARNING', 'count': 1, 'dynamics': 'PointMass',
         'controller': 'RL', 'sensor': 'PartialSensor', 'planner': 'PointMass-PID',
         'mode': 'TRAIN', 'policy_id': 'single_agent_test'},
    ]
    return _write_config(tmp_path, cfg)


@pytest.fixture
def multi_agent_config_path(tmp_path):
    """1 STANDARD + 2 MULTI_AGENT_LEARNING sharing one policy_id."""
    cfg = _load_base_config()
    cfg['fleet_composition'] = [
        {'type_name': 'STANDARD', 'count': 1, 'dynamics': 'PointMass',
         'controller': 'PIDPointMassController', 'sensor': 'PartialSensor',
         'planner': 'PointMass-PID'},
        {'type_name': 'MULTI_AGENT_LEARNING', 'count': 2, 'dynamics': 'PointMass',
         'controller': 'RL', 'sensor': 'PartialSensor', 'planner': 'PointMass-PID',
         'mode': 'TRAIN', 'policy_id': 'shared_policy'},
    ]
    return _write_config(tmp_path, cfg)


@pytest.fixture
def multi_policy_config_path(tmp_path):
    """2 MULTI_AGENT_LEARNING UAVs in two distinct, single-member policy_id groups."""
    cfg = _load_base_config()
    cfg['fleet_composition'] = [
        {'type_name': 'MULTI_AGENT_LEARNING', 'count': 1, 'dynamics': 'PointMass',
         'controller': 'RL', 'sensor': 'PartialSensor', 'planner': 'PointMass-PID',
         'mode': 'TRAIN', 'policy_id': 'policy_a'},
        {'type_name': 'MULTI_AGENT_LEARNING', 'count': 1, 'dynamics': 'PointMass',
         'controller': 'RL', 'sensor': 'PartialSensor', 'planner': 'PointMass-PID',
         'mode': 'TRAIN', 'policy_id': 'policy_b'},
    ]
    return _write_config(tmp_path, cfg)
