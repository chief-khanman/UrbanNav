"""
Shared pytest fixtures for the UAM Simulator test suite.

All tests import from the project root. pytest discovers this conftest.py
and adds the root to sys.path so imports work without a package install.
"""
import sys
import os
import pytest

# Add project root to path so all simulator modules are importable
PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.abspath(PROJECT_ROOT))

from uam_simulator import UAMSimulator

CONFIG_PATH = os.path.join(PROJECT_ROOT, 'sample_config.yaml')


@pytest.fixture(scope='module')
def config_path():
    return os.path.abspath(CONFIG_PATH)


@pytest.fixture(scope='module')
def sim(config_path):
    """
    Create and reset a UAMSimulator instance once per test module.
    scope='module' avoids re-fetching OSM map data for every test.
    """
    simulator = UAMSimulator(config_path=config_path)
    simulator.reset()
    return simulator
