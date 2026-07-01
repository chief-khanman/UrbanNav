"""TestbedSimulator — drop-in UAMSimulator subclass for the synthetic dense-airspace
testbed. Overrides only config loading, simulator-manager construction, and renderer
construction; reset()/step()/render()/get_state() are all inherited unchanged from
UAMSimulator, so existing RL gym envs (rl/single_agent, rl/multi_agent) that accept a
constructed simulator instance work against it without modification.
"""
from typing import Dict, Optional

import numpy as np

from urbannav.uam_simulator import UAMSimulator

from testbed.config_schema import TestbedConfig
from testbed.testbed_renderer import TestbedRenderer
from testbed.testbed_simulator_manager import TestbedSimulatorManager


class TestbedSimulator(UAMSimulator):
    __test__ = False  # not a pytest test class — name just starts with "Testbed"

    def _load_config(self, config_path: str) -> TestbedConfig:
        return TestbedConfig.load_from_yaml(config_path)

    def _build_simulator_manager(
        self,
        lambda_matrix: Optional[np.ndarray],
        vertiport_region_map: Optional[Dict[int, int]],
        zone_region_map: Optional[Dict],
    ) -> TestbedSimulatorManager:
        return TestbedSimulatorManager(
            self.config, lambda_matrix, vertiport_region_map, zone_region_map
        )

    def _build_renderer(self) -> TestbedRenderer:
        return TestbedRenderer(self.config.rendering, self.config.simulator.mode)
