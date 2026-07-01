"""TestbedSimulatorManager — swaps OSM-backed Airspace construction for a synthetic
TestbedAirspace. Everything else (ATC, the four engines, the per-step pipeline, the
ATC mission cycle) is inherited unchanged from SimulatorManager.
"""
from urbannav.simulator_manager import SimulatorManager

from testbed.testbed_airspace import TestbedAirspace


class TestbedSimulatorManager(SimulatorManager):
    """SimulatorManager subclass that builds a TestbedAirspace instead of an
    OSM-backed Airspace. Requires self.config.testbed_airspace (TestbedAirspaceConfig)
    in place of the usual config.vertiport/config.airspace sections."""

    def _init_airspace(self) -> TestbedAirspace:
        return TestbedAirspace(self.config.testbed_airspace, seed=self.seed)

    def _build_vertiports_random(self) -> None:
        """No-op: TestbedAirspace already builds vertiport_list deterministically
        from its pattern/placement file at construction time in _init_airspace()."""
        pass
