"""Config schema for the synthetic dense-airspace testbed.

Reuses UAMSimulatorConfig / UAVFleetInstanceConfig / LoggingConfig / RenderingConfig
from urbannav.component_schema unchanged. Replaces the OSM-driven `vertiport`/`airspace`
sections with a single `testbed_airspace` section describing a synthetic, offline
vertiport/building layout (either a generated pattern or an explicit placement file).
"""
from typing import List, Optional, Tuple

import yaml
from pydantic import BaseModel, Field, ValidationError, field_validator

from urbannav.component_schema import (
    LoggingConfig,
    RenderingConfig,
    UAMSimulatorConfig,
    UAVFleetInstanceConfig,
    validate_fleet_composition,
)

VALID_TESTBED_PATTERNS: set = {'ring'}


class BuildingConfig(BaseModel):
    """One synthetic rectangular 'building' (restricted-area) footprint.

    width/depth describe the 2D footprint (x/y extents). z_height is the
    vertical extent used only for 3D rendering (collision/detection checks
    stay altitude-agnostic, matching the main simulator's existing semantics —
    a restricted area blocks the full vertical column regardless of z_height).
    """
    center: Tuple[float, float]
    width: float
    depth: float
    buffer_radius: float = 500.0
    z_height_mean: Optional[float] = None
    z_height_std: Optional[float] = None

    @field_validator('width', 'depth', 'buffer_radius')
    @classmethod
    def must_be_positive(cls, v: float) -> float:
        if v <= 0:
            raise ValueError(f"must be > 0, got {v}")
        return v


class TestbedAirspaceConfig(BaseModel):
    """Describes a synthetic, offline vertiport/building layout.

    Two mutually exclusive placement strategies:
      - Procedural: pattern='ring' + num_vertiports/center/radius/start_angle generate
        vertiport positions on the fly (n=4 -> exact square corners).
      - File-based: placement_file points at a text file with explicit boundary +
        vertiport + building positions (see testbed.placement.load_placement_file).
        When set, the procedural fields above are ignored.
    """
    __test__ = False  # not a pytest test class — name just starts with "Testbed"

    pattern: str = 'ring'
    num_vertiports: int = 4
    center: Tuple[float, float] = (0.0, 0.0)
    radius: float = 2000.0
    start_angle: float = 0.0
    landing_pad_capacity: int = 1
    altitude_range: Tuple[float, float] = (1500.0, 3500.0)
    building_z_height_mean: float = 50.0
    building_z_height_std: float = 10.0
    buildings: List[BuildingConfig] = Field(default_factory=list)
    placement_file: Optional[str] = None
    # Footprint/buffer defaults applied to every building placed via placement_file
    # (the file only supplies positions — see testbed.placement.load_placement_file).
    building_width_default: float = 200.0
    building_depth_default: float = 200.0
    building_buffer_radius_default: float = 500.0

    @field_validator('pattern')
    @classmethod
    def pattern_must_be_valid(cls, v: str) -> str:
        if v not in VALID_TESTBED_PATTERNS:
            raise ValueError(f"pattern must be one of {sorted(VALID_TESTBED_PATTERNS)}, got '{v}'")
        return v

    @field_validator('num_vertiports')
    @classmethod
    def num_vertiports_must_be_positive(cls, v: int) -> int:
        if v < 1:
            raise ValueError(f"num_vertiports must be >= 1, got {v}")
        return v

    @field_validator('landing_pad_capacity')
    @classmethod
    def landing_pad_capacity_must_be_positive(cls, v: int) -> int:
        if v < 1:
            raise ValueError(f"landing_pad_capacity must be >= 1, got {v}")
        return v


class TestbedConfig(BaseModel):
    """Root config object for the testbed — mirrors UAMConfig.load_from_yaml's shape,
    but with `testbed_airspace` in place of `vertiport`/`airspace`."""
    simulator: UAMSimulatorConfig
    testbed_airspace: TestbedAirspaceConfig
    fleet_composition: List[UAVFleetInstanceConfig]
    logging: LoggingConfig = Field(default_factory=LoggingConfig)
    rendering: RenderingConfig = Field(default_factory=RenderingConfig)

    @classmethod
    def load_from_yaml(cls, path: str) -> 'TestbedConfig':
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        try:
            config = cls(**data)
        except ValidationError as e:
            print('Configuration Error: The testbed config file format is wrong')
            print(e)
            raise
        validate_fleet_composition(config)
        return config
