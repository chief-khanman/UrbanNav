import math
import yaml
from enum import Enum, auto
from typing import Any, List, Dict, Tuple, Optional

from dataclasses import dataclass, field
from pydantic import BaseModel, Field, ValidationError, field_validator, model_validator

from shapely import Point

from vertiport import Vertiport
from uav_template import UAV_template
from uav import UAV

from dataclasses import dataclass, asdict
import json

#### ------------ CONFIG ------------ ####

# Reserved type name for RL-training UAVs — controller is assigned at training time.
RESERVED_TYPE_LEARNING = 'LEARNING'

# Valid string identifiers for each component type.
# Dynamics, controller, sensor classes are wired separately once those modules are ready.
VALID_DYNAMICS: set[str] = {'PointMass', 'SixDOF', 'TwoDVector', 'ORCA'}
VALID_CONTROLLERS: set[str] = {'PID', 'LQR', 'MARL', 'ORCA', 'Static', 'RL'}
VALID_SENSORS: set[str] = {'PartialSensor', 'GlobalSensor', 'MapSensor'}
VALID_PLANNERS: set[str] = {'PointMass-PID', 'PointMass-RL', 'SixDOF-PID', 'SixDOF-LQR', 'N/A'}

# UAV type registry — physical parameters live here in code, not in the yaml.
# fleet_composition.type_name values must match a key in this dict.
# To add a new UAV type: add the key to VALID_UAVS and its UAVTypeConfig entry below.
VALID_UAVS: set[str] = {
    'STANDARD',
    'HEAVY',
    'LEARNING',   # reserved — used only when training RL-based controllers
    'ORCA',
}

UAV_TYPE_REGISTRY: Dict[str, 'UAVTypeConfig'] = {}  # populated after UAVTypeConfig is defined




class UAMSimulatorConfig(BaseModel):
    dt: float
    total_timestep: int
    mode: str   # '2D' or '3D'
    seed: int


class VertiportConfig(BaseModel):
    number_of_landing_pad: int


class AirspaceConfig(BaseModel):
    location_name: str
    number_of_vertiports: int
    vertiport_tag_list: List[List[str]]
    airspace_restricted_area_tag_list: List[List[str]]


class UAVTypeConfig(BaseModel):
    """Physical and kinematic parameters for a UAV type.
    Defined in code via UAV_TYPE_REGISTRY — not loaded from yaml.
    fleet_composition.type_name values resolve against this registry.
    """
    radius: float
    nmac_radius: float
    detection_radius: float
    max_speed: float
    max_acceleration: float
    max_heading_change: float   # radians
    max_velocity: float


# Populate the registry now that UAVTypeConfig is defined.
# Each key must appear in VALID_UAVS.
UAV_TYPE_REGISTRY.update({
    'STANDARD': UAVTypeConfig(
        radius=0.5,
        nmac_radius=1.0,
        detection_radius=10.0,
        max_speed=10.0,
        max_acceleration=3.0,
        max_heading_change=math.pi,
        max_velocity=15.0,
    ),
    'HEAVY': UAVTypeConfig(
        radius=1.0,
        nmac_radius=2.0,
        detection_radius=15.0,
        max_speed=7.0,
        max_acceleration=1.5,
        max_heading_change=math.pi,
        max_velocity=10.0,
    ),
    'LEARNING': UAVTypeConfig(
        radius=0.5,
        nmac_radius=1.0,
        detection_radius=10.0,
        max_speed=10.0,
        max_acceleration=3.0,
        max_heading_change=math.pi,
        max_velocity=15.0,
    ),
    'ORCA': UAVTypeConfig(
        radius=0.5,
        nmac_radius=1.0,
        detection_radius=10.0,
        max_speed=10.0,
        max_acceleration=3.0,
        max_heading_change=math.pi,
        max_velocity=15.0,
    ),
})


class UAVFleetInstanceConfig(BaseModel):
    """One entry in fleet_composition: describes a batch of UAVs of a given type.
    type_name must match a key in VALID_UAVS / UAV_TYPE_REGISTRY.
    """
    type_name: str          # must be a key in VALID_UAVS
    count: int
    dynamics: str           # must be a key in VALID_DYNAMICS
    controller: str  # None valid only for LEARNING type
    sensor: str             # must be a key in VALID_SENSORS
    planner: str            # must be a key in VALID_PLANNERS

    @field_validator('type_name')
    @classmethod
    def type_name_must_be_valid(cls, v: str) -> str:
        if v not in VALID_UAVS:
            raise ValueError(
                f"Unknown type_name '{v}'. Valid options: {sorted(VALID_UAVS)}"
            )
        return v

    @field_validator('count')
    @classmethod
    def count_must_be_positive(cls, v: int) -> int:
        if v < 1:
            raise ValueError(f"count must be >= 1, got {v}")
        return v

    @field_validator('dynamics')
    @classmethod
    def dynamics_must_be_valid(cls, v: str) -> str:
        if v not in VALID_DYNAMICS:
            raise ValueError(
                f"Unknown dynamics '{v}'. Valid options: {sorted(VALID_DYNAMICS)}"
            )
        return v

    @field_validator('controller')
    @classmethod
    def controller_must_be_valid(cls, v: Optional[str]) -> Optional[str]:
        if v is None:
            return v
        if v not in VALID_CONTROLLERS:
            raise ValueError(
                f"Unknown controller '{v}'. Valid options: {sorted(VALID_CONTROLLERS)}"
            )
        return v

    @field_validator('sensor')
    @classmethod
    def sensor_must_be_valid(cls, v: str) -> str:
        if v not in VALID_SENSORS:
            raise ValueError(
                f"Unknown sensor '{v}'. Valid options: {sorted(VALID_SENSORS)}"
            )
        return v

    @model_validator(mode='after')
    def learning_type_controller_check(self) -> 'UAVFleetInstanceConfig':
        """LEARNING UAVs may omit controller (RL policy is assigned at training time)."""
        if self.type_name != RESERVED_TYPE_LEARNING and self.controller is None:
            raise ValueError(
                f"controller must be set for non-LEARNING type '{self.type_name}'"
            )
        return self


class UAMConfig(BaseModel):
    """Root config object — mirrors the top-level keys of sample_config.yaml.
    UAV physical parameters are NOT loaded from yaml; they live in UAV_TYPE_REGISTRY.
    fleet_composition.type_name values are validated against VALID_UAVS at field level.
    """
    simulator: UAMSimulatorConfig
    vertiport: VertiportConfig
    airspace: AirspaceConfig
    fleet_composition: List[UAVFleetInstanceConfig]

    @classmethod
    def load_from_yaml(cls, path: str) -> 'UAMConfig':
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        try:
            return cls(**data)
        except ValidationError as e:
            print('Configuration Error: The config file format is wrong')
            print(e)
            raise

#### ------------ CONFIG ------------ ####


#### ------------ FLEET BUILDER ------------ ####

@dataclass
class UAVBlueprint:
    """Resolved description of one UAV instance to be created."""
    uav_id: str
    type_name: str
    type_config: UAVTypeConfig
    dynamics_name: str
    controller_name: str
    sensor_name: str
    planner_name: str


#TODO: this function needs to be placed in simulator_manager OR atc - later update
def build_fleet(config: UAMConfig) -> List[UAVBlueprint]:
    """
    Iterate over fleet_composition and produce one UAVBlueprint per UAV instance.

    UAV physical parameters are resolved from UAV_TYPE_REGISTRY (defined in code).
    UAV IDs are assigned as '<type_name>_<global_index>' across all batches.

    Returns a flat list of UAVBlueprints ordered by fleet_composition entry.
    """
    blueprints: List[UAVBlueprint] = []
    global_index: int = 0

    for entry in config.fleet_composition:
        # Resolve physical parameters from the in-code registry
        if entry.type_name not in UAV_TYPE_REGISTRY:
            raise ValueError(
                f"Cannot build fleet: type_name '{entry.type_name}' has no entry "
                f"in UAV_TYPE_REGISTRY. Add it to VALID_UAVS and UAV_TYPE_REGISTRY."
            )
        type_cfg = UAV_TYPE_REGISTRY[entry.type_name]

        for _ in range(entry.count):
            uav_id = f"{entry.type_name}_{global_index}"
            blueprints.append(UAVBlueprint(
                uav_id=uav_id,
                type_name=entry.type_name,
                type_config=type_cfg,
                dynamics_name=entry.dynamics,
                controller_name=entry.controller,
                sensor_name=entry.sensor,
                planner_name=entry.planner
            ))
            global_index += 1

    return blueprints


def validate_fleet_composition(config: UAMConfig) -> None:
    """
    Run all fleet-level checks that require the full config context.
    Raises ValueError with a descriptive message on the first failure found.

    Checks performed:
    - Each type_name is in VALID_UAVS and has an entry in UAV_TYPE_REGISTRY.
    - At most one LEARNING entry is present (only one RL training slot).
    - LEARNING entries have controller=None.
    - Non-LEARNING entries have a controller specified.
    """
    learning_count = sum(
        1 for e in config.fleet_composition
        if e.type_name == RESERVED_TYPE_LEARNING
    )
    if learning_count > 1:
        raise ValueError(
            f"At most one fleet entry may use the reserved type LEARNING, "
            f"found {learning_count}."
        )

    for entry in config.fleet_composition:
        # Confirm registry has a matching entry (guards against VALID_UAVS/registry drift)
        if entry.type_name not in UAV_TYPE_REGISTRY:
            raise ValueError(
                f"type_name '{entry.type_name}' is in VALID_UAVS but missing "
                f"from UAV_TYPE_REGISTRY. Add its UAVTypeConfig entry."
            )

        # LEARNING-specific check
        if entry.type_name == RESERVED_TYPE_LEARNING:
            if entry.controller is not None:
                raise ValueError(
                    "LEARNING fleet entry must have controller=null; "
                    "the RL policy is assigned externally at training time."
                )

#### ------------ FLEET BUILDER ------------ ####


#### ------------ COMMAND / STATE SCHEMA ------------ ####

class ActionType(Enum):
    CONTROL = auto()        # e.g., thrust, roll commands
    MISSION_PLAN = auto()   # high-level waypoints/goals
    TRAJECTORY = auto()     # time-stamped state references
    PATH = auto()           # geometric path without timing


@dataclass
class UAVCommand:
    action_type: ActionType
    payload: Any            # np.array, Plan object, etc.
    metadata: dict = field(default_factory=dict)  # optional: priority, timestamp, etc.


# Each UAV can receive one or more commands per step
# access uav using str: uav_id, and pass List of ActionType-UAVCommand
UAVCommandBundle = Dict[str, List[UAVCommand]]


@dataclass
class SimulatorState:
    """Complete state snapshot — can be serialized to JSON."""
    timestamp: float | str
    currentstep: int
    airspace_state: List[Vertiport]     # Airspace type
    atc_state: Dict[int, UAV_template|UAV]       # ATC type
    external_systems: Dict[str, Any]    # wind, obstacles, etc.

    def to_json(self) -> str:
        """States are saved for rendering."""
        return json.dumps(asdict(self))

    @classmethod
    def from_json(cls, json_str: str) -> 'SimulatorState':
        """Restore simulation from a specific saved state."""
        return cls(**json.loads(json_str))

#### ------------ COMMAND / STATE SCHEMA ------------ ####
