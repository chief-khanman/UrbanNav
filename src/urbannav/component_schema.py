import math
import yaml
from enum import Enum, auto
from typing import Any, List, Dict, Tuple, Optional

from dataclasses import dataclass, field
from pydantic import BaseModel, Field, ValidationError, field_validator, model_validator

from shapely import Point

from urbannav.vertiport import Vertiport
from urbannav.uav_template import UAV_template
from urbannav.uav import UAV

from dataclasses import dataclass, asdict
import json

#### ------------ CONFIG ------------ ####

# Reserved type name for RL-training UAVs — controller is assigned at training time. 
# Need to think about test time, and connection of policy during test.

# TODO:
# USE 'mode' for agent assignment. - if mode train 
# if mode 'train' UAV has no controller, elif mode 'test' UAV must have mapping to RL controller
# use AerBus to extract UAV state information for mode: 'test' RL controller. 

# Multi-agent RL:
# - RESERVED_TYPE_SINGLE_AGENT_LEARNING: at most one fleet_composition entry; mode
#   TRAIN requires count==1 (one agent being trained), mode TEST allows count>=1
#   (deploying a single trained policy across multiple UAVs).
# - RESERVED_TYPE_MULTI_AGENT_LEARNING: any number of fleet_composition entries;
#   each entry must set policy_id, and all entries sharing one policy_id must use
#   identical dynamics/controller/sensor/planner (they're trained/deployed as one
#   shared policy across however many UAVs use that policy_id).

RESERVED_TYPE_SINGLE_AGENT_LEARNING = 'SINGLE_AGENT_LEARNING'
RESERVED_TYPE_MULTI_AGENT_LEARNING = 'MULTI_AGENT_LEARNING'
RESERVED_TYPE_LEARNING = RESERVED_TYPE_SINGLE_AGENT_LEARNING  # backward-compat alias
RESERVED_LEARNING_TYPES: set[str] = {RESERVED_TYPE_SINGLE_AGENT_LEARNING, RESERVED_TYPE_MULTI_AGENT_LEARNING}
RESERVED_TYPE_MODE: set[str] = {'TRAIN', 'TEST'}
# Valid string identifiers for each component type.
# Dynamics, controller, sensor classes are wired separately once those modules are ready.
VALID_DYNAMICS: set[str] = {'PointMass', 'SixDOF', 'TwoDVector-Holonomic', 'ORCA'}
VALID_CONTROLLERS: set[str] = {'PIDPointMassController', 'PIDHolonomicController', 'CascadedPIDSixDOFController', 'LQR', 'MARL', 'ORCA', 'Static', 'RL'}
VALID_SENSORS: set[str] = {'PartialSensor', 'GlobalSensor', 'MapSensor'}
VALID_PLANNERS: set[str] = {'PointMass-PID', 'Holonomic-PID', 'PointMass-RL', 'SixDOF-PID', 'SixDOF-LQR', 'N/A'}

# UAV type registry — physical parameters live here in code, not in the yaml.
# fleet_composition.type_name values must match a key in this dict.
# To add a new UAV type: add the key to VALID_UAVS and its UAVTypeConfig entry below.
VALID_UAVS: set[str] = {
    'STANDARD',
    'HEAVY',
    'SINGLE_AGENT_LEARNING',   # reserved — single RL policy, trained or deployed
    'MULTI_AGENT_LEARNING',    # reserved — RL policy shared across UAVs via policy_id
    'ORCA',
}

UAV_TYPE_REGISTRY: Dict[str, 'UAVTypeConfig'] = {}  # populated after UAVTypeConfig is defined




class UAMSimulatorConfig(BaseModel):
    dt: float
    total_timestep: int
    mode: str   # '2D' or '3D'
    seed: int


class LoggingConfig(BaseModel):
    """Controls whether episode metrics are collected and where they are saved."""
    enabled: bool = True
    log_dir: str = 'logs'


class RenderingConfig(BaseModel):
    """Controls 2D rendering of simulation episodes.

    mode:
        'realtime'  — draw each step interactively (requires a display / GUI backend).
        'offline'   — collect frames during the run and save an animation at the end.
        'both'      — do both simultaneously.
    frame_skip:
        Render every (frame_skip + 1) steps.  0 = every step, 9 = every 10th step.
        Increase to keep animation file sizes manageable for long episodes.
    """
    enabled: bool = False
    mode: str = 'offline'
    output_dir: str = 'renders'
    output_filename: str = 'episode'
    realtime_sleep: float = 0.01
    frame_skip: int = 4

    @field_validator('mode')
    @classmethod
    def mode_must_be_valid(cls, v: str) -> str:
        if v not in ('realtime', 'offline', 'both'):
            raise ValueError(f"mode must be 'realtime', 'offline', or 'both', got '{v}'")
        return v


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
        radius=17.0,
        nmac_radius=200.0,
        detection_radius=500.0,
        max_speed=10.0,
        max_acceleration=3.0,
        max_heading_change=math.pi,
        max_velocity=15.0,
    ),
    'HEAVY': UAVTypeConfig(
        radius=34.0,
        nmac_radius=400.0,
        detection_radius=700.0,
        max_speed=7.0,
        max_acceleration=1.5,
        max_heading_change=math.pi,
        max_velocity=10.0,
    ),
    'SINGLE_AGENT_LEARNING': UAVTypeConfig(
        radius=17.0,
        nmac_radius=200.0,
        detection_radius=500.0,
        max_speed=10.0,
        max_acceleration=3.0,
        max_heading_change=math.pi,
        max_velocity=15.0,
    ),
    'MULTI_AGENT_LEARNING': UAVTypeConfig(
        radius=17.0,
        nmac_radius=200.0,
        detection_radius=500.0,
        max_speed=10.0,
        max_acceleration=3.0,
        max_heading_change=math.pi,
        max_velocity=15.0,
    ),
    'ORCA': UAVTypeConfig(
        radius=17.0,
        nmac_radius=200.0,
        detection_radius=500.0,
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
    mode: Optional[str] = None  # required for LEARNING types; must be in RESERVED_TYPE_MODE
    policy_id: Optional[str] = None  # required for LEARNING types; forbidden otherwise

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
        if self.type_name not in RESERVED_LEARNING_TYPES and self.controller is None:
            raise ValueError(
                f"controller must be set for non-LEARNING type '{self.type_name}'"
            )
        return self

    @model_validator(mode='after')
    def learning_type_mode_check(self) -> 'UAVFleetInstanceConfig':
        """mode must be set for LEARNING types and must NOT be set for any other type."""
        if self.type_name in RESERVED_LEARNING_TYPES:
            if self.mode is None or self.mode not in RESERVED_TYPE_MODE:
                raise ValueError(
                    f"LEARNING type '{self.type_name}' requires mode from "
                    f"{RESERVED_TYPE_MODE}, got: {self.mode!r}"
                )
        else:
            if self.mode is not None:
                raise ValueError(
                    f"mode must only be set for LEARNING types, got mode={self.mode!r} "
                    f"on type_name='{self.type_name}'"
                )
        return self

    @model_validator(mode='after')
    def learning_type_policy_id_check(self) -> 'UAVFleetInstanceConfig':
        """policy_id must be set for LEARNING types and must NOT be set for any other type."""
        if self.type_name in RESERVED_LEARNING_TYPES:
            if not self.policy_id:
                raise ValueError(
                    f"LEARNING type '{self.type_name}' requires a non-empty policy_id"
                )
        else:
            if self.policy_id is not None:
                raise ValueError(
                    f"policy_id must only be set for LEARNING types, got "
                    f"policy_id={self.policy_id!r} on type_name='{self.type_name}'"
                )
        return self

    @model_validator(mode='after')
    def single_agent_train_mode_count_check(self) -> 'UAVFleetInstanceConfig':
        """SINGLE_AGENT_LEARNING with mode=TRAIN trains exactly one agent; mode=TEST
        may deploy the same trained policy across any number of UAVs (count>=1,
        already enforced by count_must_be_positive)."""
        if self.type_name == RESERVED_TYPE_SINGLE_AGENT_LEARNING and self.mode == 'TRAIN':
            if self.count != 1:
                raise ValueError(
                    f"SINGLE_AGENT_LEARNING with mode='TRAIN' requires count==1 "
                    f"(one agent being trained), got count={self.count}"
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
    logging: LoggingConfig = Field(default_factory=LoggingConfig)
    rendering: RenderingConfig = Field(default_factory=RenderingConfig)

    @classmethod
    def load_from_yaml(cls, path: str) -> 'UAMConfig':
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        try:
            config = cls(**data)
        except ValidationError as e:
            print('Configuration Error: The config file format is wrong')
            print(e)
            raise
        validate_fleet_composition(config)
        return config

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
    policy_id: Optional[str] = None


#TODO: this function needs to be placed in simulator_manager OR atc - later update
def build_fleet_blueprint(config: UAMConfig) -> List[UAVBlueprint]:
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
                planner_name=entry.planner,
                policy_id=entry.policy_id,
            ))
            global_index += 1

    return blueprints


def validate_fleet_composition(config: UAMConfig) -> None:
    """
    Run all fleet-level checks that require the full config context (i.e. checks
    that compare multiple fleet_composition entries against each other — anything
    checkable from a single entry alone lives in UAVFleetInstanceConfig's own
    model_validators instead).
    Raises ValueError with a descriptive message on the first failure found.

    Checks performed:
    - Each type_name is in VALID_UAVS and has an entry in UAV_TYPE_REGISTRY.
    - At most one SINGLE_AGENT_LEARNING entry is present (only one RL training/
      deployment slot); any number of MULTI_AGENT_LEARNING entries are allowed.
    - All fleet entries sharing one policy_id (MULTI_AGENT_LEARNING) use identical
      dynamics/controller/sensor/planner, since they're trained/deployed as one
      shared policy.
    """
    single_agent_count = sum(
        1 for e in config.fleet_composition
        if e.type_name == RESERVED_TYPE_SINGLE_AGENT_LEARNING
    )
    if single_agent_count > 1:
        raise ValueError(
            f"At most one fleet entry may use the reserved type "
            f"{RESERVED_TYPE_SINGLE_AGENT_LEARNING}, found {single_agent_count}."
        )

    for entry in config.fleet_composition:
        # Confirm registry has a matching entry (guards against VALID_UAVS/registry drift)
        if entry.type_name not in UAV_TYPE_REGISTRY:
            raise ValueError(
                f"type_name '{entry.type_name}' is in VALID_UAVS but missing "
                f"from UAV_TYPE_REGISTRY. Add its UAVTypeConfig entry."
            )

    # All entries sharing a policy_id must describe the same component wiring,
    # since they're trained/deployed as one shared policy.
    policy_groups: Dict[str, List[UAVFleetInstanceConfig]] = {}
    for entry in config.fleet_composition:
        if entry.policy_id is not None:
            policy_groups.setdefault(entry.policy_id, []).append(entry)

    for policy_id, entries in policy_groups.items():
        first = entries[0]
        for other in entries[1:]:
            if (other.dynamics, other.controller, other.sensor, other.planner) != \
               (first.dynamics, first.controller, first.sensor, first.planner):
                raise ValueError(
                    f"All fleet entries sharing policy_id='{policy_id}' must use "
                    f"identical dynamics/controller/sensor/planner; "
                    f"found {(first.dynamics, first.controller, first.sensor, first.planner)} "
                    f"vs {(other.dynamics, other.controller, other.sensor, other.planner)}."
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
