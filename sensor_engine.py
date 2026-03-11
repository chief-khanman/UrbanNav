from typing import Dict, List
from uav import UAV
from uav_template import UAV_template
from sensor_template import Sensor
from component_schema import VALID_SENSORS
from sensor_partial import PartialSensor

# Maps VALID_SENSORS string names -> Sensor subclasses.
# Only types with a concrete implementation are listed here.
# To add a new sensor type: add it to VALID_SENSORS in component_schema.py,
# implement the class, import it above, and add its entry to this map.
SENSOR_CLASS_MAP: Dict[str, type] = {
    'PartialSensor': PartialSensor,
    # 'GlobalSensor': GlobalSensor,  # TODO: add once sensor_global.py is implemented
}


class SensorEngine:
    """Orchestrates all sensor operations for the fleet.

    Mirrors DynamicsEngine:
      - One Sensor instance per unique sensor type (shared across UAVs of that type).
      - register_uav_sensors() instantiates and fans out: uav_id -> Sensor instance.
      - Five query methods aggregate per-UAV results into Dict[int, List[int]].

    Spatial hash rebuild is triggered inside get_detection_other_uavS(), which is
    the first UAV-related sensor method called each step in _step_uavS().
    # TODO: consider calling sensor_module.update() explicitly in
    #   SimulatorManager._step_uavS() before the 5 sensor calls for cleaner ordering.
    """

    def __init__(self,
                 config,
                 sensor_uav_map: Dict[str, List[int]],
                 uav_dict: Dict[int, UAV|UAV_template],
                 airspace=None) -> None:
        """
        Args:
            config: UAMConfig instance.
            sensor_uav_map: Mapping sensor_name -> [uav_id, ...] from ATC.
            uav_dict: Mapping uav_id -> UAV instance from ATC (live reference).
            airspace: Airspace instance for restricted area data (optional;
                      RA detection is a no-op until this is provided).
        """
        self.config = config
        self.sensor_uav_map = sensor_uav_map  # mapping: sensor_name -> [uav_id, ...]
        self.uav_dict = uav_dict              # mapping: uav_id -> UAV
        self.airspace = airspace
        # Populated by register_uav_sensors(): uav_id -> Sensor instance
        self.sensor_obj_map: Dict[int, Sensor] = {}

    # ------------------------------------------------------------------
    # Registration (called once after fleet is built)
    # ------------------------------------------------------------------

    def register_uav_sensors(self) -> None:
        """Instantiate one Sensor per unique type and map every UAV id to it.

        Mirrors DynamicsEngine.register_uav_dynamics() exactly:
          - One instance per sensor type (sensors are stateless w.r.t. per-UAV
            data; the spatial hash is rebuilt from uav_dict each step).
          - Fan-out: every UAV id maps to the shared instance for its type.

        Raises:
            ValueError: if a sensor name in sensor_uav_map is not in VALID_SENSORS.
            NotImplementedError: if a sensor name is valid but not yet implemented.
        """
        type_to_instance: Dict[str, Sensor] = {}

        for sensor_name in self.sensor_uav_map:
            if sensor_name not in VALID_SENSORS:
                raise ValueError(
                    f"Unknown sensor type '{sensor_name}'. "
                    f"Valid options: {sorted(VALID_SENSORS)}"
                )
            if sensor_name not in SENSOR_CLASS_MAP:
                raise NotImplementedError(
                    f"Sensor type '{sensor_name}' is valid but not yet implemented. "
                    f"Implemented: {sorted(SENSOR_CLASS_MAP)}"
                )
            instance = SENSOR_CLASS_MAP[sensor_name]()
            # Inject restricted area geometry if available
            if self.airspace is not None:
                ra_data = getattr(self.airspace, 'restricted_airspace_geo_series', None)
                if ra_data is not None:
                    instance.set_restricted_area_data(ra_data)
            type_to_instance[sensor_name] = instance

        # Fan out: map every uav_id to its shared Sensor instance
        for sensor_name, uav_id_list in self.sensor_uav_map.items():
            sensor_obj = type_to_instance[sensor_name]
            for uav_id in uav_id_list:
                self.sensor_obj_map[uav_id] = sensor_obj

    # ------------------------------------------------------------------
    # Per-step query methods (called by SimulatorManager._step_uavS())
    # ------------------------------------------------------------------

    def get_detection_other_uavS(self) -> Dict[int, set]:
        """Rebuild spatial hashes then return detected UAV IDs per UAV.

        This is the designated once-per-step rebuild point.  Each unique
        Sensor instance has update(uav_dict) called exactly once before
        any per-UAV queries are issued.

        Returns:
            Dict[int, List[int]]: { uav_id -> [detected_uav_id, ...] }
        """
        # Rebuild hash once per unique sensor instance (guard against shared instances)
        seen_instances: set = set()
        for sensor_obj in self.sensor_obj_map.values():
            if id(sensor_obj) not in seen_instances:
                sensor_obj.update(self.uav_dict)
                seen_instances.add(id(sensor_obj))

        detection_dict: Dict[int, set] = {}
        for uav_id in self.uav_dict:
            detection_dict[uav_id] = self.sensor_obj_map[uav_id].get_uav_detection(uav_id)
        return detection_dict

    def get_nmac(self) -> Dict[int, set]:
        """Return NMAC UAV IDs per UAV.

        Precondition: get_detection_other_uavS() has been called this step
        (spatial hash already built).

        Returns:
            Dict[int, List[int]]: { uav_id -> [nmac_uav_id, ...] }
        """
        nmac_dict: Dict[int, set] = {}
        for uav_id in self.uav_dict:
            nmac_dict[uav_id] = self.sensor_obj_map[uav_id].get_nmac(uav_id)
        return nmac_dict

    def get_collision_uavS(self) -> Dict[int, set]:
        """Return colliding UAV IDs per UAV.

        Returns:
            Dict[int, List[int]]: { uav_id -> [colliding_uav_id, ...] }
        """
        collision_dict: Dict[int, set] = {}
        for uav_id in self.uav_dict:
            collision_dict[uav_id] = self.sensor_obj_map[uav_id].get_uav_collision(uav_id)
        return collision_dict

    def get_detection_restricted_area(self) -> Dict[int, set]:
        """Return detected restricted area IDs per UAV.

        Returns empty lists for each UAV until RA geometry is wired in.

        Returns:
            Dict[int, List[int]]: { uav_id -> [ra_id, ...] or [] }
        """
        detection_dict: Dict[int, set] = {}
        for uav_id in self.uav_dict:
            detection_dict[uav_id] = self.sensor_obj_map[uav_id].get_ra_detection(uav_id)
        return detection_dict

    def get_collision_restricted_area(self) -> Dict[int, set]:
        """Return collided restricted area IDs per UAV.

        Returns empty lists for each UAV until RA geometry is wired in.

        Returns:
            Dict[int, List[int]]: { uav_id -> [ra_id, ...] or [] }
        """
        collision_dict: Dict[int, set] = {}
        for uav_id in self.uav_dict:
            collision_dict[uav_id] = self.sensor_obj_map[uav_id].get_ra_collision(uav_id)
        return collision_dict
