from abc import ABC, abstractmethod
from typing import Dict, List, Any


class Sensor(ABC):

    '''
    Collect other UAVs state information
    '''
    #FIX:
    # set_data needs to be updated, currently we are only focused on dynamic obj, ie other_UAVs,
    # a sensor will sense any object, dynamic or static.
    # proposed design -
    # if intersection with object
    #   sensor will add label
    #       if sensor_label == UAV:
    #           add UAV to otherUAV_data_list
    #       else if sensor_label == restricted_airspace:
    #           add building to restricted_airspace_list
    #       else :
    #           raise ERROR - this program at the moment should have these two objects only
    #
    #
    # The above structure is flexible for accepting different kinds of objects, using a label-string
    # and adding the object to a list for variable named label-string


    @abstractmethod
    def __init__(self, *args, **kwargs) -> None:
        """Subclass __init__ must call super().__init__().
        Parameters are sensor-type-specific — see subclass docstrings.
        """
        self._ra_geo_series = None
        self._ra_buffer_geo_series = None
        return None


    def update(self, uav_dict: Dict[int, Any]) -> None:
        """Rebuild any per-step data structures (e.g., spatial hash).
        Called by SensorEngine once per simulation step before any get_*() call.
        Subclasses that require a per-step rebuild must override this method.
        Default implementation is a no-op.
        """
        pass

    def set_restricted_area_data(self, ra_geo_series, ra_buffer_geo_series=None) -> None:
        """Inject restricted area geometry for RA detection and collision checks.
        Called by SensorEngine.register_uav_sensors() after instantiation.

        Args:
            ra_geo_series: GeoDataFrame of RA polygons from airspace.
            ra_buffer_geo_series: GeoSeries of buffered RA polygons (detection zone).
        """
        self._ra_geo_series = ra_geo_series
        self._ra_buffer_geo_series = ra_buffer_geo_series


    def get_sensor_data(self, uav_id: int, sensor_active_status: bool = True) -> Dict[str, set]:
        '''Combination of uav detection(List of Dict) and ra detection(List of Dict)'''

        if sensor_active_status:
            detect_uav_id_list = self.get_uav_detection(uav_id)
            detect_ra_id_list = self.get_ra_detection(uav_id)
            nmac_uav_id_list = self.get_nmac(uav_id)
            collision_uav_id_list = self.get_uav_collision(uav_id)
            collision_ra_id_list = self.get_ra_collision(uav_id)
        else:
            detect_uav_id_list = set()
            detect_ra_id_list = set()
            nmac_uav_id_list = set()
            collision_uav_id_list = set()
            collision_ra_id_list = set()

        sensor_data_packet = {
                            'detect_uav': detect_uav_id_list,
                            'detect_ra': detect_ra_id_list,
                            'detect_nmac': nmac_uav_id_list,
                            'collision_uav': collision_uav_id_list,
                            'collision_ra': collision_ra_id_list
                            }

        return sensor_data_packet


    # UAV
    @abstractmethod
    def get_uav_detection(self, uav_id: int, *args) -> set[int]:
        # RUN broad phase collision detection
        pass

    @abstractmethod
    def get_nmac(self, uav_id: int) -> set[int]:
        '''Collect the time step and UAVs with who there was an NMAC'''
        pass

    @abstractmethod
    def get_uav_collision(self, uav_id: int) -> set[int]:
        '''returns a bool if there is a collision along with UAV id'''
        pass


    # Restricted Airspace (ra)
    def get_ra_detection(self, uav_id: int) -> set[int]:
        return set()

    def get_ra_collision(self, uav_id: int) -> set[int]:
        return set()
