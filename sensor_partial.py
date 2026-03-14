from typing import Dict, List, Optional, Tuple
from shapely.geometry import Point
from sensor_template import Sensor
from sensor_spatial_hash import SpatialHash, _euclidean_3d
from uav import UAV

class PartialSensor(Sensor):
    """Partial (range-limited) sensor using a spatial hash for broad-phase lookup.

    Each UAV using this sensor type can only detect other UAVs within its own
    detection_radius.  The spatial hash provides fast broad-phase candidate
    retrieval; a 3D distance check is then applied for exact narrow-phase filtering.

    Detection chain (each level is a strict subset of the previous):
        broad phase  -> candidates from spatial hash query
        detection    -> distance <= uav.detection_radius
        NMAC         -> distance <= uav.nmac_radius
        collision    -> distance <= (uav.radius + other.radius)

    One PartialSensor instance is shared by all UAVs of this sensor type
    (mirrors the DynamicsEngine pattern).  Per-UAV state is not stored here —
    it lives on the UAV UAV itself.

    Lifecycle per simulation step:
        1. SensorEngine calls update(uav_dict) once — rebuilds the spatial hash.
        2. SensorEngine calls get_uav_detection / get_nmac / get_uav_collision
           per UAV in turn, reusing the already-built hash.

    The commented-out SpatialHash and check_nmac/check_collision drafts that
    previously lived in this file have been moved to sensor_spatial_hash.py
    (SpatialHash class) and implemented below as get_nmac / get_uav_collision.
    Both sensor_partial.py and the future sensor_global.py import from that
    shared utility module.
    """

    def __init__(self, spacing: float|None = None, max_uavs: int = 200) -> None:
        """
        Args:
            spacing: Grid cell size in metres.  Should be >= the largest
                     detection_radius in the fleet so each query covers at
                     most a 3x3x3 block of cells.  If None, computed lazily
                     on the first update() call as max(detection_radius).
            max_uavs: Pre-allocated capacity for internal arrays.
        """
        super().__init__()
        self._spacing = spacing
        self._max_uavs: int = max_uavs
        self._spatial_hash: Optional[SpatialHash] = SpatialHash(spacing, max_uavs) if spacing is not None else None
        self._uav_dict: Dict[int, UAV] = {}

        # Restricted airspace spatial hash (built once — RA is static)
        self._ra_spatial_hash: Optional[SpatialHash] = None
        self._ra_hash_built: bool = False
        self._ra_positions: Dict[int, Tuple[float, float, float]] = {}  # int_id -> (cx, cy, 0.0)
        self._ra_polygons: dict = {}          # int_id -> shapely Polygon (actual boundary)
        self._ra_polygon_buffers: dict = {}   # int_id -> shapely Polygon (buffered detection zone)

    # ------------------------------------------------------------------
    # RA data injection (called once by SensorEngine at startup)
    # ------------------------------------------------------------------

    def set_restricted_area_data(self, ra_geo_series, ra_buffer_geo_series=None) -> None:
        """Pre-process RA geometry into int-keyed dicts for fast per-step lookup.

        Overrides Sensor.set_restricted_area_data().  Enumerates the RA
        GeoDataFrame rows and builds:
          _ra_positions        — centroid (x, y, 0) for spatial hash build
          _ra_polygons         — actual polygon for collision narrow phase
          _ra_polygon_buffers  — buffered polygon for detection narrow phase

        Args:
            ra_geo_series: GeoDataFrame of RA polygons from airspace.
            ra_buffer_geo_series: GeoSeries of buffered RA polygons (detection zone).
        """
        super().set_restricted_area_data(ra_geo_series, ra_buffer_geo_series)
        if ra_geo_series is None:
            return

        ra_gdf = ra_geo_series.reset_index(drop=True)
        ra_buf  = ra_buffer_geo_series.reset_index(drop=True) if ra_buffer_geo_series is not None else None

        for int_id, row in ra_gdf.iterrows():
            poly = row.geometry
            centroid = poly.centroid
            self._ra_positions[int_id]      = (centroid.x, centroid.y, 0.0)
            self._ra_polygons[int_id]       = poly
            if ra_buf is not None:
                self._ra_polygon_buffers[int_id] = ra_buf.iloc[int_id]

    # ------------------------------------------------------------------
    # Per-step rebuild (called by SensorEngine before any get_* queries)
    # ------------------------------------------------------------------

    def update(self, uav_dict: Dict[int, UAV]) -> None:
        """Store the live fleet reference and rebuild the spatial hash.

        Must be called once per simulation step by SensorEngine before any
        get_uav_detection / get_nmac / get_uav_collision calls.

        Stores a reference (no copy) — position updates from dynamics are
        visible immediately since dynamics runs before sensors each step.

        Args:
            uav_dict: Mapping uav_id (int) -> UAV instance from ATC.
        """
        self._uav_dict = uav_dict

        if not uav_dict:
            return

        # Lazily compute spacing from the fleet's largest detection radius
        if self._spacing is None:
            self._spacing = max(uav.detection_radius for uav in uav_dict.values())

        # Re-allocate if fleet has grown beyond initial max_uavs
        current_count = len(uav_dict)
        if self._spatial_hash is None or current_count > self._max_uavs:
            self._max_uavs = max(current_count, self._max_uavs)
            self._spatial_hash = SpatialHash(self._spacing, self._max_uavs)

        self._spatial_hash.build(uav_dict)

        # Build RA spatial hash once (restricted airspace is static)
        if not self._ra_hash_built and self._ra_positions and self._spacing:
            num_ra = len(self._ra_positions)
            self._ra_spatial_hash = SpatialHash(self._spacing, max(num_ra, 1))
            self._ra_spatial_hash.build_from_positions(self._ra_positions)
            self._ra_hash_built = True

    # ------------------------------------------------------------------
    # Abstract method implementations
    # ------------------------------------------------------------------

    def get_uav_detection(self, uav_id: int, *args) -> set[int]:
        """Return IDs of UAVs within this UAV's detection_radius.

        Broad phase: spatial hash query with detection_radius bounding box.
        Narrow phase: exact 3D distance check, exclude self.

        Args:
            uav_id: ID of the querying UAV.

        Returns:
            List of detected UAV IDs (excludes uav_id itself).
        """

        uav = self._uav_dict[uav_id]
        candidates = self._spatial_hash.query(
            (uav.px, uav.py, uav.pz), uav.detection_radius
        )

        detected: set[int] = set()
        
        # if UAV sensors are active fill the detected list 
        if uav.get_sensor_operational():
            for candidate_id in candidates:
                if candidate_id == uav_id:
                    continue
                other = self._uav_dict.get(candidate_id)
                if other is None:
                    continue
                dist = _euclidean_3d(uav.px, uav.py, uav.pz,
                                    other.px, other.py, other.pz)
                if dist <= uav.detection_radius:
                    detected.add(candidate_id)
        else:
            detected = set()
        
        return detected

    def get_nmac(self, uav_id: int) -> set[int]:
        """Return IDs of UAVs within this UAV's nmac_radius.

        Chains from get_uav_detection — only detected UAVs are checked,
        since an NMAC cannot occur with a UAV that was not detected.

        Args:
            uav_id: ID of the querying UAV.

        Returns:
            List of UAV IDs in NMAC range (subset of detected).
        """
        uav = self._uav_dict[uav_id]
        detected_ids = self.get_uav_detection(uav_id)

        nmac: set[int] = set() 
        for other_id in detected_ids:
            other = self._uav_dict[other_id]
            dist = _euclidean_3d(uav.px, uav.py, uav.pz,
                                  other.px, other.py, other.pz)
            if dist <= uav.nmac_radius:
                nmac.add(other_id)
        return nmac

    def get_uav_collision(self, uav_id: int) -> set[int]:
        """Return IDs of UAVs physically colliding with this UAV.

        Collision threshold is the sum of both UAVs' body radii — correct
        two-body overlap condition.  Chains from get_nmac.

        Args:
            uav_id: ID of the querying UAV.

        Returns:
            List of colliding UAV IDs (subset of NMAC).
        """
        uav = self._uav_dict[uav_id]
        nmac_ids = self.get_nmac(uav_id)

        collisions: set[int] = set()
        for other_id in nmac_ids:
            other = self._uav_dict[other_id]
            dist = _euclidean_3d(uav.px, uav.py, uav.pz,
                                  other.px, other.py, other.pz)
            if dist <= (uav.radius + other.radius):
                collisions.add(other_id)
        return collisions

    # ------------------------------------------------------------------
    # Restricted airspace detection and collision
    # ------------------------------------------------------------------

    def get_ra_detection(self, uav_id: int) -> set[int]:
        """Return integer IDs of restricted areas whose buffer overlaps this UAV's detection zone.

        Broad phase: RA spatial hash query at (uav.px, uav.py, 0) with detection_radius.
        Narrow phase: Shapely 2D — uav detection circle intersects buffered RA polygon.
        Sensor operational guard mirrors get_uav_detection behaviour.

        Args:
            uav_id: ID of the querying UAV.

        Returns:
            Set of integer RA IDs (subset of pre-processed _ra_polygon_buffers).
        """
        if self._ra_geo_series is None or self._ra_spatial_hash is None:
            return set()

        uav = self._uav_dict[uav_id]

        if not uav.get_sensor_operational():
            return set()

        # Broad phase: query RA hash in 2D (z=0 for ground-level RA centroids)
        candidates = self._ra_spatial_hash.query(
            (uav.px, uav.py, 0.0), uav.detection_radius
        )

        uav_detection_circle = Point(uav.px, uav.py).buffer(uav.detection_radius)

        detected_ra: set[int] = set()
        for ra_id in candidates:
            ra_buf_poly = self._ra_polygon_buffers.get(ra_id)
            if ra_buf_poly is None:
                continue
            if uav_detection_circle.intersects(ra_buf_poly):
                detected_ra.add(ra_id)

        return detected_ra

    def get_ra_collision(self, uav_id: int) -> set[int]:
        """Return integer IDs of restricted areas whose boundary overlaps this UAV's body.

        Chains from get_ra_detection — only detected RAs are checked.
        Narrow phase: Shapely 2D — uav body circle intersects actual RA polygon.

        Args:
            uav_id: ID of the querying UAV.

        Returns:
            Set of integer RA IDs (subset of get_ra_detection result).
        """
        if self._ra_geo_series is None or self._ra_spatial_hash is None:
            return set()

        detected_ra_ids = self.get_ra_detection(uav_id)
        uav = self._uav_dict[uav_id]
        uav_body_circle = Point(uav.px, uav.py).buffer(uav.radius)

        collisions_ra: set[int] = set()
        for ra_id in detected_ra_ids:
            ra_poly = self._ra_polygons.get(ra_id)
            if ra_poly is None:
                continue
            if uav_body_circle.intersects(ra_poly):
                collisions_ra.add(ra_id)

        return collisions_ra

    def _turn_off_landing_sensor(self, uav_id):
        uav = self._uav_dict[uav_id]
        if uav.current_position.distance(uav.end_vertiport.location) <= uav.sensor_shutoff_distance:
            return True 
        else:
            return False 
        
    def _turn_off_takeoff_sensor(self, uav_id):
        uav = self._uav_dict[uav_id]
        if uav.current_position.distance(uav.start_vertiport.location) <= uav.sensor_shutoff_distance:
            return True 
        else:
            return False 
        
