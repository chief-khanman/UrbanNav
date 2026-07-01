"""TestbedAirspace — a fully synthetic, offline stand-in for urbannav.airspace.Airspace.

Deliberately does NOT subclass or import internals from Airspace (airspace.py stays
untouched, per the testbed plan). Instead it exposes the same minimal attribute
surface that ATC, SensorEngine, and Renderer already read off an Airspace instance,
populated from a generated pattern or an explicit placement file instead of OSM data:

    vertiport_list, get_vp_id_list()
    location_utm_gdf                                  (ATC.airspace_mid_point_coord, Renderer)
    location_tags, location_utm, location_utm_buffer   (Renderer 2D restricted-area drawing)
    restricted_airspace_geo_series                     (SensorEngine -> set_restricted_area_data)
    restricted_airspace_buffer_geo_series               (SensorEngine -> set_restricted_area_data)
    buildings                                          (TestbedRenderer 3D extrusion)

Zero network I/O — every geometry here is built from plain numbers.
"""
import random
from dataclasses import dataclass
from typing import List, Tuple

import geopandas as gpd
import numpy as np
import shapely
from geopandas import GeoDataFrame, GeoSeries
from shapely import Point

from testbed.config_schema import BuildingConfig, TestbedAirspaceConfig
from testbed.placement import generate_ring_placement, load_placement_file
from urbannav.vertiport import Vertiport

# Margin added around the generated vertiport/building extent when deriving the
# synthetic boundary box in procedural (non-file) placement mode.
_BOUNDARY_MARGIN_FACTOR = 0.5
_MIN_BUILDING_Z_HEIGHT = 0.1


@dataclass
class BuildingSpec:
    """One synthetic building, fully resolved (position, footprint, sampled height).
    Consumed directly by TestbedRenderer for 3D extrusion."""
    center: Tuple[float, float]
    width: float
    depth: float
    z_height: float


class TestbedAirspace:
    """Synthetic, offline airspace: vertiports placed in a pattern (or loaded from a
    placement file) plus synthetic rectangular 'building' restricted areas."""

    __test__ = False  # not a pytest test class — name just starts with "Testbed"

    def __init__(self, config: TestbedAirspaceConfig, seed: int = 123) -> None:
        self.config = config
        self.seed = seed
        random.seed(seed)
        np.random.seed(seed)

        if config.placement_file is not None:
            self._build_from_placement_file(config)
        else:
            self._build_from_pattern(config)

        self._build_restricted_area_attrs()

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def _build_from_pattern(self, config: TestbedAirspaceConfig) -> None:
        vertiport_xy = generate_ring_placement(
            center=config.center,
            n=config.num_vertiports,
            radius=config.radius,
            start_angle=config.start_angle,
        )
        z_lo, z_hi = config.altitude_range
        vertiport_xyz = [(x, y, random.uniform(z_lo, z_hi)) for x, y in vertiport_xy]

        self.buildings: List[BuildingSpec] = []
        for bc in config.buildings:
            default_mean, default_std = config.building_z_height_mean, config.building_z_height_std
            mean = bc.z_height_mean if bc.z_height_mean is not None else default_mean
            std = bc.z_height_std if bc.z_height_std is not None else default_std
            self.buildings.append(
                self._resolve_building_spec(bc.center, bc.width, bc.depth, mean, std)
            )
        self._building_buffer_radii = [bc.buffer_radius for bc in config.buildings]

        self._build_vertiport_list(vertiport_xyz, config.landing_pad_capacity)
        self._build_boundary_from_extent(vertiport_xyz, config.buildings, config.radius)

    def _build_from_placement_file(self, config: TestbedAirspaceConfig) -> None:
        spec = load_placement_file(config.placement_file)

        self._build_vertiport_list(spec.vertiport_positions, config.landing_pad_capacity)

        width, depth = config.building_width_default, config.building_depth_default
        mean, std = config.building_z_height_mean, config.building_z_height_std
        self.buildings = [
            self._resolve_building_spec((x, y), width, depth, mean, std)
            for x, y, _z in spec.building_positions
        ]
        self._building_buffer_radii = [config.building_buffer_radius_default] * len(self.buildings)

        boundary_poly = shapely.geometry.box(spec.x_min, spec.y_min, spec.x_max, spec.y_max)
        self.location_utm_gdf: GeoDataFrame = GeoDataFrame({'geometry': [boundary_poly]})

    def _resolve_building_spec(
        self, center, width, depth, z_height_mean, z_height_std
    ) -> BuildingSpec:
        z_height = max(_MIN_BUILDING_Z_HEIGHT, np.random.normal(z_height_mean, z_height_std))
        return BuildingSpec(center=tuple(center), width=width, depth=depth, z_height=z_height)

    def _build_vertiport_list(
        self, positions: List[Tuple[float, float, float]], landing_pad_capacity: int
    ) -> None:
        self.vertiport_list: List[Vertiport] = []
        for x, y, z in positions:
            vp = Vertiport(Point(x, y, z))
            vp.landing_takeoff_capacity = landing_pad_capacity
            self.vertiport_list.append(vp)

    def _build_boundary_from_extent(
        self,
        vertiport_xyz: List[Tuple[float, float, float]],
        buildings: List[BuildingConfig],
        radius: float,
    ) -> None:
        xs = [p[0] for p in vertiport_xyz] + [b.center[0] for b in buildings]
        ys = [p[1] for p in vertiport_xyz] + [b.center[1] for b in buildings]
        margin = max(radius, 1.0) * _BOUNDARY_MARGIN_FACTOR
        x_min, x_max = min(xs) - margin, max(xs) + margin
        y_min, y_max = min(ys) - margin, max(ys) + margin
        boundary_poly = shapely.geometry.box(x_min, y_min, x_max, y_max)
        self.location_utm_gdf: GeoDataFrame = GeoDataFrame({'geometry': [boundary_poly]})

    def _build_restricted_area_attrs(self) -> None:
        """Build the same minimal attribute surface Airspace exposes for restricted
        areas, under one synthetic tag ('building'), so SensorEngine and Renderer
        work unmodified. See sensor_partial.py:set_restricted_area_data for the
        exact shape each attribute is required to have (GeoDataFrame vs GeoSeries)."""
        footprints = [
            shapely.geometry.box(
                b.center[0] - b.width / 2, b.center[1] - b.depth / 2,
                b.center[0] + b.width / 2, b.center[1] + b.depth / 2,
            )
            for b in self.buildings
        ]
        buffers = [
            poly.buffer(buf_radius)
            for poly, buf_radius in zip(footprints, self._building_buffer_radii)
        ]

        building_gdf: GeoDataFrame = GeoDataFrame({'geometry': footprints})
        building_buffer_series: GeoSeries = GeoSeries(buffers)

        self.location_tags = {'building': 'synthetic'}
        self.location_utm = {'building': building_gdf}
        self.location_utm_buffer = {'building': building_buffer_series}

        # SensorEngine.register_uav_sensors() reads these two directly.
        self.restricted_airspace_geo_series: GeoDataFrame = building_gdf
        self.restricted_airspace_buffer_geo_series: GeoSeries = building_buffer_series

    # ------------------------------------------------------------------
    # Public surface mirrored from Airspace
    # ------------------------------------------------------------------

    def get_vp_id_list(self) -> List[int]:
        """Returns list of vertiport ids."""
        return [vp.id for vp in self.vertiport_list]

    def get_state(self) -> List[Vertiport]:
        """Airspace state is the current vertiports — mirrors Airspace.get_state(),
        consumed by SimulatorManager._create_data_class_state() / metrics_collector.py."""
        return self.vertiport_list
