import math
import os

import pytest

from testbed.config_schema import BuildingConfig, TestbedAirspaceConfig
from testbed.testbed_airspace import TestbedAirspace

EXAMPLE_PLACEMENT_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'configs', 'example_placement.txt'
)


class TestProceduralPlacement:
    def test_square_pattern_n4(self):
        config = TestbedAirspaceConfig(
            pattern='ring', num_vertiports=4, center=(0.0, 0.0), radius=1000.0,
            start_angle=math.pi / 4, landing_pad_capacity=1,
            buildings=[BuildingConfig(center=(0.0, 0.0), width=100.0, depth=100.0)],
        )
        airspace = TestbedAirspace(config, seed=123)

        assert len(airspace.vertiport_list) == 4
        for vp in airspace.vertiport_list:
            assert math.isclose(math.hypot(vp.x, vp.y), 1000.0, rel_tol=1e-6)
            assert vp.landing_takeoff_capacity == 1

    def test_ring_pattern_n8(self):
        config = TestbedAirspaceConfig(pattern='ring', num_vertiports=8, radius=2000.0)
        airspace = TestbedAirspace(config, seed=42)
        assert len(airspace.vertiport_list) == 8
        for vp in airspace.vertiport_list:
            assert math.isclose(math.hypot(vp.x, vp.y), 2000.0, rel_tol=1e-6)

    def test_landing_pad_capacity_applied(self):
        config = TestbedAirspaceConfig(num_vertiports=4, radius=500.0, landing_pad_capacity=1)
        airspace = TestbedAirspace(config, seed=1)
        assert all(vp.landing_takeoff_capacity == 1 for vp in airspace.vertiport_list)

    def test_vp_id_list_matches_vertiport_list(self):
        config = TestbedAirspaceConfig(num_vertiports=4, radius=500.0)
        airspace = TestbedAirspace(config, seed=1)
        assert airspace.get_vp_id_list() == [vp.id for vp in airspace.vertiport_list]

    def test_building_z_height_samples_around_mean(self):
        config = TestbedAirspaceConfig(
            num_vertiports=4, radius=500.0,
            building_z_height_mean=100.0, building_z_height_std=0.001,
            buildings=[BuildingConfig(center=(0.0, 0.0), width=50.0, depth=50.0)],
        )
        airspace = TestbedAirspace(config, seed=7)
        assert len(airspace.buildings) == 1
        assert math.isclose(airspace.buildings[0].z_height, 100.0, abs_tol=0.1)

    def test_no_buildings_yields_empty_restricted_area(self):
        config = TestbedAirspaceConfig(num_vertiports=4, radius=500.0)
        airspace = TestbedAirspace(config, seed=1)
        assert len(airspace.buildings) == 0
        assert len(airspace.restricted_airspace_geo_series) == 0
        assert len(airspace.restricted_airspace_buffer_geo_series) == 0

    def test_restricted_area_attrs_shapes_match_sensor_expectations(self):
        """SensorEngine/PartialSensor.set_restricted_area_data() requires
        restricted_airspace_geo_series to support .iterrows()/.geometry (GeoDataFrame)
        and restricted_airspace_buffer_geo_series to support .iloc[i] -> Polygon
        (GeoSeries) — see sensor_partial.py:set_restricted_area_data."""
        config = TestbedAirspaceConfig(
            num_vertiports=4, radius=500.0,
            buildings=[
                BuildingConfig(center=(0.0, 0.0), width=50.0, depth=50.0),
                BuildingConfig(center=(200.0, 200.0), width=50.0, depth=50.0),
            ],
        )
        airspace = TestbedAirspace(config, seed=1)

        ra_gdf = airspace.restricted_airspace_geo_series
        ra_buf = airspace.restricted_airspace_buffer_geo_series
        assert len(ra_gdf) == 2
        for _, row in ra_gdf.reset_index(drop=True).iterrows():
            assert row.geometry.is_valid

        ra_buf_reset = ra_buf.reset_index(drop=True)
        for i in range(len(ra_buf_reset)):
            poly = ra_buf_reset.iloc[i]
            assert poly.is_valid
            # buffer must be strictly larger than the unbuffered footprint
            assert poly.area > ra_gdf.reset_index(drop=True).iloc[i].geometry.area

    def test_location_utm_gdf_centroid_accessible(self):
        """ATC.__init__ does self.airspace.location_utm_gdf.centroid unconditionally."""
        config = TestbedAirspaceConfig(num_vertiports=4, radius=500.0)
        airspace = TestbedAirspace(config, seed=1)
        centroid = airspace.location_utm_gdf.centroid
        assert centroid is not None
        assert len(centroid) == 1


class TestFilePlacement:
    def test_loads_positions_from_file(self):
        config = TestbedAirspaceConfig(
            placement_file=EXAMPLE_PLACEMENT_PATH,
            building_width_default=100.0, building_depth_default=100.0,
        )
        airspace = TestbedAirspace(config, seed=1)
        assert len(airspace.vertiport_list) == 4
        assert len(airspace.buildings) == 2

        vp_positions = sorted((vp.x, vp.y) for vp in airspace.vertiport_list)
        expected = sorted([(2000.0, 0.0), (-2000.0, 0.0), (0.0, 2000.0), (0.0, -2000.0)])
        assert vp_positions == expected

    def test_boundary_from_file_used_directly(self):
        config = TestbedAirspaceConfig(placement_file=EXAMPLE_PLACEMENT_PATH)
        airspace = TestbedAirspace(config, seed=1)
        bounds = airspace.location_utm_gdf.geometry.iloc[0].bounds
        assert bounds == (-5000.0, -5000.0, 5000.0, 5000.0)

    def test_file_placement_ignores_procedural_fields(self):
        config = TestbedAirspaceConfig(
            placement_file=EXAMPLE_PLACEMENT_PATH,
            num_vertiports=999,  # would be a contradiction if procedural path ran too
        )
        airspace = TestbedAirspace(config, seed=1)
        assert len(airspace.vertiport_list) == 4
