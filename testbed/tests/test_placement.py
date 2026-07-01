import math
import os

import pytest

from testbed.placement import generate_ring_placement, load_placement_file


class TestGenerateRingPlacement:
    def test_square_corners_n4(self):
        points = generate_ring_placement(
            center=(0.0, 0.0), n=4, radius=10.0, start_angle=math.pi / 4
        )
        assert len(points) == 4
        for x, y in points:
            assert math.isclose(math.hypot(x, y), 10.0, rel_tol=1e-9)
        # pi/4 start angle -> first point at (cos45, sin45)*10 = (~7.07, ~7.07)
        assert math.isclose(points[0][0], points[0][1], rel_tol=1e-9)

    def test_equally_spaced_ring_n8(self):
        n = 8
        points = generate_ring_placement(center=(5.0, -5.0), n=n, radius=20.0)
        assert len(points) == n
        for x, y in points:
            assert math.isclose(math.hypot(x - 5.0, y - (-5.0)), 20.0, rel_tol=1e-9)
        # angular spacing between consecutive points is uniform
        angles = [math.atan2(y - (-5.0), x - 5.0) for x, y in points]
        deltas = [(angles[i + 1] - angles[i]) % (2 * math.pi) for i in range(n - 1)]
        for d in deltas:
            assert math.isclose(d, 2 * math.pi / n, rel_tol=1e-9)

    def test_n1_returns_single_point_at_radius(self):
        points = generate_ring_placement(center=(0.0, 0.0), n=1, radius=5.0)
        assert len(points) == 1
        assert math.isclose(math.hypot(*points[0]), 5.0, rel_tol=1e-9)


class TestLoadPlacementFile:
    def test_parses_example_placement_file(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'configs', 'example_placement.txt')
        spec = load_placement_file(path)
        bounds = (spec.x_min, spec.x_max, spec.y_min, spec.y_max)
        assert bounds == (-5000.0, 5000.0, -5000.0, 5000.0)
        assert len(spec.vertiport_positions) == 4
        assert len(spec.building_positions) == 2
        assert spec.vertiport_positions[0] == (2000.0, 0.0, 1800.0)
        assert spec.building_positions[1] == (600.0, 600.0, 0.0)

    def test_comments_and_blank_lines_ignored(self, tmp_path):
        f = tmp_path / 'placement.txt'
        f.write_text(
            "# comment\n"
            "\n"
            "boundary,-1,1,-1,1\n"
            "\n"
            "# another comment\n"
            "vertiport,0,0,100\n"
        )
        spec = load_placement_file(str(f))
        assert spec.vertiport_positions == [(0.0, 0.0, 100.0)]
        assert spec.building_positions == []

    def test_missing_boundary_raises(self, tmp_path):
        f = tmp_path / 'placement.txt'
        f.write_text("vertiport,0,0,100\n")
        with pytest.raises(ValueError, match='boundary'):
            load_placement_file(str(f))

    def test_duplicate_boundary_raises(self, tmp_path):
        f = tmp_path / 'placement.txt'
        f.write_text("boundary,-1,1,-1,1\nboundary,-2,2,-2,2\n")
        with pytest.raises(ValueError, match='duplicate'):
            load_placement_file(str(f))

    def test_wrong_field_count_raises(self, tmp_path):
        f = tmp_path / 'placement.txt'
        f.write_text("boundary,-1,1,-1\n")
        with pytest.raises(ValueError, match='boundary'):
            load_placement_file(str(f))

    def test_unknown_row_kind_raises(self, tmp_path):
        f = tmp_path / 'placement.txt'
        f.write_text("boundary,-1,1,-1,1\nunicorn,0,0,0\n")
        with pytest.raises(ValueError, match='unknown row kind'):
            load_placement_file(str(f))
