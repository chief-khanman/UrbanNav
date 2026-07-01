"""Placement utilities for the synthetic testbed — shared between vertiport and
building placement, and between the two placement strategies (procedural pattern
generation vs. explicit file-based placement).

Pure geometry, no OSM/network I/O, no dependency on urbannav.airspace (kept
standalone by design — see testbed plan: airspace.py stays untouched).
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple


def generate_ring_placement(
    center: Tuple[float, float],
    n: int,
    radius: float,
    start_angle: float = 0.0,
) -> List[Tuple[float, float]]:
    """Place n points equally spaced on a circle around center.

    n=4 with start_angle=pi/4 produces exact square corners — "square" placement
    is just this pattern with n=4, no separate code path needed.

    Args:
        center: (x, y) center of the pattern.
        n: number of points to place.
        radius: distance from center to each point.
        start_angle: rotation offset (radians).

    Returns:
        List of (x, y) tuples, one per point.
    """
    cx, cy = center
    del_theta = 2 * math.pi / n
    return [
        (cx + radius * math.cos(start_angle + i * del_theta),
         cy + radius * math.sin(start_angle + i * del_theta))
        for i in range(n)
    ]


@dataclass
class PlacementSpec:
    """Explicit boundary + object positions loaded from a placement file."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    vertiport_positions: List[Tuple[float, float, float]] = field(default_factory=list)
    building_positions: List[Tuple[float, float, float]] = field(default_factory=list)


def load_placement_file(path: str) -> PlacementSpec:
    """Parse a plain-text placement file into a PlacementSpec.

    File format — one flat file, comma-delimited rows, '#' comments and blank
    lines ignored:

        # boundary,x_min,x_max,y_min,y_max
        boundary,-10000,10000,-10000,10000
        # vertiport,x,y,z
        vertiport,2000,0,2000
        # building,x,y,z   (center; footprint size/height come from config)
        building,0,0,60

    Exactly one 'boundary' row is required. Any number of 'vertiport'/'building'
    rows may follow, in any order.

    Raises:
        ValueError: on a missing/duplicate boundary row, an unknown row kind, or
            a row with the wrong number of fields.
    """
    boundary: Tuple[float, float, float, float] | None = None
    vertiport_positions: List[Tuple[float, float, float]] = []
    building_positions: List[Tuple[float, float, float]] = []

    with open(path, 'r') as f:
        for line_num, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue

            fields = [tok.strip() for tok in line.split(',')]
            kind = fields[0]

            if kind == 'boundary':
                if len(fields) != 5:
                    raise ValueError(
                        f"{path}:{line_num}: 'boundary' row needs 4 values "
                        f"(x_min,x_max,y_min,y_max), got {len(fields) - 1}"
                    )
                if boundary is not None:
                    raise ValueError(f"{path}:{line_num}: duplicate 'boundary' row")
                boundary = tuple(float(v) for v in fields[1:5])  # type: ignore[assignment]

            elif kind in ('vertiport', 'building'):
                if len(fields) != 4:
                    raise ValueError(
                        f"{path}:{line_num}: '{kind}' row needs 3 values (x,y,z), "
                        f"got {len(fields) - 1}"
                    )
                position = tuple(float(v) for v in fields[1:4])
                if kind == 'vertiport':
                    vertiport_positions.append(position)  # type: ignore[arg-type]
                else:
                    building_positions.append(position)  # type: ignore[arg-type]

            else:
                raise ValueError(f"{path}:{line_num}: unknown row kind '{kind}'")

    if boundary is None:
        raise ValueError(f"{path}: missing required 'boundary' row")

    x_min, x_max, y_min, y_max = boundary
    return PlacementSpec(
        x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max,
        vertiport_positions=vertiport_positions,
        building_positions=building_positions,
    )
