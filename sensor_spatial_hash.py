import math
import numpy as np
from typing import Dict, List, Tuple
from uav import UAV

def _euclidean_3d(x1: float, y1: float, z1: float,
                  x2: float, y2: float, z2: float) -> float:
    """3D Euclidean distance on pre-extracted float coordinates.
    Faster than constructing Shapely Point UAVs for the narrow-phase hot path.
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)


class SpatialHash:
    """Uniform-grid spatial hash for fast broad-phase neighbour lookup.

    Partitions 3-D space into cubic cells of side `spacing`.  UAV positions
    are hashed into a flat table; a bounding-box query then returns all UAV
    IDs whose cell overlaps the query region.  False positives are possible
    (hash collisions) — the caller is responsible for the narrow-phase
    distance check.

    Two-pass build algorithm [Müller et al., "Unified Particle Physics"]:
      Pass 1 – count UAVs per cell, compute prefix sums (end boundaries).
      Pass 2 – fill dense cell_entries array, decrementing start pointers.
    After Pass 2, cell_start[h] points to the first entry for hash bucket h
    and cell_start[h+1] is the exclusive end.

    References used in sensor_partial.py draft comments are preserved here.
    """

    def __init__(self, spacing: float, max_uavs: int) -> None:
        self.spacing = spacing
        # Table sized at 2x UAV count to reduce hash collisions
        self.table_size = 2 * max_uavs
        # +1 guard index prevents out-of-bounds on the end-boundary lookup
        self.cell_start = np.zeros(self.table_size + 1, dtype=int)
        self.cell_entries = np.zeros(max_uavs, dtype=int)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _int_coords(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """Map continuous position to integer grid cell indices."""
        return (math.floor(x / self.spacing),
                math.floor(y / self.spacing),
                math.floor(z / self.spacing))

    def _hash_function(self, xi: int, yi: int, zi: int) -> int:
        """XOR-mix integer cell coordinates into a table index."""
        h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481)
        return abs(h) % self.table_size

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def build(self, uav_dict: Dict[int, UAV]) -> None:
        """Rebuild the hash table from the current UAV positions.

        Must be called once per simulation step before any query() call.
        Reads uav.px, uav.py, uav.pz from each UAV in uav_dict.

        Args:
            uav_dict: Mapping of uav_id (int) -> UAV instance.
        """
        self.cell_start.fill(0)
        self.cell_entries.fill(0)

        # Pass 1a: count UAVs per hash bucket
        for uav in uav_dict.values():
            xi, yi, zi = self._int_coords(uav.px, uav.py, uav.pz)
            h = self._hash_function(xi, yi, zi)
            self.cell_start[h] += 1

        # Pass 1b: convert counts to exclusive end boundaries (prefix sum)
        total = 0
        for i in range(self.table_size):
            count = self.cell_start[i]
            total += count
            self.cell_start[i] = total
        self.cell_start[self.table_size] = total

        # Pass 2: place UAV IDs into cell_entries, decrement start pointer
        # After this pass cell_start[h] becomes the *start* (inclusive) index
        for uav_id, uav in uav_dict.items():
            xi, yi, zi = self._int_coords(uav.px, uav.py, uav.pz)
            h = self._hash_function(xi, yi, zi)
            self.cell_start[h] -= 1
            self.cell_entries[self.cell_start[h]] = uav_id

    def query(self, pos: Tuple[float, float, float], max_dist: float) -> List[int]:
        """Return candidate UAV IDs whose grid cell overlaps the query region.

        Searches the 3-D bounding box [pos ± max_dist] across all axes.
        Returns false positives — the caller must apply a narrow-phase
        distance check to remove them.

        Args:
            pos: (x, y, z) centre of the query sphere.
            max_dist: Half-width of the bounding box (query radius).

        Returns:
            List of UAV IDs (ints) that may be within max_dist of pos.
        """
        px, py, pz = pos
        min_xi, min_yi, min_zi = self._int_coords(px - max_dist, py - max_dist, pz - max_dist)
        max_xi, max_yi, max_zi = self._int_coords(px + max_dist, py + max_dist, pz + max_dist)

        candidate_ids: List[int] = []
        for xi in range(min_xi, max_xi + 1):
            for yi in range(min_yi, max_yi + 1):
                for zi in range(min_zi, max_zi + 1):
                    h = self._hash_function(xi, yi, zi)
                    start = self.cell_start[h]
                    end = self.cell_start[h + 1]
                    for i in range(start, end):
                        candidate_ids.append(int(self.cell_entries[i]))
        return candidate_ids
