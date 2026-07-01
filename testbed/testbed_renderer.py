"""TestbedRenderer — adds 3D extrusion for synthetic buildings.

Restricted areas are drawn flat at z=0 in the base Renderer (renderer.py:502-528) —
that's true for OSM-derived buildings too; no extrusion exists anywhere today. This
subclass adds the extrusion using the same bar3d-tower technique the base Renderer
already uses for vertiports (renderer.py:358-362: rooted at z=0, extruded up to
altitude) — applied here to each synthetic building's sampled z_height. Everything
else (2D rendering, UAV/trajectory drawing, animation/saving) is inherited unchanged.
"""
from urbannav.renderer import Renderer


class TestbedRenderer(Renderer):
    def _draw_map_3d(self, ax) -> None:
        super()._draw_map_3d(ax)
        if self._airspace is None:
            return
        for b in getattr(self._airspace, 'buildings', []):
            ax.bar3d(
                b.center[0] - b.width / 2, b.center[1] - b.depth / 2, 0.0,
                b.width, b.depth, b.z_height,
                color='red', alpha=0.5, zsort='average',
            )
