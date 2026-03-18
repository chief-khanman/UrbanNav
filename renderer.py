from __future__ import annotations

import math
import os
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401 — registers '3d' projection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import box as _shapely_box


class Renderer:
    """2D/3D renderer for UAM simulation episodes.

    Supports three rendering modes (set via RenderingConfig.mode):
        'realtime'  — draws each step interactively using plt.pause().
                      Requires a GUI display (e.g. run locally, not headless).
        'offline'   — collects lightweight frame snapshots during the run and
                      saves a GIF/MP4 animation when save() is called.
        'both'      — does both simultaneously.

    The dimensionality of the render is controlled by simulator_mode:
        '2D'  — standard top-down map overlay (default).
        '3D'  — matplotlib Axes3D scene; map drawn as Poly3DCollection at z=0.

    Visual style (2D):
        - Map boundary: gray fill, alpha 0.5
        - Restricted areas: red fill; buffer: orange, alpha 0.3
        - Vertiports: green squares ('gs'), markersize 2
        - UAV detection radius: green dashed circle, alpha 0.3
        - UAV NMAC radius: orange circle, alpha 0.4
        - UAV body: solid blue circle, alpha 0.7
        - UAV heading: black FancyArrowPatch
        - UAV trajectory: translucent blue trail
        - Mission line (start → goal): dashed, alpha 0.5

    Visual style (3D):
        - Map boundary: gray Poly3DCollection at z=0
        - Restricted areas: red Poly3DCollection at z=0
        - Vertiports: small green bar3d boxes at their altitude
        - UAV detection radius: equatorial ring (green dashed)
        - UAV NMAC radius: equatorial ring (orange)
        - UAV body: blue scatter point
        - UAV trajectory: 3D line
        - Mission line: 3D dashed line
        - Heading arrow: skipped in 3D
    """

    def __init__(self, config=None, simulator_mode: str = '2D') -> None:
        from component_schema import RenderingConfig
        cfg = config if config is not None else RenderingConfig()

        self.enabled: bool           = cfg.enabled
        self.mode: str               = cfg.mode          # 'realtime'|'offline'|'both'
        self.output_dir: str         = cfg.output_dir
        self.output_filename: str    = cfg.output_filename
        self.realtime_sleep: float   = cfg.realtime_sleep
        self.frame_skip: int         = cfg.frame_skip

        self._sim_mode: str          = simulator_mode    # '2D' or '3D'
        self._airspace               = None
        self._extent: Optional[Tuple[float, float, float, float]] = None  # (xmin,xmax,ymin,ymax)

        # Offline storage
        self._frames: List[Dict]     = []

        # Real-time figure
        self._fig                    = None
        self._ax                     = None
        # {uav_id: [(x, y, z), ...]} — z always stored; 2D drawing ignores it
        self._rt_traj: Dict          = {}

        # Internal frame counter for frame_skip logic
        self._step_counter: int      = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def reset(self, airspace=None) -> None:
        """Prepare the renderer for a new episode.

        Args:
            airspace: Airspace object (provides vertiport list and map GDFs).
                      Pass after SimulatorManager.reset() has built the airspace.
        """
        if not self.enabled:
            return

        self._airspace     = airspace
        self._frames       = []
        self._step_counter = 0
        self._rt_traj      = {}
        self._extent       = self._compute_extent()

        if self._fig is not None:
            plt.close(self._fig)
            self._fig = None
            self._ax  = None

    # ------------------------------------------------------------------
    # Per-step interface — called from UAMSimulator.step()
    # ------------------------------------------------------------------

    def render_step(self, uav_dict: Dict, step_num: int) -> None:
        """Snapshot the current step; draw if real-time mode is active."""
        if not self.enabled:
            return

        self._step_counter += 1
        if self._step_counter % (self.frame_skip + 1) != 0:
            return

        frame = self._build_frame(uav_dict, step_num)

        if self.mode in ('offline', 'both'):
            self._frames.append(frame)

        if self.mode in ('realtime', 'both'):
            self._draw_realtime(frame)

    # ------------------------------------------------------------------
    # Offline animation save — called from UAMSimulator.render()
    # ------------------------------------------------------------------

    def save(self, episode_id: int = 0) -> None:
        """Build an animation from collected frames and write it to disk.

        Tries GIF first (no external dependencies via Pillow), then MP4
        (requires ffmpeg).
        """
        if not self.enabled or self.mode not in ('offline', 'both'):
            return
        if not self._frames:
            print('[Renderer] No frames collected — skipping save.')
            return

        os.makedirs(self.output_dir, exist_ok=True)
        out_base = os.path.join(
            self.output_dir, f'{self.output_filename}_ep{episode_id}'
        )

        # Precompute per-UAV trajectories across all frames so animate() is fast
        traj = self._precompute_trajectories()

        if self._sim_mode == '3D':
            fig = plt.figure(figsize=(10, 10))
            ax  = fig.add_subplot(111, projection='3d')
        else:
            fig, ax = plt.subplots(figsize=(10, 10))

        frames = self._frames

        def animate(i: int):
            ax.clear()
            self._draw_scene(ax, frames[i], traj, up_to_frame=i)
            return []

        ani = FuncAnimation(
            fig, animate, frames=len(frames), interval=100, blit=False
        )

        # --- GIF (always attempted first — no ffmpeg needed) ---
        gif_path = f'{out_base}.gif'
        try:
            ani.save(gif_path, writer=PillowWriter(fps=10), dpi=150)
            print(f'[Renderer] Saved → {gif_path}')
        except Exception as exc:
            print(f'[Renderer] GIF save failed: {exc}')

        # --- MP4 (requires ffmpeg) ---
        try:
            from matplotlib.animation import FFMpegWriter
            mp4_path = f'{out_base}.mp4'
            writer = FFMpegWriter(
                fps=10,
                metadata={'title': 'UAM Simulation'},
                bitrate=5000,
                extra_args=['-vcodec', 'mpeg4', '-pix_fmt', 'yuv420p'],
            )
            ani.save(mp4_path, writer=writer, dpi=150)
            print(f'[Renderer] Saved → {mp4_path}')
        except Exception as exc:
            print(f'[Renderer] MP4 save failed (ffmpeg installed?): {exc}')

        plt.close(fig)

    # ------------------------------------------------------------------
    # Internal — real-time drawing
    # ------------------------------------------------------------------

    def _draw_realtime(self, frame: Dict) -> None:
        if self._fig is None:
            if self._sim_mode == '3D':
                self._fig = plt.figure(figsize=(10, 10))
                self._ax  = self._fig.add_subplot(111, projection='3d')
            else:
                self._fig, self._ax = plt.subplots(figsize=(10, 10))
            plt.ion()

        # Update per-UAV trajectory history — always (x, y, z)
        for uid, d in frame['uavs'].items():
            self._rt_traj.setdefault(uid, []).append((d['x'], d['y'], d['z']))

        self._ax.clear()
        self._draw_scene(self._ax, frame, self._rt_traj, up_to_frame=None)
        plt.draw()
        plt.pause(self.realtime_sleep)

    # ------------------------------------------------------------------
    # Internal — scene drawing (shared by real-time and offline)
    # ------------------------------------------------------------------

    def _draw_scene(
        self,
        ax,
        frame: Dict,
        traj: Dict,
        up_to_frame: Optional[int],
    ) -> None:
        """Draw one animation frame onto *ax*.

        Dispatches to _draw_scene_3d() when simulator_mode is '3D';
        otherwise executes the standard 2D drawing path.

        Args:
            ax:           Matplotlib axes to draw on.
            frame:        Frame dict built by _build_frame().
            traj:         {uav_id: [(x, y, z, frame_idx), ...]} for offline,
                          or {uav_id: [(x, y, z), ...]} for real-time.
            up_to_frame:  For offline mode, only draw traj points with
                          frame_idx <= up_to_frame.  None = draw all (real-time).
        """
        if self._sim_mode == '3D':
            self._draw_scene_3d(ax, frame, traj, up_to_frame)
            return

        # ----------------------------------------------------------------
        # 2D drawing path — unchanged from original implementation
        # ----------------------------------------------------------------
        ax.set_aspect('equal')
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.4)

        # ---- static map layer ----
        self._draw_map(ax)

        # ---- vertiports ----
        if self._airspace is not None:
            for vp in self._airspace.vertiport_list:
                ax.plot(vp.x, vp.y, 'gs', markersize=2, zorder=5)

        # ---- UAV trajectories ----
        for uid, pts in traj.items():
            if up_to_frame is None:
                # real-time: pts are (x, y, z) tuples — unpack first two
                draw_pts = [(x, y) for x, y, _z in pts]
            else:
                # offline: pts are (x, y, z, frame_idx) tuples
                draw_pts = [(x, y) for x, y, _z, fi in pts if fi <= up_to_frame]

            if len(draw_pts) > 1:
                xs, ys = zip(*draw_pts)
                ax.plot(xs, ys, '-', color='#1f77b4', linewidth=1.5, alpha=0.45)

        # ---- UAVs ----
        for uid, d in frame['uavs'].items():
            x, y = d['x'], d['y']
            hdg  = d['heading']

            # Mission start → goal dashed line
            if d['mission_start'] and d['mission_end']:
                sx, sy = d['mission_start']
                ex, ey = d['mission_end']
                ax.plot([sx, ex], [sy, ey], '--', color='gray',
                        linewidth=1.2, alpha=0.5, zorder=2)

            # Detection radius (dashed green)
            ax.add_patch(Circle(
                (x, y), d['detection_radius'],
                fill=False, color='green', alpha=0.3,
                linewidth=1.5, linestyle='--', zorder=3,
            ))

            # NMAC radius (orange)
            ax.add_patch(Circle(
                (x, y), d['nmac_radius'],
                fill=False, color='orange', alpha=0.4,
                linewidth=1.5, zorder=3,
            ))

            # Body (solid blue)
            ax.add_patch(Circle(
                (x, y), d['radius'],
                fill=True, color='blue', alpha=0.7, zorder=4,
            ))

            #! TODO fix PID gains so heading arrows are less chaotic before re-enabling this
            # # Heading arrow
            # arrow_len = d['radius'] * 5
            # ax.add_patch(FancyArrowPatch(
            #     (x, y),
            #     (x + arrow_len * math.cos(hdg), y + arrow_len * math.sin(hdg)),
            #     color='black', arrowstyle='->', mutation_scale=10,
            #     linewidth=2.5, zorder=5,
            # ))

            # UAV ID label
            ax.annotate(
                str(uid), (x, y),
                textcoords='offset points', xytext=(4, 4),
                fontsize=8, fontweight='bold', color='navy', zorder=6,
            )

        # ---- axis limits and title ----
        if self._extent is not None:
            xmin, xmax, ymin, ymax = self._extent
            ax.set_xlim(xmin, xmax)
            ax.set_ylim(ymin, ymax)
        else:
            self._fit_axis(ax, frame)

        ax.set_title(f'UAM Simulation — Step {frame["step"]}', fontsize=11)

    # ------------------------------------------------------------------
    # Internal — 3D scene drawing
    # ------------------------------------------------------------------

    def _draw_scene_3d(
        self,
        ax,
        frame: Dict,
        traj: Dict,
        up_to_frame: Optional[int],
    ) -> None:
        """Draw one 3D animation frame onto a matplotlib Axes3D *ax*."""
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.4)

        # ---- static map layer ----
        self._draw_map_3d(ax)

        # ---- vertiports ----
        if self._airspace is not None:
            # Box half-width proportional to map scale
            if self._extent is not None:
                dh = (self._extent[1] - self._extent[0]) * 0.008
            else:
                dh = 200.0
            for vp in self._airspace.vertiport_list:
                vp_z = vp.location.z if vp.location.has_z else 0.0
                # Draw vertiport as a tower: base at z=0, top at the vertiport altitude.
                # This roots each pad to the ground and makes altitude immediately legible.
                ax.bar3d(
                    vp.x - dh, vp.y - dh, 0.0,
                    dh * 2, dh * 2, vp_z,
                    color='green', alpha=0.7, zsort='average',
                )

        # ---- UAV trajectories ----
        _theta = np.linspace(0, 2 * math.pi, 64)

        for uid, pts in traj.items():
            if up_to_frame is None:
                # real-time: pts are (x, y, z)
                draw_pts = pts
            else:
                # offline: pts are (x, y, z, frame_idx)
                draw_pts = [(x, y, z) for x, y, z, fi in pts if fi <= up_to_frame]

            if len(draw_pts) > 1:
                xs, ys, zs = zip(*draw_pts)
                ax.plot3D(xs, ys, zs, '-', color='#1f77b4', linewidth=1.5, alpha=0.45)

        # ---- UAVs ----
        for uid, d in frame['uavs'].items():
            x, y, z = d['x'], d['y'], d['z']

            # Mission start → goal dashed 3D line
            if d['mission_start'] and d['mission_end']:
                sx, sy = d['mission_start']
                ex, ey = d['mission_end']
                sz = d['mission_start_z']
                ez = d['mission_end_z']
                ax.plot3D([sx, ex], [sy, ey], [sz, ez], '--', color='gray',
                          linewidth=1.2, alpha=0.5)

            # Detection radius — equatorial ring (green dashed)
            r_det = d['detection_radius']
            ax.plot(
                x + r_det * np.cos(_theta),
                y + r_det * np.sin(_theta),
                z,
                color='green', alpha=0.3, linewidth=1.5, linestyle='--',
            )

            # NMAC radius — equatorial ring (orange)
            r_nmac = d['nmac_radius']
            ax.plot(
                x + r_nmac * np.cos(_theta),
                y + r_nmac * np.sin(_theta),
                z,
                color='orange', alpha=0.4, linewidth=1.5,
            )

            # UAV body — scatter sphere approximation
            ax.scatter(
                [x], [y], [z],
                s=(d['radius'] / max(r_det, 1.0) * 300) ** 2,
                color='blue', alpha=0.7,
            )

            # UAV ID label
            ax.text(x, y, z, f'  {uid}', fontsize=8, fontweight='bold', color='navy')

        # ---- axis limits and title ----
        if self._extent is not None:
            xmin, xmax, ymin, ymax = self._extent
            ax.set_xlim3d(xmin, xmax)
            ax.set_ylim3d(ymin, ymax)
        else:
            self._fit_axis_3d(ax, frame)

        # z limits: cover the UAV altitude range (vertiports are 1500–3500 m)
        zs_all = [d['z'] for d in frame['uavs'].values()]
        if zs_all:
            z_lo = min(0.0, min(zs_all) - 500)
            z_hi = max(zs_all) + 500
        else:
            z_lo, z_hi = 0.0, 4000.0
        ax.set_zlim3d(z_lo, z_hi)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'UAM Simulation 3D — Step {frame["step"]}', fontsize=11)

    # ------------------------------------------------------------------
    # Internal — map drawing helpers
    # ------------------------------------------------------------------

    def _draw_map(self, ax) -> None:
        """Draw airspace boundary and restricted areas onto a 2D ax."""
        if self._airspace is None:
            return
        try:
            self._airspace.location_utm_gdf.plot(
                ax=ax, color='gray', linewidth=0.6, alpha=0.5,
            )
        except Exception:
            pass

        try:
            for tag in self._airspace.location_tags.keys():
                self._airspace.location_utm[tag].plot(
                    ax=ax, color='red', alpha=0.7,
                )
                self._airspace.location_utm_buffer[tag].plot(
                    ax=ax, color='orange', alpha=0.3,
                )
        except Exception:
            pass

    def _draw_map_3d(self, ax) -> None:
        """Draw airspace boundary and restricted areas onto a 3D ax at z=0.

        Each polygon is clipped to the current viewport extent before being
        added as a Poly3DCollection.  Without clipping, the full UTM boundary
        (tens of kilometres across) extends far beyond the visible XY plane and
        produces rendering artefacts in matplotlib's 3D projection.
        """
        if self._airspace is None:
            return

        # Build a Shapely clip box from the renderer extent.  Falls back to
        # a very large box (effectively no clip) when extent is not yet set.
        if self._extent is not None:
            xmin, xmax, ymin, ymax = self._extent
            clip = _shapely_box(xmin, ymin, xmax, ymax)
        else:
            clip = None

        def _clipped_verts(poly):
            """Return [(x, y, 0.0), ...] clipped to clip box, or None."""
            if not hasattr(poly, 'exterior'):
                return None
            geom = poly.intersection(clip) if clip is not None else poly
            if geom.is_empty:
                return None
            # intersection may return a MultiPolygon — yield each part
            parts = geom.geoms if hasattr(geom, 'geoms') else [geom]
            result = []
            for part in parts:
                if hasattr(part, 'exterior'):
                    result.append([(cx, cy, 0.0) for cx, cy in part.exterior.coords])
            return result or None

        # Airspace boundary
        try:
            geom  = self._airspace.location_utm_gdf.geometry.iloc[0]
            polys = geom.geoms if hasattr(geom, 'geoms') else [geom]
            for poly in polys:
                rings = _clipped_verts(poly)
                if rings:
                    ax.add_collection3d(
                        Poly3DCollection(rings, alpha=0.2, color='gray', linewidth=0.6)
                    )
        except Exception:
            pass

        # Restricted areas
        try:
            for tag in self._airspace.location_tags.keys():
                for _, row in self._airspace.location_utm[tag].iterrows():
                    geom  = row.geometry
                    polys = geom.geoms if hasattr(geom, 'geoms') else [geom]
                    for poly in polys:
                        rings = _clipped_verts(poly)
                        if rings:
                            ax.add_collection3d(
                                Poly3DCollection(rings, alpha=0.5, color='red')
                            )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal — frame building & utilities
    # ------------------------------------------------------------------

    def _build_frame(self, uav_dict: Dict, step_num: int) -> Dict:
        """Create a lightweight serialisable snapshot of the current step."""
        uavs = {}
        for uid, uav in uav_dict.items():
            try:
                p  = uav.mission_start_point
                ms = (p.x, p.y)
                ms_z = p.z if p.has_z else 0.0
            except AttributeError:
                ms = None
                ms_z = 0.0
            try:
                p  = uav.mission_end_point
                me = (p.x, p.y)
                me_z = p.z if p.has_z else 0.0
            except AttributeError:
                me = None
                me_z = 0.0

            uavs[uid] = {
                'x':               uav.current_position.x,
                'y':               uav.current_position.y,
                'z':               getattr(uav, 'pz', 0.0),
                'heading':         uav.current_heading,
                'radius':          uav.radius,
                'nmac_radius':     uav.nmac_radius,
                'detection_radius': uav.detection_radius,
                'mission_start':   ms,
                'mission_end':     me,
                'mission_start_z': ms_z,
                'mission_end_z':   me_z,
            }
        return {'step': step_num, 'uavs': uavs}

    def _precompute_trajectories(self) -> Dict:
        """Build {uav_id: [(x, y, z, frame_idx), ...]} from all stored frames."""
        traj: Dict = {}
        for fi, frame in enumerate(self._frames):
            for uid, d in frame['uavs'].items():
                traj.setdefault(uid, []).append((d['x'], d['y'], d['z'], fi))
        return traj

    def _compute_extent(self) -> Optional[Tuple[float, float, float, float]]:
        """Compute a fixed bounding box from vertiport locations plus margin."""
        if self._airspace is None or not self._airspace.vertiport_list:
            return None
        xs = [vp.x for vp in self._airspace.vertiport_list]
        ys = [vp.y for vp in self._airspace.vertiport_list]
        margin = max(2000.0, (max(xs) - min(xs)) * 0.25, (max(ys) - min(ys)) * 0.25)
        return (min(xs) - margin, max(xs) + margin,
                min(ys) - margin, max(ys) + margin)

    def _fit_axis(self, ax, frame: Dict) -> None:
        """Fallback: fit 2D axis to current UAV positions when no fixed extent."""
        pts = [(d['x'], d['y']) for d in frame['uavs'].values()]
        if not pts:
            return
        xs, ys = zip(*pts)
        margin = max(500.0, (max(xs) - min(xs)) * 0.15, (max(ys) - min(ys)) * 0.15)
        ax.set_xlim(min(xs) - margin, max(xs) + margin)
        ax.set_ylim(min(ys) - margin, max(ys) + margin)

    def _fit_axis_3d(self, ax, frame: Dict) -> None:
        """Fallback: fit 3D axis to current UAV positions when no fixed extent."""
        pts = [(d['x'], d['y']) for d in frame['uavs'].values()]
        if not pts:
            return
        xs, ys = zip(*pts)
        margin = max(500.0, (max(xs) - min(xs)) * 0.15, (max(ys) - min(ys)) * 0.15)
        ax.set_xlim3d(min(xs) - margin, max(xs) + margin)
        ax.set_ylim3d(min(ys) - margin, max(ys) + margin)
