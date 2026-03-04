from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from component_schema import SimulatorState


class MetricsCollector:
    """Accumulates per-step simulation data and computes episode-level metrics.

    Called each step by Logger.log_step().  Stores a lightweight snapshot of
    the simulator state (UAV positions, speeds, event counters) and defers all
    aggregation to _calculate_metrics() so callers only pay the cost once.

    Attributes:
        _steps: Ordered list of per-step records appended by record().
    """

    def __init__(self) -> None:
        self._steps: List[Dict[str, Any]] = []

    # ------------------------------------------------------------------
    # Primary interface
    # ------------------------------------------------------------------

    def record(
        self,
        state: SimulatorState,
        actions: Optional[Dict[int, Tuple[float, float]]] = None,
        collisions: Optional[Tuple[Dict, Dict, Dict, Dict, Dict]] = None,
    ) -> None:
        """Append one step record to the internal buffer.

        Args:
            state:      SimulatorState snapshot for this step.  Provides
                        current_step, atc_state (Dict[uav_id, UAV]), and
                        airspace_state (List[Vertiport]).
            actions:    Optional Dict[uav_id, (ax, ay) | (accel, yaw_rate)]
                        from the controller/AerBus for this step.
            collisions: Optional 5-tuple returned by SimulatorManager._step_uavS():
                        (ra_detect, uav_detect, nmac_dict,
                         ra_collision_dict, uav_collision_dict).
        """
        uav_dict: Dict = state.atc_state or {}

        # --- UAV snapshots ---
        uav_snapshots: Dict[int, Dict[str, Any]] = {}
        for uav_id, uav in uav_dict.items():
            pos = uav.current_position
            try:
                dist_to_goal = pos.distance(uav.mission_end_point)
            except AttributeError:
                dist_to_goal = None

            uav_snapshots[uav_id] = {
                'x':               pos.x,
                'y':               pos.y,
                'speed':           getattr(uav, 'current_speed', 0.0),
                'heading':         getattr(uav, 'current_heading', 0.0),
                'vx':              getattr(uav, 'vx', 0.0),
                'vy':              getattr(uav, 'vy', 0.0),
                'nmac_count':      getattr(uav, 'nmac_count', 0),
                'mission_complete': getattr(uav, 'current_mission_complete_status', False),
                'dist_to_goal':    dist_to_goal,
            }

        # --- Collision/NMAC summary ---
        collision_summary: Dict[str, Any] = {
            'nmac_pairs':          [],
            'uav_collision_pairs': [],
            'ra_collision_ids':    [],
        }
        if collisions is not None:
            _, _, nmac_dict, ra_collision_dict, uav_collision_dict = collisions

            # NMACs: nmac_dict = { uav_id -> [partner_ids] }
            seen_nmac: set = set()
            for uid, partners in (nmac_dict or {}).items():
                for pid in (partners or []):
                    pair = tuple(sorted((uid, pid)))
                    if pair not in seen_nmac:
                        seen_nmac.add(pair)
                        collision_summary['nmac_pairs'].append(list(pair))

            # UAV-UAV collisions
            seen_col: set = set()
            for uid, partners in (uav_collision_dict or {}).items():
                for pid in (partners or []):
                    pair = tuple(sorted((uid, pid)))
                    if pair not in seen_col:
                        seen_col.add(pair)
                        collision_summary['uav_collision_pairs'].append(list(pair))

            # Restricted-area collisions
            for uid, ra_ids in (ra_collision_dict or {}).items():
                if ra_ids:
                    collision_summary['ra_collision_ids'].append(uid)

        step_record: Dict[str, Any] = {
            'step':            state.currentstep,
            'num_active_uavs': len(uav_dict),
            'uavs':            uav_snapshots,
            'actions':         _serialize(actions or {}),
            'collisions':      collision_summary,
        }
        self._steps.append(step_record)

    def reset(self) -> None:
        """Clear all accumulated step data for a new episode."""
        self._steps = []

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------

    def _calculate_metrics(self) -> Dict[str, Any]:
        """Aggregate _steps into an episode-level metrics dictionary.

        Returns an empty dict if no steps have been recorded.

        Returns:
            Dict with keys:
              total_steps, total_nmac_events, total_uav_collision_events,
              total_ra_collision_events, unique_uavs, missions_completed,
              avg_speed, min_speed, max_speed,
              avg_dist_to_goal_final, peak_active_uavs.
        """
        if not self._steps:
            return {}

        total_nmac_events      = sum(len(s['collisions']['nmac_pairs'])          for s in self._steps)
        total_uav_col_events   = sum(len(s['collisions']['uav_collision_pairs'])  for s in self._steps)
        total_ra_col_events    = sum(len(s['collisions']['ra_collision_ids'])     for s in self._steps)
        peak_active_uavs       = max(s['num_active_uavs'] for s in self._steps)

        # Aggregate per-UAV data across all steps
        all_uav_ids: set = set()
        for s in self._steps:
            all_uav_ids.update(s['uavs'].keys())

        missions_completed = 0
        all_speeds: List[float] = []

        # Final-step distance-to-goal per UAV
        final_dists: List[float] = []
        last_step = self._steps[-1]

        for uav_id in all_uav_ids:
            uav_step_snapshots = [
                s['uavs'][uav_id] for s in self._steps if uav_id in s['uavs']
            ]
            if uav_step_snapshots:
                if any(snap['mission_complete'] for snap in uav_step_snapshots):
                    missions_completed += 1
                all_speeds.extend(snap['speed'] for snap in uav_step_snapshots)

            if uav_id in last_step['uavs']:
                dist = last_step['uavs'][uav_id]['dist_to_goal']
                if dist is not None:
                    final_dists.append(dist)

        avg_speed = float(np.mean(all_speeds)) if all_speeds else 0.0
        min_speed = float(np.min(all_speeds))  if all_speeds else 0.0
        max_speed = float(np.max(all_speeds))  if all_speeds else 0.0
        avg_dist_to_goal_final = float(np.mean(final_dists)) if final_dists else 0.0

        return {
            'total_steps':              len(self._steps),
            'total_nmac_events':        total_nmac_events,
            'total_uav_collision_events': total_uav_col_events,
            'total_ra_collision_events':  total_ra_col_events,
            'unique_uavs':              len(all_uav_ids),
            'missions_completed':       missions_completed,
            'peak_active_uavs':         peak_active_uavs,
            'avg_speed':                avg_speed,
            'min_speed':                min_speed,
            'max_speed':                max_speed,
            'avg_dist_to_goal_final':   avg_dist_to_goal_final,
        }

    def get_metrics(self) -> Dict[str, Any]:
        """Return the episode-level summary metrics dict.

        Returns:
            Dict produced by _calculate_metrics().
        """
        return self._calculate_metrics()

    def get_step_data(self) -> List[Dict[str, Any]]:
        """Return the raw per-step records list.

        Returns:
            Ordered list of step record dicts appended by record().
        """
        return self._steps


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def _serialize(obj: Any) -> Any:
    """Recursively convert numpy types and tuples to JSON-safe Python primitives.

    Args:
        obj: Any Python object that may contain numpy arrays, ndarrays, or
             other non-serializable types.

    Returns:
        JSON-serializable version of obj.
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return {str(k): _serialize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_serialize(x) for x in obj]
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (bool, int, float, str, type(None))):
        return obj
    return str(obj)
