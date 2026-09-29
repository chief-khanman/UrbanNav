from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from urbannav.component_schema import SimulatorState


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
        # Tracks each UAV's mission_complete status as of the previous record()
        # call, so a new mission assigned mid-episode (which resets
        # current_mission_complete_status back to False) doesn't get missed -
        # only a False->True transition counts as a completed mission.
        self._prev_mission_status: Dict[int, bool] = {}

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

        Note on NMAC vs. collision attribution: nmac_dict, ra_collision_dict,
        and uav_collision_dict are produced by the get_nmac() /
        get_collision_restricted_area() / get_collision_uavS() sensor queries
        made inside SimulatorManager._step_uavS(), which runs earlier in the
        same step() call, before this method ever sees `state`. By the time
        record() runs, any UAV in a uav_collision_pair or ra_collision has
        already been removed from `uav_dict` (by ATC.remove_uavs_by_id()), so
        its start/end vertiport is no longer resolvable. UAVs in an NMAC pair
        are never removed, so they are still present in `uav_dict` and get
        attributed to a specific edge. Collision and RA-collision counts are
        recorded as step-level totals; NMAC counts are recorded per-edge.
        """
        uav_dict: Dict = state.atc_state or {}
        vertiport_list = state.airspace_state or []
        # Built here (not just where the edge-snapshot loop needs it below) so the
        # UAV loop can also resolve each UAV's true mission-destination vertiport.
        vp_id_to_idx: Dict[int, int] = {id(vp): idx for idx, vp in enumerate(vertiport_list)}

        # --- UAV snapshots ---
        uav_snapshots: Dict[int, Dict[str, Any]] = {}
        new_mission_completions_this_step = 0
        for uav_id, uav in uav_dict.items():
            pos = uav.current_position
            
            #TODO: remove try/except block
            try:
                #TODO: add this as an attr, that's updated during
                # simulator_manager.step() -> dynamics_engine.step() ...
                # ... -> dynamics[uav_instance].step() <update dist_2_goal> HERE
                end_pt  = uav.mission_end_point
                end_z   = end_pt.z if end_pt.has_z else 0.0
                uav_z   = getattr(uav, 'pz', 0.0)
                dist_to_goal = math.sqrt(
                    (pos.x - end_pt.x) ** 2 +
                    (pos.y - end_pt.y) ** 2 +
                    (uav_z - end_z)    ** 2
                )
            except AttributeError:
                dist_to_goal = None

            # Count a mission as completed on the False->True transition of
            # current_mission_complete_status, rather than just sampling the
            # raw status each step. Without this, a UAV that completes a
            # mission and is reassigned a new one within the same episode
            # (assign_start_end() resets current_mission_complete_status to
            # False) would only ever show its latest mission's status, losing
            # every mission completed earlier in the episode.
            curr_mission_complete = getattr(uav, 'current_mission_complete_status', False)
            prev_mission_complete = self._prev_mission_status.get(uav_id, False)
            if curr_mission_complete and not prev_mission_complete:
                uav.num_missions_completed_in_episode = (
                    getattr(uav, 'num_missions_completed_in_episode', 0) + 1
                )
                new_mission_completions_this_step += 1
            self._prev_mission_status[uav_id] = curr_mission_complete

            # True mission-destination vertiport index, resolved the same way the
            # edge-snapshot loop below resolves start/end vertiport -> index. None
            # when the UAV has no end_vertiport assigned yet (idle, no mission).
            end_vertiport = getattr(uav, 'end_vertiport', None)
            target_vertiport_idx = (
                vp_id_to_idx.get(id(end_vertiport)) if end_vertiport is not None else None
            )

            uav_snapshots[uav_id] = {
                'x':               pos.x,
                'y':               pos.y,
                'z':               getattr(uav, 'pz', 0.0),
                'speed':           getattr(uav, 'current_speed', 0.0),
                'heading':         getattr(uav, 'current_heading', 0.0),
                'vx':              getattr(uav, 'vx', 0.0),
                'vy':              getattr(uav, 'vy', 0.0),
                'vz':              getattr(uav, 'vz', 0.0),
                #TODO: fix logic for incrementing nmac_count - sensor[uav_instance].get_nmac() -> increment uav.nmac_count
                'nmac_count':      getattr(uav, 'nmac_count', 0), #! sensor does not increment NMAC count
                'collision_status': getattr(uav, 'collision_status', 1),
                'mission_complete': curr_mission_complete,
                'num_missions_completed': getattr(uav, 'num_missions_completed_in_episode', 0),
                'dist_to_goal':    dist_to_goal,
                'target_vertiport_idx': target_vertiport_idx,
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

        # Unique UAV ids removed this step by ATC.remove_uavs_by_id() (every
        # id in every uav_collision_pair, plus every ra_collision id -- NMAC
        # never removes). Deduped via a set in case a UAV somehow appears in
        # both categories in the same step.
        removed_ids: set = set()
        for pair in collision_summary['uav_collision_pairs']:
            removed_ids.update(pair)
        removed_ids.update(collision_summary['ra_collision_ids'])
        num_removed_this_step = len(removed_ids)
        total_collision_events_this_step = len(collision_summary['uav_collision_pairs'])
        total_ra_collision_events_this_step = len(collision_summary['ra_collision_ids'])

        # --- Vertiport snapshots (graph-level surrogate data) ---
        vertiport_snapshots: Dict[int, Dict[str, Any]] = {}
        for vp_idx, vp in enumerate(vertiport_list):
            grounded_ids = list(vp.uav_id_list) + list(vp.landing_queue)
            speeds = [
                uav_snapshots[uid]['speed'] for uid in grounded_ids if uid in uav_snapshots
            ]
            vertiport_snapshots[vp_idx] = {
                'x': vp.location.x,
                'y': vp.location.y,
                'n_grounded': len(vp.uav_id_list),
                'n_landing_queue': len(vp.landing_queue),
                'capacity': vp.landing_takeoff_capacity,
                #! what am i using the speed for - what does vertiport node have to do with speed 
                'speed_sum': float(sum(speeds)),
                'speed_min': float(min(speeds)) if speeds else 0.0,
                'speed_max': float(max(speeds)) if speeds else 0.0,
            }

        # --- Edge snapshots: count in-flight UAVs per OD vertiport pair ---
        edge_snapshots: Dict[str, Dict[str, Any]] = {}
        for uav_id, uav in uav_dict.items():
            if not getattr(uav, 'uav_in_flight', False):
                continue
            src_vp = getattr(uav, 'start_vertiport', None)
            dst_vp = getattr(uav, 'end_vertiport', None)
            if src_vp is None or dst_vp is None:
                continue
            src_idx = vp_id_to_idx.get(id(src_vp))
            dst_idx = vp_id_to_idx.get(id(dst_vp))
            if src_idx is None or dst_idx is None:
                continue
            edge_key = f"{src_idx}->{dst_idx}"
            if edge_key not in edge_snapshots:
                total_dist = src_vp.location.distance(dst_vp.location)
                edge_snapshots[edge_key] = {
                    'src': src_idx,
                    'dst': dst_idx,
                    'n_in_transit': 0,
                    'progress_sum': 0.0,
                    'edge_distance': total_dist,
                    'speed_sum': 0.0,
                    'speed_min': 0.0,
                    'speed_max': 0.0,
                    'nmac_count': 0,
                }
            entry = edge_snapshots[edge_key]
            #! speed: what is the reasoning for using speed 
            #  speed: i think its to corelate speed to NMAC and collision - if there is collision then suddenly the speed wont change after collision step 
            #  suggested alternative - 
            speed = getattr(uav, 'current_speed', 0.0)
            if entry['n_in_transit'] == 0:
                entry['speed_min'] = speed
                entry['speed_max'] = speed
            else:
                entry['speed_min'] = min(entry['speed_min'], speed)
                entry['speed_max'] = max(entry['speed_max'], speed)
            entry['speed_sum'] += speed
            entry['n_in_transit'] += 1
            total_dist = entry['edge_distance']
            if total_dist > 0:
                covered = uav.current_position.distance(src_vp.location)
                entry['progress_sum'] += min(covered / total_dist, 1.0)

        # Localize NMAC events onto whichever edge each involved UAV was
        # in-flight on this step (safe: NMAC never removes, so both UAVs in
        # every nmac_pair are still present in uav_dict here). A pair
        # spanning two different edges increments both -- acceptable for
        # this per-edge signal, which is model input context, not the
        # episode-level NMAC count (that's summed across edges downstream,
        # and NMAC is the one event type where that sum can't double-count,
        # since each UAV can only be in-flight on one edge at a time).
        for pair in collision_summary['nmac_pairs']:
            for uid in pair:
                nmac_uav = uav_dict.get(uid)
                if nmac_uav is None or not getattr(nmac_uav, 'uav_in_flight', False):
                    continue
                src_vp = getattr(nmac_uav, 'start_vertiport', None)
                dst_vp = getattr(nmac_uav, 'end_vertiport', None)
                if src_vp is None or dst_vp is None:
                    continue
                src_idx = vp_id_to_idx.get(id(src_vp))
                dst_idx = vp_id_to_idx.get(id(dst_vp))
                if src_idx is None or dst_idx is None:
                    continue
                entry = edge_snapshots.get(f"{src_idx}->{dst_idx}")
                if entry is not None:
                    entry['nmac_count'] += 1

        step_record: Dict[str, Any] = {
            'step':            state.currentstep,
            'num_active_uavs': len(uav_dict),
            'uavs':            uav_snapshots,
            'actions':         _serialize(actions or {}),
            'collisions':      collision_summary,
            'vertiports':      vertiport_snapshots,
            'edges':           edge_snapshots,
            'num_removed_this_step': num_removed_this_step,
            'new_mission_completions_this_step': new_mission_completions_this_step,
            'total_collision_events_this_step': total_collision_events_this_step,
            'total_ra_collision_events_this_step': total_ra_collision_events_this_step,
        }
        self._steps.append(step_record)

    def reset(self) -> None:
        """Clear all accumulated step data for a new episode."""
        self._steps = []
        self._prev_mission_status = {}

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------

    def _calculate_metrics(self) -> Dict[str, Any]:
        """Aggregate _steps into an episode-level metrics dictionary.

        Returns an empty dict if no steps have been recorded.

        Returns:
            Dict with keys:
              total_steps, total_nmac_events, total_uav_collision_events,
              total_ra_collision_events, unique_uavs, avg_missions_completed,
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

        # Missions completed per UAV (num_missions_completed is a running
        # count maintained on the UAV by record(), incremented on every
        # False→True transition of mission_complete - so taking each UAV's
        # own last snapshot already reflects every mission it completed
        # during the episode, not just its latest one).
        mission_counts: List[int] = []
        all_speeds: List[float] = []

        # Final-step distance-to-goal per UAV
        final_dists: List[float] = []
        last_step = self._steps[-1]

        for uav_id in all_uav_ids:
            uav_step_snapshots = [
                s['uavs'][uav_id] for s in self._steps if uav_id in s['uavs']
            ]
            if uav_step_snapshots:
                mission_counts.append(uav_step_snapshots[-1]['num_missions_completed'])
                all_speeds.extend(snap['speed'] for snap in uav_step_snapshots)

            if uav_id in last_step['uavs']:
                dist = last_step['uavs'][uav_id]['dist_to_goal']
                if dist is not None:
                    final_dists.append(dist)

        avg_missions_completed = float(np.mean(mission_counts)) if mission_counts else 0.0
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
            'avg_missions_completed':   avg_missions_completed,
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
