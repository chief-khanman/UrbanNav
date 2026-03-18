from __future__ import annotations

import json
import os
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from component_schema import LoggingConfig, SimulatorState, UAMConfig
from metrics_collector import MetricsCollector, _serialize


class Logger:
    """Episode logger for UrbanNav simulation runs.

    Wraps MetricsCollector to accumulate per-step data, computes episode-level
    summary metrics, and persists everything to JSON under a timestamped
    directory tree.

    When logging is disabled via LoggingConfig (enabled=False), all record and
    save operations become no-ops so the simulation runs with zero I/O overhead.

    Directory layout (relative to log_dir)::

        logs/
          episode_0_YYYY_MM_DD_HHMM/
            metadata.json        ← start metrics + config snapshot
            step_history.json    ← raw per-step records (positions, actions, events)
            episode_metrics.json ← aggregated episode summary

    Usage::

        logger = Logger(config)          # config: LoggingConfig from UAMConfig

        # Each episode:
        logger.reset()                   # starts a new episode dir
        for _ in range(total_steps):
            sim.step(...)
            logger.log_step(state)

        logger.save()                    # flush to disk

    Attributes:
        enabled:     Whether data collection and saving are active.
        log_dir:     Root directory for all episode subdirectories.
        episode_id:  Auto-incrementing integer episode counter.
    """

    def __init__(self, config: Optional[LoggingConfig] = None,
                 full_config: Optional[UAMConfig] = None) -> None:
        cfg = config or LoggingConfig()
        self.enabled: bool = cfg.enabled
        self.log_dir: str = cfg.log_dir

        if self.enabled:
            os.makedirs(self.log_dir, exist_ok=True)

        self._metrics_collector = MetricsCollector()
        self.episode_id: int = 0
        self._episode_dir: str = ''
        self._config_snapshot: Dict[str, Any] = (
            full_config.model_dump() if full_config is not None else {}
        )

        self._init_episode_dir()

    # ------------------------------------------------------------------
    # Core step interface
    # ------------------------------------------------------------------

    def log(self, message: str) -> None:
        """Print a timestamped console message.

        Args:
            message: Human-readable log string.
        """
        ts = datetime.now().strftime('%H:%M:%S')
        print(f'[Logger | ep={self.episode_id} | {ts}] {message}')

    def log_step(
        self,
        state: Optional[SimulatorState] = None,
        actions: Optional[Dict[int, Tuple[float, float]]] = None,
        collisions: Optional[Tuple[Dict, Dict, Dict, Dict, Dict]] = None,
    ) -> None:
        """Record one simulation step.

        Called by UAMSimulator.step() after each simulator tick and once
        during reset() for the initial state.  Silently ignores calls where
        state is None (e.g. the reset() call before any state exists).

        Args:
            state:      SimulatorState snapshot for this step.
            actions:    Optional control action dict from AerBus/controller.
            collisions: Optional 5-tuple from SimulatorManager._step_uavS().
        """
        if not self.enabled or state is None:
            return
        self._metrics_collector.record(state, actions=actions, collisions=collisions)

    # ------------------------------------------------------------------
    # Metrics access
    # ------------------------------------------------------------------

    def get_simulator_start_metrics(self) -> Dict[str, Any]:
        """Return a snapshot of the very first recorded step.

        Uses the internal database (step history) to extract start-of-episode
        information: initial UAV count, step index, and episode id.

        Returns:
            Dict with episode_id, start_step, num_uavs_at_start, and
            per-UAV initial positions.
        """
        steps = self._metrics_collector.get_step_data()
        if not steps:
            return {}
        first = steps[0]
        return {
            'episode_id':        self.episode_id,
            'start_step':        first['step'],
            'num_uavs_at_start': first['num_active_uavs'],
            'initial_uav_positions': {
                uid: {'x': snap['x'], 'y': snap['y'], 'z': snap['z']} #snap aka snapshot 
                for uid, snap in first['uavs'].items()
            },
        }

    def get_simulator_end_metrics(self) -> Dict[str, Any]:
        """Compute and return end-of-episode summary metrics.

        Uses the internal database (step history) to calculate aggregated
        statistics: collisions, NMACs, missions completed, speed stats, etc.

        Returns:
            Dict produced by MetricsCollector.get_metrics(), plus episode_id.
        """
        metrics = self._metrics_collector.get_metrics()
        metrics['episode_id'] = self.episode_id
        return metrics

    def get_step_render_data(self) -> List[Dict[str, Any]]:
        """Return per-step UAV position and heading data for rendering.

        Extracts the minimal subset of per-step records needed to replay or
        render the episode: step index, and for each UAV: x, y, heading.

        Returns:
            Ordered list of dicts, one per recorded step.
        """
        render_data = []
        for step in self._metrics_collector.get_step_data():
            render_data.append({
                'step': step['step'],
                'uavs': {
                    uid: {
                        'x':       snap['x'],
                        'y':       snap['y'],
                        'z':       snap['z'],
                        'heading': snap['heading'],
                        'speed':   snap['speed'],
                        'vx':      snap['vx'],
                        'vy':      snap['vy'],
                        'vz':      snap['vz'], 
                    }
                    for uid, snap in step['uavs'].items()
                },
            })
        return render_data

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save(self) -> None:
        """Flush all episode data to disk under the current episode directory.

        Writes three JSON files:
          - metadata.json        start metrics snapshot
          - step_history.json    raw per-step records
          - episode_metrics.json aggregated episode summary

        Skips writing if no steps have been recorded this episode.
        """
        if not self.enabled:
            return
        steps = self._metrics_collector.get_step_data()
        if not steps:
            self.log('No step data to save — skipping.')
            return

        os.makedirs(self._episode_dir, exist_ok=True)

        # metadata.json
        metadata = self.get_simulator_start_metrics()
        metadata['episode_dir'] = self._episode_dir
        metadata['config'] = _serialize(self._config_snapshot)
        self._write_json(os.path.join(self._episode_dir, 'metadata.json'), metadata)

        # step_history.json
        self._write_json(
            os.path.join(self._episode_dir, 'step_history.json'),
            _serialize(steps),
        )

        # episode_metrics.json
        end_metrics = self.get_simulator_end_metrics()
        self._write_json(
            os.path.join(self._episode_dir, 'episode_metrics.json'),
            end_metrics,
        )

        self.log(f'Episode {self.episode_id} saved → {self._episode_dir}')

    def reset(self) -> None:
        """Save the current episode and prepare a new episode directory.

        Called by UAMSimulator.reset() at the start of each episode.
        Persists whatever data was collected in the previous episode before
        clearing the internal step buffer.
        """
        self.save()
        self._metrics_collector.reset()
        self.episode_id += 1
        self._init_episode_dir()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _init_episode_dir(self) -> None:
        """Create (but do not populate) the directory for the current episode."""
        ts = datetime.now().strftime('%Y_%m_%d_%H%M%S')
        dir_name = f'episode_{self.episode_id}_{ts}'
        self._episode_dir = os.path.join(self.log_dir, dir_name)

    @staticmethod
    def _write_json(filepath: str, data: Any) -> None:
        """Serialize data to a JSON file with 2-space indentation.

        Args:
            filepath: Absolute or relative path to the output file.
            data:     JSON-serializable Python object.
        """
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
