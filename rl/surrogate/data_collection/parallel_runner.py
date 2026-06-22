"""
parallel_runner.py
===================
Run multiple UrbanNav simulator instances in parallel for surrogate model
data collection.

Each worker runs the simulator with a distinct config (from sweep_config),
logging episode data via the existing Logger.  Rendering is fully decoupled:
completed episode log directories are pushed onto a queue consumed by a
dedicated render process, so simulator throughput is never blocked by
matplotlib I/O.

CLI::

    python -m rl.surrogate.data_collection.parallel_runner \\
        --base-config sample_config.yaml \\
        --num-workers 4 \\
        --output-dir logs/sweep_001 \\
        --sweep "fleet_composition.0.count=[3,5,10]" \\
        --sweep "simulator.seed=[42,123]" \\
        --render
"""

from __future__ import annotations

import argparse
import json
import os
import tempfile
import traceback
from multiprocessing import Process, Queue
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from rl.surrogate.data_collection.sweep_config import generate_sweep, _parse_sweep_arg


_SENTINEL = "__DONE__"


def _run_episode(config_dict: Dict[str, Any], episode_dir: str) -> str:
    """Run a single simulator episode and return the log directory path.

    Executed inside a worker process — imports are local to avoid issues
    with forked matplotlib backends.
    """
    import matplotlib
    matplotlib.use("Agg")

    config_dict = _prepare_config(config_dict, episode_dir)

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False
    ) as tmp:
        yaml.dump(config_dict, tmp, default_flow_style=False)
        tmp_path = tmp.name

    try:
        from urbannav.uam_simulator import UAMSimulator

        sim = UAMSimulator(config_path=tmp_path)
        sim.reset()
        for _ in range(sim.total_timestep):
            sim.step({})
        sim.logger.save()
        return sim.logger._episode_dir
    finally:
        os.unlink(tmp_path)


def _prepare_config(config_dict: Dict[str, Any], episode_dir: str) -> Dict[str, Any]:
    """Override logging/rendering settings for headless batch collection."""
    cfg = config_dict.copy()
    cfg["logging"] = {
        "enabled": True,
        "log_dir": episode_dir,
    }
    cfg["rendering"] = {
        "enabled": False,
        "mode": "offline",
        "output_dir": os.path.join(episode_dir, "renders"),
        "output_filename": "episode",
        "realtime_sleep": 0.01,
        "frame_skip": 4,
    }
    return cfg


def _worker(
    run_id: str,
    config_dict: Dict[str, Any],
    output_dir: str,
    render_queue: Optional[Queue],
) -> None:
    """Worker target: run one episode, optionally enqueue for rendering."""
    episode_dir = os.path.join(output_dir, f"run_{run_id}")
    os.makedirs(episode_dir, exist_ok=True)

    try:
        log_dir = _run_episode(config_dict, episode_dir)
        print(f"[worker {run_id}] Episode complete → {log_dir}")

        config_snapshot_path = os.path.join(episode_dir, "sweep_config.json")
        with open(config_snapshot_path, "w") as f:
            json.dump(config_dict, f, indent=2, default=str)

        if render_queue is not None:
            render_queue.put((run_id, log_dir, config_dict))

    except Exception:
        print(f"[worker {run_id}] FAILED:")
        traceback.print_exc()


def _render_worker(render_queue: Queue, output_dir: str) -> None:
    """Dedicated render process: consume log directories and produce MP4s.

    Reads step_history.json from each completed episode and renders an
    offline MP4 animation.  Runs independently so simulator workers are
    never blocked.
    """
    import matplotlib
    matplotlib.use("Agg")

    while True:
        item = render_queue.get()
        if item == _SENTINEL:
            break

        run_id, log_dir, config_dict = item
        try:
            _render_from_logs(run_id, log_dir, config_dict, output_dir)
        except Exception:
            print(f"[render {run_id}] FAILED:")
            traceback.print_exc()


def _render_from_logs(
    run_id: str,
    log_dir: str,
    config_dict: Dict[str, Any],
    output_dir: str,
) -> None:
    """Reconstruct renderer frames from step_history.json and save MP4."""
    from urbannav.component_schema import RenderingConfig, UAMConfig, UAV_TYPE_REGISTRY

    step_history_path = os.path.join(log_dir, "step_history.json")
    if not os.path.exists(step_history_path):
        print(f"[render {run_id}] No step_history.json in {log_dir}, skipping.")
        return

    with open(step_history_path, "r") as f:
        steps = json.load(f)

    if not steps:
        print(f"[render {run_id}] Empty step history, skipping.")
        return

    fleet = config_dict.get("fleet_composition", [])
    type_name = fleet[0].get("type_name", "STANDARD") if fleet else "STANDARD"
    type_cfg = UAV_TYPE_REGISTRY.get(type_name)
    default_radius = getattr(type_cfg, "radius", 5.0) if type_cfg else 5.0
    default_nmac = getattr(type_cfg, "nmac_radius", 50.0) if type_cfg else 50.0
    default_detect = getattr(type_cfg, "detection_radius", 200.0) if type_cfg else 200.0

    frames = []
    for step_record in steps:
        uavs = {}
        for uid_str, snap in step_record.get("uavs", {}).items():
            uid = int(uid_str)
            uavs[uid] = {
                "x": snap["x"],
                "y": snap["y"],
                "z": snap.get("z", 0.0),
                "heading": snap.get("heading", 0.0),
                "radius": default_radius,
                "nmac_radius": default_nmac,
                "detection_radius": default_detect,
                "mission_start": None,
                "mission_end": None,
                "mission_start_z": 0.0,
                "mission_end_z": 0.0,
            }
        frames.append({"step": step_record["step"], "uavs": uavs})

    render_dir = os.path.join(output_dir, "renders")
    os.makedirs(render_dir, exist_ok=True)

    render_cfg = RenderingConfig(
        enabled=True,
        mode="offline",
        output_dir=render_dir,
        output_filename=f"run_{run_id}",
        frame_skip=0,
    )

    from urbannav.renderer import Renderer

    sim_mode = config_dict.get("simulator", {}).get("mode", "2D")
    renderer = Renderer(config=render_cfg, simulator_mode=sim_mode)
    renderer._frames = frames
    renderer.save(episode_id=0)
    print(f"[render {run_id}] Saved render → {render_dir}")


def run_sweep(
    base_config_path: str,
    sweep_params: Dict[str, List[Any]],
    output_dir: str,
    num_workers: int = 4,
    render: bool = False,
) -> List[str]:
    """Run a full parameter sweep with parallel simulator instances.

    Args:
        base_config_path: Path to template YAML config.
        sweep_params: Parameter grid (see sweep_config.generate_sweep).
        output_dir: Root directory for all run outputs.
        num_workers: Max simultaneous simulator processes.
        render: If True, start a dedicated render process for MP4 output.

    Returns:
        List of completed episode log directory paths.
    """
    configs = generate_sweep(base_config_path, sweep_params)
    print(f"Sweep: {len(configs)} configurations, {num_workers} workers")

    os.makedirs(output_dir, exist_ok=True)

    render_queue: Optional[Queue] = None
    render_proc: Optional[Process] = None
    if render:
        render_queue = Queue()
        render_proc = Process(
            target=_render_worker,
            args=(render_queue, output_dir),
            daemon=True,
        )
        render_proc.start()

    active: List[Process] = []
    completed_dirs: List[str] = []

    for run_id, config_dict in configs:
        while len(active) >= num_workers:
            active = [p for p in active if p.is_alive()]
            if len(active) >= num_workers:
                active[0].join(timeout=1.0)

        p = Process(
            target=_worker,
            args=(run_id, config_dict, output_dir, render_queue),
        )
        p.start()
        active.append(p)
        completed_dirs.append(os.path.join(output_dir, f"run_{run_id}"))

    for p in active:
        p.join()

    if render_queue is not None and render_proc is not None:
        render_queue.put(_SENTINEL)
        render_proc.join()

    print(f"Sweep complete. {len(completed_dirs)} runs in {output_dir}")
    return completed_dirs


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run parallel simulator instances for surrogate data collection"
    )
    parser.add_argument("--base-config", required=True, help="Path to base YAML config")
    parser.add_argument("--num-workers", type=int, default=4, help="Max parallel workers")
    parser.add_argument("--output-dir", required=True, help="Root output directory")
    parser.add_argument(
        "--sweep",
        action="append",
        required=True,
        help='Sweep param: key=[v1,v2,...]. Repeat for multiple params.',
    )
    parser.add_argument("--render", action="store_true", help="Enable background MP4 rendering")
    args = parser.parse_args()

    sweep_params: Dict[str, List[Any]] = {}
    for s in args.sweep:
        key, vals = _parse_sweep_arg(s)
        sweep_params[key] = vals

    run_sweep(
        base_config_path=args.base_config,
        sweep_params=sweep_params,
        output_dir=args.output_dir,
        num_workers=args.num_workers,
        render=args.render,
    )


if __name__ == "__main__":
    main()
