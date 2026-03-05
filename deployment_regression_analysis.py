"""deployment_regression_analysis.py

Step-time and memory regression for SimulatorManager.step().

Sweep : UAV count 500 → 10 000 in steps of 500 (20 data points).
        2 000 steps per run.  Vertiports = n_uavs + 1.
        Logging and rendering disabled.

Measures (step loop only — reset / init excluded):
  - Total wall-clock time (s)
  - Peak Python-level allocation (KB) via tracemalloc

Outputs (written next to this file):
  regression_results.csv   — raw data table
  regression_analysis.png  — two-panel plot
      · Time panel overlays an n log n reference scaled to the first point
      · Memory panel shows peak KB vs n

Run from the UrbanNav directory:
    python deployment_regression_analysis.py
"""

import csv
import os
import sys
import time
import tracemalloc

import matplotlib
matplotlib.use('Agg')          # no display required
import matplotlib.pyplot as plt
import numpy as np

# ── make UrbanNav importable regardless of cwd ──────────────────────────────
_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _DIR)

from component_schema import (
    AirspaceConfig,
    LoggingConfig,
    RenderingConfig,
    UAMConfig,
    UAMSimulatorConfig,
    UAVFleetInstanceConfig,
    VertiportConfig,
)
from simulator_manager import SimulatorManager

# ── sweep parameters ─────────────────────────────────────────────────────────
UAV_COUNTS = list(range(500, 10_001, 500))   # [500, 1000, …, 10000]
N_STEPS    = 2_000
SEED       = 42


# ── helpers ──────────────────────────────────────────────────────────────────

def _build_config(n_uavs: int) -> UAMConfig:
    """Build a minimal UAMConfig for n_uavs UAVs with n_uavs+1 vertiports.

    Empty tag lists → vertiports are sampled randomly inside Austin's city
    boundary (no per-run OSM feature queries; only the boundary geocode is
    needed, which osmnx caches to disk after the first run).
    """
    return UAMConfig(
        simulator=UAMSimulatorConfig(
            dt=1.0,
            total_timestep=N_STEPS,
            mode='2D',
            seed=SEED,
        ),
        vertiport=VertiportConfig(number_of_landing_pad=4),
        airspace=AirspaceConfig(
            location_name='Austin, Texas, USA',
            number_of_vertiports=n_uavs + 1,
            vertiport_tag_list=[],                  # random placement — no building query
            airspace_restricted_area_tag_list=[],   # no restricted areas in perf test
        ),
        fleet_composition=[UAVFleetInstanceConfig(
            type_name='STANDARD',
            count=n_uavs,
            dynamics='PointMass',
            controller='PIDPointMassController',
            sensor='PartialSensor',
            planner='PointMass-PID',
        )],
        logging=LoggingConfig(enabled=False),
        rendering=RenderingConfig(enabled=False),
    )


def _benchmark(n_uavs: int) -> tuple[float, float]:
    """Time the step loop and measure the initialized simulator's memory footprint.

    Memory is sampled once after reset() — it captures the steady-state footprint
    of all UAV objects, vertiports, and data structures for n_uavs UAVs.
    tracemalloc is stopped before the step loop so it adds zero timing overhead.

    Returns:
        (step_loop_seconds, peak_kb)
    """
    config = _build_config(n_uavs)
    sm = SimulatorManager(config)

    # --- memory: snapshot the initialized simulator state ---
    tracemalloc.start()
    sm.reset()
    # current_bytes = live footprint after init (what we care about for scaling).
    # peak_bytes would include transient allocations made during reset that have
    # already been freed, which overstates the steady-state memory.
    current_bytes, _ = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    # --- time: pure step loop with no tracemalloc overhead ---
    t0 = time.perf_counter()
    for _ in range(N_STEPS):
        sm.step({})
    elapsed = time.perf_counter() - t0

    return elapsed, current_bytes / 1024.0    # seconds, KB


def _append_csv_row(n_uavs: int, t: float, m: float) -> None:
    """Append one result row immediately after each benchmark run."""
    path = os.path.join(_DIR, 'regression_results.csv')
    write_header = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(['n_uavs', 'step_loop_time_s', 'live_mem_kb'])
        writer.writerow([n_uavs, round(t, 4), round(m, 2)])


def _save_csv(uav_counts: list, times: list, mems: list) -> None:
    """Overwrite CSV with complete results at the end of a full sweep."""
    path = os.path.join(_DIR, 'regression_results.csv')
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['n_uavs', 'step_loop_time_s', 'live_mem_kb'])
        for n, t, m in zip(uav_counts, times, mems):
            writer.writerow([n, round(t, 4), round(m, 2)])
    print(f'[Regression] Data  → {path}')


def _plot(uav_counts: list, times: list, mems: list) -> None:
    ns  = np.asarray(uav_counts, dtype=float)
    ref = ns * np.log(ns)
    ref = ref * (times[0] / ref[0])    # scale so reference passes through first point

    fig, (ax_t, ax_m) = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle(
        f'SimulatorManager.step() — scaling analysis  '
        f'({N_STEPS} steps / run, logging & rendering OFF)',
        fontsize=12,
    )

    # ── time ─────────────────────────────────────────────────────────────────
    ax_t.plot(ns, times, 'o-', color='steelblue', lw=2, ms=6, label='measured')
    ax_t.plot(ns, ref,   '--', color='tomato',    lw=1.5,     label=r'$n\,\log n$ (scaled)')
    ax_t.set_xlabel('UAV count  (n)', fontsize=11)
    ax_t.set_ylabel('Total step-loop wall time  (s)', fontsize=11)
    ax_t.set_title('Time vs UAV count')
    ax_t.legend(fontsize=10)
    ax_t.grid(True, alpha=0.35)
    ax_t.set_xlim(left=0)
    ax_t.set_ylim(bottom=0)

    # ── memory ───────────────────────────────────────────────────────────────
    ax_m.plot(ns, mems, 's-', color='darkorange', lw=2, ms=6)
    ax_m.set_xlabel('UAV count  (n)', fontsize=11)
    ax_m.set_ylabel('Live memory footprint after reset  (KB)', fontsize=11)
    ax_m.set_title('Memory vs UAV count')
    ax_m.grid(True, alpha=0.35)
    ax_m.set_xlim(left=0)
    ax_m.set_ylim(bottom=0)

    plt.tight_layout()
    path = os.path.join(_DIR, 'regression_analysis.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'[Regression] Plot  → {path}')


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    print(
        f'UAV regression sweep: n = {UAV_COUNTS[0]} … {UAV_COUNTS[-1]}  '
        f'(step={UAV_COUNTS[1]-UAV_COUNTS[0]}),  {N_STEPS} steps / run\n'
    )

    times: list[float] = []
    mems:  list[float] = []

    for i, n in enumerate(UAV_COUNTS, 1):
        print(f'  [{i:>2}/{len(UAV_COUNTS)}]  n={n:>5}  ', end='', flush=True)
        t, m = _benchmark(n)
        times.append(t)
        mems.append(m)
        print(f'time={t:7.2f}s   peak_mem={m:8.1f} KB')
        _append_csv_row(n, t, m)   # flush immediately — safe if process is killed

    # ── summary table ────────────────────────────────────────────────────────
    print('\n── Results ────────────────────────────────────────────────')
    print(f'  {"n":>7}   {"time (s)":>10}   {"live mem (KB)":>13}')
    print('  ' + '─' * 36)
    for n, t, m in zip(UAV_COUNTS, times, mems):
        print(f'  {n:>7}   {t:>10.3f}   {m:>13.1f}')

    _save_csv(UAV_COUNTS, times, mems)
    _plot(UAV_COUNTS, times, mems)


if __name__ == '__main__':
    main()
