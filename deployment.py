# run_simulation.py
import os
import numpy as np
from uam_simulator import UAMSimulator


# config_file
config_file = './sample_config.yaml'

# [PHASE 1] Create SIMULATOR
print("[PHASE 1] Creating UAMSimulator...")
sim = UAMSimulator(config_path=config_file)
print("[PHASE 1] UAMSimulator created.")

# [PHASE 2] Reset - builds airspace, ATC, components, UAVs
print("[PHASE 2] Resetting simulator (builds airspace, ATC, UAVs)...")
sim.reset()
uav_count = len(sim.simulator_manager.atc.uav_dict)
vp_count = len(sim.simulator_manager.airspace.vertiport_list)
print(f"[PHASE 2] Reset complete. UAVs: {uav_count}, Vertiports: {vp_count}")

for vertiport in sim.simulator_manager.airspace.vertiport_list:
    print(f'Vertiport id: {vertiport.id} UAV ids: {vertiport.uav_id_list}')

# [PHASE 3] Step loop
print(f"[PHASE 3] Starting step loop for {sim.total_timestep} steps...")
for step in range(sim.total_timestep):
    sim.step({})

    if step % 1000 == 0:
        uav_count = len(sim.simulator_manager.atc.uav_dict)
        print(f"  Step {step:>5}: UAVs active = {uav_count}")

print("[PHASE 3] Simulation complete.")
