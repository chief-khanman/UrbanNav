"""deployment.py — Quick test runner for the UAM simulator, metrics logger, and renderer.

Run from the UrbanNav directory:
    python deployment.py

Rendering is controlled entirely by sample_config.yaml:
    rendering.enabled      — true/false
    rendering.mode         — 'offline', 'realtime', or 'both'
    rendering.frame_skip   — render every N+1 steps (reduces animation size)
    rendering.output_dir   — where to write GIF/MP4 files
"""
#TODO: general TODO list and directives to follow when updating/editing code 
# 1. remove [BAND2-DIRECT/INDIRECT] from docstring 
# 2. do not change/update/edit parts of code that are not relevant - helps with diffing and understanding changes easily
#  

import os

from uam_simulator_vp_design import UAMSimulatorVPDesign

CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'sample_config.yaml')


#TODO: design choice selection - what would be the best place to load the OD matrix
# CONFIG OR path argument - keep in mind this is for Vertiport design problem 
#TODO: the OD matrix 
#band 1 output for OD matrix - there are two available OD matrix outputs - od_lambda_matrix.csv OR .npy -
#place code for accepting OD matrix - this is a one time entry of data into simulator. 
#Even for Vertiport design RL problem - the OD matrix will be loaded once. 

def main() -> None:
    sim = UAMSimulatorVPDesign(config_path=CONFIG_FILE)
    sim.reset()

    rendering_on = sim.renderer.enabled
    print(f'\nRunning {sim.total_timestep} steps '
          f'(logging: {sim.logger.enabled} | '
          f'rendering: {rendering_on}'
          + (f', mode={sim.renderer.mode}' if rendering_on else '')
          + ')\n')

    for step in range(sim.total_timestep):
        sim.step({})   # empty command bundle — all UAVs use internal controllers

        if step % 100 == 0:
            num_uavs = len(sim.simulator_manager.atc.uav_dict)
            print(f'  step {step:4d}/{sim.total_timestep} | active UAVs: {num_uavs}')

    # --- save metrics ---
    #TODO: add sim_manager.get_episode_metrics 
    episode_metrics = sim.simulator_manager.get_episode_metrics()
    #TODO: need to save metrics during test procedure to ensure all outputs are correct
    # later, this variable will be passed to the GNN-RL module for VP design problem 
    
    sim.logger.save()

    metrics = sim.logger.get_simulator_end_metrics()
    print('\n--- Episode Metrics ---')
    for key, value in metrics.items():
        print(f'  {key}: {value}')

    # --- save offline animation (no-op if rendering disabled or mode=realtime) ---
    sim.render()


if __name__ == '__main__':
    main()
