# run_simulation.py
import os
import numpy as np
from uam_simulator import UAMSimulator
from controller import RLController, LQRController, ATCController
from controller import ControllerManifest
from aer_bus import ExecutionMode, AerBus

from stable_baselines3 import PPO

#config_file
config_file = './Path/to/config'

# Create SIMULATOR - AerBus is now integrated into UAMSimulator
sim = UAMSimulator(config_path=config_file)

# Load trained RL controller
rl_policy = PPO.load("./trained_rl_controller")

rl_controller = RLController(
    manifest=ControllerManifest(
        controller_id='rl_agent_1', # WHO
        controlled_state='uav', # WHAT 
        controlled_state_id=['uav_0', 'uav_1', 'uav_2', 'uav_3'], # WHAT
        #controlled_uav_ids=['uav_0', 'uav_1', 'uav_2', 'uav_3'],
        required_state_keys=['uav_states', 'nearby_uavs'], # WHAT - change this so that it contains controlled_state.attr, in this case uav.sensor
        output_type='action', # HOW - change this to have controller output command, and also how many commands/data is being sent, like acceleration+heading_change OR d_vx+d_vy, etc
        execution_mode='inline' # WHEN - at what time steps of the simulator should the simulator expect the outputs, every time step, some time step, etc
    ),
    policy=rl_policy
)

# Create LQR controller
lqr_controller = LQRController(
    manifest=ControllerManifest(
        controller_id='rl_agent_1',
        controlled_state='uav',
        controlled_state_id=['uav_0', 'uav_1', 'uav_2', 'uav_3'],
        #controlled_uav_ids=['uav_0', 'uav_1', 'uav_2', 'uav_3'],
        required_state_keys=['uav_states', 'nearby_uavs'],
        output_type='action',
        execution_mode='inline'
    ),
    Q=np.eye(6),
    R=np.eye(2)
)

# Create ATC
atc = ATCController(
    manifest=ControllerManifest(
        controller_id='rl_agent_1',
        controlled_state='uav',
        controlled_state_id=['uav_0', 'uav_1', 'uav_2', 'uav_3'],
        #controlled_uav_ids=['uav_0', 'uav_1', 'uav_2', 'uav_3'],
        required_state_keys=['uav_states', 'nearby_uavs'],
        output_type='action',
        execution_mode='inline'
    )
)

# vertiport_controller = ATCVertiportController(
#     manifest=ControllerManifest(
#         controller_id='atc_vertiport_controller',
#         controlled_state='atc',  # All UAVs
#         controlled_state_id=['atc'],
#         required_state_keys=['uav_states', 'airspace'],
#         output_type='directive',
#         execution_mode='process'  # Run in separate process
#     )
# )

# AER_BUS is initiated inside UAM_SIMULATOR, 
# Register controllers
sim.aer_bus.register_controller(rl_controller, ExecutionMode.INLINE)
sim.aer_bus.register_controller(lqr_controller, ExecutionMode.INLINE)
sim.aer_bus.register_controller(atc, ExecutionMode.PROCESS)

# Note: External RRT controller connects via ZeroMQ automatically
# Just run: python external_rrt_controller.py



for step in range(1000):
    sim_state, done, info = sim.step() # what is the purpose of DONE ??
    sim.metrics.log_step_metrics(sim_state, info)
    
    if done:
        break
    
    if step % 10 == 0:
        print(f"Step {step}: {len(info['collisions'])} collisions")
    
    #rendering
    if sim.render_each_step:
        sim.renderer.render(sim_state)
    else:
        sim.renderer.log_render(sim_state, info)

# Metrics
sim.metrics.calculate_metrics(sim.metrics.log)
