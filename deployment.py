# run_simulation.py
import os
import numpy as np
from uam_simulator import UAMSimulator


#config_file
config_file = './sample_config.yaml'

# Create SIMULATOR - AerBus is now integrated into UAMSimulator
sim = UAMSimulator(config_path=config_file)



# for step in range(sim.total_timestep):
#     sim_state, done, info = sim.step() # what is the purpose of DONE ??
#     sim.metrics.log_step_metrics(sim_state, info)
    
#     if done:
#         break
    
#     if step % 10 == 0:
#         print(f"Step {step}: {len(info['collisions'])} collisions")
    
#     #rendering
#     if sim.render_each_step:
#         sim.renderer.render(sim_state)
#     else:
#         sim.renderer.log_render(sim_state, info)

# # Metrics
# sim.metrics.calculate_metrics(sim.metrics.log)
