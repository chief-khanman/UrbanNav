# run_simulation.py
from simulator import UAMSimulator, RLController, LQRController, ATCController
from simulator import ControllerManifest, ExecutionMode

# Create SIMULATOR
sim = UAMSimulator(config={
    'dt': 0.1,
    'n_uavs': 50,
    'mode': 'deployment'
})

# Load trained RL controller
from stable_baselines3 import PPO
rl_policy = PPO.load("trained_rl_controller")

rl_controller = RLController(
    manifest=ControllerManifest(
        controller_id='rl_agent_1',
        controlled_uav_ids=['uav_0', 'uav_1', 'uav_2', 'uav_3'],
        required_state_keys=['uav_states', 'nearby_uavs'],
        output_type='action',
        execution_mode='inline'
    ),
    policy=rl_policy
)

# Create LQR controller
lqr_controller = LQRController(
    manifest=ControllerManifest(
        controller_id='lqr_ctrl',
        controlled_uav_ids=[f'uav_{i}' for i in range(4, 29)],  # 25 UAVs
        required_state_keys=['uav_states'],
        output_type='action',
        execution_mode='inline'
    ),
    Q=np.eye(6),
    R=np.eye(2)
)

# Create ATC
atc = ATCController(
    manifest=ControllerManifest(
        controller_id='atc',
        controlled_uav_ids=[f'uav_{i}' for i in range(50)],  # All UAVs
        required_state_keys=['uav_states', 'airspace'],
        output_type='directive',
        execution_mode='process'  # Run in separate process
    )
)

# Register controllers
sim.register_controller(rl_controller, ExecutionMode.INLINE)
sim.register_controller(lqr_controller, ExecutionMode.INLINE)
sim.register_controller(atc, ExecutionMode.PROCESS)

# Note: External RRT controller connects via ZeroMQ automatically
# Just run: python external_rrt_controller.py

# Run simulation
for step in range(1000):
    state, done, info = sim.step()
    
    if done:
        break
    
    if step % 10 == 0:
        print(f"Step {step}: {len(info['collisions'])} collisions")

# Save final state
sim.save_state('final_state.json')
