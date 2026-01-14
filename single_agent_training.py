# train_rl_controller.py
from stable_baselines3 import PPO
from simulator import UAMSimulator, UAMSimEnv

# Create simulator
sim = UAMSimulator(config={
    'dt': 0.1,
    'n_uavs': 4,
    'mode': 'training'
})

# Wrap in Gym environment
env = UAMSimEnv(
    simulator=sim,
    controlled_uav_ids=['uav_0', 'uav_1', 'uav_2', 'uav_3'],
    observation_config={'state_keys': ['uav_states', 'nearby_uavs']},
    action_config={'action_dim': 2}
)

# Train RL policy
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)

# Save policy
model.save("trained_rl_controller")
