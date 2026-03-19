# train_rl_controller.py
from stable_baselines3 import PPO
from uam_simulator import UAMSimulator
from single_agent_gym_env import UAMSimEnv
from stable_baselines3.common.env_checker import check_env

# Create simulator
sim = UAMSimulator(config_path='sample_config.yaml')

# # Wrap in Gym environment
env = UAMSimEnv(
    simulator=sim,
    obs_type='AGENT-INTRUDER',
    n_intruder = 1 
)

print('Observation space sample: ')
print(env.observation_space.sample(), '\n')
print('Observation space shape: ')
print(env.observation_space.sample().shape, '\n')

print('Action space sample: ')
print(env.action_space.sample(), '\n')
print('Action space shape: ')
print(env.action_space.sample().shape, '\n')

print(env.reset(seed=123))
# # Train RL policy
# model = PPO("MlpPolicy", env, verbose=1)
# model.learn(total_timesteps=100000)

# # Save policy
# model.save("trained_rl_controller")
