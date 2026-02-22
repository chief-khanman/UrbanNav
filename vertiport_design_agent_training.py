from stable_baselines3 import PPO
from vertiport_design_env import VertiportDesignEnv

config_path = 'Path/to/config'
env = VertiportDesignEnv(config_path=config_path, simulator_timesteps=12)

model = PPO('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=1000)

model.save('trained_vertiport_design_agent')