from map_env_revised import MapEnv

env = MapEnv(3,5, obs_space_str='UAM_UAV')
obs, info = env.reset()
# print('Reset env')
# print(f'obs: {obs}')
# print(f'info: {info}')


# print(env.observation_space.shape)
# print(env.action_space.shape)

# print(env.observation_space.sample())
# print(env.action_space.sample())


for _ in range(10000):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info =  env.step(action)


