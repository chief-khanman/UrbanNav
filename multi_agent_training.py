from uam_simulator import UAMSimulator
from multi_agent_gym_env import UAMMultiAgentEnv

# ---------------------------------------------------------------------------
# Create simulator and wrap in multi-agent env
# ---------------------------------------------------------------------------
# Requires a config yaml with at least one LEARNING fleet entry.
# Set count > 1 on that entry to control multiple agents, e.g.:
#
#   fleet_composition:
#     - type_name: LEARNING
#       count: 3
#       dynamics: PointMass
#       controller: null
#       sensor: PartialSensor
#       planner: N/A
#       mode: TRAIN
#     - type_name: STANDARD
#       count: 5
#       dynamics: PointMass
#       controller: PIDPointMassController
#       sensor: PartialSensor
#       planner: PointMass-PID
# ---------------------------------------------------------------------------

sim = UAMSimulator(config_path='sample_config.yaml')

env = UAMMultiAgentEnv(
    simulator=sim,
    obs_type='AGENT-N-INTRUDER',
    n_intruder=3,
    reward_type='r1r2r3',
)

print('Possible agents:  ', env.possible_agents)
print('Number of agents: ', env._n_agents)
print('Observation space:', env.observation_space(env.possible_agents[0]))
print('Action space:     ', env.action_space(env.possible_agents[0]))
print()

# ---------------------------------------------------------------------------
# Random rollout — verifies the API contract before training
# ---------------------------------------------------------------------------
observations, infos = env.reset()

print('Initial observations:')
for agent, obs in observations.items():
    print(f'  {agent}: shape={obs.shape}')
print()

for step_idx in range(10):
    if not env.agents:
        print('All agents done — ending early.')
        break

    actions = {agent: env.action_space(agent).sample() for agent in env.agents}
    observations, rewards, terminations, truncations, infos = env.step(actions)

    print(f'Step {step_idx + 1}:')
    for agent in sorted(rewards):
        print(
            f'  {agent} | reward={rewards[agent]:.3f} '
            f'terminated={terminations[agent]} truncated={truncations[agent]}'
        )
    print(f'  Active agents after step: {env.agents}')
    print()


# ---------------------------------------------------------------------------
# MARL training — uncomment and adapt once a MARL library is installed
# ---------------------------------------------------------------------------

# --- Option A: Independent PPO via SuperSuit + Stable-Baselines3 ---
# Each agent trains its own PPO policy; no parameter sharing.
#
# from supersuit import pettingzoo_env_to_vec_env_v1, concat_vec_envs_v1
# from stable_baselines3 import PPO
#
# vec_env = pettingzoo_env_to_vec_env_v1(env)
# vec_env = concat_vec_envs_v1(vec_env, num_vec_envs=1, num_cpus=1, base_class='stable_baselines3')
# model = PPO('MlpPolicy', vec_env, verbose=1)
# model.learn(total_timesteps=500_000)
# model.save('ippo_uam_multi_agent')

# --- Option B: RLlib (IPPO or MAPPO) ---
# from ray.rllib.env.wrappers.pettingzoo_env import ParallelPettingZooEnv
# import ray
# from ray import tune
#
# ray.init()
# tune.register_env('uam_marl', lambda cfg: ParallelPettingZooEnv(
#     UAMMultiAgentEnv(UAMSimulator('sample_config.yaml'), obs_type='AGENT-N-INTRUDER')
# ))
# tune.run(
#     'PPO',
#     config={
#         'env': 'uam_marl',
#         'num_agents': env._n_agents,
#         'framework': 'torch',
#     }
# )
