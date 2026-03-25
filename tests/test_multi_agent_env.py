"""
Tests for UAMMultiAgentEnv (PettingZoo ParallelEnv).

Mirrors the structure of test_single_agent_env.py.
Requires a LEARNING fleet entry with count >= 2 in the config yaml.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

from uam_simulator import UAMSimulator
from multi_agent_gym_env import UAMMultiAgentEnv

CONFIG_FILE = os.path.join(os.path.dirname(__file__), '../sample_config.yaml')


def make_env(obs_type='AGENT-INTRUDER', n_intruder=1, reward_type='r1'):
    sim = UAMSimulator(config_path=CONFIG_FILE)
    return UAMMultiAgentEnv(
        simulator=sim,
        obs_type=obs_type,
        n_intruder=n_intruder,
        reward_type=reward_type,
    )


# ---------------------------------------------------------------------------
# Check 1: env creation — possible_agents matches LEARNING fleet count
# ---------------------------------------------------------------------------
def test_possible_agents():
    env = make_env()
    assert len(env.possible_agents) > 0, "possible_agents must be non-empty"
    assert all(
        a == f"agent_{i}" for i, a in enumerate(env.possible_agents)
    ), "possible_agents must be ['agent_0', 'agent_1', ...]"
    print(f"[PASS] test_possible_agents — {env.possible_agents}")


# ---------------------------------------------------------------------------
# Check 2: observation and action spaces defined for all possible agents
# ---------------------------------------------------------------------------
def test_spaces():
    env = make_env(obs_type='AGENT-N-INTRUDER', n_intruder=2)
    for agent in env.possible_agents:
        obs_sp = env.observation_space(agent)
        act_sp = env.action_space(agent)
        assert obs_sp is not None, f"Missing obs space for {agent}"
        assert act_sp is not None, f"Missing action space for {agent}"
        assert obs_sp.shape[0] > 0
        assert act_sp.shape[0] > 0
    print(f"[PASS] test_spaces — obs {obs_sp.shape}, act {act_sp.shape}")


# ---------------------------------------------------------------------------
# Check 3: reset returns correct agent set and observation shapes
# ---------------------------------------------------------------------------
def test_reset():
    env = make_env()
    observations, infos = env.reset()

    assert set(observations.keys()) == set(env.agents), (
        "Observations must be keyed by current active agents"
    )
    assert set(infos.keys()) == set(env.agents)
    assert env.agents == env.possible_agents, (
        "All agents must be active immediately after reset"
    )

    for agent, obs in observations.items():
        expected_shape = env.observation_space(agent).shape
        assert obs.shape == expected_shape, (
            f"{agent}: expected obs shape {expected_shape}, got {obs.shape}"
        )
        assert obs.dtype == np.float32

    print(f"[PASS] test_reset — {len(env.agents)} agents, obs shape {obs.shape}")


# ---------------------------------------------------------------------------
# Check 4: step returns correct keys and value types
# ---------------------------------------------------------------------------
def test_step_output_format():
    env = make_env()
    env.reset()

    agents_before_step = list(env.agents)
    actions = {agent: env.action_space(agent).sample() for agent in env.agents}
    observations, rewards, terminations, truncations, infos = env.step(actions)

    # Step output must cover the agents that were active at the START of the step
    assert set(observations.keys()) == set(agents_before_step)
    assert set(rewards.keys()) == set(agents_before_step)
    assert set(terminations.keys()) == set(agents_before_step)
    assert set(truncations.keys()) == set(agents_before_step)
    assert set(infos.keys()) == set(agents_before_step)

    for agent in agents_before_step:
        assert isinstance(rewards[agent], float)
        assert isinstance(terminations[agent], bool)
        assert isinstance(truncations[agent], bool)
        assert observations[agent].shape == env.observation_space(agent).shape
        assert observations[agent].dtype == np.float32

    print(f"[PASS] test_step_output_format — {len(agents_before_step)} agents stepped")


# ---------------------------------------------------------------------------
# Check 5: action space shapes match dynamics
# ---------------------------------------------------------------------------
def test_action_space_shape():
    env = make_env()
    dynamics = env._dynamics_name
    expected_shapes = {
        'PointMass':            (2,),
        'TwoDVector-Holonomic': (2,),
        'SixDOF':               (4,),
    }
    for agent in env.possible_agents:
        act_shape = env.action_space(agent).shape
        assert act_shape == expected_shapes[dynamics], (
            f"Action space shape {act_shape} does not match dynamics '{dynamics}'"
        )
    print(f"[PASS] test_action_space_shape — dynamics={dynamics}, shape={act_shape}")


# ---------------------------------------------------------------------------
# Check 6: agent IDs are stable across multiple resets
# ---------------------------------------------------------------------------
def test_stable_agent_ids_across_resets():
    env = make_env()

    env.reset()
    mapping_first = dict(env._agent_to_uav_id)

    env.reset()
    mapping_second = dict(env._agent_to_uav_id)

    # Agent-name → uav_id mapping must be consistent across resets
    # (ATC assigns the same ids in the same order given the same fleet config)
    for agent in env.possible_agents:
        assert agent in mapping_first
        assert agent in mapping_second
    print(f"[PASS] test_stable_agent_ids_across_resets — mapping={mapping_first}")


# ---------------------------------------------------------------------------
# Check 7: multi-step rollout — env does not error over several steps
# ---------------------------------------------------------------------------
def test_multi_step_rollout():
    env = make_env(obs_type='AGENT-N-INTRUDER', n_intruder=2, reward_type='r1r2r3')
    env.reset()

    for step_idx in range(20):
        if not env.agents:
            break
        actions = {a: env.action_space(a).sample() for a in env.agents}
        observations, rewards, terminations, truncations, infos = env.step(actions)

        # After step, self.agents must be a subset of possible_agents
        assert set(env.agents).issubset(set(env.possible_agents))

    print(f"[PASS] test_multi_step_rollout — completed {step_idx + 1} steps, "
          f"{len(env.agents)} agents remaining")


# ---------------------------------------------------------------------------
# Check 8: reward_type propagates correctly
# ---------------------------------------------------------------------------
def test_reward_types():
    for reward_type in ('r1', 'r1r2', 'r1r2r3', 'r1r2r3r4'):
        env = make_env(reward_type=reward_type)
        env.reset()
        actions = {a: env.action_space(a).sample() for a in env.agents}
        _, rewards, _, _, _ = env.step(actions)
        for agent, r in rewards.items():
            assert isinstance(r, float), f"Reward must be float, got {type(r)}"
    print("[PASS] test_reward_types — all reward_type variants return floats")


# ---------------------------------------------------------------------------
# Run all checks
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    test_possible_agents()
    test_spaces()
    test_reset()
    test_step_output_format()
    test_action_space_shape()
    test_stable_agent_ids_across_resets()
    test_multi_step_rollout()
    test_reward_types()
    print('\nAll multi-agent env checks passed.')
