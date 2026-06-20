import numpy as np
import pytest
from pydantic import ValidationError

from urbannav.uam_simulator import UAMSimulator
from urbannav.component_schema import UAMConfig, UAVFleetInstanceConfig, validate_fleet_composition

from rl.multi_agent.multi_agent_gym_env import UAMMultiAgentEnv

try:
    from pettingzoo.test import parallel_api_test
    HAVE_PETTINGZOO_TEST = True
except ImportError:
    HAVE_PETTINGZOO_TEST = False


class TestPettingZooConformance:
    @pytest.mark.skipif(not HAVE_PETTINGZOO_TEST, reason="pettingzoo.test not installed")
    def test_parallel_api_conformance(self, multi_agent_config_path):
        sim = UAMSimulator(config_path=multi_agent_config_path)
        env = UAMMultiAgentEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        parallel_api_test(env, num_cycles=20)


class TestReset:
    def test_reset_returns_obs_and_info_for_all_agents(self, multi_agent_config_path):
        sim = UAMSimulator(config_path=multi_agent_config_path)
        env = UAMMultiAgentEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        obs, infos = env.reset()
        assert set(obs.keys()) == set(env.possible_agents)
        assert set(infos.keys()) == set(env.possible_agents)
        for agent_obs in obs.values():
            assert agent_obs.shape == env.observation_space(env.possible_agents[0]).shape

    def test_agents_grouped_by_shared_policy_id(self, multi_agent_config_path):
        sim = UAMSimulator(config_path=multi_agent_config_path)
        env = UAMMultiAgentEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        env.reset()
        assert len(env.possible_agents) == 2
        assert set(env.agent_policy_id.values()) == {'shared_policy'}


class TestPolicyIdValidation:
    def test_policy_id_required_for_multi_agent_learning(self):
        with pytest.raises(ValidationError, match="policy_id"):
            UAVFleetInstanceConfig(
                type_name='MULTI_AGENT_LEARNING', count=1, dynamics='PointMass',
                controller='RL', sensor='PartialSensor', planner='PointMass-PID',
                mode='TRAIN',
            )

    def test_policy_id_forbidden_for_non_learning_type(self):
        with pytest.raises(ValidationError, match="policy_id"):
            UAVFleetInstanceConfig(
                type_name='STANDARD', count=1, dynamics='PointMass',
                controller='PIDPointMassController', sensor='PartialSensor',
                planner='PointMass-PID', policy_id='oops',
            )

    def test_shared_policy_id_requires_identical_wiring(self):
        config = UAMConfig(
            simulator={'dt': 1.0, 'total_timestep': 10, 'mode': '3D', 'seed': 1},
            vertiport={'number_of_landing_pad': 2},
            airspace={
                'location_name': 'Austin, Texas, USA', 'number_of_vertiports': 5,
                'vertiport_tag_list': [], 'airspace_restricted_area_tag_list': [],
            },
            fleet_composition=[
                {'type_name': 'MULTI_AGENT_LEARNING', 'count': 1, 'dynamics': 'PointMass',
                 'controller': 'RL', 'sensor': 'PartialSensor', 'planner': 'PointMass-PID',
                 'mode': 'TRAIN', 'policy_id': 'shared'},
                {'type_name': 'MULTI_AGENT_LEARNING', 'count': 1, 'dynamics': 'SixDOF',
                 'controller': 'RL', 'sensor': 'PartialSensor', 'planner': 'SixDOF-PID',
                 'mode': 'TRAIN', 'policy_id': 'shared'},
            ],
        )
        with pytest.raises(ValueError, match="identical"):
            validate_fleet_composition(config)


class TestPolicyIsolation:
    def test_two_independent_policy_groups_have_isolated_obs_and_reward(
        self, multi_policy_config_path
    ):
        sim = UAMSimulator(config_path=multi_policy_config_path)
        env = UAMMultiAgentEnv(simulator=sim, obs_type='AGENT', n_intruder=1)
        assert set(env.agent_policy_id.values()) == {'policy_a', 'policy_b'}

        obs, infos = env.reset()
        agents = list(env.agents)
        assert len(agents) == 2
        # obs arrays must be independent objects, not aliased across agents
        assert obs[agents[0]] is not obs[agents[1]]

        actions = {a: env.action_space(a).sample() for a in agents}
        obs, rewards, terminations, truncations, infos = env.step(actions)

        assert set(rewards.keys()) == set(agents)
        assert isinstance(rewards[agents[0]], float)
        assert isinstance(rewards[agents[1]], float)
        assert obs[agents[0]].shape == env.observation_space(agents[0]).shape
        assert obs[agents[1]].shape == env.observation_space(agents[1]).shape
