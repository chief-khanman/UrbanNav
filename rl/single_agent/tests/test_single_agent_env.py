import numpy as np
import pytest

from urbannav.uam_simulator import UAMSimulator
from urbannav.component_schema import RESERVED_TYPE_SINGLE_AGENT_LEARNING

from rl.single_agent.single_agent_gym_env import UAMSimEnv
from rl.common import agent_logic


class TestReset:
    def test_reset_returns_obs_and_info(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        obs, info = env.reset()
        assert isinstance(obs, np.ndarray)
        assert isinstance(info, dict)

    def test_reset_discovers_correct_learning_uav_id(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        env.reset()
        uav = sim.simulator_manager.atc.uav_dict[env._learning_uav_id]
        assert uav.type_name == RESERVED_TYPE_SINGLE_AGENT_LEARNING

    def test_learning_uav_has_correct_dynamics(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        env.reset()
        assert env._dynamics_name == 'PointMass'

    def test_other_uavs_instantiated_correctly(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        env.reset()
        # fixture fleet: 1 STANDARD + 1 SINGLE_AGENT_LEARNING
        assert len(sim.simulator_manager.atc.uav_dict) == 2
        non_learning = [
            uav for uav in sim.simulator_manager.atc.uav_dict.values()
            if uav.type_name != RESERVED_TYPE_SINGLE_AGENT_LEARNING
        ]
        assert len(non_learning) == 1
        assert non_learning[0].type_name == 'STANDARD'


class TestSpaces:
    @pytest.mark.parametrize("obs_type,n_intruder,expected_shape", [
        ('AGENT', 3, (9,)),
        ('AGENT-INTRUDER', 1, (15,)),
        ('AGENT-N-INTRUDER', 2, (21,)),
        ('AGENT-RA', 3, (10,)),
        ('AGENT-INTRUDER-RA', 1, (16,)),
        ('AGENT-N-INTRUDER-RA', 2, (22,)),
    ])
    def test_observation_space_shape(
        self, single_agent_config_path, obs_type, n_intruder, expected_shape
    ):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type=obs_type, n_intruder=n_intruder)
        assert env.observation_space.shape == expected_shape

    def test_action_space_is_normalized_box_for_point_mass(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT', n_intruder=1)
        assert env.action_space.shape == (2,)
        assert (env.action_space.low == -1.0).all()
        assert (env.action_space.high == 1.0).all()

    def test_reset_observation_matches_declared_space(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        obs, _ = env.reset()
        assert obs.shape == env.observation_space.shape
        assert obs.dtype == np.float32


class TestActionConversion:
    def test_format_action_scales_to_physical_units_for_point_mass(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT', n_intruder=1)
        env.reset()
        uav = sim.simulator_manager.atc.uav_dict[env._learning_uav_id]

        action = np.array([1.0, -1.0], dtype=np.float32)
        cmd_bundle = agent_logic.format_action(
            action, env._learning_uav_id, env._dynamics_name,
            sim.simulator_manager.atc.uav_dict,
        )
        payload = cmd_bundle[env._learning_uav_id][0].payload
        assert np.isclose(payload[0], uav.max_acceleration)
        assert np.isclose(payload[1], -uav.max_heading_change)

    def test_format_action_rejects_unknown_dynamics(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT', n_intruder=1)
        env.reset()
        with pytest.raises(ValueError):
            agent_logic.format_action(
                np.array([0.0, 0.0], dtype=np.float32), env._learning_uav_id,
                'NotARealDynamics', sim.simulator_manager.atc.uav_dict,
            )


class TestStep:
    def test_step_returns_expected_types(self, single_agent_config_path):
        sim = UAMSimulator(config_path=single_agent_config_path)
        env = UAMSimEnv(simulator=sim, obs_type='AGENT-INTRUDER', n_intruder=1)
        env.reset()
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        assert obs.shape == env.observation_space.shape
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert isinstance(truncated, bool)
        assert isinstance(info, dict)
