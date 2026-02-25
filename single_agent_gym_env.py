import gymnasium as gym
from gymnasium import spaces
import numpy as np

class UAMSimEnv(gym.Env):
    """
    Gym environment wrapper for RL training
    
    Supports both single-agent and multi-agent training
    """
    
    def __init__(self, 
                 simulator,
                 controlled_uav_ids: List[str],
                 observation_config: Dict[str, Any],
                 action_config: Dict[str, Any]):
        
        self.simulator = simulator
        self.controlled_uav_ids = controlled_uav_ids
        
        # Define observation space
        self.observation_space = self._create_observation_space(observation_config)
        
        # Define action space
        self.action_space = self._create_action_space(action_config)
        
        # Create temporary controller for training
        self.training_manifest = ControllerManifest(
            controller_id='rl_training',
            controlled_uav_ids=controlled_uav_ids,
            required_state_keys=observation_config['state_keys'],
            output_type='action',
            execution_mode='inline'
        )
    
    def _create_observation_space(self, config):
        """Define observation space based on config"""
        # Example: position, velocity, nearby UAVs
        n_features = config.get('n_features', 10)
        return spaces.Box(
            low=-np.inf, 
            high=np.inf, 
            shape=(n_features,), 
            dtype=np.float32
        )
    
    def _create_action_space(self, config):
        """Define action space"""
        # Example: thrust and heading rate
        return spaces.Box(
            low=np.array([0, -1]), 
            high=np.array([1, 1]), 
            dtype=np.float32
        )
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Reset simulator
        self.simulator.reset()
        
        # Get initial observation
        state = self.simulator.get_state()
        obs = self._extract_observation(state)
        
        return obs, {}
    
    def step(self, action):
        """
        Execute action in simulator
        
        Args:
            action: numpy array from RL policy
        
        Returns:
            observation, reward, terminated, truncated, info
        """
        
        # Convert action to simulator format
        sim_action = self._format_action(action)
        
        # Step simulator (only this controller active during training)
        state, done, info = self.simulator.step(sim_action)
        
        # Extract observation for policy
        obs = self._extract_observation(state)
        
        # Compute reward
        reward = self._compute_reward(state, action, info)
        
        terminated = done
        truncated = info.get('truncated', False)
        
        return obs, reward, terminated, truncated, info
    
    def _extract_observation(self, state: SimulatorState) -> np.ndarray:
        """Convert simulator state to RL observation"""
        # Extract features for controlled UAVs
        obs_list = []
        
        for uav_id in self.controlled_uav_ids:
            uav = next(u for u in state.uavs if u['id'] == uav_id)
            
            # Own state
            obs_list.extend(uav['position'])
            obs_list.extend(uav['velocity'])
            
            # Relative positions to nearby UAVs
            nearby = self._get_nearby_uavs(uav, state.uavs)
            # ... add features
        
        return np.array(obs_list, dtype=np.float32)
    
    def _format_action(self, action: np.ndarray) -> Dict[str, Any]:
        """Convert RL action to simulator action format"""
        actions = {}
        
        for i, uav_id in enumerate(self.controlled_uav_ids):
            actions[uav_id] = {
                'thrust': float(action[i, 0]),
                'heading_rate': float(action[i, 1])
            }
        
        return actions
    
    def _compute_reward(self, state, action, info):
        """Compute reward for RL training"""
        reward = 0.0
        
        # Goal reaching
        for uav_id in self.controlled_uav_ids:
            uav = next(u for u in state.uavs if u['id'] == uav_id)
            goal_dist = np.linalg.norm(uav['position'] - uav['goal'])
            reward -= goal_dist
        
        # Collision penalty
        if info.get('collision', False):
            reward -= 100.0
        
        # Energy efficiency
        energy_cost = np.sum(action**2)
        reward -= 0.01 * energy_cost
        
        return reward
