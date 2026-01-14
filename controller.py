from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Dict, Any
import numpy as np

@dataclass
class ControllerManifest:
    """Metadata describing what a controller needs and provides"""
    controller_id: str
    controlled_uav_ids: List[str]  # Which UAVs this controls
    required_state_keys: List[str]  # What state info it needs
    output_type: str  # 'action', 'waypoint', 'velocity', etc.
    execution_mode: str  # 'inline', 'process', 'external'

class BaseController(ABC):
    """Abstract base class for all controllers"""
    
    def __init__(self, manifest: ControllerManifest):
        self.manifest = manifest
    
    @abstractmethod
    def compute_action(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        """
        Given state observation, return control actions
        
        Args:
            observation: Dict with keys from required_state_keys
                Example: {
                    'uav_states': [(id, pos, vel, heading), ...],
                    'nearby_uavs': [...],
                    'airspace_constraints': [...],
                    'wind_vector': [wx, wy, wz]
                }
        
        Returns:
            Dict mapping UAV IDs to actions
                Example: {
                    'uav_0': {'thrust': 0.5, 'heading_rate': 0.1},
                    'uav_1': {'thrust': 0.7, 'heading_rate': -0.05}
                }
        """
        pass
    
    @abstractmethod
    def reset(self):
        """Reset controller state (important for episodic RL)"""
        pass


class RLController(BaseController):
    """Wrapper for RL policies (single or multi-agent)"""
    
    def __init__(self, manifest: ControllerManifest, policy):
        super().__init__(manifest)
        self.policy = policy  # Trained policy (e.g., from stable-baselines3)
    
    def compute_action(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        # Convert observation to policy input format
        policy_obs = self._prepare_observation(observation)
        
        # Get action from policy
        action, _states = self.policy.predict(policy_obs, deterministic=True)
        
        # Convert action to simulator format
        return self._format_action(action)
    
    def _prepare_observation(self, obs):
        """Convert simulator state to policy observation space"""
        # Extract only the UAVs this controller manages
        my_uavs = [u for u in obs['uav_states'] 
                   if u['id'] in self.manifest.controlled_uav_ids]
        
        # Format according to policy's observation space
        # (This depends on how you trained the policy)
        return np.array([...])  # Implementation specific
    
    def _format_action(self, action):
        """Convert policy action to simulator commands"""
        commands = {}
        for i, uav_id in enumerate(self.manifest.controlled_uav_ids):
            commands[uav_id] = {
                'thrust': action[i, 0],
                'heading_rate': action[i, 1]
            }
        return commands
    
    def reset(self):
        # RL policies might have internal state (LSTM, etc.)
        self.policy.reset()


class LQRController(BaseController):
    """Classic LQR controller"""
    
    def __init__(self, manifest: ControllerManifest, Q, R):
        super().__init__(manifest)
        self.Q = Q  # State cost matrix
        self.R = R  # Control cost matrix
        # Compute LQR gains...
    
    def compute_action(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        commands = {}
        for uav_id in self.manifest.controlled_uav_ids:
            # Find this UAV's state
            uav_state = next(u for u in observation['uav_states'] 
                           if u['id'] == uav_id)
            
            # Compute LQR control
            x = np.array([uav_state['pos'], uav_state['vel']]).flatten()
            u = -self.K @ x  # LQR feedback
            
            commands[uav_id] = {'control': u}
        
        return commands
    
    def reset(self):
        pass  # LQR is stateless


class ATCController(BaseController):
    """Air Traffic Control - provides directives to all UAVs"""
    
    def __init__(self, manifest: ControllerManifest):
        super().__init__(manifest)
        self.assigned_routes = {}
    
    def compute_action(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        """ATC doesn't send control actions, but high-level directives"""
        directives = {}
        
        # Detect conflicts
        conflicts = self._detect_conflicts(observation['uav_states'])
        
        # Issue directives
        for uav_id in conflicts:
            directives[uav_id] = {
                'type': 'altitude_change',
                'target_altitude': self._compute_safe_altitude(uav_id)
            }
        
        return directives
    
    def _detect_conflicts(self, uav_states):
        # Conflict detection logic
        return []
    
    def reset(self):
        self.assigned_routes.clear()
