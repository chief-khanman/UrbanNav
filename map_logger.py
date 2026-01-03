from datetime import datetime
import json
import numpy as np
import os

class MapLogger:
    """
    Unified logger for both non-learning agents and autonomous learning agents
    in UAM environment. Manages separate file structures for both agent types
    while sharing serialization logic.
    """
    
    def __init__(self, base_log_dir="logs"):
        """Initialize the integrated logger
        
        Args:
            base_log_dir (str): Base directory to store all log files
        """
        self.base_log_dir = base_log_dir
        
        # Create base directory if it doesn't exist
        os.makedirs(base_log_dir, exist_ok=True)
        
        # Create timestamp for this logging session
        self._initialize_new_episode()
    
    def _initialize_new_episode(self):
        """Initialize or reinitialize for a new episode with a new timestamp"""
        # Create a unique timestamp for this episode
        self.timestamp = datetime.now().strftime("%Y_%m_%d_%H%M")
        
        # Create episode directory with timestamp
        self.episode_dir = os.path.join(self.base_log_dir, f"episode_{self.timestamp}")
        os.makedirs(self.episode_dir, exist_ok=True)
        
        # Create subdirectories for each agent type
        self.non_learning_dir = os.path.join(self.episode_dir, "non_learning_agents")
        self.learning_dir = os.path.join(self.episode_dir, "learning_agents")
        
        os.makedirs(self.non_learning_dir, exist_ok=True)
        os.makedirs(self.learning_dir, exist_ok=True)
        
        # Tracking data structures
        self.non_learning_trajectories = {}  # Store state-action pairs for non-learning agents
        self.learning_transitions = {}  # Store state, action, reward, next state, next action transitions for learning agents
        self.completed_agents = set()  # Track which agents have completed their mission
        self.collision_data = None  # Store collision information if it occurs
        
        # Episode metadata
        self.episode_metadata = {
            'timestamp': self.timestamp,
            'collision_occurred': False,
            'collision_agents': None,
            'collision_type': None,
            'non_learning_agents': [],
            'learning_agents': [],
            'completed_agents': []
        }
    
    def _serialize_array(self, arr):
        """Convert numpy arrays and other types to JSON-serializable format"""
        if isinstance(arr, np.ndarray):
            return arr.tolist()
        elif isinstance(arr, dict):
            return {k: self._serialize_array(v) for k, v in arr.items()}
        elif isinstance(arr, (list, tuple)):
            return [self._serialize_array(x) for x in arr]
        elif isinstance(arr, (bool, int, float, str, type(None))):
            return arr
        else:
            return str(arr)  # Convert other types to string
    
    def log_non_learning_step(self, agent_id, state, action):
        """Log a single step for a non-learning agent"""
        if agent_id not in self.non_learning_trajectories:
            self.non_learning_trajectories[agent_id] = []
            if agent_id not in self.episode_metadata['non_learning_agents']:
                self.episode_metadata['non_learning_agents'].append(agent_id)
            
        step_data = {
            'state': self._serialize_array(state),
            'action': self._serialize_array(action)
            # TODO: possible location to add current distance
            # distance += state.distance_covered
            # TODO: might have to add a new attribute to UAV to calculate the distance it has covered
        }
        self.non_learning_trajectories[agent_id].append(step_data)

    def log_learning_transition(self, agent_id, state, action, reward, next_state, next_action, info=None):
        """Log a single state, action, reward, next state, next action transition for a learning agent"""
        if agent_id not in self.learning_transitions:
            self.learning_transitions[agent_id] = []
            if agent_id not in self.episode_metadata['learning_agents']:
                self.episode_metadata['learning_agents'].append(agent_id)
            
        transition = {
            # TODO: if the UAV has new attribute for distance covered, then need to add that attr to its state/obs 
            # TODO: OR - use the attr of UAV and store it here as a key:value  
            'state': self._serialize_array(state),
            'action': self._serialize_array(action),
            'reward': reward,
            'next_state': self._serialize_array(next_state),
            'next_action': self._serialize_array(next_action),
            'info': self._serialize_array(info) if info is not None else {}
        }
        self.learning_transitions[agent_id].append(transition)

    def mark_agent_complete(self, agent_id):
        """Mark an agent as having completed its mission"""
        self.completed_agents.add(agent_id)
        if agent_id not in self.episode_metadata['completed_agents']:
            self.episode_metadata['completed_agents'].append(agent_id)

    def record_nmac(self, nmac_ids, time_step=None):
        """Record Near Mid-Air Collision (NMAC) information
        
        Args:
            nmac_ids: List of UAV IDs involved in the NMAC
            time_step: Current time step when NMAC occurred (optional)
        """
        if not hasattr(self, 'nmac_data'):
            self.nmac_data = []
        
        nmac_record = {
            'nmac_ids': self._serialize_array(nmac_ids),
            'time_step': time_step
        }
        
        self.nmac_data.append(nmac_record)
        
        # Update metadata
        if 'nmac_occurred' not in self.episode_metadata:
            self.episode_metadata['nmac_occurred'] = True
            self.episode_metadata['nmac_events'] = []
        
        self.episode_metadata['nmac_events'].append(nmac_record)
    
    def record_collision(self, collision_ids, collision_type):
        """Record collision information"""
        self.collision_data = {
            'collision_ids': collision_ids,
            'collision_type': collision_type
        }
        
        # Update metadata
        self.episode_metadata['collision_occurred'] = True
        self.episode_metadata['collision_agents'] = self._serialize_array(collision_ids)
        self.episode_metadata['collision_type'] = collision_type
    
    def save_episode(self):
        """Save all agent data to their respective files within the episode directory"""
        if not self.non_learning_trajectories and not self.learning_transitions:
            print("No data to save in this episode")
            return
        
        # Update final metadata values
        self.episode_metadata['num_non_learning_agents'] = len(self.non_learning_trajectories)
        self.episode_metadata['num_learning_agents'] = len(self.learning_transitions) #why is transitions = num_learning_agents
        self.episode_metadata['completed_agents'] = list(self.completed_agents) 
        
        # Save metadata for the episode
        with open(os.path.join(self.episode_dir, 'metadata.json'), 'w') as f:
            json.dump(self.episode_metadata, f, indent=2)
        
        # Save each non-learning agent's trajectory
        for agent_id, trajectory in self.non_learning_trajectories.items():
            agent_data = {
                'agent_id': agent_id,
                'agent_type': 'non_learning',
                'trajectory_length': len(trajectory),
                'terminated_by_collision': (self.collision_data and 
                                         agent_id in (self.collision_data['collision_ids'] or [])),
                'completed_successfully': agent_id in self.completed_agents,
                'trajectory': trajectory
            }
            
            filename = f"agent_{agent_id}_trajectory.json"
            filepath = os.path.join(self.non_learning_dir, filename)
            
            with open(filepath, 'w') as f:
                json.dump(agent_data, f, indent=2)
        
        # Save each learning agent's transitions
        for agent_id, transitions in self.learning_transitions.items():
            agent_data = {
                'agent_id': agent_id,
                'agent_type': 'learning',
                'transitions_length': len(transitions),
                'terminated_by_collision': (self.collision_data and 
                                         agent_id in (self.collision_data['collision_ids'] or [])),
                'completed_successfully': agent_id in self.completed_agents,
                'transitions': transitions
            }
            
            filename = f"agent_{agent_id}_transitions.json"
            filepath = os.path.join(self.learning_dir, filename)
            
            with open(filepath, 'w') as f:
                json.dump(agent_data, f, indent=2)
    
    def reset(self):
        """Reset the logger for a new episode with a new timestamp"""
        # Save current episode data if exists
        self.save_episode()
        
        # Initialize a completely new episode with new timestamp and directories
        self._initialize_new_episode()
        
    def close(self):
        """Save the episode data and close the logger"""
        self.save_episode()