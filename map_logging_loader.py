import json
import os
from typing import List, Dict, Tuple, Union
import numpy as np

class MapLoader:
    """
    Unified loader class for analyzing both non-learning agent trajectories
    and learning agent transitions from the integrated logger.
    """
    
    def __init__(self, base_log_dir="logs"):
        """Initialize the integrated loader
        
        Args:
            base_log_dir (str): Base directory containing episode logs
        """
        self.base_log_dir = base_log_dir
    
    def list_episodes(self) -> List[str]:
        """List all available episode directories
        
        Returns:
            List of episode directory names
        """
        return [d for d in os.listdir(self.base_log_dir) 
                if os.path.isdir(os.path.join(self.base_log_dir, d)) and d.startswith("episode_")]
    
    def load_episode_metadata(self, episode_dir: str) -> Dict:
        """Load metadata for a specific episode
        
        Args:
            episode_dir (str): Name of the episode directory
            
        Returns:
            Dict containing episode metadata
        """
        metadata_path = os.path.join(self.base_log_dir, episode_dir, 'metadata.json')
    
        # Check if the metadata file exists
        if not os.path.exists(metadata_path):
            print(f"Warning: Metadata file not found: {metadata_path}")
            # Return empty metadata dictionary
            return {
                "timestamp": "unknown",
                "collision_occurred": False,
                "collision_agents": [],
                "collision_type": "none",
                "non_learning_agents": [],
                "learning_agents": [],
                "completed_agents": [],
                "num_non_learning_agents": 0,
                "num_learning_agents": 0
            }
            
        # Load and return metadata if it exists
        try:
            with open(metadata_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error loading metadata from {metadata_path}: {e}")
            # Return empty metadata dictionary on error
            return {
                "timestamp": "unknown",
                "collision_occurred": False,
                "collision_agents": [],
                "collision_type": "none",
                "non_learning_agents": [],
                "learning_agents": [],
                "completed_agents": [],
                "num_non_learning_agents": 0,
                "num_learning_agents": 0
            }
    
    def get_non_learning_agents(self, episode_dir: str) -> List[str]:
        """Get list of non-learning agent IDs in an episode
        
        Args:
            episode_dir (str): Name of the episode directory
            
        Returns:
            List of non-learning agent IDs
        """
        non_learning_dir = os.path.join(self.base_log_dir, episode_dir, "non_learning_agents")
        if not os.path.exists(non_learning_dir):
            return []
            
        return [f.split('_')[1] for f in os.listdir(non_learning_dir) 
                if f.startswith('agent_') and f.endswith('_trajectory.json')]
    
    def get_learning_agents(self, episode_dir: str) -> List[str]:
        """Get list of learning agent IDs in an episode
        
        Args:
            episode_dir (str): Name of the episode directory
            
        Returns:
            List of learning agent IDs
        """
        learning_dir = os.path.join(self.base_log_dir, episode_dir, "learning_agents")
        if not os.path.exists(learning_dir):
            return []
            
        return [f.split('_')[1] for f in os.listdir(learning_dir) 
                if f.startswith('agent_') and f.endswith('_transitions.json')]
    
    def load_non_learning_trajectory(self, episode_dir: str, agent_id: str) -> Dict:
        """Load trajectory for a specific non-learning agent
        
        Args:
            episode_dir (str): Name of the episode directory
            agent_id (str): ID of the non-learning agent
            
        Returns:
            Dict containing the agent's trajectory data
        """
        filepath = os.path.join(
            self.base_log_dir, 
            episode_dir, 
            "non_learning_agents", 
            f"agent_{agent_id}_trajectory.json"
        )
        with open(filepath, 'r') as f:
            return json.load(f)
    
    def load_learning_transitions(self, episode_dir: str, agent_id: str) -> Dict:
        """Load transitions for a specific learning agent
        
        Args:
            episode_dir (str): Name of the episode directory
            agent_id (str): ID of the learning agent
            
        Returns:
            Dict containing the agent's transition data
        """
        filepath = os.path.join(
            self.base_log_dir, 
            episode_dir, 
            "learning_agents", 
            f"agent_{agent_id}_transitions.json"
        )
        with open(filepath, 'r') as f:
            return json.load(f)
    
    def get_non_learning_step(self, episode_dir: str, agent_id: str, step: int) -> Dict:
        """Get a specific step from a non-learning agent's trajectory
        
        Args:
            episode_dir (str): Name of the episode directory
            agent_id (str): ID of the non-learning agent
            step (int): Step number to retrieve
            
        Returns:
            Dict containing state and action for that step
        """
        trajectory = self.load_non_learning_trajectory(episode_dir, agent_id)
        if step >= len(trajectory['trajectory']):
            raise IndexError(f"Step {step} out of range. Trajectory has {len(trajectory['trajectory'])} steps.")
        return trajectory['trajectory'][step]
    
    def get_learning_transition(self, episode_dir: str, agent_id: str, step: int) -> Dict:
        """Get a specific transition from a learning agent's data
        
        Args:
            episode_dir (str): Name of the episode directory
            agent_id (str): ID of the learning agent
            step (int): Step number to retrieve
            
        Returns:
            Dict containing state, action, reward, next_state, next_action for that step
        """
        transitions = self.load_learning_transitions(episode_dir, agent_id)
        if step >= len(transitions['transitions']):
            raise IndexError(f"Step {step} out of range. Transitions has {len(transitions['transitions'])} steps.")
        return transitions['transitions'][step]
    
    def get_non_learning_step_range(self, episode_dir: str, agent_id: str, start_step: int, end_step: int) -> List[Dict]:
        """Get a range of steps from a non-learning agent's trajectory
        
        Args:
            episode_dir (str): Name of the episode directory
            agent_id (str): ID of the non-learning agent
            start_step (int): Starting step (inclusive)
            end_step (int): Ending step (exclusive)
            
        Returns:
            List of state-action pairs for the specified range
        """
        trajectory = self.load_non_learning_trajectory(episode_dir, agent_id)
        return trajectory['trajectory'][start_step:end_step]
    
    def get_learning_transition_range(self, episode_dir: str, agent_id: str, start_step: int, end_step: int) -> List[Dict]:
        """Get a range of transitions from a learning agent's data
        
        Args:
            episode_dir (str): Name of the episode directory
            agent_id (str): ID of the learning agent
            start_step (int): Starting step (inclusive)
            end_step (int): Ending step (exclusive)
            
        Returns:
            List of SARSA transitions for the specified range
        """
        transitions = self.load_learning_transitions(episode_dir, agent_id)
        return transitions['transitions'][start_step:end_step]
    
    def get_non_learning_full_trajectory(self, episode_dir: str, agent_id: str) -> Tuple[List, List]:
        """Get the complete trajectory for a non-learning agent
        
        Returns:
            Tuple of (states, actions)
        """
        trajectory = self.load_non_learning_trajectory(episode_dir, agent_id)
        states = [step['state'] for step in trajectory['trajectory']]
        actions = [step['action'] for step in trajectory['trajectory']]
        return states, actions
    
    def get_learning_full_trajectory(self, episode_dir: str, agent_id: str) -> Tuple[List, List, List, List, List]:
        """Get a complete trajectory for a learning agent
        
        Returns:
            States: List of state dictionaries
            Actions: List of action arrays
            Rewards: List of reward values
            Next_states: List of next state dictionaries
            Next_actions: List of next action arrays
        """
        transitions = self.load_learning_transitions(episode_dir, agent_id)
        states = []
        actions = []
        rewards = []
        next_states = []
        next_actions = []
        
        for t in transitions['transitions']:
            # Skip the first transition where state and action are null
            if t['state'] is not None:
                states.append(t['state'])
                actions.append(t['action'])
                rewards.append(t['reward'])
                next_states.append(t['next_state'])
                next_actions.append(t['next_action'])
            else:
                # For first transition, only collect next_state and next_action
                next_states.append(t['next_state'])
                next_actions.append(t['next_action'])
                
        return states, actions, rewards, next_states, next_actions
    
    def print_non_learning_step_details(self, step_data: Dict):
        """Print detailed information about a non-learning agent step"""
        print("\nState:")
        if isinstance(step_data['state'], list) and len(step_data['state']) > 0:
            state = step_data['state'][0]  # Access first state in the list if it's a list
            for key, value in state.items():
                print(f"  {key}: {value}")
        else:
            state = step_data['state']
            for key, value in state.items():
                print(f"  {key}: {value}")
            
        print(f"\nAction: {step_data['action']}")
    
    def print_learning_transition_details(self, transition: Dict):
        """Print detailed information about a learning agent transition"""
        print("\nLearning Transition:")
        
        # Handle state (which might be null for first transition)
        print("\nState:")
        if transition['state'] is not None:
            for key, value in transition['state'].items():
                print(f"  {key}: {value}")
        else:
            print("  [Initial state - None]")
        
        # Handle action (which might be null for first transition)
        if transition['action'] is not None:
            print(f"\nAction taken: {transition['action']}")
        else:
            print("\nAction taken: [Initial action - None]")
            
        print(f"Reward: {transition['reward']}")
        
        print(f"\nNext State:")
        for key, value in transition['next_state'].items():
            print(f"  {key}: {value}")
        
        print(f"\nNext Action: {transition['next_action']}")
        
        if transition['info'] and len(transition['info']) > 0:
            print("\nAdditional Info:")
            for key, value in transition['info'].items():
                print(f"  {key}: {value}")
    
    def print_episode_summary(self, episode_dir: str):
        """Print a summary of all agents in an episode"""
        metadata = self.load_episode_metadata(episode_dir)
        print(f"\nEpisode Summary ({episode_dir}):")
        print(f"Timestamp: {metadata['timestamp']}")
        
        # Print collision information
        if metadata['collision_occurred']:
            print(f"Collision occurred between agents: {metadata['collision_agents']}")
            print(f"Collision type: {metadata['collision_type']}")
        
        # Print agent counts
        print(f"Non-learning agents: {metadata.get('num_non_learning_agents', len(metadata.get('non_learning_agents', [])))}")
        print(f"Learning agents: {metadata.get('num_learning_agents', len(metadata.get('learning_agents', [])))}")
        print(f"Completed agents: {metadata['completed_agents']}")
        
        # Print summary for non-learning agents
        non_learning_agents = self.get_non_learning_agents(episode_dir)
        if non_learning_agents:
            print("\n=== Non-Learning Agents ===")
            for agent_id in non_learning_agents:
                trajectory = self.load_non_learning_trajectory(episode_dir, agent_id)
                print(f"\nAgent {agent_id}:")
                print(f"Trajectory length: {trajectory['trajectory_length']}")
                print(f"Completed successfully: {trajectory['completed_successfully']}")
                print(f"Terminated by collision: {trajectory['terminated_by_collision']}")
        
        # Print summary for learning agents
        learning_agents = self.get_learning_agents(episode_dir)
        if learning_agents:
            print("\n=== Learning Agents ===")
            for agent_id in learning_agents:
                transitions = self.load_learning_transitions(episode_dir, agent_id)
                print(f"\nAgent {agent_id}:")
                print(f"Transitions length: {transitions['transitions_length']}")
                print(f"Completed successfully: {transitions['completed_successfully']}")
                print(f"Terminated by collision: {transitions['terminated_by_collision']}")
    
    def print_agent_details(self, episode_dir: str, agent_id: str):
        """Print detailed information about a specific agent"""
        # Check if agent is non-learning or learning
        non_learning_agents = self.get_non_learning_agents(episode_dir)
        learning_agents = self.get_learning_agents(episode_dir)
        
        if agent_id in non_learning_agents:
            # Print details for non-learning agent
            trajectory = self.load_non_learning_trajectory(episode_dir, agent_id)
            print(f"\nNon-Learning Agent {agent_id}:")
            print(f"Trajectory length: {trajectory['trajectory_length']}")
            print(f"Completed successfully: {trajectory['completed_successfully']}")
            print(f"Terminated by collision: {trajectory['terminated_by_collision']}")
            
            # Show first and last state-action pairs
            if trajectory['trajectory']:
                print("\nFirst step:")
                self.print_non_learning_step_details(trajectory['trajectory'][0])
                print("\nLast step:")
                self.print_non_learning_step_details(trajectory['trajectory'][-1])
                
        elif agent_id in learning_agents:
            # Print details for learning agent
            transitions = self.load_learning_transitions(episode_dir, agent_id)
            print(f"\nLearning Agent {agent_id}:")
            print(f"Transitions length: {transitions['transitions_length']}")
            print(f"Completed successfully: {transitions['completed_successfully']}")
            print(f"Terminated by collision: {transitions['terminated_by_collision']}")
            
            # Show first and last transitions, handling the case where first state/action are null
            if transitions['transitions']:
                print("\nFirst transition:")
                self.print_learning_transition_details(transitions['transitions'][0])
                
                # Show the first complete transition (state, action, reward, next_state, next_action)
                if len(transitions['transitions']) > 1:
                    print("\nFirst complete transition:")
                    self.print_learning_transition_details(transitions['transitions'][1])
                
                print("\nLast transition:")
                self.print_learning_transition_details(transitions['transitions'][-1])
        else:
            print(f"Agent {agent_id} not found in episode {episode_dir}")