from gymnasium import Env, spaces
from typing import List

from vertiport import Vertiport
from uam_simulator import UAMSimulator

class VertiportDesignEnv(Env):
    def __init__(self, 
                 config_path:str,
                 simulator_timesteps:int):
        
        self.config_path = config_path
        self.simulator = UAMSimulator(config_path=self.config_path)
        self.simulator_timesteps = simulator_timesteps
        self.seed = self.simulator.seed

        self.observation_space = spaces.
        self.action_space = spaces.
        pass

    def _create_obs(self, metrics, action) -> spaces.Box:
        return formatted_obs
    
    def get_reward(self, obs, previous_obs) -> float:
        reward = func(obs, previous_obs)
        return reward
    
    def step(self, 
             action:List[Vertiport]):
        self.simulator.reset()
        self.simulator.set_vertiports(action)
        for step in self.simulator_timesteps:
            state, some_bool, some_dict = self.simulator.step()
        
        simulator_metrics = self.simulator.metrics.get_metrics()

        obs = self._create_obs(simulator_metrics, action)

        reward = self.get_reward(obs, self.previous_obs)

        self.previous_obs = obs

        return obs, info, reward, terminated, truncated 

        
        pass
    
    def reset(self, seed:int=self.seed, options=None):
        self.simulator.reset()
        #run simulator with random vertiport initialization
        for step in self.simulator_timesteps:
            state, some_bool, some_dict = self.simulator.step()
        metrics = self.simulator.metrics.
        #create obs with metrics and selected_vertiports
        obs = self._create_obs(state)
        # store OBS:(current_metrics and actions) as current_obs
        self.previous_obs = obs
        #create info
        info = some_dict 

        return obs, info
        
    
    def close(self):
        pass
