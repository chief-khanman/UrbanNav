from gymnasium import Env
from gymnasium import spaces 
import numpy as np 

from uam_simulator import UAMSimulator


class SingleAgentEnv(Env):
    def __init__(self,
                 config_file_path): # config_file needs to have only one external_autonomous_uav where the controller will be empty
        self.config_file_path = config_file_path
        self.sim = UAMSimulator(self.config_file_path)

    def step(self, action):

        return obs, reward, terminated, truncated, info


    def reset(self,):
        rl_controller = RL
        return obs, info