#! rename - main modules/scripts to have a airspace/aeronautics theme 
from typing import List, Dict, Any, Tuple 
import numpy as np 
from simulator_manager import SimulatorManager
from renderer import Renderer
from logger import Logger 
from component_schema import UAMConfig, ActionType, UAVCommand, UAVCommandBundle, SimulatorState
class UAMSimulator:
    """Main simulator coordinating all components"""
    
    def __init__(self, 
                 config_path:str):
        
        # config - file with ATC, airspace, vertiport, UAV 
        self.config = UAMConfig.load_from_yaml(config_path) # what are the variables in config 
        # Build a config checker  
        # total_time_step 
        self.total_timestep = self.config.simulator.total_timestep

        ##### Simulator Manager #####
        # what does state manager do - manage/hold the data of UAV, Airspace, ATC, vertiport
        self.simulator_manager = SimulatorManager(self.config) 

        ##### Rendering #####
        # let renderer have its own data storeage 
        # needs a storage
        # needs to communicate with StateManager
        self.renderer = Renderer() 
        
        ##### Metrics #####
        # collect and store 
        # 1. step metrics 
        # 2. episode metrics 
        self.logger = Logger() # need to define a path to metrics directory - get from config file  
    
    
    def reset(self):
        """Reset simulator to initial state"""
        # RESET sim manager
        self.simulator_manager.reset() 
        # log 
        self.logger.reset()
        self.logger.log_step()
        

    
    def step(self, commands:UAVCommandBundle):

        current_state = self.simulator_manager.get_state()

        self.simulator_manager.step(commands)
        self.logger.log_step(current_state)

        return None
    

    def render(self,):
        pass

    
    
    
    def get_state(self):
        """Get current state snapshot"""
        return self.simulator_manager.get_state()
    
    def save_state(self, filepath):
        """Save state to JSON for replay"""
        state = self.simulator_manager.get_state()
        with open(filepath, 'w') as f:
            f.write(state.to_json())
    
    def load_state(self, filepath):
        """Load state from JSON"""
        with open(filepath, 'r') as f:
            state = SimulatorState.from_json(f.read()) # state is of simulator_state type 
        self.simulator_manager.set_state(state)


                

