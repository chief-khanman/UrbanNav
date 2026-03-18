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
        self.config = UAMConfig.load_from_yaml(config_path)
        # total_time_step
        self.total_timestep = self.config.simulator.total_timestep

        ##### Simulator Manager #####
        self.simulator_manager = SimulatorManager(self.config)

        ##### Rendering #####
        self.renderer = Renderer(self.config.rendering, self.config.simulator.mode)

        ##### Metrics #####
        self.logger = Logger(self.config.logging, full_config=self.config)


    def reset(self):
        """Reset simulator to initial state"""
        self.simulator_manager.reset()
        # Pass the freshly-built airspace to the renderer so it can draw the map
        self.renderer.reset(self.simulator_manager.airspace)
        self.logger.reset()
        self.logger.log_step()
        


    def step(self, commands: UAVCommandBundle):
        current_state = self.simulator_manager.get_state()
        collisions = self.simulator_manager.step(commands)
        self.logger.log_step(current_state, collisions=collisions)
        # Snapshot / draw current step (no-op when rendering.enabled=false)
        self.renderer.render_step(
            self.simulator_manager.atc.uav_dict,
            self.simulator_manager._state.currentstep,
        )
        return collisions

    def render(self) -> None:
        """Save offline animation for the current episode (no-op for realtime-only mode)."""
        self.renderer.save(episode_id=self.logger.episode_id)


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
            state = SimulatorState.from_json(f.read())
        self.simulator_manager.set_state(state)
