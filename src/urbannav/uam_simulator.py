#! rename - main modules/scripts to have a airspace/aeronautics theme
from typing import List, Dict, Any, Tuple, Optional
import numpy as np
import pandas as pd
from urbannav.simulator_manager import SimulatorManager
from urbannav.renderer import Renderer
from urbannav.logger import Logger
from urbannav.component_schema import UAMConfig, ActionType, UAVCommand, UAVCommandBundle, SimulatorState, RESERVED_TYPE_SINGLE_AGENT_LEARNING


class UAMSimulator:
    """Main simulator coordinating all components"""

    def __init__(self,
                 config_path:str,
                 od_matrix_path: Optional[str] = None,
                 zone_region_map_path: Optional[str] = None):
        '''
        Args:
            config_path         : path to the YAML simulator config file.
            od_matrix_path      : optional path to the Band 1 OD lambda matrix
                                  (.npy or .csv). If None, the simulator falls
                                  back to random mission assignment (no
                                  demand-driven dispatch) — this is the
                                  default, unchanged behavior.
            zone_region_map_path: optional path to the Band 1 zone-region
                                  mapping CSV (zone_id, region_id columns).
                                  Static for the run; used by the vertiport-
                                  design RL env to build vertiport_region_map
                                  per episode. None in standalone/mission-sim use.
        '''

        # config - file with ATC, airspace, vertiport, UAV
        self.config = UAMConfig.load_from_yaml(config_path)
        # total_time_step
        self.total_timestep = self.config.simulator.total_timestep

        ##### Simulator Manager #####
        # Load OD demand matrix from Band 1 output if a path is supplied.
        # Supports .npy (numpy array) and .csv (comma-delimited) formats.
        # None -> SimulatorManager runs the original random mission
        # assignment so the simulator runs unchanged in standalone use.
        if od_matrix_path is not None:
            if od_matrix_path.endswith('.npy'):
                lambda_matrix = np.load(od_matrix_path)
            else:
                lambda_matrix = np.loadtxt(od_matrix_path, delimiter=',')
        else:
            lambda_matrix = None

        # vertiport_region_map is built per episode by the RL environment
        # when it selects one vertiport per region. Start as None here;
        # update via simulator_manager.update_vertiport_region_map() before
        # each episode when running under the RL wrapper.
        vertiport_region_map = None

        # Load static zone->region mapping from Band 1 output if supplied.
        # CSV format: first column = zone_id, second column = region_id.
        if zone_region_map_path is not None:
            _df = pd.read_csv(zone_region_map_path)
            zone_region_map: Optional[Dict] = {
                row[0]: int(row[1]) for row in _df.itertuples(index=False, name=None)
            }
        else:
            zone_region_map = None

        self.simulator_manager = SimulatorManager(
            self.config, lambda_matrix, vertiport_region_map, zone_region_map
        )

        ##### Rendering #####
        self.renderer = Renderer(self.config.rendering, self.config.simulator.mode)

        ##### Metrics #####
        self.logger = Logger(self.config.logging, full_config=self.config)


    def reset(self, rebuild_airspace: bool = True):
        """Reset simulator to initial state.

        Args:
            rebuild_airspace: forwarded to SimulatorManager.reset(). When False,
                reuse the existing airspace (vertiport_list / regions_dict) —
                used by VertiportDesignEnv to avoid re-fetching OSM and
                losing the agent's vertiport selection each design-step.
        """
        self.simulator_manager.reset(rebuild_airspace=rebuild_airspace)
        # Pass the freshly-built airspace to the renderer so it can draw the map
        self.renderer.reset(self.simulator_manager.airspace)
        self.logger.reset()

        # ---- log zero-th step ---- # 
        #TODO: call simulator_manager.get_state()
        #      pass state to log_step()
        self.logger.log_step()

        


    def step(self, commands: UAVCommandBundle):
        current_state = self.simulator_manager.get_state()
        #TODO: change simulator_manager.step() to return None
        #TODO: create get functions for returning and capturing necessary outputs 
        collisions = self.simulator_manager.step(commands)
        #TODO: place collisions
        # collisions = self.simulator_manager.get_collisions()
        # current_state = self.simulator_manager.get_state()
        # sim_data = {'current_state':current_state, 
        #             'collisions':collisions, 
        #              kwargs ...
        #             }
        #TODO: let logger and renderer decide what to do with the data
        #      this way sim_data can contain multiple things and loggers and renderers use what they need 
        # self.logger.log_step(sim_data)
        # self.renderer.render_step(sim_data)

        self.logger.log_step(current_state, collisions=collisions)
        # Snapshot / draw current step (no-op when rendering.enabled=false)
        #TODO: definition of render_step to follow above reccomendation 
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
