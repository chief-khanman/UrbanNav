from typing import List, Dict, Any, Tuple, Optional
import numpy as np
import pandas as pd
from urbannav.simulator_manager_vp_design import SimulatorManagerVPDesign
from urbannav.renderer import Renderer
from urbannav.logger import Logger
from urbannav.component_schema import UAMConfig, ActionType, UAVCommand, UAVCommandBundle, SimulatorState, RESERVED_TYPE_SINGLE_AGENT_LEARNING


class UAMSimulatorVPDesign:
    """Main simulator coordinating all components"""

    def __init__(self,
                 config_path: str,
                 od_matrix_path: Optional[str] = None,
                 zone_region_map_path: Optional[str] = None):
        '''Initialize the simulator.

        Args:
            config_path         : path to the YAML simulator config file.
            od_matrix_path      : optional path to the Band 1 OD lambda matrix.
                                  Supports .npy (numpy array) and .csv formats.
                                  If None, the simulator falls back to random
                                  mission assignment (no demand-driven dispatch).
            zone_region_map_path: optional path to Band 1 zone-region mapping
                                  (zone_region_map.csv). Two-column CSV:
                                  first column = zone_id, second = region_id.
                                  Static for the lifetime of the run — does not
                                  change between episodes. Used by the RL env
                                  to build vertiport_region_map per episode.
                                  None in standalone test mode.
        '''
        # config - file with ATC, airspace, vertiport, UAV
        self.config = UAMConfig.load_from_yaml(config_path)
        # total_time_step
        self.total_timestep = self.config.simulator.total_timestep

        ##### Simulator Manager #####
        # Load OD demand matrix from Band 1 output if a path is supplied.
        # Supports .npy (numpy array) and .csv (comma-delimited) formats.
        # When None, SimulatorManagerVPDesign falls back to random mission
        # assignment so the simulator still runs in a standalone test context.
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

        # Load static zone→region mapping from Band 1 output if supplied.
        # CSV format: first column = zone_id, second column = region_id.
        # Keys are kept as-is (str or int depending on CSV content).
        if zone_region_map_path is not None:
            _df = pd.read_csv(zone_region_map_path)
            zone_region_map: Optional[Dict] = {
                row[0]: int(row[1]) for row in _df.itertuples(index=False, name=None)
            }
        else:
            zone_region_map = None

        self.simulator_manager = SimulatorManagerVPDesign(self.config, lambda_matrix, vertiport_region_map, zone_region_map)

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
