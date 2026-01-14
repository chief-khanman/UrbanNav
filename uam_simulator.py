import numpy as np 
from state_manager import StateManager
from simulator_state import SimulatorState
from aer_bus import AerBus, ExecutionMode
from dynamics_engine import DynamicsEngine
from metrics_collector import MetricsCollector
from collision_detection import AdaptiveCollisionDetector
from renderer import Renderer

class UAMSimulator:
    """Main simulator coordinating all components"""
    
    def __init__(self, config):
        # config - file with ATC, airspace, vertiport, UAV 
        self.config = config # what are the variables in config 
        #Build a config checker  
        self.dt = config['dt']  # Time step
        # total_time_step 
        self.total_timestep = config.total_timestep
        
        ##### Core components #####
        # use config to send data to statemanager 
        self.state_manager = StateManager(self.config) # what does state manager do - manage/hold the data of UAV, Airspace, ATC, vertiport
        self.aer_bus = AerBus(mode='deployment') 
        
        ##### Physics/Dynamics #####
        # Dynamics engine will ingest a dict mapping UAV.id to dynamics type(class)
        self.dynamics = DynamicsEngine(self.config) # make dynamics engine handle different kind/form of dynamics 
        ##### Collision #####
        self.collision_detector = AdaptiveCollisionDetector(
                                                            sensor_radius=5.0,
                                                            mode='3D'
                                                            )
        
        ##### Rendering #####
        self.renderer = Renderer() # let renderer have its own data storeage 
        
        ##### Metrics #####
        self.metrics = MetricsCollector() 
    
    def register_controller(self, controller, mode=ExecutionMode.INLINE): # what is the use of execution-mode
        """Register a controller with the AER_BUS"""
        self.aer_bus.register_controller(controller, mode)
    
    def step(self, external_actions=None):
        """
        Main simulation step
        
        Args:
            external_actions: Optional actions (used during RL training)
        
        Returns:
            state, done, info
        """
        
        # 1. Get current state
        # current_state is of type SimulatorState
        current_state = self.state_manager.get_state()
        
        # 2. Get actions from controllers (unless in training mode)
        # during RL training some UAVs can still get actions from external controllers 
        # during TRAINING actions can come from mixed sources 
        # during TESTING all actions are from analytic/learned controllers 
        if external_actions is None:
            actions = self.aer_bus.step(current_state)
        else:
            actions = external_actions # in training mode actions will come from RL controllers 
        
        # 3. Apply dynamics - (NEW) current_state: UPDATED position & velocity 
        self.dynamics.step(current_state, actions, self.dt) # should dynamics.step return SimulatorState type, what are the pros and cons
        
        # 4. Check collisions
        collisions = self.collision_detector.detect_collisions(
            current_state.uavs
        )
        
        # 5. Update state
        self.state_manager.update(current_state)
        
        # 6. Collect metrics
        # There are two kind of metrics 
        #   1. for rendering simulator
        #   2. simulator snap_shot for external processing 
        # IS there any other kind of metrics that I might need 
        self.metrics.record(actions, collisions, current_state) # save action first so that when we read the log, we know which action led to new_state, after reset, action, and collision is saved as zero, 0
        
        # 7. Check termination
        terminated = self._check_terminated(current_state, collisions)

        # 8. Check done
        done = True if current_state.step >= self.total_timestep else False 
        
        info = {
            'collisions': collisions,
            'metrics': self.metrics.get_latest()
        }
        
        return current_state, done, info
    
    def reset(self):
        """Reset simulator to initial state"""
        self.state_manager.reset()
        self.aer_bus.reset()
        self.metrics.reset()
        # should we store the starting state here ??
        # after state_manager.reset() - state_manager.current_state is defined
        # after metrics.reset() - metrics is empty 
        # I think state_manager.current_state/get_current_state() should be called and saved in metrics
        # because step() -  uses the current_state and moves forward
    
    def get_state(self):
        """Get current state snapshot"""
        return self.state_manager.get_state()
    
    def save_state(self, filepath):
        """Save state to JSON for replay"""
        state = self.state_manager.get_state()
        with open(filepath, 'w') as f:
            f.write(state.to_json())
    
    def load_state(self, filepath):
        """Load state from JSON"""
        with open(filepath, 'r') as f:
            state = SimulatorState.from_json(f.read()) # state is of simulator_state type 
        self.state_manager.set_state(state)

    def _check_done(self, current_state, collisions):
        for uav in current_state.uavs:
            if uav.mission_complete:
                uav.complete_mission.append(uav.mission_id)

    def _check_terminated(self,current_state, collisions):
        for uav in current_state.uavs:
            if uav.collision:
                return True
            return False
                

