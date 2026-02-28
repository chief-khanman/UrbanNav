from typing import Dict, List, Optional, Any
import json
from enum import Enum
import multiprocessing as mp
from queue import Queue, Empty
import zmq
from component_schema import VALID_CONTROLLERS
from controller_template import Controller
from controller_pid_point_mass import PIDPointMassController


CONTROLLER_CLASS_MAP: Dict[str, type] = {
    'PIDPointMassController': PIDPointMassController, 
    # other controllers will be defined here 
}

class ExecutionMode(Enum):
    INLINE = "inline"          # Controller runs in same process
    PROCESS = "process"        # Controller in separate Python process
    EXTERNAL = "external"      # Controller via ZeroMQ


class AerBus:
    """
    Central message router and controller orchestrator
    Handles both training and deployment modes
    """
    
    def __init__(self, config, controller_uav_map, uav_dict, mode: str = 'deployment'):
        
        self.config = config
        # mappping: uav_id -> UAV 
        self.uav_dict = uav_dict
        # mapping: Controller_str -> [uav_id1, uav_id2, ...], other_controller_str -> [uav_id4, uav_id5, ....]
        self.controller_uav_map = controller_uav_map

        # mapping: controller_str -> (internal)Controller
        self.mode = mode  # 'training' or 'deployment' # what is the purpose of mode ??
        

        #TODO: 
        # need to clarify different controller types and who will use which source - base(internal)/external/process 
        # also need to clarify the RL in training and RL during evaluation 
        
        # base(internal) controllers
        self.controllers: Dict[str, Controller] = {}
        
        # For external controllers
        self.zmq_context = None
        self.external_sockets = {}
        
        # For process-based controllers
        self.process_queues = {}
        self.processes = {}
        
    
    def register_controller(self, 
                          controller: Controller,
                          mode: ExecutionMode = ExecutionMode.INLINE):
        """Register a controller with the bus"""
        

        
        if mode == ExecutionMode.INLINE:
            # Store controller directly
            self.controllers[controller_id] = controller
        
        elif mode == ExecutionMode.PROCESS:
            # Spawn separate process
            self._spawn_controller_process(controller)
        
        elif mode == ExecutionMode.EXTERNAL:
            # Setup ZeroMQ socket for external script
            self._setup_external_socket(controller_id)
        
        print(f"Registered controller: {controller_id} in {mode.value} mode")

    #TODO: how to resolve registering all the internal controllers 
    def register_uav_controllers(self,):

        # pseudo code
        # check if UAV has correct controller defined in VALID_CONTROLLERS
        # check if UAV has correct controller with mapping defined in CONTROLLER_CLASS_MAP
        # register controller - this is the extra part compared to dynamics, planner, and sensor
        #       if internal controller - register using internal_controller_registration method
        #       if external controller - same for external 
        #       if separate python process - same but for python process 
        # WHAT DOES REGISTRATION DO - 
        # no matter what the controller - 1. create the instance  
        #                                 2. map uav_id to instance 

        
        #TODO: is this the correct way to use the function ??
        self.register_controller()
    
    def _spawn_controller_process(self, controller: BaseController):
        """Spawn controller in separate process"""
        controller_id = controller.manifest.controller_id
        
        # Create communication queues
        state_queue = mp.Queue()
        action_queue = mp.Queue()
        
        # Spawn process
        process = mp.Process(
            target=self._controller_process_worker,
            args=(controller, state_queue, action_queue)
        )
        process.start()
        
        self.process_queues[controller_id] = {
            'state': state_queue,
            'action': action_queue
        }
        self.processes[controller_id] = process
    
    @staticmethod
    def _controller_process_worker(controller, state_queue, action_queue):
        """Worker function running in separate process"""
        while True:
            try:
                # Wait for state
                state_dict = state_queue.get(timeout=1.0)
                
                if state_dict is None:  # Shutdown signal
                    break
                
                # Compute action
                action = controller.compute_action(state_dict)
                
                # Send back action
                action_queue.put(action)
            
            except Empty:
                continue
            except Exception as e:
                print(f"Controller error: {e}")
                action_queue.put({})  # Send empty action on error
    
    def _setup_external_socket(self, controller_id: str):
        """Setup ZeroMQ socket for external controller"""
        if self.zmq_context is None:
            self.zmq_context = zmq.Context()
        
        # Create REQ-REP socket pair
        socket = self.zmq_context.socket(zmq.REP)
        port = 5555 + len(self.external_sockets)  # Incremental ports
        socket.bind(f"tcp://*:{port}")
        
        self.external_sockets[controller_id] = {
            'socket': socket,
            'port': port
        }
        
        print(f"External controller {controller_id} listening on port {port}")
    
    #TODO: what should be the correct return signature - once plan_dict is handed to get_actions(), it will use that plan and compare against uav current attrs to generate control actions
    def get_actions(self, plan_dict) -> Dict[int, Any]: 
        """
        Main step function - distribute state, collect actions
        
        Returns:
            Dict mapping UAV IDs to aggregated actions
        """
        
        
        # Collect actions from all controllers
        all_actions = {}
        
        # 1. Inline controllers (fastest)
        for controller_id, controller in self.controllers.items():
            
            
            # Filter state for this controller
            obs = 
            
            # Get action
            action = controller.compute_action(obs)
            all_actions[controller_id] = action
        
        # 2. Process-based controllers
        for controller_id, queues in self.process_queues.items():
            manifest = self.controller_manifests[controller_id]
            obs = current_state.filter_for_controller(manifest)
            
            # Send state
            queues['state'].put(obs)
            
            # Receive action (with timeout)
            try:
                action = queues['action'].get(timeout=0.1)
                all_actions[controller_id] = action
            except Empty:
                print(f"Warning: Controller {controller_id} timed out")
                all_actions[controller_id] = {}
        
        # 3. External controllers (via ZeroMQ)
        for controller_id, socket_info in self.external_sockets.items():
            manifest = self.controller_manifests[controller_id]
            obs = current_state.filter_for_controller(manifest)
            
            socket = socket_info['socket']
            
            # Send state as JSON
            socket.send_json(obs)
            
            # Receive action
            try:
                action = socket.recv_json(flags=zmq.NOBLOCK)
                all_actions[controller_id] = action
            except zmq.Again:
                print(f"Warning: External controller {controller_id} not responding")
                all_actions[controller_id] = {}
        
        # 4. Merge actions (handle overlapping control)
        merged_actions = self.action_merger.merge(all_actions, 
                                                   self.controller_manifests)
        
        return merged_actions
    
    def reset(self):
        """Reset all controllers (important for RL episodes)"""
        for controller in self.controllers.values():
            controller.reset()
        
        # Send reset signal to process-based controllers
        for controller_id, queues in self.process_queues.items():
            queues['state'].put({'reset': True})
        
        # Send reset to external controllers
        for controller_id, socket_info in self.external_sockets.items():
            socket = socket_info['socket']
            socket.send_json({'reset': True})
            socket.recv_json()  # Wait for ACK
    
    def shutdown(self):
        """Clean shutdown of all resources"""
        # Terminate processes
        for process in self.processes.values():
            process.terminate()
            process.join(timeout=1.0)
        
        # Close ZeroMQ sockets
        for socket_info in self.external_sockets.values():
            socket_info['socket'].close()
        
        if self.zmq_context:
            self.zmq_context.term()


#* ActionMerger -
# For future version of UrbanNav 
# Feature of ActionMerger will resolve conflict when multiple controllers are sending action to same UAV 
# There will be a priority list 
# and based on that we will select the action for the UAV  