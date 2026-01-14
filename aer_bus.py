from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
import json
from enum import Enum
import multiprocessing as mp
from queue import Queue, Empty
import zmq
from controller import BaseController, ControllerManifest
from simulator_state import SimulatorState

class ExecutionMode(Enum):
    INLINE = "inline"          # Controller runs in same process
    PROCESS = "process"        # Controller in separate Python process
    EXTERNAL = "external"      # Controller via ZeroMQ


class AerBus:
    """
    Central message router and controller orchestrator
    Handles both training and deployment modes
    """
    
    def __init__(self, mode: str = 'deployment'):
        self.mode = mode  # 'training' or 'deployment'
        self.controllers: Dict[str, BaseController] = {}
        self.controller_manifests: Dict[str, ControllerManifest] = {}
        
        # For external controllers
        self.zmq_context = None
        self.external_sockets = {}
        
        # For process-based controllers
        self.process_queues = {}
        self.processes = {}
        
        # State management
        self.current_state: Optional[SimulatorState] = None
        
        # Action merging strategy
        self.action_merger = ActionMerger()
    
    def register_controller(self, 
                          controller: BaseController,
                          mode: ExecutionMode = ExecutionMode.INLINE):
        """Register a controller with the bus"""
        
        manifest = controller.manifest
        controller_id = manifest.controller_id
        
        self.controller_manifests[controller_id] = manifest
        
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
    
    def step(self, current_state: SimulatorState) -> Dict[str, Dict[str, Any]]:
        """
        Main step function - distribute state, collect actions
        
        Returns:
            Dict mapping UAV IDs to aggregated actions
        """
        self.current_state = current_state
        
        # Collect actions from all controllers
        all_actions = {}
        
        # 1. Inline controllers (fastest)
        for controller_id, controller in self.controllers.items():
            manifest = self.controller_manifests[controller_id]
            
            # Filter state for this controller
            obs = current_state.filter_for_controller(manifest)
            
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


class ActionMerger:
    """Handles conflicts when multiple controllers affect same UAV"""
    
    def merge(self, 
              all_actions: Dict[str, Dict[str, Any]],
              manifests: Dict[str, ControllerManifest]) -> Dict[str, Any]:
        """
        Merge actions from multiple controllers
        
        Strategy:
        1. Direct control (RL, LQR) takes precedence
        2. ATC directives modify control if safe
        3. External systems (wind) are additive
        """
        
        merged = {}
        
        # Priority levels
        direct_control = {}  # Highest priority
        directives = {}      # Medium priority
        external = {}        # Additive
        
        for controller_id, actions in all_actions.items():
            manifest = manifests[controller_id]
            
            if manifest.output_type in ['action', 'control']:
                direct_control.update(actions)
            elif manifest.output_type == 'directive':
                directives.update(actions)
            elif manifest.output_type == 'external':
                external.update(actions)
        
        # Merge: direct control + directives (if compatible) + external
        merged = direct_control.copy()
        
        # Apply directives that don't conflict
        for uav_id, directive in directives.items():
            if uav_id not in merged:
                merged[uav_id] = directive
            else:
                # Merge directive with existing control
                merged[uav_id] = self._merge_directive(
                    merged[uav_id], directive
                )
        
        # Add external effects (e.g., wind disturbance)
        for uav_id, external_effect in external.items():
            if uav_id in merged:
                merged[uav_id]['external_disturbance'] = external_effect
        
        return merged
    
    def _merge_directive(self, control, directive):
        """Merge ATC directive with low-level control"""
        # Example: ATC says "climb", modify thrust accordingly
        if directive.get('type') == 'altitude_change':
            control['target_altitude'] = directive['target_altitude']
        return control
