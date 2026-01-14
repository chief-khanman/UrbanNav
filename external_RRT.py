# external_rrt_controller.py
"""
Example external controller using RRT for path planning
Communicates with simulator via ZeroMQ
"""

import zmq
import numpy as np
import json

class RRTController:
    def __init__(self, uav_ids):
        self.uav_ids = uav_ids
        self.paths = {}  # Store planned paths
    
    def plan_path(self, start, goal, obstacles):
        """RRT path planning logic"""
        # Your RRT implementation
        pass
    
    def compute_action(self, observation):
        """Given state, return control action"""
        actions = {}
        
        for uav_id in self.uav_ids:
            # Find this UAV in observation
            uav_state = next(u for u in observation['uav_states'] 
                           if u['id'] == uav_id)
            
            # Plan path if needed
            if uav_id not in self.paths:
                self.paths[uav_id] = self.plan_path(
                    start=uav_state['position'],
                    goal=uav_state['goal'],
                    obstacles=observation.get('obstacles', [])
                )
            
            # Follow path
            path = self.paths[uav_id]
            next_waypoint = path[0] if path else uav_state['goal']
            
            # Compute action towards waypoint
            direction = next_waypoint - uav_state['position']
            direction = direction / np.linalg.norm(direction)
            
            actions[uav_id] = {
                'velocity': direction * 5.0  # m/s
            }
        
        return actions

def main():
    # Setup ZeroMQ connection
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:5555")  # Connect to AER_BUS
    
    # Initialize controller
    controller = RRTController(uav_ids=['uav_10', 'uav_11', 'uav_12'])
    
    print("RRT Controller connected and running...")
    
    while True:
        # Receive state from simulator
        message = socket.recv_json()
        
        # Handle reset
        if 'reset' in message:
            controller.paths.clear()
            socket.send_json({'ack': 'reset'})
            continue
        
        # Compute action
        action = controller.compute_action(message)
        
        # Send action back
        socket.send_json(action)

if __name__ == "__main__":
    main()
