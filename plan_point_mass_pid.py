from typing import List
from shapely import Point
import numpy as np
from plan_template import PlannerTemplate

class PointMassPIDPlanner(PlannerTemplate):
    def __init__(self, waypoints:List[Point]):
        self.waypoints = waypoints
        self.current_idx = 0
        self.threshold = 2.0 # unit: meters, distance to consider waypoint 'reached'

    def get_plan(self, current_pos:Point) -> List[Point]:
        if self.current_idx >= len(self.waypoints):
            return [self.waypoints[-1]]  # Stay at last waypoint
        
        target = self.waypoints[self.current_idx]
        dist = target.distance(current_pos)
        
        if dist < self.threshold:
            self.current_idx += 1
            
        return [self.waypoints[min(self.current_idx, len(self.waypoints)-1)]]  

