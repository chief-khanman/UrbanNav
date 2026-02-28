from typing import Dict 
from shapely import Point
from plan_template import PlannerTemplate
from abc import ABC
from typing import Dict, Any, List
from uav_template import UAV_template
from uav import UAV
from component_schema import UAMConfig, VALID_DYNAMICS





class PlannerEngine:

    def __init__(self, 
                 config:UAMConfig, 
                 plan_uav_map: Dict[str, List[int]], 
                 uav_dict:Dict[int, UAV|UAV_template]):
        
        self.config = config
        self.dt = self.config.simulator.dt
        
        self.plan_uav_map = plan_uav_map # {'PointMassPIDPlanner':[uav_id1, uav_id2, ...], 'HolonomicPIDPlanner': [uav_id3, uav_id4]}
        self.uav_dict = uav_dict # {uav_id<int>:UAV, ....}

        self.plan_str_obj_map:Dict[int, PlannerTemplate] = {} #{'PointMassPIDPlanner':PointMassPIDPlanner, 'HolonomicPIDPlanner':HolonomicPIDPlanner}
        
        self.plan_dict: Dict[int, List[Point]] = {}        
        return None

    def register_uav_planners(self, ) -> None:
        #TODO: complete planner registration - will follow similar pattern defined in dynamics_engine.py
        for uav_id in self.uav_dict.keys():    
            uav = self.uav_dict[uav_id]
            # need to refactor the calling style/signature and provided parameters 
            #TODO: plan init requires providing waypoints - why is there a error 
            plan_model:PlannerTemplate = self.plan_str_obj_map[uav_id](waypoints = [uav.start_vertiport.location, uav.end_vertiport.location])

        return None 
            

        pass

    def get_plans(self, *args, **kwargs) -> Dict[int, List[Point]]:
        # use uav_id to access plan of each UAV 
        # use individual UAV plan_template to get plan and assign to UAV 
        
        # 
        #
        for uav_id in self.uav_dict.keys():
            
            plan_model:PlannerTemplate = self.plan_str_obj_map[uav_id]
            uav_plan = plan_model.get_plan()
            
            self.plan_dict[uav_id] = uav_plan

        return self.plan_dict
    
    def set_plans(self, *args, **kwargs):
        pass


