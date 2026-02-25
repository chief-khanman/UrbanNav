from typing import Dict 
from plan_template import PlannerTemplate






class PlannerEngine:

    def __init__(self, config, plan_uav_map, uav_dict):
        self.config = config
        self.plan_uav_map = plan_uav_map
        self.uav_dict = uav_dict

        self.plan_dict:Dict[str, PlannerTemplate]
        pass


    def get_plans(self, *args, **kwargs) -> Dict[str, PlannerTemplate]:
        # use uav_id to access plan of each UAV 
        # use individual UAV plan_template to get plan and assign to UAV 
        
        return self.plan_dict
    
    def set_plans(self, *args, **kwargs):
        pass


