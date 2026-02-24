from typing import Dict 
from plan_template import PlannerTemplate






class PlannerEngine:

    def __init__(self, config, uav_list):
        self.config = config
        self.uav_list = uav_list

        self.plan_dict:Dict[str, PlannerTemplate]
        pass


    def get_plans(self, *args, **kwargs) -> Dict[str, PlannerTemplate]:
        # use uav_id to access plan of each UAV 
        # use individual UAV plan_template to get plan and assign to UAV 
        
        return self.plan_dict
    
    def set_plans(self, *args, **kwargs):
        pass


