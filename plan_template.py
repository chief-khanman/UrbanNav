from abc import ABC
from typing import Any



class PlannerTemplate(ABC):
    def __init__(self, *args, **kwargs):
        pass

    def mission_plan(self, *args, **kwargs):
        pass

    def path_plan(self, *args, **kwargs):
        pass

    def trajectory_plan(self, *args, **kwargs):
        pass

    def get_plan(self, *args, **kwargs) -> Any:
        pass 

    def get_velocity_plan(self, *args, **kwargs):
        pass