from abc import ABC, abstractmethod
from typing import List
from shapely import Point



class PlannerTemplate(ABC):
    def __init__(self, *args, **kwargs):
        pass

    def mission_plan(self, *args, **kwargs):
        pass

    def path_plan(self, *args, **kwargs):
        pass

    def trajectory_plan(self, *args, **kwargs):
        pass

    @abstractmethod
    def get_plan(self, current_pos: Point) -> List[Point]:
        pass

    def get_velocity_plan(self, *args, **kwargs):
        pass
