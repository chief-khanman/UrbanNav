from abc import ABC, abstractmethod
from typing import Any




class Controller(ABC):
    def __init__(self, dt, mode='undef'):
        self.mode:str = mode
        self.controller_type = None
        self.dt = dt

    @abstractmethod 
    def get_control_action(self, *args, **kwargs) -> Any:
        pass

    @abstractmethod
    def set_control_action(self):
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reset controller state between episodes. Override in stateful subclasses."""
        pass

