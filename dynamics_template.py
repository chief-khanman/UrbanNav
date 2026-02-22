from abc import ABC, abstractmethod
from typing import Tuple


class Dynamics(ABC):
    
    def __init__(self, dt:float = 0.1):
        self.dt = dt
        
    
    @abstractmethod
    def update(self, uav_id:str, action:Tuple):
        ''' Apply actions to update state of UAV.
            This method has side effects.
        '''
        return None