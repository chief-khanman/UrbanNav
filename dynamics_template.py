from abc import ABC, abstractmethod
from typing import Tuple


class DynamicsTemplate(ABC):
    
    def __init__(self, dt = 0.1):
        self.dt = dt
    
    @abstractmethod
    def update(self, uav, action):
        ''' Apply actions to update state of UAV.
            This method has side effects.
        '''
        return None