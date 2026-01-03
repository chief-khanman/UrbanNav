from abc import ABC, abstractmethod
from typing import Tuple

class ControllerTemplate(ABC):

    def __init__(self, max_acceleration, max_heading_change):
        self.max_acceleration = max_acceleration
        self.max_heading_change = max_heading_change

    @abstractmethod
    def __call__(self, observation) -> Tuple:
        '''Returns action based on observation.
        
            Args: observation array

            Returns: action tuple
        
        '''

        pass