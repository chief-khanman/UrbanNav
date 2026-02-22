from abc import ABC, abstractmethod





class Controller(ABC):
    def __init__(self):
        self.controller_type = None
        self.mode:str

    @abstractmethod 
    def get_control_action(self):
        pass

    @abstractmethod
    def set_control_action(self):
        pass

