from controller_template import ControllerTemplate

class StaticController(ControllerTemplate):

    def __init__(self, max_acceleration, max_heading_change):
        super().__init__(max_acceleration, max_heading_change)

    def __call__(self, observation):
        return (0,0)