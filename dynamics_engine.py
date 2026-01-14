from abc import ABC
from typing import List
from simulator_state import SimulatorState
# in the config file - 
# dynamics mode and solver will be defined for use with DynamicsEngine()
#  
class DynamicsEngine:
    def __init__(self, config):
        self.dynamics_mode:str = config.dynamics_mode #2D or 3D or External
        self.dynamics_equation:DynamicsEquation = config.dynamics_equation # for 2D: PointMass, 2DVector. for 3D: 6DOF, .. for external: PyORCA
        self.solver:Solver = config.solver
        # each uav in UAVs can have unique dynamics
        # 2D:
        #   1. point_mass
        #   2. ...
        # 3D:
        #   1. 6DOF
        #   2. ... 
        # External:
        #   1. PyRVO2
        #   2. ...   
        # Additionally, Dynamics engine will collect solver type from config 


    def step(self, current_state:SimulatorState, actions:List, dt:float):
        
        
        
        ##### update: current_state.UAVs #####
        for uav in current_state.uavs:
            # 1. collect uav_dynamics class 
            # 2. collect corresponding action for uav 
            # 3. make sure action is of correct type for uav_dynamics class
            # 4. use the step of that class to update velocity and position

            # 1. determine uav_dynamics type
            if isinstance(uav.dynamics_type,PointMassDynamics):
                self.dynamics_equation.step(uav)
            
            elif uav.dynamics_type == SixDOFDynamics():
                # 1. update velocity
                uav.velocity += actions[] * dt
                # 2. update position
                uav.position += uav_dict['uav_id'].velocity * dt 
            
            elif uav.dynamics_type == PyORCA():
                # 1. update velocity
                uav.velocity += actions[] * dt
                # 2. update position
                uav.position += uav_dict['uav_id'].velocity * dt 
        
        


class DynamicsEquation(ABC):
    def __init__(self):
        self.solver:Solver 
        self.dynamics_mode:str 
    def step(self, *args, **kwargs):
        pass
    

class PointMassDynamics(DynamicsEquation):
    def __init__(self,):
        self.dynamics_mode = '2D'
        self.solver = dynamics_engine.solver #FirstOrderEuler or RK45 or something else 
        
        
    def step(self, uav, action, dt):
        if isinstance(self.solver,FirstOrderEuler):
            uav.speed += action.acceleration * dt
            uav.heading += action.heading_rate * dt
            uav.position += uav.speed * dt 
        elif self.solver == RK45:
            pass
        else:
            raise RuntimeError('Solver not defined - use existing solvers or define a solver in "dynamics_engine.py". ')
        pass


class TwoDVectorDynamcis(DynamicsEquation):
    def __init__(self, ):
        self.dynamics_mode = '2D'
        self.solver = dynamics_engine.solver
    def step(self, uav, action, dt):
        if self.solver == FirstOrderEuler:
            uav.velocity += uav.velocity + action.acceleration * dt
            uav.position += uav.position + uav.velocity * dt
        elif self.solver == RK45:
            pass
        else:
            raise RuntimeError('Solver not defined - ...')


class SixDOFDynamics(DynamicsEquation):
    def __init__(self,):
        self.dynamics_mode = '3D'
    def step(self):
        pass

class PyORCA(DynamicsEquation):
    def __init__(self,):
        self.dynamics_mode = '2D'

    def step(self,):
        pass


class Solver(ABC):
    def __init__(self,):
        self.name = 'GenericSolver'

class FirstOrderEuler(Solver):
    def __init__(self):
        super().__init__()
        self.name = 'FirstOrderEuler'
    

class RK45(Solver):
    def __init__(self):
        super().__init__()
        self.name = 'RK45'