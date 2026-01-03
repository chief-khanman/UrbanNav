#utils_vel.py
'''In this script I am trying to make utility functions 
that will help calculate relative velocities. 
I also want to place other velocity related calculations in this script'''



from typing import Self, AnyStr




# what i want to do in this utils file is
# create a simple utility so that i can perform velocity/relative velocity calculation with utmost ease
# in any dynamic system 
# 1. 





class Vel:
    def __init__(self, 
                 vel_item:str, 
                 vx, 
                 vy, 
                 vz=0, 
                 # rename to ref_frame 
                 rel2str = 'E'): # rel2str = 'E' means relative to Earth
        
        '''Initiate velocity type object.
           Provide objects name, and x,y,z velocity coordinate values,
           and reference frame string.'''
        
        self.vel_item = vel_item
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.rel2str = rel2str

        self.rel_vel_tag = self.vel_item + self.rel2str

    def update_vel(self, other_ref:Self) -> None:
        
        return None
    


    def rel_vel_str(self, other_ref):
        pass
    