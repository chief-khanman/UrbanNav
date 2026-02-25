from collections import deque

from shapely import Point
from typing import List

# WORKING:  --- Feb 25, 2026
class Vertiport:
    def __init__(self, location: Point, uav_id_list: list = []) -> None:
        self.id = id(self)
        self.location = location
        self.uav_id_list: List = uav_id_list
        # vertiport capacity 
        self.landing_takeoff_capacity = 4
        # vertiport region id/number
        self.region = None
        # passenger arrival rate - an exponential distribution learned from metro data
        
        #landing queue
        self.landing_queue: deque = deque()
        #takeoff queue
        self.takeoff_queue: deque = deque()
    
    def __repr__(
        self,
    ) -> str:
        return "Vertiport({location}, {uav_list})".format(
            location=self.location, uav_list=self.uav_id_list
        )
    
    # Add x and y properties that delegate to the location Point object for rendering
    @property
    def x(self):
        return self.location.x
    
    @property
    def y(self):
        return self.location.y

    def get_landing_queue(self):
        '''Return list of UAV_id waiting to land.'''
        return self.landing_queue


    def get_takeoff_queue(self):
        '''Return list of UAV_id waiting to take off'''
        return self.takeoff_queue


    def get_uav_list(self,):
        pass

    def update_queue(self,):
        pass
    

if __name__ == '__main__':
    random_vertiport = Vertiport(Point(12,13))
    print(random_vertiport.location)