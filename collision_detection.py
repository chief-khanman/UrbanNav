# collision detection algorithm for UrbanNav
# broad phase - spatial hashing 
# narrow phase - 2D: shapely.intersect, 3D: pyvista.boolean_intersection.number_cells
from typing import Tuple, List
from uav_template import UAV_template

class AdaptiveCollisionDetector:
    def __init__(self, ):
        pass
    # create for both 2D and 3D 
    def position_2_grid(self,):
        pass

    # hashing function
    def grid_2_hash_index(self):
        pass

    def _spatial_hash(self,coords:Tuple[float, float, float]) -> float:
        self.position_2_grid()
        self.grid_2_hash_index()
        hash_value = coords[0] * coords[1] * coords[2]
        return hash_value

    def create_hash_array(self,):
        pass

    def query_hash_array(self):
        pass





    def narrow_phase_collision_check(self, uav_list:List[UAV_template]) -> List[UAV_template]:
        collided_uav = []
        return collided_uav
    
    def detect_collisions(self, uavs:List[UAV_template]) -> List[UAV_template]:
        collided_uav = []
        # broad phase 
        # narrow phase
        
        return collided_uav



        


