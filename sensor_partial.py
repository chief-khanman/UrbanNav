from sensor_template import Sensor



class PartialSensor(Sensor):

    def __init__(self,):
        super().__init__()



#### ---- SPATIAL HASHING ---- ####
# import math
# import numpy as np

# class SpatialHash:
#     def __init__(self, spacing, max_uavs):
#         self.spacing = spacing
#         # Table size chosen to be 2x the number of particles for fewer collisions [4]
#         self.table_size = 2 * max_uavs 
        
#         # cell_start includes the +1 guard to prevent out-of-bounds in the query [3]
#         self.cell_start = np.zeros(self.table_size + 1, dtype=int)
#         # cell_entries is the dense array [3, 4]
#         self.cell_entries = np.zeros(max_uavs, dtype=int)

#     def int_coords(self, x, y, z):
#         # Floors coordinates to map physical space into grid indices [2, 6]
#         return math.floor(x / self.spacing), math.floor(y / self.spacing), math.floor(z / self.spacing)

#     def hash_function(self, xi, yi, zi):
#         # Evenly distributes cells using bitwise XOR, mapping to an index [5, 7]
#         h = (xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481)
#         return abs(h) % self.table_size

#     def run_broad_phase_search(self, uav_dict):
#         """Builds the spatial hash data structure from the current UAV positions"""
#         # 1. Reset arrays to 0 [4, 8]
#         self.cell_start.fill(0)
#         self.cell_entries.fill(0)

#         # 2. Count UAVs per cell [4, 8]
#         for uav_id, uav in uav_dict.items():
#             xi, yi, zi = self.int_coords(uav.px, uav.py, uav.pz)
#             hash_id = self.hash_function(xi, yi, zi)
#             self.cell_start[hash_id] += 1

#         # 3. Compute partial sums (End boundaries) [1, 8]
#         # Each number will point to the last cell entry + 1
#         total_sum = 0
#         for i in range(self.table_size):
#             count = self.cell_start[i]
#             total_sum += count
#             self.cell_start[i] = total_sum
#         # Apply the final total sum to the "+1" guard index
#         self.cell_start[self.table_size] = total_sum 

#         # 4. Fill the dense array and mutate cell_start [8, 9]
#         for uav_id, uav in uav_dict.items():
#             xi, yi, zi = self.int_coords(uav.px, uav.py, uav.pz)
#             hash_id = self.hash_function(xi, yi, zi)
            
#             # Decrease the pointer and place the ID [8, 9]
#             self.cell_start[hash_id] -= 1 
#             particle_placement_id = self.cell_start[hash_id]
#             self.cell_entries[particle_placement_id] = uav_id

#     def query(self, pos, max_dist):
#         """Returns all UAV IDs in the cells surrounding the queried position"""
#         px, py, pz = pos
#         query_ids = []
        
#         # Calculate the bounding box of cells to search [1]
#         # If max_dist <= spacing, this naturally results in a 3x3x3 (27 cell) block [2, 6]
#         min_xi, min_yi, min_zi = self.int_coords(px - max_dist, py - max_dist, pz - max_dist)
#         max_xi, max_yi, max_zi = self.int_coords(px + max_dist, py + max_dist, pz + max_dist)

#         # Loop through the block of cells [1]
#         for xi in range(min_xi, max_xi + 1):
#             for yi in range(min_yi, max_yi + 1):
#                 for zi in range(min_zi, max_zi + 1):
#                     hash_id = self.hash_function(xi, yi, zi)
                    
#                     # Because cell_start was mutated, it now points to the first entry
#                     # The next index acts as the end boundary [1, 3]
#                     start = self.cell_start[hash_id]
#                     end = self.cell_start[hash_id + 1]
                    
#                     for i in range(start, end):
#                         query_ids.append(self.cell_entries[i])
                        
#         return query_ids 

#    def check_nmac(query_ids):
#       nmac_ids = []
#       for query_id in query_ids:
#           if uav.distance(self.uav_dict[query_id]) <= nmac_distance:
#               nmac_ids.append(query_id)
#       return nmac_ids
# 
#     def check_collision(nmac_ids):
#       collision_ids = []
#       for nmac_id in nmac_ids:
#           if uav.distance(self.uav_dict[nmac_id]) <= collision_distance:
#               collision_ids.append(nmac_id)
# 
#       return collision_ids 