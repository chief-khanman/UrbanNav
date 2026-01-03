# import modules 
import rvo2
from uav_v2_template import UAV_v2_template
from controller_template import ControllerTemplate
import shapely
from typing import List, Dict 





# connect this file with all ORCA agents 
class RVO2_simulator:
    def __init__(self, timestep,radius,max_speed, mapped_env_orca_agent_list:List[UAV_v2_template] = None):
        # create the ORCA sim 
        self.rvo2_sim = rvo2.PyRVOSimulator(# the arguments should be added to __init__ above
                                        timestep, #sim timestep 
                                        3*radius, # neighborDist - 3 times of radius
                                        10, #  maxNeighbor - number of neighbor to keep in account when performing collision avoidance 
                                        5, # time horizon other agents - secondss use the website for definition: https://gamma.cs.unc.edu/RVO2/documentation/2.0/params.html 
                                        5, # time horizon obstacles - seconds
                                        radius, # size of UAV
                                        max_speed, # max speed of UAV
                                        )
        
        # list of restricted areas in env/map
        self.orca_polygon_list = []
        # UAV list from MAPPED_ENV          List[UAV, UAV, ...]
        self.mapped_env_orca_agent_list = mapped_env_orca_agent_list 
        
        # RVO2_SIM - orca agent dict
        #                        key        :     value
        #                        str(uav_id), RVO2_agent_number
        self.orca_agent_id_2_RVO2_number:Dict = {}
        
        # MAPPING between env/sim uav_id_str and env/sim UAV_obj
        #
        #                                    env/sim uav_id_str,   env/sim UAV_obj                                            
        self.orca_agent_id_to_mapped_agent_obj:Dict[str, UAV_v2_template] = {}


    

    def step(self):
        '''This method needs to talk with MappedEnv, 
           Internally updates the position of UAV to MappedEnv'''
        # step moves all agents in ORCA/RVO2
        self.rvo2_sim.doStep()

        # update prefVelocity to all agents 
        # the following 'for-loop' takes ORCA agents in env/sim
        # collects their current position
        # sets prefVelocity based on current position and end in RVO2 sim
        #   env/sim agent_id_str,   RVO2_agent_number             dict(), this dict maps env/sim to RVO2_sim
        for orca_agent_key, orca_agent_value in self.orca_agent_id_2_RVO2_number.items():
            
            mapped_agent = self.orca_agent_id_to_mapped_agent_obj[orca_agent_key]
            # collect the current position of agent in RVO2
            # use this current position and update current_position of ORCA_UAV in env/sim
            orca_agent_current_position = self.rvo2_sim.getAgentPosition(self.orca_agent_id_2_RVO2_number[orca_agent_key])
            
            #CONVERT - TUPLE TO POINT
            orca_agent_current_position = shapely.Point(orca_agent_current_position)
            
            # updating the MAPPED_ENV UAV agent's current_position
            mapped_agent.current_position = orca_agent_current_position
            
            #
            self.rvo2_sim.setAgentPrefVelocity(orca_agent_value, self.getPrefVelocity(mapped_agent))

        # get agent current position and update orca_agent_id_2_RVO2_number


        return None


    def reset(self,):
        '''Collect information from MappedEnv
           There is no information that needs to go back from this method to MappedEnv'''
        
        
        # ADD agents to RVO2_SIM
        #           UAV           in        List[UAV, UAV, ... ] 
        for mapped_env_orca_agent in self.mapped_env_orca_agent_list:
            # collecting start and end point of UAV 
            (x,y) = mapped_env_orca_agent.start.x, mapped_env_orca_agent.start.y 
            
            # key string - using mapped_agent.id 
            uav_id_str = str(mapped_env_orca_agent.id)
            # mapping agent uav_id_str to mapped_agent
            self.orca_agent_id_to_mapped_agent_obj[uav_id_str] = mapped_env_orca_agent
            # adding agent to PyORCA/RVO2 sim
            self.orca_agent_id_2_RVO2_number[uav_id_str] = self.rvo2_sim.addAgent((x,y)) # <- return: number of RVO2_agent in RVO2 sim
        

        # ADD OBSTACLE IN RVO2_SIM - restriced airspace polygons 
        for orca_polygon in self.orca_polygon_list:
            self.rvo2_sim.addObstacle(orca_polygon)
            self.rvo2_sim.processObstacles()
        
        # ASSIGN prefVelocity IN RVO2_SIM - to agents
        # orca_agent_key:id_str, orca_agent_value:RVO2_sim_agent_number
        for orca_agent_key, orca_agent_value in self.orca_agent_id_2_RVO2_number.items():
            
            # UAV        =      Dict[key:id_str]
            mapped_agent = self.orca_agent_id_to_mapped_agent_obj[orca_agent_key]
            #                                  RVO2_sim_agent_no,     getPrefVelocity(UAV)
            self.rvo2_sim.setAgentPrefVelocity(orca_agent_value, self.getPrefVelocity(mapped_agent)) 
        
        return None
    
    def addAgent(self, new_agent:UAV_v2_template):
        # add new_agent to ORCA_agent_list - List[UAV]
        new_agent_id_str = str(new_agent.id)
        self.mapped_env_orca_agent_list.append(new_agent)
        # add new agent dicts
        # add to orca_agent_id_to_mapped_agent_obj
        self.orca_agent_id_to_mapped_agent_obj[new_agent_id_str] = new_agent
        # collect start point of new_agent
        (x,y) = new_agent.start.x, new_agent.start.y
        # add to orca_agent_id_to_RVO2_number
        self.orca_agent_id_2_RVO2_number[new_agent_id_str] = self.rvo2_sim.addAgent((x,y))

        return None
        
    def getPrefVelocity(self, agent:UAV_v2_template):
        '''Return the prefVelocity tuple for RVO2 sim'''
        
        current_position = agent.current_position
        goal = agent.end
        goalDirection_x, goalDirection_y = goal.x - current_position.x, goal.y - current_position.y
        # norm_goalDirection = goalDirection/shapely.distance(goal, current_position) #need to define magnitude
        magnitude = shapely.distance(goal, current_position)
        norm_x, norm_y = goalDirection_x/magnitude, goalDirection_y/magnitude

        # prefVelocity = norm_goalDirection * agent.max_speed
        prefVelocity_x, prefVelocity_y = norm_x*agent.max_speed, norm_y*agent.max_speed
        prefVelocityTuple = (prefVelocity_x, prefVelocity_y)
        return prefVelocityTuple #prefVelocity has to be a tuple
    
    def set_polygon_coords(self, poly_list):
        '''Call this method in Mapped_env's reset 
        ASSIGN argument - mapped_env.restricted_airspace_buffer_geo_series'''

        # poly-list is [poly1, poly2, poly3, poly4, poly5, ...... poly_n]
        # poly_n = [[x1 y1], [x2 y2], ....[xn yn]]
        
        # TODO: given a polygon list, I will form a rectangle around the polygon
        # TODO: remember must use CCW, counter clockwise direction for OBSTACLE polygon coordinates
        for i in range(len(poly_list)):
            airspace_polygon = poly_list[i]
            orca_polygon = []
            for coords in shapely.get_coordinates(airspace_polygon):
                tuple_coords = (coords[0], coords[1])
                orca_polygon.append(tuple_coords)
            self.orca_polygon_list.append(orca_polygon)
        
        return None




