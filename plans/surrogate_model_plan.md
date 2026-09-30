# Date: Sep 29, 2026


## Surrogate Model 
1. check current state of surrogate model 
2. 

## Vertiport Design Env and Simulator connection - 
when running vertiport design env - the following are disconnected :
1. demand_model.py#179 get_episode_metrics(), is not connected with simulator, and vp design, moreover, the simple reward function is looking for a distance based metric, we are not collecting/designed distance based metric in get_episode_metrics() - we could add the following - pair_dist, pair_avg_speed, etc 
 
2. Access/Egress time - for each region there are multiple zones, find the access and egress times from each zone to selected zone of a region. 
For each zone, compute the ground access time to its region's selected vertiport on the OSMnx drive network (add_edge_speeds / add_edge_travel_times), and do the same for egress at the destination. This doesn't need the simulator. It's a static shortest-path computation you can cache per candidate.

3. restating issue with get_episode_metric() - comment in simulator_manager.py #314-316 state its use, but we have not integrated it yet. 

4. graph_builder.py - this script is for building graph object of vertiport network, which will be used by GNN-RL algorithm, the GNN-RL algorithm that we use for selecting the vertiports. all the methods defined in this script is used in vertiport_design_env.py. To make the GNN-RL generalizeable to many different maps - <answer in claude project UrbanNav/vertiport placement optimization metrics>

5. MCTS for vertiport selection has not been implemented 