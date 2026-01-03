import math 
import numpy as np



def _get_reward_only_agent(self):
    # use this reward for for single_agent training,
    # progressively add newer terms to this reward function
    # as we introduce other_agent and restricted airspace 
    
    reward = 0.0


    # variables
    punishment_existence = -0.1
    heading_efficiency = 0
    progress = 0

    reward += punishment_existence

    # obs is a DICT
    obs = self._get_obs() #this returns transformed data for 'UAM-UAV'
    
    # Agent info
    current_distance = self.agent.current_position.distance(self.agent.end)
    #TODO: use current_speed as a reward parameter 
    current_speed = obs['agent_speed']
    current_heading = obs['agent_current_heading']
    current_deviation = obs['agent_deviation']

    if hasattr(self, 'previous_distance') and self.previous_distance is not None:
        progress = self.previous_distance - current_distance
        # Exponential scaling for distance factor to emphasize final approach
        distance_factor =  10000 * np.exp(-0.0005 * current_distance)
        reward += progress *  distance_factor
    self.previous_distance = current_distance

    heading_efficiency = current_deviation**2
    reward -= heading_efficiency * 10

    # Add goal reached reward (using mission_complete_distance)
    if current_distance < self.agent.mission_complete_distance:
        reward += 1000.0


    return reward















"""Old simple goal direction dense reward function with punishments for static and dynamic collision avoidance.
    
    Reward worked for goal direction task but failed to conduct collision avoidance.
    Trained on 25,000 steps with PPO.
    Removed dynamic and static collision avoidance rewards.
    Suggestion: Include speed management reward for quick convergence."""
def _get_reward_simple(self):
    #FIX: 
    # Depending on the type of obs_constructor used,
    # this method will need to choose the correct reward function
    # reward functions should be defined in mapped_env_utils 

    # """Calculate the reward for the learning agent."""
    reward = 0.0

    # Variables
    punishment_existence = -0.1
    uav_punishment_closeness = 0
    ra_punishment_closeness = 0
    heading_efficiency = 0
    progress = 0


    reward += punishment_existence
    
    # Get current state information
    obs = self._get_obs()
    current_distance = self.agent.current_position.distance(self.agent.end)
    if self.obs_space_str == 'UAM_UAV':
        # Agent info
        current_speed = obs['agent_speed']
        current_heading = obs['agent_current_heading']
        current_deviation = obs['agent_deviation']
        # UAV intruder info
        uav_intruder = obs['intruder_detected']
        dist_to_uav = obs['distance_to_intruder']
        uav_relative_heading = obs['relative_heading_intruder']
        # RA intruder info
        ra_intruder = obs['restricted_airspace_detected']
        dist_to_ra = obs['distance_to_restricted_airspace']
        ra_relative_heading = obs['relative_heading_restricted_airspace']
    
    # Progress reward
    if hasattr(self, 'previous_distance') and self.previous_distance is not None:
        progress = self.previous_distance - current_distance
        # Exponential scaling for distance factor to emphasize final approach
        distance_factor = np.exp(-current_distance / 5000)
        reward += progress * 15.0 * (1.0 + distance_factor)
    self.previous_distance = current_distance
    
    # Heading efficiency reward
    #! DELETE ref_heading --- heading_efficiency can be calculated using agent_deviation
    ref_heading = math.atan2(
        self.agent.end.y - self.agent.current_position.y,
        self.agent.end.x - self.agent.current_position.x
    )
    heading_diff = ((math.degrees(ref_heading) - math.degrees(self.agent.current_heading) + 180) % 360) - 180
    heading_efficiency = np.cos(np.deg2rad(heading_diff))

    # # UAV intruder collision avoidance
    # # No UAV intruder detection
    # if uav_intruder == 0:
    #     uav_punishment_closeness = 0
    # # UAV intruder detected 
    # else:
    #     normed_namc_distance = self.agent.nmac_radius / self.agent.detection_radius
    #     uav_punishment_closeness = -math.exp(normed_namc_distance - dist_to_uav * 10.0)
    
    # # RA intruder collision avoidance
    # # No RA intruder detection
    # if ra_intruder == 0:
    #     ra_punishment_closeness = 0
    # # RA intruder detected
    # else:
    #     #!TODO mess around with constants to put notion of relative importance for UAV versus RA collision avoidance
    #     normed_namc_distance = self.agent.namc_radius / self.agent.detection_radius
    #     ra_punishment_closeness = -math.exp(normed_namc_distance - dist_to_ra * 10.0)
    
    # # Static collision avoidance rewards
    # static_detection, static_info = self.agent.sensor.get_static_detection(self.agent)
    # if static_detection:
    #     # Strong penalty for being near restricted airspace
    #     distance_to_restricted = static_info.get('distance', float('inf'))
    #     distance_factor = 1.0 - min(1.0, distance_to_restricted / self.agent.detection_radius)
    #     reward -= distance_factor * 50.0
        
    #     # Reduce heading importance during avoidance
    #     if abs(heading_diff) < 30.0:
    #         reward += heading_efficiency * 1.0  # Reduced reward during avoidance
    # else:
    #     # Normal heading rewards when no threats
    #     if abs(heading_diff) < 5.0:
    #         reward += 5.0
    #     elif abs(heading_diff) < 30.0:
    #         reward += heading_efficiency * 3.0
    #     elif abs(heading_diff) > 90.0:
    #         reward -= 10.0
    
    # # Dynamic collision avoidance rewards
    # is_nmac, nmac_list = self.agent.sensor.get_nmac(self.agent)
    # if is_nmac:
    #     for nmac_uav in nmac_list:
    #         distance = self.agent.current_position.distance(nmac_uav.current_position)
    #         nmac_factor = 1.0 - (distance / self.agent.nmac_radius)
    #         reward -= nmac_factor * 30.0
    
    ### We're keeping this for later
    # Speed management reward
    # agent.dynamics.max_speed updated to be agent.max_speed for env compatibility
    target_speed = self.agent.max_speed
    if current_distance < 1000:
        # Reduce target speed when approaching goal
        target_speed = max(5.0, self.agent.max_speed * (current_distance / 1000))
    speed_efficiency = 1.0 - abs(current_speed - target_speed) / self.agent.max_speed
    reward += speed_efficiency * 2.0
    
    # Terminal rewards
    if current_distance < self.agent.mission_complete_distance:
        reward += 1000.0
    
    return float(reward)


    """Intermediate recent complex goal direction dense reward function.
    
    Removed dynamic collision avoidance rewards. 
    Only static collision avoidance rewards are included.
    Only tested on small number of steps with PPO.
    Ensure you add the normaliation to relative_heading parameter in UAM_UAV observation space.
    Suggestion: Test on larger number of steps, upwards of 500,000 to see if it converges."""




def _get_reward_ra(self):
    """
    Calculate the reward for the learning agent with the updated observation format.
    Handles both the goal-seeking behavior and collision avoidance.
    """
    # Base reward from existing function
    reward = 0.0

    # Key reward/punishment parameters
    punishment_existence = -0.1
    uav_punishment_closeness = 0
    ra_punishment_closeness = 0
    heading_efficiency = 0
    progress = 0

    reward += punishment_existence
    
    # Get current state information
    obs = self._get_obs()
    current_distance = self.agent.current_position.distance(self.agent.end)
    
    if self.obs_space_str == 'UAM_UAV':
        # Agent info - extract values from numpy arrays
        current_speed = obs['agent_speed'][0]
        current_heading = obs['agent_current_heading'][0]
        current_deviation = obs['agent_deviation'][0]
        
        # UAV intruder info - extract values from numpy arrays
        uav_intruder = obs['intruder_detected'][0]
        dist_to_uav = obs['distance_to_intruder'][0]
        uav_relative_heading = obs['relative_heading_intruder'][0]
        
        # Intruder position info
        intruder_position_x = obs['intruder_position_x'][0]
        intruder_position_y = obs['intruder_position_y'][0]
        intruder_speed = obs['intruder_speed'][0]
        
        # Create a Point object from coordinates for _get_nmac_state_reward function
        from shapely import Point
        if uav_intruder > 0.5:  # If intruder is detected
            intruder_position = Point(intruder_position_x, intruder_position_y)
        else:
            intruder_position = None
        
        # RA intruder info - extract values from numpy arrays
        ra_intruder = obs['restricted_airspace_detected'][0]
        dist_to_ra = obs['distance_to_restricted_airspace'][0]
        ra_relative_heading = obs['relative_heading_restricted_airspace'][0]
    
    # Progress reward
    if hasattr(self, 'previous_distance') and self.previous_distance is not None:
        progress = self.previous_distance - current_distance
        # Exponential scaling for distance factor to emphasize final approach
        distance_factor = np.exp(-current_distance / 5000)
        reward += progress * 15.0 * (1.0 + distance_factor)
    self.previous_distance = current_distance
    
    # Heading efficiency reward
    ref_heading = math.atan2(
        self.agent.end.y - self.agent.current_position.y,
        self.agent.end.x - self.agent.current_position.x
    )
    heading_diff = ((math.degrees(ref_heading) - math.degrees(self.agent.current_heading) + 180) % 360) - 180
    heading_efficiency = np.cos(np.deg2rad(heading_diff))
    reward += heading_efficiency * 0.5

    # # UAV intruder collision avoidance
    # if uav_intruder < 0.5:  # No UAV intruder detection
    #     uav_punishment_closeness = 0
    # else:  # UAV intruder detected
    #     normed_nmac_distance = self.agent.nmac_radius / self.agent.detection_radius
    #     uav_punishment_closeness = -math.exp(normed_nmac_distance - dist_to_uav * 10.0)
        
    #     # NMAC-specific state-based rewards
    #     if dist_to_uav <= self.agent.nmac_radius and intruder_position is not None:
    #         # NMAC situation detected - apply state-based rewards
    #         nmac_reward = self._get_nmac_state_reward(intruder_position, intruder_speed, dist_to_uav, uav_relative_heading)
    #         reward += nmac_reward
            
    #         # Add reward/penalty for response time
    #         if hasattr(self, 'nmac_detected_time'):
    #             # If we already detected NMAC before
    #             response_time = self.current_time_step - self.nmac_detected_time
    #             # Penalize longer response times exponentially
    #             response_penalty = -0.5 * math.exp(min(response_time / 2.0, 5.0))
    #             reward += response_penalty
    #         else:
    #             # First time detecting NMAC
    #             self.nmac_detected_time = self.current_time_step
    #     else:
    #         # If we're no longer in NMAC but were previously
    #         if hasattr(self, 'nmac_detected_time'):
    #             # Reward for successfully exiting NMAC
    #             reward += 2.0
    #             # Reset NMAC detection time
    #             delattr(self, 'nmac_detected_time')
    
    # RA intruder collision avoidance
    if ra_intruder < 0.5:  # No RA intruder detection
        ra_punishment_closeness = 0
    else:  # RA intruder detected
        normed_nmac_distance = self.agent.nmac_radius / self.agent.detection_radius
        ra_punishment_closeness = -math.exp(normed_nmac_distance - dist_to_ra * 10.0)
    
    # Add proximity penalties to total reward
    reward += uav_punishment_closeness + ra_punishment_closeness
    
    # Add goal reached reward (using mission_complete_distance)
    if current_distance < self.agent.mission_complete_distance:
        reward += 1000.0
    
    return reward

    """Most recent complex goal direction dense reward function.
    
    Only tested on small number of steps with PPO.
    Ensure you add the normaliation to relative_heading parameter in UAM_UAV observation space.
    Suggestion: Test on larger number of steps, upwards of 500,000 to see if it converges."""




def _get_reward_intruder(self):
    """
    Calculate the reward for the learning agent with the updated observation format.
    Handles both the goal-seeking behavior and collision avoidance.
    """
    # Base reward from existing function
    reward = 0.0

    # Key reward/punishment parameters
    punishment_existence = -0.1
    uav_punishment_closeness = 0
    # ra_punishment_closeness = 0
    heading_efficiency = 0
    progress = 0

    reward += punishment_existence
    
    # Get current state information
    obs = self._get_obs()
    current_distance = self.agent.current_position.distance(self.agent.end)
    
    if self.obs_space_str == 'UAM_UAV':
        # Agent info - extract values from numpy arrays
        current_speed = obs['agent_speed'][0]
        current_heading = obs['agent_current_heading'][0]
        current_deviation = obs['agent_deviation'][0]
        
        # UAV intruder info - extract values from numpy arrays
        uav_intruder = obs['intruder_detected'][0] # this returns a bool
        dist_to_uav = obs['distance_to_intruder'][0]
        uav_relative_heading = obs['relative_heading_intruder'][0]
        
        # Intruder position info
        intruder_position_x = obs['intruder_position_x'][0]
        intruder_position_y = obs['intruder_position_y'][0]
        intruder_speed = obs['intruder_speed'][0]
        
        # Create a Point object from coordinates for _get_nmac_state_reward function
        from shapely import Point
        if uav_intruder:  # If intruder is detected
            intruder_position = Point(intruder_position_x, intruder_position_y)
        else:
            intruder_position = None
        
        # RA intruder info - extract values from numpy arrays
        ra_intruder = obs['restricted_airspace_detected'][0]
        dist_to_ra = obs['distance_to_restricted_airspace'][0]
        ra_relative_heading = obs['relative_heading_restricted_airspace'][0]
    
    # Progress reward
    if hasattr(self, 'previous_distance') and self.previous_distance is not None:
        progress = self.previous_distance - current_distance
        # Exponential scaling for distance factor to emphasize final approach
        distance_factor = np.exp(-current_distance / 5000)
        reward += progress * 15.0 * (1.0 + distance_factor)
    self.previous_distance = current_distance
    
    # Heading efficiency reward
    ref_heading = math.atan2(
        self.agent.end.y - self.agent.current_position.y,
        self.agent.end.x - self.agent.current_position.x
    )
    heading_diff = ((math.degrees(ref_heading) - math.degrees(self.agent.current_heading) + 180) % 360) - 180
    heading_efficiency = np.cos(np.deg2rad(heading_diff))
    reward += heading_efficiency * 0.5

    # UAV intruder collision avoidance
    if uav_intruder:  # No UAV intruder detection
        uav_punishment_closeness = 0
    else:  # UAV intruder detected
        normed_nmac_distance = self.agent.nmac_radius / self.agent.detection_radius
        uav_punishment_closeness = -math.exp(normed_nmac_distance - dist_to_uav * 10.0)
        
        # NMAC-specific state-based rewards
        if dist_to_uav <= self.agent.nmac_radius and intruder_position is not None:
            # NMAC situation detected - apply state-based rewards
            nmac_reward = self._get_nmac_state_reward(intruder_position, intruder_speed, dist_to_uav, uav_relative_heading)
            reward += nmac_reward
            
            # Add reward/penalty for response time
            if hasattr(self, 'nmac_detected_time'):
                # If we already detected NMAC before
                response_time = self.current_time_step - self.nmac_detected_time
                # Penalize longer response times exponentially
                response_penalty = -0.5 * math.exp(min(response_time / 2.0, 5.0))
                reward += response_penalty
            else:
                # First time detecting NMAC
                self.nmac_detected_time = self.current_time_step
        else:
            # If we're no longer in NMAC but were previously
            if hasattr(self, 'nmac_detected_time'):
                # Reward for successfully exiting NMAC
                reward += 2.0
                # Reset NMAC detection time
                delattr(self, 'nmac_detected_time')
    
    # RA intruder collision avoidance
    # if ra_intruder < 0.5:  # No RA intruder detection
    #     ra_punishment_closeness = 0
    # else:  # RA intruder detected
    #     normed_nmac_distance = self.agent.nmac_radius / self.agent.detection_radius
    #     ra_punishment_closeness = -math.exp(normed_nmac_distance - dist_to_ra * 10.0)
    
    # Add proximity penalties to total reward
    reward += uav_punishment_closeness # + ra_punishment_closeness
    
    # Add goal reached reward (using mission_complete_distance)
    if current_distance < self.agent.mission_complete_distance:
        reward += 1000.0
    
    return reward





def _get_reward_complex(self):
    """
    Calculate the reward for the learning agent with the updated observation format.
    Handles both the goal-seeking behavior and collision avoidance.
    """
    # Base reward from existing function
    reward = 0.0

    # Key reward/punishment parameters
    punishment_existence = -0.1
    uav_punishment_closeness = 0
    ra_punishment_closeness = 0
    heading_efficiency = 0
    progress = 0

    reward += punishment_existence
    
    # Get current state information
    obs = self._get_obs()
    current_distance = self.agent.current_position.distance(self.agent.end)
    
    if self.obs_space_str == 'UAM_UAV':
        # Agent info - extract values from numpy arrays
        current_speed = obs['agent_speed'][0]
        current_heading = obs['agent_current_heading'][0]
        current_deviation = obs['agent_deviation'][0]
        
        # UAV intruder info - extract values from numpy arrays
        uav_intruder = obs['intruder_detected'][0] # this returns a bool
        dist_to_uav = obs['distance_to_intruder'][0]
        uav_relative_heading = obs['relative_heading_intruder'][0]
        
        # Intruder position info
        intruder_position_x = obs['intruder_position_x'][0]
        intruder_position_y = obs['intruder_position_y'][0]
        intruder_speed = obs['intruder_speed'][0]
        
        # Create a Point object from coordinates for _get_nmac_state_reward function
        from shapely import Point
        if uav_intruder:  # If intruder is detected
            intruder_position = Point(intruder_position_x, intruder_position_y)
        else:
            intruder_position = None
        
        # RA intruder info - extract values from numpy arrays
        ra_intruder = obs['restricted_airspace_detected'][0]
        dist_to_ra = obs['distance_to_restricted_airspace'][0]
        ra_relative_heading = obs['relative_heading_restricted_airspace'][0]
    
    # Progress reward
    if hasattr(self, 'previous_distance') and self.previous_distance is not None:
        progress = self.previous_distance - current_distance
        # Exponential scaling for distance factor to emphasize final approach
        distance_factor = np.exp(-current_distance / 5000)
        reward += progress * 15.0 * (1.0 + distance_factor)
    self.previous_distance = current_distance
    
    # Heading efficiency reward
    ref_heading = math.atan2(
        self.agent.end.y - self.agent.current_position.y,
        self.agent.end.x - self.agent.current_position.x
    )
    heading_diff = ((math.degrees(ref_heading) - math.degrees(self.agent.current_heading) + 180) % 360) - 180
    heading_efficiency = np.cos(np.deg2rad(heading_diff))
    reward += heading_efficiency * 0.5

    # UAV intruder collision avoidance
    if uav_intruder:  # No UAV intruder detection
        uav_punishment_closeness = 0
    else:  # UAV intruder detected
        normed_nmac_distance = self.agent.nmac_radius / self.agent.detection_radius
        uav_punishment_closeness = -math.exp(normed_nmac_distance - dist_to_uav * 10.0)
        
        # NMAC-specific state-based rewards
        if dist_to_uav <= self.agent.nmac_radius and intruder_position is not None:
            # NMAC situation detected - apply state-based rewards
            nmac_reward = self._get_nmac_state_reward(intruder_position, intruder_speed, dist_to_uav, uav_relative_heading)
            reward += nmac_reward
            
            # Add reward/penalty for response time
            if hasattr(self, 'nmac_detected_time'):
                # If we already detected NMAC before
                response_time = self.current_time_step - self.nmac_detected_time
                # Penalize longer response times exponentially
                response_penalty = -0.5 * math.exp(min(response_time / 2.0, 5.0))
                reward += response_penalty
            else:
                # First time detecting NMAC
                self.nmac_detected_time = self.current_time_step
        else:
            # If we're no longer in NMAC but were previously
            if hasattr(self, 'nmac_detected_time'):
                # Reward for successfully exiting NMAC
                reward += 2.0
                # Reset NMAC detection time
                delattr(self, 'nmac_detected_time')
    
    # RA intruder collision avoidance
    if ra_intruder < 0.5:  # No RA intruder detection
        ra_punishment_closeness = 0
    else:  # RA intruder detected
        normed_nmac_distance = self.agent.nmac_radius / self.agent.detection_radius
        ra_punishment_closeness = -math.exp(normed_nmac_distance - dist_to_ra * 10.0)
    
    # Add proximity penalties to total reward
    reward += uav_punishment_closeness + ra_punishment_closeness
    
    # Add goal reached reward (using mission_complete_distance)
    if current_distance < self.agent.mission_complete_distance:
        reward += 1000.0
    
    return reward
    
    """NMAC state-based helper reward function."""





def _get_nmac_state_reward(self, position, speed, distance, relative_heading):
    """
    Calculate reward/penalty based on NMAC state tuple:
    (quadrant, relative_speed, relative_heading)
    """
    # Get the quadrant of the intruder
    intruder_pos = position
    agent_pos = self.agent.current_position
    
    # Determine quadrant (1-4)
    dx = intruder_pos.x - agent_pos.x
    dy = intruder_pos.y - agent_pos.y
    quadrant = 0
    if dx > 0 and dy > 0:
        quadrant = 1
    elif dx < 0 and dy > 0:
        quadrant = 2
    elif dx < 0 and dy < 0:
        quadrant = 3
    else:  # dx > 0 and dy < 0
        quadrant = 4
    
    # Determine relative speed category
    intruder_speed = speed
    agent_speed = self.agent.current_speed
    speed_diff = intruder_speed - agent_speed
    
    if abs(speed_diff) < 0.5:  # Within 0.5 units of speed
        rel_speed_category = "same"
    elif speed_diff > 0:
        rel_speed_category = "faster"
    else:
        rel_speed_category = "slower"
    
    # Discretize relative heading to categories
    # 0-45: same direction
    # 46-135: perpendicular
    # 136-180: opposing direction
    abs_rel_heading = abs(relative_heading)
    if abs_rel_heading <= 45:
        heading_category = "same"
    elif abs_rel_heading <= 135:
        heading_category = "perpendicular"
    else:
        heading_category = "opposing"
    
    # Create the state tuple
    state_tuple = (quadrant, rel_speed_category, heading_category)
    
    # Define rewards/penalties for each state tuple
    state_rewards = {
        # Quadrant 1 (intruder ahead and to the right)
        (1, "same", "same"): -0.5,        # Low risk but monitoring needed
        (1, "same", "perpendicular"): -1.0,  # Medium risk - crossing paths
        (1, "same", "opposing"): -2.0,     # High risk - heading toward each other
        (1, "faster", "same"): -0.8,       # Moderate risk - intruder pulling away but in same direction
        (1, "faster", "perpendicular"): -1.5,  # High risk - fast crossing paths
        (1, "faster", "opposing"): -3.0,    # Severe risk - closing fast head-on
        (1, "slower", "same"): -0.3,       # Low risk - catching up to intruder
        (1, "slower", "perpendicular"): -0.7,  # Moderate risk - crossing paths but slower
        (1, "slower", "opposing"): -1.5,    # High risk - opposing but closing more slowly

        # Quadrant 2 (intruder ahead and to the left)
        (2, "same", "same"): -0.5,        # Low risk but monitoring needed
        (2, "same", "perpendicular"): -1.0,  # Medium risk - crossing paths
        (2, "same", "opposing"): -2.0,     # High risk - heading toward each other
        (2, "faster", "same"): -0.8,       # Moderate risk - intruder pulling away but in same direction
        (2, "faster", "perpendicular"): -1.5,  # High risk - fast crossing paths
        (2, "faster", "opposing"): -3.0,    # Severe risk - closing fast head-on
        (2, "slower", "same"): -0.3,       # Low risk - catching up to intruder
        (2, "slower", "perpendicular"): -0.7,  # Moderate risk - crossing paths but slower
        (2, "slower", "opposing"): -1.5,    # High risk - opposing but closing more slowly

        # Quadrant 3 (intruder behind and to the left)
        (3, "same", "same"): -0.2,        # Very low risk - going same direction behind
        (3, "same", "perpendicular"): -0.7,  # Moderate risk - could intersect paths
        (3, "same", "opposing"): 0.5,      # Low risk - moving away from each other
        (3, "faster", "same"): -1.0,       # Moderate to high risk - intruder catching up
        (3, "faster", "perpendicular"): -1.2,  # High risk - intruder could intercept
        (3, "faster", "opposing"): 0.3,     # Very low risk - fast separation
        (3, "slower", "same"): 0.1,        # Very low risk - pulling away from intruder
        (3, "slower", "perpendicular"): -0.3,  # Low risk - slow crossing behind
        (3, "slower", "opposing"): 0.7,     # Very low risk - both moving apart

        # Quadrant 4 (intruder behind and to the right)
        (4, "same", "same"): -0.2,        # Very low risk - going same direction behind
        (4, "same", "perpendicular"): -0.7,  # Moderate risk - could intersect paths
        (4, "same", "opposing"): 0.5,      # Low risk - moving away from each other
        (4, "faster", "same"): -1.0,       # Moderate to high risk - intruder catching up
        (4, "faster", "perpendicular"): -1.2,  # High risk - intruder could intercept
        (4, "faster", "opposing"): 0.3,     # Very low risk - fast separation
        (4, "slower", "same"): 0.1,        # Very low risk - pulling away from intruder
        (4, "slower", "perpendicular"): -0.3,  # Low risk - slow crossing behind
        (4, "slower", "opposing"): 0.7,     # Very low risk - both moving apart

        # Edge cases - immediate collision risks
        # These are special cases that represent extremely high risk scenarios
        
        # Head-on collision course
        (1, "same", "opposing"): -2.5,     # Direct collision course from front
        (2, "same", "opposing"): -2.5,     # Direct collision course from front
        
        # Fast intruder from behind
        (3, "faster", "same"): -1.5,       # Being overtaken quickly from behind
        (4, "faster", "same"): -1.5,       # Being overtaken quickly from behind
        
        # Perpendicular crossing with high speed differential
        (1, "faster", "perpendicular"): -2.0,  # Fast perpendicular crossing from front-right
        (2, "faster", "perpendicular"): -2.0,  # Fast perpendicular crossing from front-left
        
        # Special case: imminent collision (assuming this can be detected)
        # This could be identified by additional logic checking time-to-collision
        # For very small distances combined with certain headings
    }
    
    # Default penalty if state not explicitly defined
    default_penalty = -1.0
    
    # Get the reward from the dictionary, or use default penalty
    reward = state_rewards.get(state_tuple, default_penalty)
    
    # Additional scaling based on distance (more severe as distance decreases)
    # Normalize distance within NMAC radius (0 to 1)
    norm_distance = max(0.01, min(1.0, distance / self.agent.nmac_radius))
    distance_factor = 1.0 / norm_distance
    
    # Scale reward by distance factor (limited to avoid extreme values)
    distance_scaling = min(3.0, distance_factor)
    reward = reward * distance_scaling
    
    return reward