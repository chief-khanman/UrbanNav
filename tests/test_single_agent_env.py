#check imports 
import os
from uam_simulator import UAMSimulator
from single_agent_gym_env import UAMSimEnv




# perform all checks on gym.Env before training/testing 
CONFIG_FILE = os.path.join(os.path.dirname(__file__), '../sample_config.yaml')
print(CONFIG_FILE)
# check - env reset 

# check env connect correct LEARNING UAV to correct LEARNING UAV id 

# LEARNING UAV has correct dynamics 

# other uavs are instantiated correctly 

# if there is RA, RA is instantiated correctly 

# check 1 - correct obs space 

# check 2 - correct action space 

# check 3 - for dynamics action space to action conversion is correct 

