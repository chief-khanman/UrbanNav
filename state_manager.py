from simulator_state import SimulatorState
import datetime

class StateManager:
    def __init__(self, config):
        self.config = config
        return None
    
    
    def reset(self, ):
        uavs = self.config.uavs
        airspace = self.config.airspace
        vertiports = self.config.vertiports
        atc_state = self.config.atc_state
        timestamp = datetime.datetime.now().strftime('%m %d %Y -- %H:%M')
        step = 0
        
        self._state = SimulatorState(timestamp,
                                     step,
                                     uavs,
                                     airspace,
                                     vertiports,
                                     atc_state,
                                     external_systems)
        return None
    
    def update(self, current_state:SimulatorState):

        ##### update: current_state.STEP
        current_state.step += 1

        ##### update: current_state.UAVs #####
        # already updated using dynamics.step()

        ##### update: current_state.AIRSPACE #####

        ##### update: current_state.VERTIPORTS #####
        for vertiport in current_state.vertiports:
            # if UAV landed at vertiport == True
            # add UAV to vertiport queue 
            vertiport.update_queue()
        
        ##### update: current_state.ATC_STATE #####
        # atc_state.update() will loop through all the vertiports in atc
        # for vertiport in atc.vertiport_list:
        # if vertiport.queue is not empty:
        # for uav in vertiport.queue:
        # assign new_mission/new_goal to uav   
        current_state.atc_state.update()
        
        ##### update: current_state.EXTERNAL_SYSTEMS #####
        
        return None
    
    def get_state(self,):
        return self._state
    
    def set_state(self, state):
        self._state = state
        return None
        

    
