from dataclasses import dataclass, asdict
import json
from typing import List, Any, Dict
from controller import ControllerManifest
from airspace import Airspace
from uav import UAV_template
from atc import ATC
from vertiport import Vertiport

@dataclass
class SimulatorState:
    """Complete state snapshot - can be serialized to JSON"""
    timestamp: float|str
    step: int
    uavs: List[UAV_template] # use UAV type
    airspace: Airspace # use Airspace type
    vertiports: List[Vertiport] # use vertiport type
    atc_state: ATC # use ATC type 
    external_systems: Dict[str, Any]  # wind, obstacles, etc.
    
    #
    def to_json(self) -> str:
        '''states are saved for rendering'''
        return json.dumps(asdict(self))
    
    @classmethod
    def from_json(cls, json_str: str):
        '''to continue simulation from specific state'''
        return cls(**json.loads(json_str))
    
    def filter_for_controller(self, manifest: ControllerManifest) -> Dict[str, Any]:
        """Extract only the state information a controller needs"""
        filtered = {}
        
        if 'uav_states' in manifest.required_state_keys:
            # Only include UAVs this controller cares about
            filtered['uav_states'] = [
                u for u in self.uavs 
                if u['id'] in manifest.controlled_uav_ids
            ]
        #TODO: develop _get_nearby_uavs()
        if 'nearby_uavs' in manifest.required_state_keys:
            # Include UAVs within sensor range
            filtered['nearby_uavs'] = self._get_nearby_uavs(
                manifest.controlled_uav_ids
            )
        
        if 'airspace' in manifest.required_state_keys:
            filtered['airspace'] = self.airspace
        
        if 'wind' in manifest.required_state_keys:
            filtered['wind'] = self.external_systems.get('wind', {})
        
        return filtered
