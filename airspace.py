import numpy as np
import shapely
import pandas as pd
import geopandas as gpd
from geopandas import GeoSeries, GeoDataFrame
from osmnx import features as ox_features
from osmnx import geocode_to_gdf as geocode_to_gdf
from osmnx import projection as ox_projection
from typing import List, Tuple, Dict
import math
from shapely import Point, Polygon
from shapely.validation import make_valid
import warnings 
import random
from sklearn.cluster import KMeans as KM 

from vertiport import Vertiport
#! FIX:
# this module will now handle creating objects in/on airspace
# vertiport creation
# restricted airspace creation
#
class Airspace: 
    
    """Primary Airspace asset and methods for adding vertiports. """
    
    def __init__(
        self,
        location_name: str,
        number_of_vertiports: int,
        vertiport_tag_list: List[Tuple[str,str]] | List = [],
        airspace_restricted_area_tag_list: List[Tuple[str,str]] | List = [], #airspace_restricted_area_tag_list: List[Tuple('building:commercial', ...)]
        buffer_radius: float = 500,
        seed=123
    ) -> None: 
        
        """ 
        Initialize the Airspace asset, 
        and if vertiport_tag_list, and airspace_restricted_area_tag_list are defined
        user can build vertiports and restricted areas using 
        vertiport_utm 
        AND 
        airspace_restricted_area_buffer_array
        
        Args:
            number_of_vertiports: total number of vertiports in the airspace for simulation
            location_name (string): Location of the Airspace ie. "Austin, Texas, USA"
            airspace_restricted_area_tag_list: Tags from OSMNx used for selecting restricted airspace
            vertiport_tag_list: Tags from OSMNx used for selecting vertiports in airspace
            buffer_radius (int): Distance around restriced airspace
            seed: for seeding random generator 

        """
        self.seed = seed
        self.location_name = location_name  #'Austin, Texas, USA'
        self.buffer_radius = buffer_radius
        self.airspace_restricted_area_tag_list = airspace_restricted_area_tag_list #airspace_restricted_area_tag_list: [('building','commercial'), ...]
        self.vertiport_tag_list = vertiport_tag_list

        location_gdf = geocode_to_gdf(self.location_name)  # converts named geocode - 'Austin,Texas' location to gdf
        input_geom = []
        temp_input_geom = location_gdf['geometry'].iloc[0]
        # print(temp_input_geom.is_valid)
        # print(type(temp_input_geom))
        # print(len(temp_input_geom.geoms))
        if not temp_input_geom.is_valid:
            temp_input_geom = shapely.GeometryCollection(make_valid(temp_input_geom))
        for geom in temp_input_geom.geoms:
            if isinstance(geom, Polygon):
                # print('Found polygon')
                input_geom.append(geom)
        
        input_geom = shapely.MultiPolygon(input_geom)
                

             

        self.location_utm_gdf: gpd.GeoDataFrame = ox_projection.project_gdf(location_gdf)  # default projection - UTM projection #! GeoDataFrame has deprication warning - need quick fix
        self.location_utm_gdf["boundary"] = (self.location_utm_gdf.boundary)  # adding column 'boundary'

        #* IMPROVEMENT PLAN: a function will create vertiports dict for given tag_list dicts - the vertiports_utm dict is used for accessing data and building veriports 
        # Airspace vertiport location data
        if self.vertiport_tag_list:  # example vertiport_tag_list: List[Tuple('building', 'commercial'), ... , ... , ... ]
            # create empty dict containers for geodataframe
            self.vertiport_tags:Dict[str,str] = {}
            self.vertiport_feat:Dict[str,GeoDataFrame] = {}
            self.vertiport_utm:Dict[str,GeoDataFrame] = {} #this is a dict of GDF
            #                                                           tag     ,  tag_value
            #                          vertiport_tag_list: List[Tuple('building', 'commercial'), ... , ... , ... ]
            for tag, tag_value in self.vertiport_tag_list:
                #assign k,v = tag_value,tag
                self.vertiport_tags[tag_value] = tag
                try:
                    with warnings.catch_warnings():
                        warnings.filterwarnings('ignore', category=RuntimeWarning)
                        feat = ox_features.features_from_polygon(input_geom, tags={tag:tag_value})
                    # print(type(feat))
                    # print(feat['geometry'])
                    #assign k,v = tag_value, GeoDataFrame - for each tag_value store corresponding GDF
                    self.vertiport_feat[tag_value] = self._fix_invalid_geometries(feat)#ox_features.features_from_polygon(location_gdf["geometry"].iloc[0], tags={tag:tag_value})
                    #convert from lat,long format to x,y format based on defaultCRS
                    self.vertiport_utm[tag_value] = ox_projection.project_gdf(self.vertiport_feat[tag_value])
                except Exception as e:
                    print(f"Warning: Failed to get features for {tag}={tag_value}: {e}")
                    continue


        #* IMPROVEMENT PLAN: a function will create restricted_airspace for given tag_list dicts
        # Airspace restricted area data
        if self.airspace_restricted_area_tag_list:
            # create empty dict container for restricted areas
            self.location_tags:Dict[str,str] = {}
            self.location_feature:Dict[str, GeoDataFrame] = {}
            self.location_utm:Dict[str, GeoDataFrame] = {}
            self.location_utm_buffer:Dict[str, GeoSeries] = {}

            self.airspace_restricted_area_buffer_array:List[GeoSeries] = []
            self.airspace_restricted_area_array:List[GeoDataFrame] = []

            for tag, tag_value in self.airspace_restricted_area_tag_list:
                self.location_tags[tag_value] = tag
                try:
                    with warnings.catch_warnings():
                        warnings.filterwarnings('ignore', category=RuntimeWarning)
                        feat = ox_features.features_from_polygon(input_geom, tags={tag:tag_value})
                    #use tag,tag_value to create geodataframe of restricted areas
                    self.location_feature[tag_value] = self._fix_invalid_geometries(feat)#ox_features.features_from_polygon(location_gdf["geometry"].iloc[0], tags={tag:tag_value})
                    #convert from long,lat to x,y based on defaultCRS
                    self.location_utm[tag_value] = ox_projection.project_gdf(self.location_feature[tag_value])
                    #apply buffer zone around restricted area polygon
                    self.location_utm_buffer[tag_value] = self.location_utm[tag_value].buffer(self.buffer_radius)
                    
                    #! why feed the same information above into a list
                    self.airspace_restricted_area_array.append(self.location_utm[tag_value])
                    self.airspace_restricted_area_buffer_array.append(self.location_utm_buffer[tag_value])
                except Exception as e:
                    print(f'Warning: Failed to get features for {tag}={tag_value}: {e}')
                    continue

            #! who is using these two variables
            self.restricted_airspace_buffer_geo_series = pd.concat(self.airspace_restricted_area_buffer_array)
            self.restricted_airspace_geo_series = pd.concat(self.airspace_restricted_area_array)

        # Vertiport data
        self.max_num_vps_airspace = number_of_vertiports #! change the name of this variable
        self.vertiport_list:List[Vertiport] = []
        self.polygon_dict:Dict[str,List[Polygon]] = {} #key,value = str, Polygon #! where and why is this needed 

        return None

    def __repr__(self) -> str:
        return "Airspace({location_name})".format(location_name=self.location_name)
    
    def get_state(self,):
        """
        Returns the list of vertiports.
        Airspace state is current vertiports. 
        Returns:
            List: The list of vertiports.
        """
        return self.vertiport_list

    def _fix_invalid_geometries(self, gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
        """Fix invalid geometries in a GeoDataFrame."""
        if gdf.empty:
            return gdf
        
        invalid_mask = ~gdf.geometry.is_valid
        if invalid_mask.any():
            gdf.loc[invalid_mask, 'geometry'] = gdf.loc[invalid_mask, 'geometry'].map(lambda geom: make_valid(geom) if geom is not None else geom)
        
        # Remove None geometries
        gdf = gdf[gdf.geometry.notna()]
        return gdf

    def set_vertiport(self,vertiport) -> None:
        """
        Adds a vertiport to the vertiport list.

        Args:
            vertiport: The vertiport to add.
        
        Returns:
            None
        """
        if len(self.vertiport_list) < self.max_num_vps_airspace:
            self.vertiport_list.append(vertiport)
        else:
            print('Max number of vertiports reached, additonal vertiports will not be added')
        return None 

    def set_vps_in_vp_list(self, vp_list:List[Vertiport]) -> None:
        for vp in vp_list:
            self.set_vertiport(vp)
        return None
    
    def set_random_sample_vertiports(self, vertiports:List[Vertiport], sample_number=None) -> None:
        '''Given a list of vertiports, 
            add randomly sampled 'sample_number' of vertiports 
            to airspace's vertiport_list'''

        if sample_number:
            sampled_vertiports = random.sample(vertiports, sample_number)
        else:
            raise UnboundLocalError('sample_number is NONE') 

        for vertiport in sampled_vertiports:
            self.set_vertiport(vertiport)

        return None

    def get_vertiport_list(self) -> List[Vertiport]:
        """
        Returns the list of vertiports.

        Returns:
            List: The list of vertiports.
        """
        return self.vertiport_list

    def get_vp_id_list(self,) -> List[int]:
        '''Returns list of vertiport ids'''

        vp_id_list = [vp.id for vp in self.vertiport_list]
        return vp_id_list
    
    def create_vertiport_at_location(self, location:Tuple)-> Vertiport:
        """Create a vertiport at position(x,y)."""
        position = Point(location[0], location[1])
        
        # if Airspace instance has defined restricted airspace 
        if self.airspace_restricted_area_tag_list:
            sample_space = self.location_utm_gdf['geometry'].iloc[0]
            for tag_value in self.location_tags.keys():
                # sample_space_geoseries:GeoSeries = self.location_utm_gdf.iloc[0,0]
                sample_space = shapely.difference(sample_space, self.location_utm_buffer[tag_value].union_all())
                
            sample_space_gdf = GeoSeries(sample_space)
        else: 
            sample_space = self.location_utm_gdf
            sample_space_gdf = sample_space.geometry

        sample_space_array: np.ndarray = shapely.get_parts(sample_space_gdf)

        for sample in sample_space_array:
            if sample.contains(position):
                print('Valid location for vertiport at: ', position)
                _vertiport = Vertiport(position)
                return _vertiport
            
        raise RuntimeError('Not a valid location')

    def _convert_lat_long_xy(self, lat_long:Tuple[float, float]) -> Tuple[float, float]:
        x,y = 0,0
        # return x,y
        raise NotImplementedError
    
    def create_vertiport_from_lat_long(self, lat_long:Tuple) -> Vertiport:
        x,y = self._convert_lat_long_xy(lat_long)
        vertiport = Vertiport(Point(x,y))
        return vertiport 
    
    def create_vertiport_from_polygon(self,polygon:Polygon) -> Vertiport:
        '''Given a polygon, find the centeroid of the polygon, 
        and place a vertiport at that polygon'''

        poly_centeroid = polygon.centroid
        return Vertiport(poly_centeroid)

    def create_vertiports_from_polygons(self,polygon_list:List[Polygon]) -> List[Vertiport]:
        '''Use polygons from polygon_list to create vertiports at each polygon'''

        vertiport_list = []
        for polygon in polygon_list:
            vertiport_list.append(self.create_vertiport_from_polygon(polygon))
        return vertiport_list

    def _make_polygon_dict(self, tag_str) -> None:
        # TODO: check if tag_str in tag_list
        # if True, then use tag_str as key for dict

        '''Internal method for building vertiports using vertiport_tags. 
        Add polygons of specific "tag_str" to an local dictionary.
        These polygons will be used to create vertiports using OSMNx tags'''

        #                                                               tag_str: 'commercial' etc.
        try:
            assert tag_str in self.vertiport_tags.keys()
        except:
            raise AssertionError('airspace - _make_polygon_dict() is not using correct tag_str')

        self.polygon_dict[tag_str] = [obj for obj in self.vertiport_utm[tag_str].geometry if isinstance(obj, Polygon)]

        return None

    def assign_region_to_vertiports(self, vertiport_list:List[Vertiport], num_regions) -> List[Vertiport]:
        # TODO: this needs to be an internal method

        '''Using K-Means, assign regions to each vertioport from vertiport list. '''

        location_tuple = [(vertiport.x, vertiport.y) for vertiport in vertiport_list]
        #           n_clusters needs to be a variable
        kmeans = KM(n_clusters=num_regions, random_state=0, n_init="auto").fit(location_tuple)

        # print(f' These are the labels: {np.unique(kmeans.labels_)}')

        for i in range(len(kmeans.labels_)):
            vertiport = vertiport_list[i]
            # assigning region to vertiport
            vertiport.region = kmeans.labels_[i]

        return vertiport_list

    def assign_vertiports_to_regions(self, vertiport_list:List[Vertiport], num_regions:int) -> Dict:

        region_vertiport_dict = {}
        for region_id in range(num_regions):
            region_vertiport_dict[region_id] = []
            for vertiport in vertiport_list:
                if vertiport.region == region_id:
                    region_vertiport_dict[region_id].append(vertiport)

        return region_vertiport_dict

    def _sample_vertiport_from_region(self, region_dict:Dict[int, List[Vertiport]], n_sample_from_region:int = 1) -> List[Vertiport]:
        '''From the dictionary of regions with vertiports, 
        sample "n_sample_from_region" number of vertiports from vertiports list of that region'''

        sampled_vertiports = []

        for region in region_dict.keys():
            sampled_vertiports += random.sample(region_dict[region], n_sample_from_region)

        return sampled_vertiports

    # VERTIPORT CREATION - OPTION 1
    def add_n_random_vps_to_vplist(self, num_vertiports: int) -> None:
        """
        Creates a specified number of random vertiports within the airspace.

        Args:
            num_vertiports (int): The number of vertiports to create.

        Returns:
            None

        Side Effects:
            - Creates the vertiports and updates the vertiports in the airspace list.
        """

        # Set seed if provided
        if self.seed is not None:
            print(f"Creating vertiports with seed: {self.seed}")
            random.seed(self.seed)
            np.random.seed(self.seed)

        if num_vertiports > len(self.vertiport_list) - self.max_num_vps_airspace:
            raise RuntimeError('Exceeds max vertiport number defined for airspace, reduce number of vertiports to be added to vertiport_list')

        if self.airspace_restricted_area_tag_list:
            sample_space = self.location_utm_gdf['geometry'].iloc[0]
            for tag_value in self.location_tags.keys():
                sample_space = shapely.difference(sample_space, self.location_utm_buffer[tag_value].union_all())
            sample_space_gdf = GeoSeries(sample_space)
        else: 
            sample_space = self.location_utm_gdf
            sample_space_gdf = sample_space.geometry

        sample_vertiport: GeoSeries = sample_space_gdf.sample_points(num_vertiports, rng=self.seed)#TODO: change seed to rng, to avoid warning 
        sample_vertiport_array: np.ndarray = shapely.get_parts(sample_vertiport[0])

        for location in sample_vertiport_array:
            self.vertiport_list.append(
                Vertiport(location=location, uav_list=[])
            )

        print(f"Created {len(self.vertiport_list)} vertiports with seed {self.seed}")

    # VERTIPORT CREATION - OPTION 2
    def add_vps_from_regions_to_vplist(self, tag_str, num_regions, n_sample_from_region):
        '''create vertiports and update vertiport_list by adding,
            n_sample_from_region vertiports to vertiport_list'''

        # TODO: place a check
        # check if self.polygon_dict is an attribute if not DO SOMETHING -- ??
        try: 
            assert hasattr(self, 'polygon_dict')
        except:
            AttributeError("Missing polygon_dict, __init__'s vertiport_tag_list is empty")
        # step 1 - makes self.poly_dict
        self._make_polygon_dict(tag_str)
        # step 2
        vertiport_list = self.create_vertiports_from_polygons(self.polygon_dict[tag_str])

        # step 3
        vertiport_list_with_region = self.assign_region_to_vertiports(vertiport_list, num_regions)

        # step 4
        regions_dict = self.assign_vertiports_to_regions(vertiport_list_with_region, num_regions)
        # for region in regions_dict.keys():
        #     print(f'Region {region} has {len(regions_dict[region])} vertiports')

        # step 5
        self.vertiport_list += self._sample_vertiport_from_region(regions_dict, n_sample_from_region)

        return None

    def make_regions_dict(self, tag_str:str, num_regions:int):
        '''Using tag_str, and num_region, make an airspace dict attribute that hold regions and vertiports'''
        # TODO: place a check
        # check if self.polygon_dict is an attribute if not DO SOMETHING -- ??
        try: 
            assert hasattr(self, 'polygon_dict')
        except:
            AttributeError("Missing polygon_dict, __init__'s vertiport_tag_list is empty")
        # step 1 - makes self.poly_dict
        self._make_polygon_dict(tag_str)
        # step 2
        vertiport_list = self.create_vertiports_from_polygons(self.polygon_dict[tag_str])
        print('Adding vertiports to vertiport_list...')
        time.sleep(1)
        self.set_vps_in_vp_list(vertiport_list)

        # step 3
        vertiport_list_with_region = self.assign_region_to_vertiports(vertiport_list, num_regions)

        # step 4
        #! vertiports are not added to vertiport_list 
        self.regions_dict =  self.assign_vertiports_to_regions(vertiport_list_with_region, num_regions)

        self.num_regions = len(self.regions_dict.keys())

        return None

    def get_random_vertiport_from_region(self, region) -> List[Vertiport]:
        # ensure self.regions_dict is present
        if not hasattr(self, 'regions_dict'):
            raise RuntimeError('Cannot execute method, initialize attr: regions_dict with make_regions_dict()')
        print(f'Retriving vertiport from region: {region}')
        vertiport_list_of_region = self.regions_dict[region]
        return random.sample(vertiport_list_of_region, k=1)    

    def fill_vertiport_from_region(self, partial_vertiport_list):
        # find how many regions there are for this env
        required_vertiports = self.num_regions
        # determine how many vertiports need to be collected
        region_index_for_sampling =  len(partial_vertiport_list)
        if region_index_for_sampling: 
            for region in range(region_index_for_sampling, required_vertiports):
                vertiport = random.sample(self.regions_dict[region], k=1)[0] #! random.sample() returns a list
                partial_vertiport_list.append(vertiport)

        complete_list_vertiport = partial_vertiport_list
        # print(f'In file airspace.fill_vertiport_from_region(), printing complete_list_vertiport{complete_list_vertiport}')
        return complete_list_vertiport

        # using the previous information about
        # how many vertiports there are in partial_vertiport_list and
        # how many more I need
        #
        # I will determine the current region to sample from
        # and fill the remaining requirement for vertiport

    def set_vertiport_list_vp_design(self, complete_vertiport_list):
        #! why is this +=, that would mean argument is added to previous self.vertiport,
        #! complete_vertiport_list consists of all required vertiports for running map_env simulation
        #! complete_vertiport_list comes from airspace.fill_vertiport_from_region()
        self.vertiport_list = complete_vertiport_list 
        return None 

    def get_vertiports_of_region(self, region):
        vertiports = self.regions_dict[region]
        return vertiports

    def remove_vp(self, vertiport):
        '''remove vertiport from list'''
        pass

if __name__ == '__main__':
    print('Starting Airspace class instance ...')
    import time 
    import matplotlib.pyplot as plt 
    time.sleep(1)
    
    airspace = Airspace(number_of_vertiports=28, #! what is the use of this argument/attr 
                        location_name="Austin, Texas, USA", 
                        airspace_restricted_area_tag_list=[], 
                        vertiport_tag_list=[('building', 'commercial')])
    
    
    print('Number of vertiports: ',airspace.max_num_vps_airspace)
    print('vertiport list: ', airspace.vertiport_list)
    print('polygon dict: ', airspace.polygon_dict)
    print()
    print(airspace.location_utm_gdf.crs)
    print()

    print(airspace.location_utm_gdf.columns)
    # airspace.set_vertiport(Vertiport(location=Point(60000,300000)))
    # print(airspace.vertiport_list)
    # print('calling get_vertiport_list(): ', airspace.get_vertiport_list())

    # airspace.create_vertiport_at_location(location=(620000, 3340000))
    # print('calling get_vertiport_list(): ', airspace.get_vertiport_list())
    fig, ax = plt.subplots()
    airspace.location_utm_gdf.plot(ax=ax)
    ax.plot()
    plt.show()
    
    # airspace.make_regions_dict('commercial', 5)
    # print(airspace.regions_dict)
    # print(airspace.get_random_vertiport_from_region(4))
    # print('Vertiports in airspace: ', airspace.get_vertiport_list())

    print()
    print('Closing Airspace ...')
    # airspace.create_vertiports_from_regions('commercial', num_regions=5, n_sample_from_region=2)
    # for vertiport in airspace.get_vertiport_list():
    #     print(vertiport)
    #     print(vertiport.region)
