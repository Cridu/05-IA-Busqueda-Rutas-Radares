# Required imports
import numpy as np
from Location import Location
from Boundaries import Boundaries
from Radar import Radar
from tqdm import tqdm

# Constant that avoids setting cells to have an associated cost of zero
EPSILON = 1e-4

class Map:
    """ Class that models the map for the simulation """
    def __init__(self, 
                 boundaries: Boundaries,
                 height:     np.int32, 
                 width:      np.int32, 
                 radars:     np.array=None):
        self.boundaries = boundaries        # Boundaries of the map
        self.height     = height            # Number of coordinates in the y-axis
        self.width      = width             # Number of coordinates int the x-axis
        self.radars     = radars            # List containing the radars (objects)

    def generate_radars(self, n_radars: np.int32) -> None:
        """ Generates n-radars randomly and inserts them into the radars list """
        # Select random coordinates inside the boundaries of the map
        lat_range = np.linspace(start=self.boundaries.min_lat, stop=self.boundaries.max_lat, num=self.height)
        lon_range = np.linspace(start=self.boundaries.min_lon, stop=self.boundaries.max_lon, num=self.width)
        rand_lats = np.random.choice(a=lat_range, size=n_radars, replace=False)
        rand_lons = np.random.choice(a=lon_range, size=n_radars, replace=False)
        self.radars = []        # Initialize 'radars' as an empty list

        # Loop for each radar that must be generated
        for i in range(n_radars):
            # Create a new radar
            new_radar = Radar(location=Location(latitude=rand_lats[i], longitude=rand_lons[i]),
                              transmission_power=np.random.uniform(low=1, high=1000000),
                              antenna_gain=np.random.uniform(low=10, high=50),
                              wavelength=np.random.uniform(low=0.001, high=10.0),
                              cross_section=np.random.uniform(low=0.1, high=10.0),
                              minimum_signal=np.random.uniform(low=1e-10, high=1e-15),
                              total_loss=np.random.randint(low=1, high=10),
                              covariance=None)

            # Insert the new radar
            self.radars.append(new_radar)
        return
    
    def get_radars_locations_numpy(self) -> np.array:
        """ Returns an array with the coordiantes (lat, lon) of each radar registered in the map """
        locations = np.zeros(shape=(len(self.radars), 2), dtype=np.float32)
        for i in range(len(self.radars)):
            locations[i] = self.radars[i].location.to_numpy()
        return locations
    
    def compute_detection_map(self) -> np.array:
        """ Computes the detection map for each coordinate in the map (with all the radars) """
        lat_range = np.linspace(self.boundaries.min_lat, self.boundaries.max_lat, self.height)
        lon_range = np.linspace(self.boundaries.min_lon, self.boundaries.max_lon, self.width)
        lon_grid, lat_grid = np.meshgrid(lon_range, lat_range)      # Both of shape (H, W)

        detection_map = np.zeros(shape=(self.height, self.width), dtype=np.float64)

        for radar in tqdm(self.radars, desc="Computing detection map"):
            # Radar equation (1): maximum range in meters
            max_range = radar.compute_max_range()

            # Differences (in degrees) w.r.t. the radar and approximate distance (6)
            d_lat = lat_grid - radar.location.latitude
            d_lon = lon_grid - radar.location.longitude
            distance = np.sqrt(d_lat ** 2 + d_lon ** 2) * 111000

            # 2D gaussian (2), evaluated over the whole grid at once
            inv_cov = np.linalg.inv(radar.covariance)
            det_cov = np.linalg.det(radar.covariance)
            exponent = -0.5 * (inv_cov[0, 0] * d_lat ** 2
                               + (inv_cov[0, 1] + inv_cov[1, 0]) * d_lat * d_lon
                               + inv_cov[1, 1] * d_lon ** 2)
            psi = np.exp(exponent) / (2.0 * np.pi * np.sqrt(det_cov))

            # Equation (4): outside the range the detection value is 0
            psi = np.where(distance <= max_range, psi, 0.0)

            # Equation (3): maximum over all radars
            detection_map = np.maximum(detection_map, psi)

        # Equation (8): MinMax scaling with epsilon
        psi_min, psi_max = detection_map.min(), detection_map.max()
        if psi_max == psi_min:
            return np.full_like(detection_map, EPSILON)
        return (detection_map - psi_min) / (psi_max - psi_min) * (1 - EPSILON) + EPSILON