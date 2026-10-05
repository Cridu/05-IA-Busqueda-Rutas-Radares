# Required imports
import numpy as np

class Boundaries:
    """ Geodetic boundaries of the area covered by the map """
    def __init__(self, max_lat: np.float32, min_lat: np.float32,
                       max_lon: np.float32, min_lon: np.float32):
        self.max_lat = max_lat
        self.min_lat = min_lat
        self.max_lon = max_lon
        self.min_lon = min_lon