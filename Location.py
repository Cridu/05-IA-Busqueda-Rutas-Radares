# Required imports
import numpy as np

class Location:
    """ Geodetic coordinates of a point """
    def __init__(self, latitude: np.float32, longitude: np.float32):
        self.latitude  = latitude
        self.longitude = longitude

    def to_numpy(self) -> np.array:
        """ Returns the location as an array (lat, lon) """
        return np.array([self.latitude, self.longitude], dtype=np.float32)