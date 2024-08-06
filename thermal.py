# self
import utils

# standard
from multiprocessing.synchronize import Lock

# 3rd party
from mavsdk.telemetry import Position
from pyproj import CRS



class Thermal():
    """
    Class responsible for encapsulating the information of a thermal
    """
    def __init__(
            self, x_m: float, y_m: float, force_ms: float,
            radius_m: float, projection: CRS, lock: Lock):
        """
        Inits a thermal object

        Parameters
        ----------
        - x, y: flat
            - Coordinates in meters
        - force_ms: float
            - Force exerted by the thermal
        - radius_m: float
            - Radius of the thermal
        - projection: CRS
            - Projection standard reference
        """
        self.x = x_m
        self.y = y_m
        lat, lon = utils.xy_to_global(x_m, y_m, projection)
        self.global_position = Position(
            lat, lon, 0, 0
        )
        self.force = force_ms
        self.radius = radius_m
        self.lock = lock


def thermal_dict_to_obj(thermals: list, locks: list, projection: CRS):
    """
    Converts the thermals config dict into Thermal objects

    Parameters
    ----------
    - thermals: list
        - Thermals to be converted
    - projection: CRS
        - Projection standard reference
    """
    therms = []
    for index, t in enumerate(thermals):
        therms.append(Thermal(**t, radius_m=0, projection=projection, lock=locks[index]))
    return therms
