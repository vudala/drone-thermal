# standard
import pickle
import math
import time

# 3rd party
from mavsdk.telemetry import Position, VelocityNed
from pyproj import CRS, Transformer


def obj_to_bytearray(obj: object):
    """
    Turns an Object into a byte list to be used as ByteMultiArray by ROS2

    Parameters
    ----------
    - obj: Object
        - Any object

    Return
    ------
    - data: list
    """
    blob = pickle.dumps(obj)
    data = []
    for x in blob:
        data.append(x.to_bytes(1, 'little'))
    return data


def bytearray_to_obj(arr: list):
    """
    Turns a byte list back to an object

    Parameters
    ----------
    - arr: list
        - Array of bytes

    Return
    ------
    - obj: Object
        - An unserialized object
    """
    blob = b''.join(arr)
    return pickle.loads(blob)


def latlonhei_to_xyz(lat_rad: float, lon_rad: float, hei_cm: float):
    """
    Convert latitude, longitude and height to XYZ coordinates in centimeters

    Parameters
    ----------
    - lat_rad: float
        - Latitude in radians
    - lon_rad: float
        - Longitude in radians
    - hei_cm: float
        - Height in centimeters

    Return
    ------
    - (x, y, z): (float, float, float) 
    """
    EARTH_RADIUS_CENTIMETERS = 637100000.0
    R = EARTH_RADIUS_CENTIMETERS

    x = (R + hei_cm) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (R + hei_cm) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (R + hei_cm) * math.sin(lat_rad)

    return x, y, z


def distance_cm(pos1: Position, pos2: Position):
    """
    Calculates the distance between two points in centimeters

    Parameters
    ----------
    - pos1: Position
        - First point
    - pos2: Position
        - Second point

    Return
    ------
    - distance: float
        - Distance in centimeters
    """
    lat1, lat2 = (pos1.latitude_deg, pos2.latitude_deg)
    lon1, lon2 = (pos1.longitude_deg, pos2.longitude_deg)
    hei1, hei2 = (pos1.relative_altitude_m, pos2.relative_altitude_m)

    lat1, lat2 = (math.radians(lat1), math.radians(lat2))
    lon1, lon2 = (math.radians(lon1), math.radians(lon2))

    x1, y1, z1 = latlonhei_to_xyz(lat1, lon1, hei1 * 100.0)
    x2, y2, z2 = latlonhei_to_xyz(lat2, lon2, hei2 * 100.0)

    d = math.sqrt(
        math.pow((x2 - x1), 2.0) +
        math.pow((y2 - y1), 2.0) +
        math.pow((z2 - z1), 2.0)
    )

    return d


def ground_speed_ms(vel: VelocityNed):
    """
    Returns the ground speed in m/s using the NED velocity of the drone

    Parameters
    ----------
    - vel: VelocityNED
        - NED velocity of the drone
    
    Return
    ------
    - gnd_speed: float
    """
    return math.sqrt(
        math.pow(vel.north_m_s, 2.0) + math.pow(vel.east_m_s, 2.0)
    )


# Define the WGS84 geographic coordinate system (latitude, longitude)
wgs84 = CRS.from_epsg(4326)  # EPSG code for WGS84

def xy_to_global(x_local: float, y_local: float, projection):
    # Create a transformer object
    transformer = Transformer.from_crs(projection, wgs84)
    return transformer.transform(x_local, y_local)


def global_to_xy(latitude: float, longitude: float, projection):
    # Create a transformer object (switched order for reverse conversion)
    transformer = Transformer.from_crs(wgs84, projection)
    return transformer.transform(longitude, latitude)
