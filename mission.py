# self
from drone_core import DroneCore

# 3rd party
import asyncio
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.telemetry import Position
from mavsdk.offboard import OffboardError, PositionNedYaw
from pyproj import CRS, Transformer

import utils

ACCEPTANCE_RADIUS_CM = 100

t_list = [
    {"x": 1000, "y": 220}
]

class Thermal():
    def __init__(self, x, y, projection, force, radius):
        self.x = x
        self.y = y
        lat, lon = xy_to_global(x, y, projection)
        self.global_position = Position(
            lat, lon, 0, 0
        )
        self.force = force
        self.radius = radius
        self.visited = False

thermals = []

p_list = [
    {"x":  500, "y": 0, "z": 150},
    {"x": 750, "y": 200, "z": 250},
    {"x": 1250, "y": 200, "z": 300},
    {"x": 1500, "y": 0, "z": 225},
    {"x": 2000, "y": 0, "z": 300},
]

class Waypoint():
    def __init__(self, x, y, projection, altitude, speed):
        self.x = x
        self.y = y
        self.z = altitude
        lat, lon = xy_to_global(x, y, projection)
        self.global_position = Position(
            lat, lon, altitude, 0
        )
        self.altitude = altitude
        self.speed = speed

points = []

# Define the WGS84 geographic coordinate system (latitude, longitude)
wgs84 = CRS.from_epsg(4326)  # EPSG code for WGS84
def xy_to_global(x_local, y_local, projection):
    # Create a transformer object
    transformer = Transformer.from_crs(projection, wgs84)
    return transformer.transform(x_local, y_local)


def global_to_xy(latitude, longitude, projection):
    # Create a transformer object (switched order for reverse conversion)
    transformer = Transformer.from_crs(wgs84, projection)
    return transformer.transform(longitude, latitude)


async def get_next_waypoint(drone: DroneCore):
    m_progress = await drone.get_mission_progress()
    if m_progress.current >= m_progress.total:
        return None
    return points[m_progress.current]


async def worth_it(drone: DroneCore, thermal: Thermal):
    drone_pos = await drone.get_position()
    next_waypoint = await get_next_waypoint(drone)
    if not next_waypoint:
        return False
    thermal_pos = thermal.global_position
    thermal_pos.relative_altitude_m = drone_pos.relative_altitude_m
    thermal_pos.absolute_altitude_m = drone_pos.absolute_altitude_m
    # if it needs to gain at least 20 meters of height to reach the next waypoint
    # and if it is at least 30 meters from the thermal
    print(next_waypoint.z - drone_pos.relative_altitude_m, end=' ')
    print(utils.distance_cm(drone_pos, thermal_pos) / 100)
    if next_waypoint.z - drone_pos.relative_altitude_m > 20:
        if utils.distance_cm(drone_pos, thermal_pos) / 100 < 30:
            return True
    
    return False


async def reach_position(drone: DroneCore, target: Position):
    """
    It hold the execution until the drones has gotten close enough to the
    desired position
    """
    await drone.offboard.set_position_global(target)

    while utils.distance_cm(drone.position, target) < ACCEPTANCE_RADIUS_CM:
        await asyncio.sleep(0.1)


async def follow_thermal(drone: DroneCore, thermal: Thermal):
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.return_to_launch()
        return
    
    target = thermal.global_position
    target.relative_altitude_m = drone.position.relative_altitude_m

    # check if has got close enough to the thermal position
    await reach_position(drone, target)


async def ride_thermal(drone: DroneCore, thermal: Thermal, height: float):
    drone.apply_thermal_force(thermal.force)
    
    # thermal xy, next waypoint z
    target = thermal.global_position
    target.relative_altitude_m = height
    await reach_position(drone, target)

    drone.clear_thermal_force()


"""
Assume that the drone is navigating through a mission and following a given
waypoint. This coroutine exits to determine if its worth it to stop the
mission, activate offboard and go torwards a thermal in order to gain lift.
"""
async def scan_for_thermal(drone: DroneCore):
    while True:
        for t in thermals:
            if not t.visited and await worth_it(drone, t):
                t.visited = True

                print("WORTH IIIIIIIIIIIT")

                # await drone.system.mission.pause_mission()

                # await follow_thermal(drone, t)

                # # SET NAV_LOITER_RAD
                # await drone.system.param.set_param_float("NAV_LOITER_RAD", float(40.0))

                # next_waypoint = await get_next_waypoint(drone)

                # await ride_thermal(drone, t, next_waypoint.z)

                # await drone.system.mission.start_mission()
        await asyncio.sleep(0.1)


async def run(drone: DroneCore):
    await drone.system.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    mission_items = []

    pos = await drone.get_position()

    # Initial GPS coordinates and altitude
    origin_lat, origin_lon = pos.latitude_deg, pos.longitude_deg

    # Define the azimuthal equidistant projection with center point coordinates
    projection = CRS.from_string(
        f"+proj=aeqd +lon_0={origin_lon} +lat_0={origin_lat}"
    )

    for t in t_list:
        thermals.append(Thermal(t["x"], t["y"], projection, 0, 0))
    
    for p in p_list:
        points.append(Waypoint(p["x"], p["y"], projection, p["z"], 0))

    for waypoint in points:
        mission_item = MissionItem(
            waypoint.global_position.latitude_deg,
            waypoint.global_position.longitude_deg,
            waypoint.z,
            10,
            True,
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            MissionItem.VehicleAction.NONE
        )
        mission_items.append(mission_item)

    mission_plan = MissionPlan(mission_items)

    await drone.system.mission.set_return_to_launch_after_mission(False)
    # await drone.system.param.set_param_int("RTL_TYPE", int(1))
    await drone.system.param.set_param_int("MIS_TKO_LAND_REQ", int(0))

    drone.logger.info("-- Uploading mission")
    await drone.system.mission.upload_mission(mission_plan)

    drone.logger.info("-- Arming")
    await drone.system.action.arm()

    drone.logger.info("-- Taking off")
    await drone.system.action.takeoff()

    await asyncio.sleep(10)

    drone.logger.info("-- Starting mission")
    await drone.system.mission.start_mission()

    await scan_for_thermal(drone)
