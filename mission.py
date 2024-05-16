# self
from drone_core import DroneCore

# 3rd party
import asyncio
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.telemetry import Position
from mavsdk.offboard import PositionNedYaw, PositionGlobalYaw, OffboardError
from pyproj import CRS, Transformer

import utils

ACCEPTANCE_RADIUS_CM = 100

thermals = [
    {"x": 25, "y": 50, "visited": False}
]

points = [
    {"x":  0, "y": 0, "z": 10},
    {"x": 50, "y": 0, "z": 10},
    {"x": 50, "y": 50, "z": 10},
    {"x": 0, "y": 50, "z": 10},
    {"x": 0, "y": 0, "z": 10},
]

class Thermal():
    def __init__(self, x, y, projection, force, radius):
        self.x = x
        self.y = y
        lat, lon = xy_to_global(x, y, projection)
        self.global_position = PositionGlobalYaw(
            lat, lon, 0, 0, 0
        )
        self.force = force
        self.radius = radius


class Waypoint():
    def __init__(self, x, y, projection, altitude, speed):
        self.x = x
        self.y = y
        self.global_position = xy_to_global(x, y, projection)
        self.altitude = altitude
        self.speed = speed


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


async def worth_it(drone: DroneCore, next_waypoint: Position, thermal: Thermal):
    drone_pos = await drone.get_position()

    thermal_pos = Position(thermal)
    thermal_pos.absolute_altitude_m = drone_pos.absolute_altitude_m
    if next_waypoint.absolute_altitude_m - drone_pos.absolute_altitude_m > 20:
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
    
    target = PositionGlobalYaw(thermal.global_position)
    target.alt_m = drone.position.relative_altitude_m
    target.altitude_type = PositionGlobalYaw.AltitudeType.REL_HOME

    # check if has got close enough to the thermal position
    await reach_position(drone, target)


async def ride_thermal(drone: DroneCore, thermal: Thermal, height: float):
    drone.apply_thermal_force(thermal.force)
    
    # thermal xy, next waypoint z
    target = PositionGlobalYaw(thermal.global_position)
    target.alt_m = height
    target.altitude_type = PositionGlobalYaw.AltitudeType.REL_HOME
    await reach_position(drone, target)

    drone.clear_thermal_force()


"""
Assume that the drone is navigating through a mission and following a given
waypoint. This coroutine exits to determine if its worth it to stop the
mission, activate offboard and go torwards a thermal in order to gain lift.
"""
async def scan_for_thermal(drone: DroneCore, thermals: list):
    while True:
        for t in thermals:
            if not t["visited"] and worth_it(drone, t):
                t["visited"] = True

                await drone.mission.pause_mission()

                await follow_thermal(drone, t)

                # SET NAV_LOITER_RAD
                await drone.param.set_param_float("NAV_LOITER_RAD", float(40.0))

                await ride_thermal(drone, t, )

                await drone.mission.start_mission()


async def run(drone: DroneCore):

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    mission_items = []

    pos = await drone.get_position()

    # Initial GPS coordinates and altitude
    origin_lat, origin_lon, origin_alt = pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m

    # Define the azimuthal equidistant projection with center point coordinates
    projection = CRS.from_string(
        f"+proj=aeqd +lon_0={origin_lon} +lat_0={origin_lat}"
    )

    for waypoint in points:
        lat, lon = xy_to_global(waypoint["x"], waypoint["y"], projection)

        mission_item = MissionItem(lat, lon, waypoint["z"],
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
                                    MissionItem.VehicleAction.NONE)
        mission_items.append(mission_item)
    
    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await scan_for_thermal(drone, thermals)
