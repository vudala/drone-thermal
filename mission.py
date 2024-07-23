# self
from drone_core import DroneCore

# 3rd party
import asyncio
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.telemetry import Position
from mavsdk.offboard import OffboardError, PositionNedYaw, PositionGlobalYaw
from pyproj import CRS, Transformer

# standard
import utils
from typing import Optional

ACCEPTANCE_RADIUS_M = 5

# Minimum altitude to gain from a thermal
MIN_WORTH_ALTITUDE_M = 20
# Maximum worth distance from a thermal
MAX_WORTH_DISTANCE_M = 70


class Thermal():
    """
    Class responsible for encapsulating the information of a thermal
    """
    def __init__(self, x, y, force, radius, projection):
        self.x = x
        self.y = y
        lat, lon = xy_to_global(x, y, projection)
        self.global_position = Position(
            lat, lon, 0, 0
        )
        self.force = force
        self.radius = radius
        self.available = True


class Waypoint():
    """
    Class responsible for encapsulating the information of a waypoint
    """
    def __init__(self, x, y, altitude_m, speed_ms, projection):
        self.x = x
        self.y = y
        self.altitude_m = altitude_m
        lat, lon = xy_to_global(x, y, projection)
        self.global_position = Position(
            lat, lon, altitude_m, 0
        )
        self.speed_ms = speed_ms


def pos_to_pos_global_yaw(pos: Position):
    """
    Converts a Position object into a PositionGlobalYaw object

    Parameters
    ----------
    - pos: Position
        - Source position
    """
    return PositionGlobalYaw(
        pos.latitude_deg,
        pos.longitude_deg,
        pos.relative_altitude_m,
        0, PositionGlobalYaw.AltitudeType.REL_HOME
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


async def get_next_waypoint(drone: DroneCore, points: list):
    m_progress = await drone.get_mission_progress()
    if m_progress.current >= m_progress.total:
        return None
    return points[m_progress.current]


async def worth_it(
        drone: DroneCore, next_waypoint: Optional[Waypoint], thermal: Thermal
    ):
    """
    Given a thermal, it determines if its worth it to go for it, considering the
    next waypoint

    Parameters
    ----------
    - drone: DroneCore
        - Target drone
    - next_waypoint: Waypoint | None
        - Next waypoint to reach
    - thermal: Thermal
        - Thermal do be analyzed
    """
    if not next_waypoint:
        return False
    
    drone_pos = await drone.get_position()
    thermal_pos = thermal.global_position
    thermal_pos.relative_altitude_m = drone_pos.relative_altitude_m
    thermal_pos.absolute_altitude_m = drone_pos.absolute_altitude_m

    alt_diff = next_waypoint.altitude_m - drone_pos.relative_altitude_m
    if alt_diff > MIN_WORTH_ALTITUDE_M:
        dist = utils.distance_cm(drone_pos, thermal_pos) 
        if dist < MAX_WORTH_DISTANCE_M * 100:
            return True
    
    return False


async def reach_position(drone: DroneCore, target: Position):
    """
    It hold the execution until the drone has gotten close enough to the
    desired position

    Parameters
    ----------
    - drone: DroneCore
        - Target drone
    - target: Position
        - Desired position
    """
    target_global_yaw = pos_to_pos_global_yaw(target)
    await drone.system.offboard.set_position_global(target_global_yaw)

    pos = await drone.get_position()
    while utils.distance_cm(pos, target) > ACCEPTANCE_RADIUS_M * 100:
        await asyncio.sleep(0.1)
        pos = await drone.get_position()


async def start_offboard(drone: DroneCore):
    """
    Triggers offboard mode

    Parameters
    ----------
    - drone: DroneCore
        - Target drone
    """

    pos = pos_to_pos_global_yaw(await drone.get_position())
    await drone.system.offboard.set_position_global(pos)

    drone.logger.info("-- Starting offboard")
    try:
        await drone.system.offboard.start()
    except OffboardError as error:
        drone.logger.info(f"Starting offboard mode failed \
                with error: {error}")
        # TODO: Define failure behaviour


async def follow_thermal(drone: DroneCore, thermal: Thermal):
    """
    Makes the drone follow a thermal

    Parameters
    ----------
    - drone: DroneCore
        - Target drone
    - thermal: Thermal
        - Desired thermal to follow
    """
    await start_offboard(drone)
    
    target = thermal.global_position
    target.relative_altitude_m = drone.position.relative_altitude_m

    await reach_position(drone, target)


async def ride_thermal(drone: DroneCore, thermal: Thermal, height: float):
    drone.apply_thermal_force(thermal.force)

    target_global_yaw = pos_to_pos_global_yaw(thermal.global_position)
    target_global_yaw.alt_m = height
    await drone.system.offboard.set_position_global(target_global_yaw)

    pos = await drone.get_position()
    diff = (height - pos.relative_altitude_m)
    while diff > ACCEPTANCE_RADIUS_M:
        await asyncio.sleep(0.1)
        pos = await drone.get_position()
        diff = (height - pos.relative_altitude_m)

    drone.clear_thermal_force()


async def scan_for_thermal(drone: DroneCore, points: list, thermals: list):
    """
    Assume that the drone is navigating through a mission and following a given
    waypoint. This coroutine exits to determine if its worth it to stop the
    mission, activate offboard and go torwards a thermal in order to gain lift.
    """
    while True:
        next_waypoint = await get_next_waypoint(drone, points)

        for t in thermals:
            if t.available and await worth_it(drone, next_waypoint, t):
                t.available = False
                
                await drone.system.mission.pause_mission()

                await follow_thermal(drone, t)

                # SET NAV_LOITER_RAD
                await drone.system.param.set_param_float("NAV_LOITER_RAD", float(40.0))

                await ride_thermal(drone, t, next_waypoint.altitude_m)

                await drone.system.mission.start_mission()
                t.available = True

        await asyncio.sleep(0.1)


def create_mission(waypoints: list):
    """
    Creates a mission plan object, given a list of Waypoints

    Paremeters
    ----------
    - waypoints: list
        - List of waypoints to be followed during mission

    Return
    ------
    - mission_plan: MissionPlan
    """
    mission_items = []
    for wp in waypoints:
        mission_item = MissionItem(
            wp.global_position.latitude_deg,
            wp.global_position.longitude_deg,
            wp.altitude_m,
            wp.speed_ms,
            True,
            float('nan'),
            float('nan'),
            MissionItem.CameraAction.NONE,
            float('nan'),
            float('nan'),
            ACCEPTANCE_RADIUS_M,
            float('nan'),
            float('nan'),
            MissionItem.VehicleAction.NONE
        )
        mission_items.append(mission_item)

    return MissionPlan(mission_items)


async def upload_mission(drone: DroneCore, waypoints: list):
    """
    Uploads waypoints as a mission for the given drone

    Parameters
    ----------
    - drone: DroneCore
        - The target drone
    - waypoints: list
        - A list of Waypoint objects
    """
    await drone.system.mission.set_return_to_launch_after_mission(False)
    await drone.system.param.set_param_int("MIS_TKO_LAND_REQ", int(0))

    mission_plan = create_mission(waypoints)

    drone.logger.info("-- Uploading mission")
    await drone.system.mission.upload_mission(mission_plan)


async def run(drone: DroneCore, waypoints: list, thermals: list):
    """
    Makes the target drone execute a given mission
    - drone: DroneCore
        - Target drone
    - waypoints: list
        - List of the waypoints of the mission
    - thermals: list
        - List of thermals present in the environment
    """

    pos = await drone.get_position()

    # Initial GPS coordinates and altitude
    origin_lat, origin_lon = pos.latitude_deg, pos.longitude_deg

    # Define the azimuthal equidistant projection with center point coordinates
    projection = CRS.from_string(
        f"+proj=aeqd +lon_0={origin_lon} +lat_0={origin_lat}"
    )

    points = []    
    for p in waypoints:
        points.append(Waypoint(**p, projection=projection))

    await upload_mission(drone, points)    

    drone.logger.info("-- Arming")
    await drone.system.action.arm()

    drone.logger.info("-- Taking off")
    await drone.system.action.takeoff()

    drone.logger.info("-- Starting mission")
    await drone.system.mission.start_mission()

    therms = []
    for t in thermals:
        therms.append(Thermal(t["x"], t["y"], t["force"], 0, projection))
    
    await scan_for_thermal(drone, points, therms)
