#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.telemetry import Position, VelocityNed, Heading

from pyproj import CRS, Transformer

# Define the WGS84 geographic coordinate system (latitude, longitude)
wgs84 = CRS.from_epsg(4326)  # EPSG code for WGS84
def ned_from_global(x_local, y_local, projection):
    # Create a transformer object
    transformer = Transformer.from_crs(projection, wgs84)
    return transformer.transform(x_local, y_local)


def global_from_ned(latitude, longitude, projection):
    # Create a transformer object (switched order for reverse conversion)
    transformer = Transformer.from_crs(wgs84, projection)
    x_local, y_local = transformer.transform(longitude, latitude)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))

    mission_items = []

    async for p in drone.telemetry.position():
        pos = p
        break
    
    # Initial GPS coordinates and altitude
    origin_lat, origin_lon, origin_alt = pos.latitude_deg, pos.longitude_deg, pos.absolute_altitude_m

    # Define the azimuthal equidistant projection with center point coordinates
    projection = CRS.from_string(
        f"+proj=aeqd +lon_0={origin_lon} +lat_0={origin_lat}"
    )

    points = [
        {"x": 50, "y": 50, "z": 30},
        {"x": 100, "y": 50, "z": 30},
        {"x": 100, "y": 100, "z": 30},
        {"x": 50, "y": 100, "z": 30},
        {"x": 50, "y": 50, "z": 30},
    ]

    for waypoint in points:
        lat, lon = ned_from_global(waypoint["x"], waypoint["y"], projection)
        print("{} {}".format(lat, lon))
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



    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())