#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.telemetry import Position
from pyproj import CRS, Transformer

import utils

# Define the WGS84 geographic coordinate system (latitude, longitude)
wgs84 = CRS.from_epsg(4326)  # EPSG code for WGS84
def ned_to_global(x_local, y_local, projection):
    # Create a transformer object
    transformer = Transformer.from_crs(projection, wgs84)
    return transformer.transform(x_local, y_local)


def global_to_ned(latitude, longitude, projection):
    # Create a transformer object (switched order for reverse conversion)
    transformer = Transformer.from_crs(wgs84, projection)
    return transformer.transform(longitude, latitude)


thermals = [
    {"x": 25, "y": 50}
]

async def monitor_pos(drone: System, projection):
    async for p in drone.telemetry.position():
        for t in thermals:
            t_lat, t_lon = ned_to_global(t["x"], t["y"], projection)
            t_pos = Position(t_lat, t_lon, p.absolute_altitude_m, p.relative_altitude_m)

            dist = utils.distance_cm(p, t_pos)

            if dist < 300:
                print("In thermal range")
        

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

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
        {"x":  0, "y": 0, "z": 10},
        {"x": 50, "y": 0, "z": 10},
        {"x": 50, "y": 50, "z": 10},
        {"x": 0, "y": 50, "z": 10},
        {"x": 0, "y": 0, "z": 10},
    ]

    for waypoint in points:
        lat, lon = ned_to_global(waypoint["x"], waypoint["y"], projection)
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

    await monitor_pos(drone, projection)


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
