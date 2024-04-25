#!/usr/bin/env python3

import asyncio

from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("-- Transitioning to MR")
    await drone.action.transition_to_multicopter()

    await asyncio.sleep(10)

    print("-- Transitioning to FW")
    await drone.action.transition_to_fixedwing()

    await asyncio.sleep(10)




if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
