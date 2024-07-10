# standard
from multiprocessing.synchronize import Barrier
import time

# 3rd party
import asyncio
import rclpy
from std_msgs.msg import ByteMultiArray
from mavsdk.offboard import (PositionNedYaw)

# self
import utils
from drone_core import DroneCore
import mission


PX4_SITL_DEFAULT_PORT = 14540

# delay in seconds
POSITION_REFRESH_DELAY = 0.1
VELOCITY_REFRESH_DELAY = 0.1


drones = []

async def create(name: str, instance: int, priority: int, logger_path: str):
    """
    Creates a drone and expects its health checks for GPS and that kind of stuff

    Parameters
    ----------
    - name: str
        - Name of the drone
    - instance: int
        - Instance number of drone
    
    Return
    ------
    - drone: DroneCore
    """
    drone = DroneCore(name, instance, priority, logger_path)

    sys_addr = 'udp://:' + str(PX4_SITL_DEFAULT_PORT + instance)

    drone.logger.info('Trying to connect to ' + sys_addr)
    await drone.system.connect(system_address=sys_addr)
    drone.logger.info('Connected to PX4')

    await drone.stabilize()

    return drone


# callback function of the position subscription
def subscribe_position(drone: DroneCore, topic: str, ref: int, msg):
    data = msg.data
    pos = utils.bytearray_to_obj(data)
    
    drones[ref]['position'] = pos


def subscribe_to_topics(drone: DroneCore, total_instances: int):
    """
    Subscribes to the topics of the other drones

    Parameters
    ---------
    - drone: DroneCore
        - The drone that will be subscribing
    - total_instances: int
        - Total number of drones in the swarm
    """
    for i in range(total_instances):
        if i != drone.instance:
            # Position
            drone.subscribe_to(
                'drone_{}/position'.format(i),
                ByteMultiArray,
                subscribe_position,
                i
            )


async def refresher(drone: DroneCore):
    """
    Creates and executes all of the refresher coroutines

    Parameters
    - drone: DroneCore
        - Target drone of the refreshing tasks
    """
    position_ref_coro = drone.position_refresher()
    velocity_ref_coro = drone.velocity_refresher()
    mission_ref_coro = drone.mission_progress_refresher()

    group = asyncio.gather(
        position_ref_coro,
        velocity_ref_coro,
        mission_ref_coro
    )

    await group


async def proceed():
    """
    Await for some logic allowing the drone to proceed with its flight
    """
    while True:
        await asyncio.sleep(1)


async def start_coroutines(
        drone: DroneCore, waypoints: list, thermals: list, total: int
    ):
    """
    Setup and kickstart all the coroutines of the drone

    Parameters
    - drone: DroneCore
        - Target drone
    - waypoints: list
        - List of waypoints to be followed during mission
    - thermals: thermals
        - Thermals of the environment
    - total: int
        - Total number of drones in the swarm
    """
    coros = []

    # ros2 spin on separate thread
    coros.append(
        asyncio.to_thread(rclpy.spin, drone.ros2_node)
    ) 

    # the refreshers
    coros.append(refresher(drone))

    # the mission
    coros.append(mission.run(drone, waypoints, thermals))

    # create tasks for all coroutines
    group = asyncio.gather(*coros)

    drone.logger.info('Running all coroutines')

    await group


async def execute_core(
        name: str, waypoints: list, thermals: list,
        inst: int, total: int,
        barrier: Barrier,
        logger_path: str
    ):
    """
    Wraps the functionalities
    """
    # init the drones data storage
    for i in range(total):
        if i != inst:
            drones.append(dict())
        else:
            drones.append(None)
    
    # defines the priority of the drone, for now is an arbitrary value
    priority = inst

    dro = await create(name, inst, priority, logger_path)
    dro.logger.info('Drone successfully created')
    
    dro.logger.info('Synchronizing with the other UAVs')
    barrier.wait()
    dro.logger.info('Synchronized')

    dro.logger.info('Subscribing to the other drones topics')
    subscribe_to_topics(dro, total)
    dro.logger.info('Subscribed to the topics')

    dro.logger.info('All drones synced')
    dro.logger.info('Starting the coroutines')
    await start_coroutines(dro, waypoints, thermals, total)


def execute(
        name: str, waypoints: list, thermals: list,
        inst: int, total: int,
        barrier: Barrier,
        logger_path: str):
    """
    Executes all the tasks of a drone

    Parameters
    ----------
    - name: str
        - Name of the drone
    - waypoints: list
        - List of waypoints to be followed during mission
    - thermals: thermals
        - Thermals of the environment
    - inst: int
        - Number of the drone's instance
    - total: int
        - How many drones are being simulated
    - barrier: Barrier
        - A Barrier used to synchronize all the drones, must have the parties
        number the same as the total param
    - logger_path: str
        - Root dir of the logger system
    """
    rclpy.init()
    asyncio.run(
        execute_core(
            name,
            waypoints,
            thermals,
            inst,
            total,
            barrier,
            logger_path
        )
    )
    rclpy.shutdown()
