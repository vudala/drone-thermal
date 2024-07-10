# standard
import time
from functools import partial
from typing import Callable
import subprocess

# 3rd party
import asyncio
from rclpy.node import Node
from mavsdk import System
from mavsdk.offboard import PositionGlobalYaw
from std_msgs.msg import ByteMultiArray, ByteMultiArray

# self
import utils
from logger import Logger


MAVSDK_SERVER_DEFAULT_PORT = 50051


class DroneCore():
    """
    Wraps the ROS2 and MAVSDK functionalities into an abstraction of a UAV
    """
    def __init__(
            self, name: str,
            instance: int, priority: int,
            logger_path: str
        ):
        """
        Inits the attributes, mavsdk_server, ros2 node and publishers
        """
        self.system = System(port=(MAVSDK_SERVER_DEFAULT_PORT + instance))

        self.name = name
        self.priority = priority

        self.node_name = 'drone_{}'.format(instance)
        self.ros2_node = Node(self.node_name)
        self.instance = instance

        self.subscribed = set()

        self.logger = Logger(
            logger_path,
            'drone_{}'.format(instance)
        )

        self.position = None
        self.position_publisher = self.create_publisher(
            '/position',
            ByteMultiArray
        )
        self.relative_alt_m = None

        self.velocity_ned = None
        self.ground_speed_ms = None

        self.fixedwing_metrics = None
        self.airspeed_ms = None
        self.throttle_pct = None
        self.climb_rate_ms = None

        self.mission_progress = None


    async def stabilize(self):
        """
        Wait for the sensors to stabilize
        """
        self.logger.info("Waiting for drone to connect...")
        async for state in self.system.core.connection_state():
            if state.is_connected:
                self.logger.info(f"-- Connected to drone!")
                break

        self.logger.info(
            "Waiting for drone to have a global position estimate..."
        )
        async for health in self.system.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.logger.info("-- Global position estimate OK")
                break


    def create_publisher(self, topic: str, data_type: any):
        """
        Creates a publisher for some topic, with QoS 1 to allow only the latest
        info to be transmited

        Parameters
        ----------
        - topic: str
            - Name of the ROS2 topic you want to create
        - data_type: any
            - Data type of the topic
        """
        node = self.ros2_node
        return node.create_publisher(data_type, self.node_name + topic, 1)


    def publish_position(self):
        """
        Publish its own position at the position topic
        """
        if self.position != None:
            msg = ByteMultiArray()
            msg.data = utils.obj_to_bytearray(self.position)
            self.position_publisher.publish(msg)


    def subscribe_to(
            self,
            topic: str, data_type: any,
            callback: Callable[[any, str, int, any], None],
            ref: int
        ):
        """
        Subscribes to a topic

        Parameters
        ----------
        - topic: str
            - Name of the topic to subscribe to
        - data_type: any
            - ROS2 data type to be used at the callback
        - callback: function(drone, topic, ref, msg)
            - A callback function with the first param being the topic and the
            second the message of the callback param
        - ref: int
            - Instance that it refers to
        """
        sub = self.ros2_node.create_subscription(
            data_type,
            topic,
            partial(callback, self, topic, ref),
            1
        )
        self.subscribed.add(sub)


    async def position_refresher(self):
        """
        Keeps updating and publishing the drone position
        """
        async for pos in self.system.telemetry.position():
            self.position = pos
            self.relative_alt_m = pos.relative_altitude_m
            self.publish_position()
            await asyncio.sleep(0)


    async def get_position(self):
        while self.position == None:
            await asyncio.sleep(0.01)
        return self.position


    async def velocity_refresher(self):
        """
        Keeps updating and publishing the drones NED velocity
        """
        async for v in self.system.telemetry.velocity_ned():
            self.velocity_ned = v
            await asyncio.sleep(0)


    async def get_velocity_ned(self):
        while self.velocity_ned == None:
            await asyncio.sleep(0.01)
        return self.velocity_ned


    async def gnd_speed_refresher(self, delay):
        """
        Keeps updating drones ground speed

        Parameters
        ----------
        - delay: float
            - Delay in seconds between iterations
        """
        while True:
            vel = await self.get_velocity_ned()
            self.ground_speed_ms = utils.ground_speed_ms(vel)
            await asyncio.sleep(delay)


    async def get_ground_speed_ms(self):
        while self.ground_speed_ms == None:
            await asyncio.sleep(0.01)
        return self.ground_speed_ms


    async def fixedwing_metrics_refresher(self):
        """
        Keeps updating drones airspeed, throttle and climb rate
        """
        async for met in self.system.telemetry.fixedwing_metrics():
            self.fixedw = met
            self.airspeed_ms = met.airspeed_m_s
            self.throttle_pct = met.throttle_percentage
            self.climb_rate_ms = met.climb_rate_m_s
            await asyncio.sleep(0)

    
    async def get_fixedwing_metrics(self):
        while self.fixedwing_metrics == None:
            await asyncio.sleep(0.01)
        return self.fixedwing_metrics


    def apply_thermal_force(self, force: float):
        command = """
        gz topic -t /world/default/wrench/persistent -m gz.msgs.EntityWrench -p\\
        'entity: {
            type: 2,
            name: "{}"
        },
        wrench: {
            force: {
                z: {}
            }
        }'
        """.format(self.name, force)

        subprocess.run(command, shell = True, executable="/bin/bash")


    def clear_thermal_force(self):
        command = """
        gz topic -t /world/default/wrench/clear -m gz.msgs.EntityWrench -p\\
        'entity: {
            type: 2,
            name: "{}"
        }
        """.format(self.name)

        subprocess.run(command, shell = True, executable="/bin/bash")


    async def mission_progress_refresher(self):
        async for mission_progress in self.system.mission.mission_progress():
            self.mission_progress = mission_progress
            await asyncio.sleep(0)


    async def get_mission_progress(self):
        while self.mission_progress == None:
            await asyncio.sleep(0.1)
        return self.mission_progress
