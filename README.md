# Maestro
Implementing a drone swarm controller on PX4 SITL, simulating the effect of thermals

### Requirements
You will need all these libraries:

- Python client for ROS 2 Humble:
    - Follow the standard process of instalation and setup for ROS2
    https://docs.ros.org/

- asyncio
    - Install: pip install asyncio

- MAVSDK-Python
    - Install: pip install mavsdk

- pyproj
    - Install: pip install pyproj

### Usage

#### Running without Docker

Display usage:
```bash
python3 maestro.py -h
```

### Config file

#### Structure
The JSON file must be organized in the following way:
```json
{
    "drones": [
        {
            "name": "drone1",
            "mission_waypoints": [
                {
                    "x": 10,
                    "y": 10,
                    "z": 10
                    
                }
            ]
        }
    ],
    "thermals": [
        {
            "x": 10,
            "y": 10,
            "force": 5
        }
    ]
}
```
You can define multiple drones using the "drones" attribute, each drone is
described by it's name, and the waypoints of it's mission.

Using the "thermals" attribute you can define it's XY position, and also the
force that the thermal will apply over the drone if it ever come in contact with
it.

#### -c, --config
You can point to a configuration file using this option, it can be written in
any of the forms, like this:
```bash
python3 maestro.py -c path/to/config.json
# or
python3 maestro.py --config path/to/config.json
```

#### Relative path
If in the directory that you are executing the script, exists a config.json
file, and no other configuration file was issued to the script using -c or
–config, the script will try to read that file as configuration.

#### Absolute path
If none of the situations above happen, maestro.py will look for config.json
file in the folder of the script itself.
