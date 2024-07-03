# standard
import os
import argparse
import json
from multiprocessing import Process, Barrier

# self
from logger import Logger
import drone_tasks


def root_path():
    """
    Gets the root path of the project

    Return
    ------
    - rpath: str
    """
    return os.path.dirname(os.path.abspath(__file__))


def read_config(logger: Logger, path: str):
    """
    Parses the configuration file

    Parameters
    ----------
    - logger: Logger
        - Which logger to use
    - path: str
        - Path to config file
    
    Return
    ------
    - config: dict
    """
    if os.path.exists(path):
        logger.info('Reading drones config from {}'.format(path))
        f = open(path)
        return json.load(f)
    else:
        at_root_path = os.path.join(root_path(), 'config.json')
        if os.path.exists(at_root_path):
            logger.info('Reading drones config from {}'.format(at_root_path))
            f = open(at_root_path)
            return json.load(f)
    return None


def main(conf_path: str):
    """
    Creates multiple drones and syncs them

    Parameters
    ----------
    - conf_path: str
        - Filepath to configuration file
    """
    log_path = os.path.join(root_path(), 'log')
    log = Logger(log_path)

    log.info('Maestro initialized')
    log.info('Creating the drones')

    config = read_config(log, conf_path)

    if config == None:
        log.error("Please provide a configuration file")
        quit()

    total_drones = len(config["drones"])
    barrier = Barrier(parties=total_drones)

    procs = []
    
    for inst, drone in enumerate(config["drones"]):
        p = Process(
            target=drone_tasks.execute,
            args=[
                drone["name"],
                drone["mission_waypoints"],
                config["thermals"],
                inst,
                total_drones,
                barrier,
                log_path
            ],
            name='maestro_drone_' + str(inst)
        )
        p.start()
        procs.append(p)

        log.info('Drone {} created'.format(inst))
    
    log.info('All drones were created')
    log.info('Waiting for the drones to finish execution') 

    for p in procs:
        p.join()

    log.info('Done')


def get_args():
    """
    Process the arguments, displays a help message if arguments were not given
    correctly
    """
    parser = argparse.ArgumentParser(
        prog='python3 maestro.py',
        description='Initiates the drone swarm'
    )

    parser.add_argument(
        '-c', '--config', dest='conf_path',
        metavar='filepath', type=str,
        default='config.json',
        help='path for .json config file, check README.md'
    )

    return parser.parse_args()


if __name__ == '__main__':
    args = get_args()
    main(args.conf_path)
