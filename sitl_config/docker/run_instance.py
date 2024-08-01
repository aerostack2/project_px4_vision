# /usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Utility to get drone namespaces from a configuration file and run the docker"""

__authors__ = "Rafael Pérez Seguí, Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"

import os
import argparse
import json
from time import sleep
import yaml
from pathlib import Path

RUN_DOCKER_SCRIPT='sitl_config/docker/docker_run.bash'

    
def read_file(filename: Path) -> str:
    """Read file content

    :param filename: Path to file
    :type filename: Path
    :return: File content
    :rtype: str
    """
    # Check extension of config file
    if filename.endswith('.json'):
        with open(filename, 'r', encoding='utf-8') as stream:
            config = json.load(stream)
    elif filename.endswith('.yaml') or filename.endswith('.yml'):
        with open(filename, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
    else:
        raise ValueError('Invalid configuration file extension.')
    return config

# Function to find the index of a namespace in the OrderedDict and print it
def find_index_by_namespace(json_data, namespace):
    """
    Finds the index of the namespace in the OrderedDict json_data and prints it.
    If namespace is not found, prints -1.
    
    Args:
    - json_data (OrderedDict): JSON data as an OrderedDict.
    - namespace (str): Namespace to search for in the keys of json_data.
    """
    # Access the "drones" array from the json_data
    drones = json_data.get("drones", [])
    
    # Search for namespace within each drone entry
    for index, drone in enumerate(drones):
        if drone.get("namespace") == namespace:
            return index
    
    return -1  # -1 if namespace is not found in OrderedDict

class DroneConfig:
    """Class to read and parse the configuration file."""

    def __init__(self, config_file: str):
        self.config_file = config_file
        self.config_data = read_file(config_file)

    def get_namespace_index(self, namespace: str):
        """Get the index of the namespace in the configuration file."""
        # Check namespace is in self.config_data["drones"] and return index
        return find_index_by_namespace(self.config_data, namespace)

    def construct_command(self, namespace: str):
        """Construct the command using the drone parameters."""
        world = self.config_data["world"]
        drones = self.config_data["drones"]

        drone_found = False
        for drone in drones:
            if drone["namespace"] == namespace:
                airframe = drone["airframe"]
                model = drone["model"]
                pose = drone["pose"]
                instance = drone["instance"]
                drone_found = True
                break

        if not drone_found:
            print(f"No drone found with namespace: {namespace}")
            return None
        
        arguments = f"-n {namespace} -a {airframe} -m {model} -p {pose} -w {world} -i {instance}"
        return arguments

def main():
    parser = argparse.ArgumentParser(
        description="Run drone configuration command from YAML simulation file.")
    parser.add_argument(
        '-p', '--config_file',
        type=str,
        required=True,
        help='Path to the YAML simulation configuration file.')
    parser.add_argument(
        '-n', '--namespace',
        type=str,
        required=True,
        help='Namespace of the drone.')

    args = parser.parse_args()

    # Check if file exists
    config_file = Path(args.config_file)
    if not config_file.exists():
        raise FileNotFoundError(f"File {config_file} not found")
    
    # Check if RUN_DOCKER_SCRIPT file exists
    run_docker_script = Path(RUN_DOCKER_SCRIPT)
    if not run_docker_script.exists():
        raise FileNotFoundError(f"File {run_docker_script} not found")
    
    drone_config = DroneConfig(args.config_file)
    arguments = drone_config.construct_command(args.namespace)

    if drone_config.get_namespace_index(args.namespace) != 0:
        sleep(2.0) # Wait for the first drone to start the container

    if arguments:
        print(f"Executing command: {RUN_DOCKER_SCRIPT} {arguments}")
        os.system(f"{RUN_DOCKER_SCRIPT} {arguments}")

if __name__ == "__main__":
    main()
