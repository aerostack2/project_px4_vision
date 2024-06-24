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

"""Utility to get drone namespaces from a JSON configuration file and run the docker"""

__authors__ = "Rafael Pérez Seguí, Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"

import sys

from ament_index_python.packages import get_package_share_directory
package_folder = get_package_share_directory(
    'as2_state_estimator')
sys.path.append(package_folder + 'launch')

import os
import argparse
from get_drone_index import load_json_ordered, find_index_by_namespace
from pathlib import Path

RUN_DOCKER_SCRIPT="./docker/docker_run.bash"

class DroneConfig:
    """Class to read and parse the JSON configuration file."""

    def __init__(self, config_file: str):
        self.config_file = config_file
        self.config_data = load_json_ordered(config_file)

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
                drone_found = True
                break

        if not drone_found:
            print(f"No drone found with namespace: {namespace}")
            return None
        
        instance = find_index_by_namespace(self.config_data, namespace)
        if instance == -1:
            print(f"Namespace {namespace} not found in the configuration file")
            return None
        
        arguments = f"-n {namespace} -a {airframe} -m {model} -p {pose} -w {world} -i {instance}"
        return arguments

def main():
    parser = argparse.ArgumentParser(description="Run drone configuration command from JSON file.")
    parser.add_argument('-p', '--config_file', type=str, required=True, help='Path to the JSON configuration file.')
    parser.add_argument('-n', '--namespace', type=str, required=True, help='Namespace of the drone.')

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

    if arguments:
        print(f"Executing command: {RUN_DOCKER_SCRIPT} {arguments}")
        os.system(f"{RUN_DOCKER_SCRIPT} {arguments}")

if __name__ == "__main__":
    main()
