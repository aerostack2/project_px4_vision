#!/usr/bin/env python3

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

"""Get a specific drone field value from config file"""

__authors__ = "Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"

import argparse
import yaml
from pathlib import Path
import json


def get_drone_value_json(filename: Path, namespace: str, field: str):
    """Get specific drone field value listed in JSON config file

    :param filename: Path to drones JSON config file
    :type filename: Path
    :param namespace: The namespace of the drone
    :type namespace: str
    :param field: The field to retrieve
    :type field: str
    :return: Value of the specified field for the given namespace
    :rtype: str
    """
    with open(filename, "r", encoding="utf-8") as file:
        config = json.load(file)
        for drone in config.get("drones", []):
            if drone["namespace"] == namespace:
                return drone.get(field)
        raise ValueError(f"No drone with namespace '{namespace}' found in config file")


def get_drone_value_yaml(filename: Path, namespace: str, field: str):
    """Get specific drone field value listed in YAML config file

    :param filename: Path to drones YAML config file
    :type filename: Path
    :param namespace: The namespace of the drone
    :type namespace: str
    :param field: The field to retrieve
    :type field: str
    :return: Value of the specified field for the given namespace
    :rtype: str
    """
    with open(filename, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
        drone = config.get(namespace)
        if drone:
            return drone.get(field)
        raise ValueError(f"No drone with namespace '{namespace}' found in config file")


def get_drone_value(filename: Path, namespace: str, field: str):
    """Get specific drone field value listed in config file (JSON or YAML)

    :param filename: Path to drones config file
    :type filename: Path
    :param namespace: The namespace of the drone
    :type namespace: str
    :param field: The field to retrieve
    :type field: str
    :return: Value of the specified field for the given namespace
    :rtype: str
    """
    # Determine file type based on extension
    file_extension = filename.suffix.lower()
    if file_extension == ".json":
        return get_drone_value_json(filename, namespace, field)
    elif file_extension in [".yaml", ".yml"]:
        return get_drone_value_yaml(filename, namespace, field)
    else:
        raise ValueError("Unsupported file format. Only JSON and YAML files are supported.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--config_file", type=str, required=True, help="Path to drones config file")
    parser.add_argument("-n", "--namespace", type=str, required=True, help="Namespace of the drone")
    parser.add_argument("-f", "--field", type=str, required=True, help="Field to retrieve")
    args = parser.parse_args()

    config_file = Path(args.config_file)
    if not config_file.exists():
        raise FileNotFoundError(f"File {config_file} not found")

    # Get drone field value from config file
    value = get_drone_value(config_file, args.namespace, args.field)

    # Return for use in bash scripts
    print(value)
