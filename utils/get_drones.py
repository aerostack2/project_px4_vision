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

"""Get drones names from config file"""

__authors__ = "Rafael Pérez Seguí, Pedro Arias Pérez"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"

import argparse
import yaml
from pathlib import Path
import json


def get_drones_namespaces_json(filename: Path) -> list[str]:
    """Get drone namespaces listed in JSON config file

    :param filename: Path to drones JSON config file
    :type filename: Path
    :return: List of drone namespaces
    :rtype: list[str]
    """
    with open(filename, "r", encoding="utf-8") as file:
        config = json.load(file)
        # Attempt to retrieve model names, fallback to namespaces if not found
        model_names = [drone.get("model_name") for drone in config.get("drones", [])]
        model_names = [name for name in model_names if name is not None]  # Filter out None values
        
        if len(model_names) == 0:
            namespace_names = [drone["namespace"] for drone in config["drones"]]
            if len(namespace_names) == 0:
                raise ValueError("No drones found in config file")
            return namespace_names
        
        return model_names


def get_drones_namespaces_yaml(filename: Path) -> list[str]:
    """Get drone names listed in swarm config file
    Open yaml file, read as a dictionary and return main keys as list

    :param filename: Path to drones config file
    :type filename: Path
    :return: List of drones names
    :rtype: list[str]
    """
    with open(filename, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
        return list(config.keys())


def get_drones_namespaces(filename: Path) -> list[str]:
    """Get drone namespaces listed in config file (JSON or YAML)
    Open file, read as JSON or YAML depending on file extension, and return namespaces as list

    :param filename: Path to drones config file
    :type filename: Path
    :return: List of drone namespaces
    :rtype: list[str]
    """
    # Determine file type based on extension
    file_extension = filename.suffix.lower()
    if file_extension == ".json":
        return get_drones_namespaces_json(filename)
    elif file_extension in [".yaml", ".yml"]:
        return get_drones_namespaces_yaml(filename)
    else:
        raise ValueError("Unsupported file format. Only JSON and YAML files are supported.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--config_file', type=str, required=True, help="Path to drones config file")
    parser.add_argument("--sep", help="Separator", default=":")
    args = parser.parse_args()

    config_file = Path(args.config_file)
    if not config_file.exists():
        raise FileNotFoundError(f"File {config_file} not found")

    # Get drones from config file
    drones = get_drones_namespaces(config_file)

    # Return for use in bash scripts
    print(args.sep.join(drones))
