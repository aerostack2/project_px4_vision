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

"""Get a specific drone index from config file"""

__authors__ = "Rafael Pérez Seguí"
__copyright__ = "Copyright (c) 2024 Universidad Politécnica de Madrid"
__license__ = "BSD-3-Clause"

import argparse
import json
from collections import OrderedDict
import sys
from pathlib import Path

# Function to load JSON as an OrderedDict
def load_json_ordered(file_path):
    """
    Loads JSON from the specified file_path as an OrderedDict.
    
    Args:
    - file_path (str): Path to the JSON file.
    
    Returns:
    - OrderedDict: JSON data loaded as an OrderedDict.
    """
    with open(file_path, 'r') as file:
        # Load JSON data
        data = json.load(file, object_pairs_hook=OrderedDict)
        # Sort the keys in the JSON data
        sorted_data = OrderedDict(sorted(data.items()))
        return sorted_data

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

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--config_file", type=str, required=True, help="Path to drones config file")
    parser.add_argument("-n", "--namespace", type=str, required=True, help="Namespace of the drone")
    
    args = parser.parse_args()

    config_file = Path(args.config_file)
    if not config_file.exists():
        raise FileNotFoundError(f"File {config_file} not found")
    
    json_data = load_json_ordered(args.config_file)
    index = find_index_by_namespace(json_data, args.namespace)
    if index == -1:
        print(f"Error: No drone with namespace '{args.namespace}' found in config file")
        sys.exit(1)
    
    # Return for use in bash scripts
    print(index)
    sys.exit(0)