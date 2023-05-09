#!/bin/bash

export PX4_HOME_LAT=40.441055
export PX4_HOME_LON=-3.688814
export PX4_HOME_ALT=0.0

# Get first argument and stored it in simulation_config variable
simulation_config=$1

# If no argument is passed, return an error
if [ -z "$simulation_config" ]
then
    echo "Error: no simulation config file provided"
    exit 1
fi

# If $AS2_GZ_ASSETS_SCRIPT_PATH is not set, return an error
if [ -z "$AS2_GZ_ASSETS_SCRIPT_PATH" ]
then
    echo "Error: AS2_GZ_ASSETS_SCRIPT_PATH is not set"
    exit 1
fi

echo "Simulation config file: $simulation_config"

$AS2_GZ_ASSETS_SCRIPT_PATH/default_run.sh $simulation_config