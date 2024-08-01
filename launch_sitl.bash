#!/bin/bash

drones_namespace_comma=$(python3 utils/get_drones.py -p sitl_config/world.yaml --sep ',')

echo "Launching SITL simulation for drones: ${drones_namespace_comma}"

tmuxinator start -n pixhawk_sitl -p tmuxinator/sitl_simulation.yaml \
  drone_namespace=${drones_namespace_comma} \
  wait

tmux attach-session -t pixhawk_sitl