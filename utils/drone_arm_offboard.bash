#!/bin/bash

# Arguments
drone_namespace=$1

# If no drone namespace is provided, use the default one
if [ -z "$drone_namespace" ]; then
    drone_namespace="drone0"
fi 

ros2 service call /$drone_namespace/set_offboard_mode std_srvs/srv/SetBool data:\ true\

ros2 service call /$drone_namespace/set_arming_state std_srvs/srv/SetBool data:\ true\