#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: namespace of the drone. For simulation must match world.json"
    echo "      -e: estimator_type for real fly, choices: [raw_odometry, mocap_pose]"
    echo "      -g: use gps, choices: [true | false]. Default: false"
    echo "      -r: record rosbag"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -m: launch MicroXRCEAgent, choices: [true | false]. Default: false"
}

# Initialize variables with default values
drone_namespace="drone0"
estimator_plugin="raw_odometry"
use_gps="false"
record_rosbag="false"
simulated="false"
micro_xrce_agent="false"

# Arg parser
while getopts "n:e:grtsmv" opt; do
  case ${opt} in
    n )
      drone_namespace="${OPTARG}"
      ;;
    e )
      estimator_plugin="${OPTARG}"
      ;;
    g )
      use_gps="true"
      ;;
    r )
      record_rosbag="true"
      ;;
    s )
      simulated="true"
      ;;
    m )
      micro_xrce_agent="false"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# If simulation is true, set the estimator_plugin to raw_odometry
instance="1"
if [[ ${simulated} == "true" ]]; then
  estimator_plugin="raw_odometry"
  instance=$(python3 utils/get_drone_index.py -p sim_config/world.json -n ${drone_namespace})
  # Add +1 to the instance to match the target_system_id
  instance=$((instance+1))
fi

# Launch aerostack2
tmuxinator start -n ${drone_namespace} -p tmuxinator/aerostack2.yml \
    drone_namespace=${drone_namespace} \
    micro_xrce_agent=${micro_xrce_agent} \
    estimator_plugin=${estimator_plugin} \
    use_gps=${use_gps} \
    simulation=${simulated} \
    instance=${instance} &
wait

# Launch rosbag
# if [[ ${record_rosbag} == "true" ]]; then
#   tmuxinator start -n rosbag -p tmuxinator/rosbag.yml \
#       drone_namespaces=${drone_namespace} &
#   wait
# fi

# # Attach to tmux session
# tmux attach-session -t ${drone_namespace}:mission
