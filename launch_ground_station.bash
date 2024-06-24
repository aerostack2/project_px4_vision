#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: namespaces of the drones only for real fly, separated by comma. For simulation world.json is used"
    echo "      -t: launch keyboard teleoperation"
    echo "      -s: simulated, choices: [true | false]"
    echo "      -v: open rviz, choices: [true | false]"
    echo "      -m: launch mocap4ros2, choices: [true | false]"
}

# Initialize variables with default values
drone_namespaces_comma="drone0"
keyboard_teleop="false"
simulated="false"
rviz="false"
mocap4ros2="false"

# Arg parser
while getopts "n:tsvm" opt; do
  case ${opt} in
    n )
      drone_namespaces_comma="${OPTARG}"
      ;;
    t )
      keyboard_teleop="true"
      ;;
    s )
      simulated="true"
      ;;
    v )
      rviz="true"
      ;;
    m )
      mocap4ros2="true"
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

if [[ ${simulated} == "true" ]]; then
  simulation_config_file="sim_config/world.json"
  drone_namespaces_comma=$(python3 utils/get_drones.py -p ${simulation_config_file} --sep ',')
fi
# Convert drone_namespaces_comma to a list
IFS=',' read -r -a drone_namespaces <<< "${drone_namespaces_comma}"

# Launch ground station
# If any is true, launch tmuxinator
if [[ ${keyboard_teleop} == "true" || ${rviz} == "true" || ${mocap4ros2} == "true" ]]; then
  tmuxinator start -p tmuxinator/ground_station.yml \
    drone_namespaces=${drone_namespaces_comma} \
    keyboard_teleop=${keyboard_teleop} \
    simulated=${simulated} \
    rviz=${rviz} \
    mocap4ros2=${mocap4ros2} &
  wait
fi

# If simulated, launch docker
if [[ ${simulated} == "true" ]]; then
  for namespace in ${drone_namespaces[@]}; do
    tmuxinator start -n gazebo_${namespace} -p tmuxinator/gazebo.yml \
        simulation_config=${simulation_config_file} namespace=${namespace} &
    wait

    # Break
    # break

    # Wait for the gazebo to start if not last drone
    if [[ ${namespace} != ${drone_namespaces[-1]} ]]; then
      echo "Waiting for gazebo to start..."
      sleep 10
    fi
  done
fi

# Attach to tmux session
if [[ ${simulated} == "true" ]]; then
  tmux attach-session -t gazebo_${drone_namespaces[0]}
else
  tmux attach-session -t ground_station
fi

