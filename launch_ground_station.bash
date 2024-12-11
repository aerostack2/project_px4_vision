#!/bin/bash

usage() {
    echo "  options:"
    echo "      -n: drone namespaces, comma separated. Default is 'drone0'"
    echo "      -t: launch keyboard teleoperation. Default not launch"
    echo "      -v: open rviz. Default not launch"
    echo "      -m: launch mocap4ros2. Default not launch"
    echo "      -r: record rosbag. Default not launch"
    echo "      -g: launch using gnome-terminal instead of tmux. Default not set"
}

# Initialize variables with default values
drones_namespace_comma="drone0"
keyboard_teleop="false"
rviz="false"
mocap4ros2="false"
rosbag="false"
use_gnome="false"

# Parse command line arguments
while getopts "n:tvmrg" opt; do
  case ${opt} in
    n )
      drones_namespace_comma="${OPTARG}"
      ;;
    t )
      keyboard_teleop="true"
      ;;
    v )
      rviz="true"
      ;;
    m )
      mocap4ros2="true"
      ;;
    r )
      rosbag="true"
      ;;
    g )
      use_gnome="true"
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

# If no drone namespaces are provided, finish the execution
if [ -z "$drones_namespace" ]; then
  echo "No drone namespace provided. Set it using the -n option"
  exit 1
fi
IFS=',' read -r -a drone_namespaces <<< "$drones_namespace_comma"

# Select between tmux and gnome-terminal
tmuxinator_mode="start"
tmuxinator_end="wait"
tmp_file="/tmp/as2_project_launch_${drone_namespaces[@]}.txt"
if [[ ${use_gnome} == "true" ]]; then
  tmuxinator_mode="debug"
  tmuxinator_end="> ${tmp_file} && python3 utils/tmuxinator_to_genome.py -p ${tmp_file} && wait"
fi

# Launch aerostack2 ground station
eval "tmuxinator ${tmuxinator_mode} -n ground_station -p tmuxinator/ground_station.yaml \
  drone_namespace=${drones_namespace_comma} \
  keyboard_teleop=${keyboard_teleop} \
  rviz=${rviz} \
  rosbag=${rosbag} \
  mocap4ros2=${mocap4ros2} \
  ${tmuxinator_end}"

# Attach to tmux session
if [[ ${use_gnome} == "false" ]]; then
  tmux attach-session -t ground_station
# If tmp_file exists, remove it
elif [[ -f ${tmp_file} ]]; then
  rm ${tmp_file}
fi