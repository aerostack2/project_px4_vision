#!/bin/bash

# Stop docker containers
./docker/docker_stop.bash

# For each drone namespace in argument
for ns in "$@"; do
  tmux_session_list+=("$ns")
  tmux_session_list+=("gazebo_$ns")
done

# List of Tmux sessions to be killed
tmux_session_list=("keyboard_teleop" "rosbag" "mocap" "gazebo" "rviz" "ground_station")

# Get drone namespaces from simulation config file
drone_namespaces_sim=$(python3 utils/get_drones.py -p sim_config/world.json --sep ' ')
for namespace in ${drone_namespaces_sim[@]}; do
    tmux_session_list+=("${namespace}")
    tmux_session_list+=("gazebo_${namespace}")
done

# If inside tmux session, get the current session name
if [[ -n "$TMUX" ]]; then
  current_session=$(tmux display-message -p '#S')
fi

# Send Ctrl+C signal to each window of each session
for session in ${tmux_session_list[@]}; do
  # Check if session exists
  if tmux has-session -t "$session" 2>/dev/null; then
    # Get the list of windows in the session
    windows=($(tmux list-windows -t "$session" -F "#{window_index}"))
    # Iterate through each window and send Ctrl+C
    for window in "${windows[@]}"; do
      # Send Ctrl+C to the window
      tmux send-keys -t "$session:$window" C-c
      sleep 0.1 # Add a small delay to allow the signal to be processed
    done
  fi
done

# Kill all tmux sessions from the list except for the current one
for session in ${tmux_session_list[@]}; do
  if [[ "$session" != "$current_session" ]]; then
    tmux kill-session -t "$session" 2>/dev/null
  fi
done

# Kill the current tmux session, if in a tmux session
if [[ -n "$TMUX" ]]; then
  tmux kill-session -t "$current_session" 2>/dev/null
fi
