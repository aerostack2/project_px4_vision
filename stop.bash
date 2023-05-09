#!/bin/bash

# Set default input element
if [ $# -eq 0 ]; then
  set -- "drone0"
fi

# Make a tmux list of sessions to be killed
tmux_session_list=("keyboard_teleop" "rosbag" "mocap" "gazebo")

# For each drone namespace, add to the list
for ns in "$@"; do
  tmux_session_list+=("$ns")
done

# If inside tmux session, get the current session name
if [[ -n "$TMUX" ]]; then
  current_session=$(tmux display-message -p '#S')
fi

# Kill gazebo
pkill -9 -f "gzclient" < /dev/null

# Kill all tmux sessions from the list except for the current one
for session in "${tmux_session_list[@]}"; do
  if [[ "$session" != "$current_session" ]]; then
    tmux kill-session -t "$session" 2>/dev/null
  fi
done

# Kill the current tmux session, if in a tmux session
if [[ -n "$TMUX" ]]; then
  tmux kill-session -t "$current_session" 2>/dev/null
fi