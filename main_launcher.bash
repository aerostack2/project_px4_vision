#!/bin/bash

# Arguments
drone_namespace=$1

# If no drone namespace is provided, use the default one
if [ -z "$drone_namespace" ]; then
    drone_namespace="drone0"
fi 


use_sim_time=false
controller="pid_speed_controller" # "differential_flatness_controller" or "pid_speed_controller"
behavior_type="position" # "position" or "trajectory"

if [[ "$controller" == "differential_flatness_controller" ]]
then
    behavior_type="trajectory"
fi

source ./utils/launch_tools.bash

new_session $drone_namespace

new_window 'alphanumeric_viewer' "ros2 run as2_alphanumeric_viewer as2_alphanumeric_viewer_node \
    --ros-args -r  __ns:=/$drone_namespace"

new_window 'MicroXRCE Agent' "MicroXRCEAgent serial -b 921600 --dev /dev/ttyUSB0"

new_window 'platform' "ros2 launch as2_platform_pixhawk pixhawk_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    config:=config/platform_default.yaml \
    external_odom:=false"

new_window 'controller' "ros2 launch as2_motion_controller controller_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    cmd_freq:=100.0 \
    info_freq:=10.0 \
    use_bypass:=true \
    plugin_name:=${controller} \
    plugin_config_file:=config/${controller}.yaml"

new_window 'state_estimator' "ros2 launch as2_state_estimator state_estimator_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    plugin_name:=raw_odometry \
    plugin_config_file:=config/default_state_estimator.yaml"

new_window 'behaviors' "ros2 launch as2_behaviors_motion motion_behaviors_launch.py \
    namespace:=$drone_namespace \
    use_sim_time:=$use_sim_time \
    follow_path_plugin_name:=follow_path_plugin_$behavior_type \
    go_to_plugin_name:=go_to_plugin_$behavior_type \
    takeoff_plugin_name:=takeoff_plugin_$behavior_type \
    land_plugin_name:=land_plugin_speed \
    land_trajectory_height:=-5.0 \
    go_to_threshold:=0.2 \
    takeoff_threshold:=0.2"

if [[ "$behavior_type" == "trajectory" ]]
then
    new_window 'traj_generator' "ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py  \
        namespace:=$drone_namespace \
        use_sim_time:=$use_sim_time"
fi


# if inside a tmux session detach before attaching to the session
if [ -n "$TMUX" ]; then
    tmux switch-client -t $drone_namespace
else
    tmux attach -t $drone_namespace:0
fi
