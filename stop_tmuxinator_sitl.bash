#!/bin/bash

# Get the directory of the current script
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

${script_dir}/sitl_config/docker/docker_stop.bash

tmux kill-session -t pixhawk_sitl