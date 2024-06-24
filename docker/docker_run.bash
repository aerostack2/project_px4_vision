#!/bin/bash

usage() {
    echo "Usage: $0 [-n PX4_UXRCE_DDS_NS] [-a PX4_SYS_AUTOSTART] [-m PX4_GZ_MODEL] [-p PX4_GZ_MODEL_POSE] [-w PX4_GZ_WORLD] [-i INSTANCE] [-c COMMAND]"
    echo "  options:"
    echo "      -n: PX4_UXRCE_DDS_NS"
    echo "      -a: PX4_SYS_AUTOSTART (default: 4001)"
    echo "      -m: PX4_GZ_MODEL (default: x500)"
    echo "      -p: PX4_GZ_MODEL_POSE (default: 0,1,1)"
    echo "      -w: PX4_GZ_WORLD (default: default)"
    echo "      -i: INSTANCE (default: 1)"
    echo "      -c: COMMAND (default: ./build/px4_sitl_default/bin/px4)"
    exit 1
}

# Initialize variables with default values
PX4_UXRCE_DDS_NS="0"
PX4_SYS_AUTOSTART="4001"
PX4_GZ_MODEL="x500"
PX4_GZ_MODEL_POSE="0,1,1"
PX4_GZ_WORLD="default"
INSTANCE="0"
COMMAND="./build/px4_sitl_default/bin/px4"

# Arg parser
while getopts "n:a:m:p:w:i:c:" opt; do
  case ${opt} in
    n )
      PX4_UXRCE_DDS_NS="${OPTARG}"
      ;;
    a )
      PX4_SYS_AUTOSTART="${OPTARG}"
      ;;
    m )
      PX4_GZ_MODEL="${OPTARG}"
      ;;
    p )
      PX4_GZ_MODEL_POSE="${OPTARG}"
      ;;
    w )
      PX4_GZ_WORLD="${OPTARG}"
      ;;
    i )
      INSTANCE="${OPTARG}"
      ;;
    c )
      COMMAND="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[amwpic]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

# Check if PX4_UXRCE_DDS_NS is empty and fail if it is
if [ -z "$PX4_UXRCE_DDS_NS" ]; then
    echo "Error: PX4_UXRCE_DDS_NS is required using -n flag."
    usage
    exit 1
fi

# Command to launch PX4 SITL
full_command="PX4_UXRCE_DDS_NS=${PX4_UXRCE_DDS_NS} PX4_SYS_AUTOSTART=${PX4_SYS_AUTOSTART} PX4_GZ_MODEL=${PX4_GZ_MODEL} PX4_GZ_MODEL_POSE='${PX4_GZ_MODEL_POSE}' PX4_GZ_WORLD=${PX4_GZ_WORLD} ${COMMAND} -i ${INSTANCE}"
echo "Command: ${full_command}"

IMAGE_NAME="px4-autopilot-gz:latest"
CONTAINER_NAME="px4-autopilot-gz-container"

# Enable X11 forwarding
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# Build the Docker image if it does not exist
if ! docker image inspect "${IMAGE_NAME}" &> /dev/null; then
    echo "Image not found. Building the Docker image..."
    if [ -f docker/docker_build.bash ]; then
        source docker/docker_build.bash
    else
        echo "Error: docker/docker_build.bash not found."
        exit 1
    fi
fi

# Check if the container is already running
if docker ps | grep -q "${CONTAINER_NAME}"; then
    echo "The container is already running."
else
    # Check if the container exists but is not running
    if docker ps -a | grep -q "${CONTAINER_NAME}"; then
        echo "Starting the existing container..."
        docker start ${CONTAINER_NAME}
    else
        echo "Creating a new container..."
        # docker run -d -it --name ${CONTAINER_NAME} -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY -v ~/.Xauthority:/root/.Xauthority:ro px4-autopilot-gz:latest
        docker run -it --name ${CONTAINER_NAME} \
          --detach \
          --privileged \
          -e DISPLAY=$DISPLAY \
          -e XAUTHORITY=$XAUTHORITY \
          -v ~/.Xauthority:/root/.Xauthority:ro \
          --network host \
          px4-autopilot-gz:latest \
          tail -f /dev/null
    fi
fi
echo "Attaching to the running container..."
docker exec -it -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY ${CONTAINER_NAME} /bin/bash -c "${full_command}; exec bash"
