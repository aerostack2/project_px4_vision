#!/bin/bash

CONTAINER_NAME="px4-autopilot-gz-container"

# Check if the container is running
if docker ps | grep -q "${CONTAINER_NAME}"; then
    echo "Stopping the container..."
    docker stop ${CONTAINER_NAME}
fi

# Check if the container exists (running or stopped)
if docker ps -a | grep -q "${CONTAINER_NAME}"; then
    echo "Removing the container..."
    docker rm ${CONTAINER_NAME}
else
    echo "Container ${CONTAINER_NAME} does not exist."
fi
