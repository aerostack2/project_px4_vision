#!/bin/bash

IMAGE_NAME="px4-autopilot-gz:latest"

# Build the Docker image if it does not exist
if ! docker images | grep -q "${IMAGE_NAME}"; then
    echo "Building the Docker image..."
    docker build -t ${IMAGE_NAME} .
fi