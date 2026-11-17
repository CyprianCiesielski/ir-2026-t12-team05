#!/bin/bash

set -e

ACCOUNT="$(id -un)"
CONTAINER_NAME="intelligent-robotics-${ACCOUNT}"
IMAGE_NAME="intelligent-robotics:humble-${ACCOUNT}"
WORKSPACE="$HOME/ir_ws"

# Allow local Docker containers to access X11
xhost +local:docker > /dev/null

docker run --rm -it \
    --name "$CONTAINER_NAME" \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device=/dev/dri \
    -v "$WORKSPACE:/home/student/ir_ws" \
    "$IMAGE_NAME"