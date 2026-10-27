#!/usr/bin/env bash
xhost +local:docker
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

docker build -t rmtt_slam_proto:latest "${REPO_ROOT}/docker"

docker run -it --rm \
    --net=host \
    --ipc=host \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${REPO_ROOT}/ros2_ws:/root/ros2_ws:rw" \
    --name rmtt_proto \
    rmtt_slam_proto:latest
