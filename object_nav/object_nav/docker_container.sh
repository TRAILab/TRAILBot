#!/bin/bash

# Absolute path of your local project folder
LOCAL_DIR=$(realpath ./my_project)

# Docker image name
IMAGE_NAME=graph_rag_docker_image:latest

# Container name
CONTAINER_NAME=ours_docker

# Run the container
docker run -it \
    --gpus all \
    --name $CONTAINER_NAME \
    -v /home/trailbot:/workspace \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network=host \
    --privileged \
    $IMAGE_NAME \
    /bin/bash
