#!/usr/bin/env bash

ROOT=$(pwd)

CONTAINER="coordinated-robot-search"

# Remove container if the `--rebuild` flag is set
if [ "$1" == "--rebuild" ]; then
    echo "Removing container..."
    docker rm -f $CONTAINER > /dev/null
fi

# Build container if it does not exist
if [ ! "$(docker ps -aq -f name=$CONTAINER)" ]; then
    echo "Container '$CONTAINER' does not exist. Building..."
    docker build -t $CONTAINER . || exit 1
fi

# Start if container is stopped
if [ "$(docker ps -aq -f status=exited -f name=$CONTAINER)" ]; then
        echo "Starting container..."
        docker start $CONTAINER > /dev/null
fi

XAUTH=/tmp/.docker.xauth

[ "$(docker ps | grep $CONTAINER)" ] && {
        echo "Attaching to running container..."
        docker exec -it \
            --privileged \
            --workdir="/root/ws" \
            $CONTAINER bash
        exit 0
}

echo "Running container..."
docker run -it \
        --name $CONTAINER \
        --env="DISPLAY=$DISPLAY" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="QT_X11_NO_MITSHM=1" \
        --net=host \
        --workdir="/root/ws" --volume="$ROOT:/root/ws" \
        --workdir="/root/ws" --volume="$ROOT:/root/ws" \
        --volume="$HOME/.config/nvim:/root/.config/nvim" \
        --privileged \
        $CONTAINER \
        bash
