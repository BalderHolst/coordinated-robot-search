#!/usr/bin/env bash

ROOT=$(pwd)

CONTAINER="coordinated-robot-search"

# Build container if it does not exist
if [ ! "$(docker ps -aq -f name=$CONTAINER)" ]; then
    echo "Container '$CONTAINER' does not exist. Building..."
    docker build -t $CONTAINER .
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
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --net=host \
        --workdir="/root/ws" \
        --volume="$ROOT:/root/ws" \
        --privileged \
        $CONTAINER \
        bash
