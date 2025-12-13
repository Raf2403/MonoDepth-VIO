#!/bin/bash

# Определяем пути
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Разрешаем доступ к X-серверу для графики
xhost +local:root

docker rm -f ros_openvins_container &> /dev/null

echo "Starting OpenVINS Docker (CPU mode)..."
echo "Mounting $ROOT_DIR into /catkin_ws"

docker run -it --rm \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$ROOT_DIR:/catkin_ws:rw" \
    --name ros_openvins_container \
    openvins_ros_noetic \
    bash

