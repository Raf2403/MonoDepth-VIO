#!/usr/bin/env bash

# Определяем корневую директорию (на уровень выше, где лежит скрипт)
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD

echo "---- Building Docker for OpenVINS (ROS Noetic) ----"
echo "Root Dir: $ROOT_DIR"

# Переходим в корень репозитория для правильного контекста сборки
cd $ROOT_DIR

# Основная команда сборки
# Используем osrf/ros:noetic-desktop-full как базу — там уже есть ROS и Ubuntu 20.04
# Это решает 90% проблем с зависимостями OpenVINS
docker build -t openvins_ros_noetic \
    -f $ROOT_DIR/docker/Dockerfile \
    $ROOT_DIR \
    --network=host \
    --build-arg from=osrf/ros:noetic-desktop-full

# Проверка статуса выполнения
if [ $? -eq 0 ]; then
    echo "---- Build Successful! Image: openvins_ros_noetic ----"
else
    echo "!!!! Build Failed !!!!"
    exit 1
fi

cd $EXEC_PATH

