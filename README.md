# MonoDepth-VIO: Visual-Inertial Odometry (RealSense D435 + External IMU)

Этот проект реализует систему визуально-инерциальной одометрии (VIO) на базе фреймворка **OpenVINS**. 
Особенность конфигурации заключается в использовании камеры **Intel RealSense D435** (которая не имеет встроенного IMU) в связке с **внешним источником инерциальных данных** (iPhone/CSV).

## Архитектура системы

*   **Visual Sensor:** Intel RealSense D435 (RGB/Mono поток).
*   **Inertial Sensor:** Внешний IMU (данные с iPhone, передаваемые через CSV/ROS topic).
*   **VIO Core:** OpenVINS (`ov_msckf`).
*   **Environment:** ROS Noetic (Docker Container).

### Схема топиков (Remapping)
В файле запуска `system.launch` настроена следующая маршрутизация данных:
*   Камера: `/camera/color/image_raw` -> `/cam0/image_raw`
*   IMU: `/imu` (из CSV) -> `/imu0`

---

## Требования

### Аппаратное обеспечение
1.  Камера Intel RealSense D435.
2.  Кабель USB 3.0 (обязательно подключение к порту USB 3.0+).
3.  iPhone (или иной источник) для записи CSV с данными акселерометра и гироскопа.

### Программное обеспечение
*   Linux (Host)
*   Docker
*   NVIDIA Drivers (рекомендуется для GPU-ускорения, если используется `nvidia-docker`).

---

## Установка и запуск

Ниже приведена инструкция для развертывания окружения "с нуля" после клонирования репозитория.

### 1. Сборка и запуск Docker-контейнера

Выполните эти команды на хост-машине (терминал Ubuntu):

```
# 1. Сборка образа
./build_docker.sh

# 2. Запуск контейнера (проброс USB и графики включен в скрипте)
sudo ./run_docker.sh
```

### 2. Сборка пакетов ROS (Внутри контейнера)

После входа в контейнер (приглашение `root@...:/catkin_ws#`):

```
cd /catkin_ws

# Очистка кэша сборки (во избежание конфликтов)
catkin clean -y

# Сборка пакетов (используем 2 ядра для стабильности)
catkin build -j2 -p2
```
*Убедитесь, что сборка завершилась сообщением `[build] Summary: All packages succeeded!`.*

### 3. Установка драйверов RealSense

По умолчанию в базовом образе отсутствуют udev-правила и библиотеки для работы с физическим устройством. Выполните установку внутри контейнера:

```
# Добавление ключей и репозитория Intel
apt-get update && apt-get install -y software-properties-common curl
apt-key adv --keyserver keyserver.ubuntu.com --recv-key C8B3A55A6F3EFCDE
add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u

# Установка библиотек и ROS-обертки
apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev ros-noetic-realsense2-camera ros-noetic-realsense2-description

# Обновление профиля ROS (ВАЖНО!)
source /opt/ros/noetic/setup.bash
rospack profile
```

### 4. Запуск системы

1.  Подключите камеру RealSense по USB.
2.  Убедитесь, что файл с данными IMU находится по пути, указанному в `system.launch` (по умолчанию: `src/imu_csv_publisher/ros_imu_data.csv`).
3.  Запустите основной launch-файл:

```
source devel/setup.bash
roslaunch MonoDepth-VIO system.launch
```

---

## Конфигурация файлов

### system.launch
Если файл отсутствует, создайте его в `/catkin_ws/src/MonoDepth-VIO/launch/system.launch`:

```
<launch>
    <!-- 1. КАМЕРА REALSENSE -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="enable_gyro" value="false"/>
        <arg name="enable_accel" value="false"/>
        <arg name="align_depth" value="false"/>
        <arg name="enable_sync" value="true"/>
        <arg name="color_width" value="640"/>
        <arg name="color_height" value="480"/>
        <arg name="color_fps" value="30"/>
    </include>

    <!-- 2. IPHONE IMU PUBLISHER -->
    <node pkg="imu_csv_publisher" type="imu_data_start.py" name="imu_publisher_node" output="screen">
        <param name="csv_path" value="$(find imu_csv_publisher)/ros_imu_data.csv"/>
        <remap from="/imu" to="/imu0"/>
    </node>

    <!-- 3. OPENVINS ESTIMATOR -->
    <node name="ov_msckf" pkg="ov_msckf" type="run_subscribe_msckf" output="screen">
        <param name="config_path" value="$(find ov_msckf)/../config/euroc_mav/estimator_config.yaml"/>
        <remap from="/cam0/image_raw" to="/camera/color/image_raw"/>
    </node>

    <!-- 4. ВИЗУАЛИЗАЦИЯ -->
    <node type="rviz" name="rviz" pkg="rviz" args="-d $(find ov_msckf)/launch/display.rviz" />
</launch>
```
```
