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

## Исправление ошибок первого запуска (Критично!)

Из-за несовместимости версий OpenCV внутри Docker-контейнера и на хосте, при первом запуске может возникать ошибка `core dumped` или `Input file is invalid`.
Необходимо один раз пересобрать пакет внутри контейнера с ограничением памяти.

1.  Запустите контейнер:
    ```
    docker start -ai <имя_вашего_контейнера>
    ```

2.  Внутри контейнера выполните пересборку (флаг `-j 1` обязателен, чтобы избежать вылета по нехватке памяти):
    ```
    cd /catkin_ws
    catkin build ov_msckf --no-deps -j 1 -DCMAKE_BUILD_TYPE=Release
    source devel/setup.bash
    ```

---

## Инструкция по запуску (Runtime)

Для работы системы потребуются два терминала, подключенных к одному контейнеру.

### Терминал 1: Драйверы сенсоров и ROS Core

1.  Подключитесь к работающему контейнеру:
    ```
    sudo docker exec -it <имя_вашего_контейнера> bash
    source /catkin_ws/devel/setup.bash
    ```

2.  Запустите компоненты:
    
    ```
    # 1. Запуск ядра ROS
    roscore &
    sleep 3

    # 2. Запуск камеры RealSense (RGB 640x480, 30fps, без встроенного IMU)
    roslaunch realsense2_camera rs_camera.launch \
      enable_color:=true \
      color_width:=640 \
      color_height:=480 \
      color_fps:=30 \
      enable_gyro:=false \
      enable_accel:=false &

    # 3. Запуск моста данных с iPhone (IMU)
    # (Замените на вашу команду запуска скрипта, который публикует в /iphone/imu)
    # python3 iphone_bridge.py &
    ```

3.  Проверьте наличие топиков:
    ```
    rostopic list
    # Вы должны увидеть: /camera/color/image_raw И /iphone/imu
    ```

### Терминал 2: Алгоритм VIO (OpenVINS)

1.  Подключитесь к тому же контейнеру в новом окне:
    ```
    sudo docker exec -it <имя_вашего_контейнера> bash
    source /catkin_ws/devel/setup.bash
    ```

2.  Сгенерируйте XML-конфигурацию (Обязательно при каждом перезапуске контейнера):
    Использование XML вместо YAML обходит баг парсера OpenCV 4.2.0. Выполните этот блок целиком:

    ```
    python3 -c '
    xml_content = """<?xml version="1.0"?>
    <opencv_storage>
    <verbosity>INFO</verbosity>
    <use_stereo>0</use_stereo>
    <max_cameras>1</max_cameras>
    <topic_imu>/iphone/imu</topic_imu>
    <topic_camera0>/camera/color/image_raw</topic_camera0>
    <use_fej>1</use_fej>
    <integration>rk4</integration>
    <perform_online_calibration>1</perform_online_calibration>
    <estimate_td>1</estimate_td>
    <estimate_extrinsics>1</estimate_extrinsics>
    <calib_cam_extrinsics>1</calib_cam_extrinsics>
    <calib_cam_intrinsics>0</calib_cam_intrinsics>
    <calib_cam_timeoffset>1</calib_cam_timeoffset>
    <calib_imu_intrinsics>1</calib_imu_intrinsics>
    <calib_imu_g_sensitivity>0</calib_imu_g_sensitivity>
    <relative_config_imu>0</relative_config_imu>
    <relative_config_imucam>0</relative_config_imucam>
    <accelerometer_noise_density>2.00e-03</accelerometer_noise_density>
    <accelerometer_random_walk>3.00e-03</accelerometer_random_walk>
    <gyroscope_noise_density>1.70e-04</gyroscope_noise_density>
    <gyroscope_random_walk>2.00e-05</gyroscope_random_walk>
    <max_imu_rate>100</max_imu_rate>
    <gravity_mag>9.81</gravity_mag>
    <cam0_is_fisheye>0</cam0_is_fisheye>
    <cam0_resolution>640 480</cam0_resolution>
    <cam0_intrinsics>386.0 386.0 320.0 240.0</cam0_intrinsics>
    <cam0_distortion_model>radial-tangential</cam0_distortion_model>
    <cam0_distortion_coeffs>0.0 0.0 0.0 0.0</cam0_distortion_coeffs>
    <T_C0toI type_id="opencv-matrix">
      <rows>4</rows>
      <cols>4</cols>
      <dt>d</dt>
      <data>1. 0. 0. 0. 0. 1. 0. 0. 0. 0. 1. 0. 0. 0. 0. 1.</data>
    </T_C0toI>
    <init_window_time>1.0</init_window_time>
    <init_imu_thresh>1.0</init_imu_thresh>
    <max_clones>11</max_clones>
    <max_slam>50</max_slam>
    <max_slam_in_update>25</max_slam_in_update>
    <max_msckf_in_update>40</max_msckf_in_update>
    <feat_rep_msckf>GLOBAL_3D</feat_rep_msckf>
    <feat_rep_slam>GLOBAL_3D</feat_rep_slam>
    <feat_rep_aruco>GLOBAL_3D</feat_rep_aruco>
    <num_aruco>0</num_aruco>
    </opencv_storage>
    """
    with open("/tmp/config.xml", "w") as f:
        f.write(xml_content)
    print("XML config generated.")
    '
    ```

3.  Запустите алгоритм:
    ```
    rosrun ov_msckf run_subscribe_msckf /tmp/config.xml
    ```
