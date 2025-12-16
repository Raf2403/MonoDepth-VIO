# Полная интеграция D435 с внешней IMU в ROS

---

## Архитектура системы

```
[D435 камера] --USB--> ROS узел realsense2_camera
                      │ выдаёт /camera/depth/image_rect_raw
                      │ выдаёт /camera/color/image_raw
                      │ выдаёт /camera/*/camera_info

[Внешняя IMU] --UART--> ROS узел imu_driver
              (микроконтроллер)  │ выдаёт /imu_raw (sensor_msgs/Imu)

                      └---> robot_localization (EKF)
                            │ принимает /imu_raw + другие источники
                            └ выдаёт /odometry/filtered
```

---

## Шаг 1. Установка необходимых пакетов

```bash
# ROS 1 (если используешь ROS Noetic / Melodic)
sudo apt update
sudo apt install ros-noetic-realsense2-camera ros-noetic-robot-localization ros-noetic-message-filters

# ROS 2 (если используешь Humble / Jazzy)
sudo apt install ros-humble-realsense2-camera ros-humble-robot-localization
```

Дополнительно для Python‑узлов:

```bash
pip install pyserial numpy scipy
```

---

## Шаг 2. Создание ROS узла для IMU (imu_driver)

Создай пакет:

```bash
# ROS 1
catkin_create_pkg imu_driver rospy sensor_msgs std_msgs

# ROS 2
ros2 pkg create --build-type ament_cmake imu_driver
```

### ROS 1: `scripts/imu_driver.py`

```python
#!/usr/bin/env python3

import rospy
import serial
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import traceback

class IMUDriver:
    def __init__(self):
        rospy.init_node('imu_driver', anonymous=False)
        
        # Параметры из roslaunch
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.imu_topic = rospy.get_param('~imu_topic', '/imu_raw')
        
        # Издатель
        self.pub_imu = rospy.Publisher(self.imu_topic, Imu, queue_size=10)
        
        # Коннект к порту
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            rospy.loginfo(f"IMU driver: Подключено к {self.port} @ {self.baudrate} baud")
        except Exception as e:
            rospy.logerr(f"Ошибка подключения: {e}")
            exit(1)
    
    def parse_imu_data(self, line):
        """
        Парсинг строки от микроконтроллера.
        Формат: ax,ay,az,gx,gy,gz
        (где ускорение в м/с^2, угловая скорость в рад/с)
        """
        try:
            parts = line.strip().split(',')
            if len(parts) >= 6:
                ax, ay, az, gx, gy, gz = map(float, parts[:6])
                return ax, ay, az, gx, gy, gz
        except ValueError:
            pass
        return None
    
    def run(self):
        rate = rospy.Rate(100)  # 100 Гц (или настраивай под свой IMU)
        
        while not rospy.is_shutdown():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    
                    data = self.parse_imu_data(line)
                    if data:
                        ax, ay, az, gx, gy, gz = data
                        
                        # Создать сообщение
                        imu_msg = Imu()
                        imu_msg.header.stamp = rospy.Time.now()
                        imu_msg.header.frame_id = self.frame_id
                        
                        # Линейное ускорение (м/с^2)
                        imu_msg.linear_acceleration.x = ax
                        imu_msg.linear_acceleration.y = ay
                        imu_msg.linear_acceleration.z = az
                        
                        # Угловая скорость (рад/с)
                        imu_msg.angular_velocity.x = gx
                        imu_msg.angular_velocity.y = gy
                        imu_msg.angular_velocity.z = gz
                        
                        # Ковариационные матрицы (подобрать эмпирически)
                        imu_msg.linear_acceleration_covariance = [0.01, 0, 0,
                                                                   0, 0.01, 0,
                                                                   0, 0, 0.01]
                        imu_msg.angular_velocity_covariance = [0.001, 0, 0,
                                                               0, 0.001, 0,
                                                               0, 0, 0.001]
                        
                        self.pub_imu.publish(imu_msg)
            
            except Exception as e:
                rospy.logwarn(f"Ошибка чтения: {e}")
                traceback.print_exc()
            
            rate.sleep()
    
    def shutdown(self):
        if self.ser.is_open:
            self.ser.close()
        rospy.loginfo("IMU driver: Отключено")

if __name__ == '__main__':
    driver = IMUDriver()
    rospy.on_shutdown(driver.shutdown)
    driver.run()
```

### ROS 1: `launch/imu_driver.launch`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <node pkg="imu_driver" type="imu_driver.py" name="imu_driver" output="screen">
        <param name="port" value="/dev/ttyUSB0" />
        <param name="baudrate" value="115200" />
        <param name="frame_id" value="imu_link" />
        <param name="imu_topic" value="/imu_raw" />
    </node>
</launch>
```

---

## Шаг 3. Конфигурация robot_localization (EKF)

### ROS 1: `config/ekf.yaml`

```yaml
# Параметры узла
ekf_filter_node:
  use_sim_time: false
  frequency: 30
  sensor_timeout: 0.1
  two_d_mode: false  # false для 3D, true для 2D-робота
  transform_time_offset: 0.0
  print_diagnostics: true
  debug: false
  debug_out_file: /tmp/robot_localization_debug.txt

  # Матрица переходов (шум процесса)
  process_noise_covariance: [
    1e-2, 0,    0,    0,    0,    0,    0,     0,     0,    0, 0, 0,
    0,    1e-2, 0,    0,    0,    0,    0,     0,     0,    0, 0, 0,
    0,    0,    1e-2, 0,    0,    0,    0,     0,     0,    0, 0, 0,
    0,    0,    0,    1e-2, 0,    0,    0,     0,     0,    0, 0, 0,
    0,    0,    0,    0,    1e-2, 0,    0,     0,     0,    0, 0, 0,
    0,    0,    0,    0,    0,    1e-2, 0,     0,     0,    0, 0, 0,
    0,    0,    0,    0,    0,    0,    1e-4,  0,     0,    0, 0, 0,
    0,    0,    0,    0,    0,    0,    0,     1e-4,  0,    0, 0, 0,
    0,    0,    0,    0,    0,    0,    0,     0,     1e-4, 0, 0, 0,
    0,    0,    0,    0,    0,    0,    0,     0,     0,    1e-4, 0, 0,
    0,    0,    0,    0,    0,    0,    0,     0,     0,    0, 1e-4, 0,
    0,    0,    0,    0,    0,    0,    0,     0,     0,    0, 0, 1e-4
  ]

  # Шум измерений (начальные оценки, подобрать позже)
  initial_estimate_covariance: [
    1e-9, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,
    0,    1e-9, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,
    0,    0,    1e-9, 0,    0,    0,    0,     0,     0,    0,    0,    0,
    0,    0,    0,    1e-9, 0,    0,    0,     0,     0,    0,    0,    0,
    0,    0,    0,    0,    1e-9, 0,    0,     0,     0,    0,    0,    0,
    0,    0,    0,    0,    0,    1e-9, 0,     0,     0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    1e-9,  0,     0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,     1e-9,  0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,     0,     1e-9, 0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,     0,     0,    1e-9, 0,    0,
    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    1e-9, 0,
    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    1e-9
  ]

  # ============ IMU ============
  imu0: /imu_raw
  imu0_config: [
    false, false, false,  # x, y, z (положение)
    true,  true,  true,   # roll, pitch, yaw (ориентация)
    true,  true,  true    # ang_vel_x, ang_vel_y, ang_vel_z (угловая скорость)
  ]
  imu0_nodelay: false
  imu0_differential: false
  imu0_relative: true
  imu0_queue_size: 10
  imu0_remove_gravitational_acceleration: true

  # ============ Одометрия (если есть) ============
  # odom0: /visual_odom  # или /odom от другого источника
  # odom0_config: [
  #   true,  true,  false,  # x, y, z
  #   false, false, true,   # roll, pitch, yaw
  #   false, false, false   # угловые скорости
  # ]
  # odom0_nodelay: false
  # odom0_differential: false
  # odom0_relative: false
  # odom0_queue_size: 10

  # ============ Выходные параметры ============
  output_final_estimate: true
  output_location: "world"  # или "odom"
```

### ROS 1: `launch/robot_localization.launch`

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
    <!-- Запустить realsense2_camera -->
    <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="camera" value="camera" />
    </include>

    <!-- Запустить imu_driver -->
    <include file="$(find imu_driver)/launch/imu_driver.launch" />

    <!-- Запустить EKF -->
    <node pkg="robot_localization" type="ekf_node" name="ekf_filter_node" clear_params="true">
        <rosparam command="load" file="$(find my_robot_pkg)/config/ekf.yaml" />
        <remap from="odometry/filtered" to="odometry/filtered" />
    </node>

    <!-- TF: static transform между IMU и камерой -->
    <!-- Измерить расстояние IMU от камеры и заполнить ниже -->
    <node pkg="tf" type="static_transform_publisher" name="imu_to_camera_tf"
          args="0 0 0 0 0 0 camera_link imu_link 100" />
    <!-- args: x y z roll pitch yaw parent_frame child_frame rate -->
</launch>
```

---

## Шаг 4. Трансформы (TF) между датчиками

Это **критично**: EKF должна знать, как расположены IMU и камера друг относительно друга.

### Измерить физическое расположение

1. Возьми рулетку, измерь расстояния IMU от камеры (в метрах):
   - смещение по X (вперёд/назад),
   - смещение по Y (влево/вправо),
   - смещение по Z (вверх/вниз).

2. Если IMU повёрнута относительно камеры, измерь углы (в радианах):
   - roll (вращение вокруг X),
   - pitch (вращение вокруг Y),
   - yaw (вращение вокруг Z).

Пример: IMU висит под камерой, смещение на 5 см вниз (Z = -0.05 м), без поворотов:

```xml
<node pkg="tf" type="static_transform_publisher" name="camera_to_imu_tf"
      args="0 0 -0.05 0 0 0 camera_link imu_link 100" />
```

Если IMU поворачивается на 90° вокруг Y (pitch = π/2 ≈ 1.57):

```xml
<node pkg="tf" type="static_transform_publisher" name="camera_to_imu_tf"
      args="0.05 0 0 0 1.57 0 camera_link imu_link 100" />
```

---

## Шаг 5. Запуск всего

```bash
# Открыть 4 терминала:

# Терминал 1: запустить основной launch-файл
roslaunch my_robot_pkg robot_localization.launch

# Терминал 2: проверить топики (опционально)
rostopic list
# Должны быть: /imu_raw, /camera/depth/image_rect_raw, /odometry/filtered

# Терминал 3: visualize в RViz
rviz

# В RViz:
# 1. Fixed Frame: odom
# 2. Add Display -> Image -> /camera/color/image_raw
# 3. Add Display -> PointCloud2 -> /camera/depth/points (если доступно)
# 4. Add Display -> Odometry -> /odometry/filtered
```

---

## Шаг 6. Отладка и калибровка

### Проверить, есть ли данные

```bash
rostopic hz /imu_raw
# Должно быть ~100 Гц

rostopic hz /odometry/filtered
# Должно быть ~30 Гц (частота EKF)

rostopic echo /imu_raw | head -20
# Должны видеть изменяющиеся значения acc и gyro
```

### Если IMU «шумит» или дрейфит

Изменить в `ekf.yaml`:

```yaml
# Увеличить доверие к IMU (уменьшить коварианс шума):
imu0_config: [
  false, false, false,
  true,  true,  true,
  true,  true,  true
]

# Увеличить шум процесса, если фильтр отстаёт:
process_noise_covariance: [
  1e-1, 0, ...  # увеличить с 1e-2 на 1e-1
]
```

### Если камера и IMU не синхронны

Добавить в коде Python временную синхронизацию:

```python
# В imu_driver.py, вместо rospy.Time.now():
import time
start_time = rospy.Time.now()
start_clock = time.time()

# В цикле:
elapsed = time.time() - start_clock
imu_msg.header.stamp = start_time + rospy.Duration(elapsed)
```

---

## Шаг 7. Использование фьюзированных данных

### Для SLAM (rtabmap, ORB-SLAM3 и т.д.)

```bash
roslaunch rtabmap_ros rtabmap.launch \
  depth_topic:=/camera/depth/image_rect_raw \
  rgb_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  imu_topic:=/imu_raw \
  approx_sync:=true
```

### Для простой навигации

Использовать `/odometry/filtered` как основной источник для `move_base`:

```xml
<param name="odom_frame_id" value="odom" />
<param name="base_frame_id" value="base_link" />
<remap from="odom" to="odometry/filtered" />
```

---
