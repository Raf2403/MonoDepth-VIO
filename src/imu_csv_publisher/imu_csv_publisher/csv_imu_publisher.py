#!/usr/bin/env python3
"""
ROS2 нода для воспроизведения IMU данных из CSV файла
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import pandas as pd
import numpy as np
from pathlib import Path
import sys

class CsvImuPublisher(Node):
    def __init__(self):
        super().__init__('csv_imu_publisher')
        
        # Параметры
        self.declare_parameter('csv_file', 'ros_imu_data.csv')
        self.declare_parameter('publish_frequency', 100.0)  # Hz
        self.declare_parameter('publish_mag', True)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Получаем параметры
        csv_file = self.get_parameter('csv_file').value
        self.publish_freq = self.get_parameter('publish_frequency').value
        publish_mag = self.get_parameter('publish_mag').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Загрузка данных
        self.get_logger().info(f"Загрузка данных из {csv_file}")
        try:
            self.data = pd.read_csv(csv_file)
        except FileNotFoundError:
            self.get_logger().error(f"Файл {csv_file} не найден")
            sys.exit(1)
            
        self.get_logger().info(f"Загружено {len(self.data)} записей")
        
        # Проверка столбцов
        required = ['timestamp', 'accel_x', 'accel_y', 'accel_z', 
                   'gyro_x', 'gyro_y', 'gyro_z']
        missing = [col for col in required if col not in self.data.columns]
        if missing:
            self.get_logger().error(f"Отсутствуют столбцы: {missing}")
            sys.exit(1)
        
        # Публикаторы
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        if publish_mag and all(col in self.data.columns for col in ['mag_x', 'mag_y', 'mag_z']):
            self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
            self.has_mag = True
            self.get_logger().info("Магнитометр будет опубликован")
        else:
            self.has_mag = False
            self.get_logger().info("Магнитометр не будет опубликован")
        
        # Инициализация индекса
        self.current_index = 0
        self.data_length = len(self.data)
        
        # Рассчитываем период таймера
        timer_period = 1.0 / self.publish_freq
        
        # Создаем таймер для публикации
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Время начала
        self.start_time = self.get_clock().now()
        self.first_timestamp = self.data['timestamp'].iloc[0]
        
        self.get_logger().info(f"Начало публикации с частотой {self.publish_freq} Hz")
    
    def timer_callback(self):
        if self.current_index >= self.data_length:
            self.get_logger().info("Все данные опубликованы")
            self.timer.cancel()
            return
        
        # Публикуем текущую запись
        self.publish_current_data()
        self.current_index += 1
        
        # Логирование прогресса
        if self.current_index % 500 == 0:
            progress = (self.current_index / self.data_length) * 100
            self.get_logger().info(f"Прогресс: {progress:.1f}% ({self.current_index}/{self.data_length})")
    
    def publish_current_data(self):
        """Публикует данные текущего индекса"""
        idx = self.current_index
        
        # Получаем временную метку
        timestamp_ns = int(self.data['timestamp'].iloc[idx])
        
        # Создаем IMU сообщение
        imu_msg = Imu()
        
        # Заполняем заголовок
        imu_msg.header.stamp.sec = timestamp_ns // 1_000_000_000
        imu_msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        imu_msg.header.frame_id = self.frame_id
        
        # Линейное ускорение (м/с²)
        imu_msg.linear_acceleration.x = float(self.data['accel_x'].iloc[idx])
        imu_msg.linear_acceleration.y = float(self.data['accel_y'].iloc[idx])
        imu_msg.linear_acceleration.z = float(self.data['accel_z'].iloc[idx])
        
        # Угловая скорость (рад/с)
        imu_msg.angular_velocity.x = float(self.data['gyro_x'].iloc[idx])
        imu_msg.angular_velocity.y = float(self.data['gyro_y'].iloc[idx])
        imu_msg.angular_velocity.z = float(self.data['gyro_z'].iloc[idx])
        
        # Ковариационные матрицы (единичные по умолчанию)
        imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                                  0.0, 0.01, 0.0,
                                                  0.0, 0.0, 0.01]
        
        imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                               0.0, 0.01, 0.0,
                                               0.0, 0.0, 0.01]
        
        # Ориентация если есть
        orient_cols = ['orientation_qx', 'orientation_qy', 'orientation_qz', 'orientation_qw']
        if all(col in self.data.columns for col in orient_cols):
            imu_msg.orientation.x = float(self.data['orientation_qx'].iloc[idx])
            imu_msg.orientation.y = float(self.data['orientation_qy'].iloc[idx])
            imu_msg.orientation.z = float(self.data['orientation_qz'].iloc[idx])
            imu_msg.orientation.w = float(self.data['orientation_qw'].iloc[idx])
            imu_msg.orientation_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]
        else:
            # Если нет ориентации, заполняем нулями
            imu_msg.orientation.w = 1.0
            imu_msg.orientation_covariance = [-1.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0,
                                              0.0, 0.0, 0.0]
        
        # Публикация IMU
        self.imu_pub.publish(imu_msg)
        
        # Публикация магнитометра если есть
        if self.has_mag:
            mag_msg = MagneticField()
            mag_msg.header.stamp.sec = timestamp_ns // 1_000_000_000
            mag_msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
            mag_msg.header.frame_id = self.frame_id
            
            # Конвертация из микротесла в тесла
            mag_msg.magnetic_field.x = float(self.data['mag_x'].iloc[idx]) * 1e-6
            mag_msg.magnetic_field.y = float(self.data['mag_y'].iloc[idx]) * 1e-6
            mag_msg.magnetic_field.z = float(self.data['mag_z'].iloc[idx]) * 1e-6
            
            # Ковариация магнитометра
            mag_msg.magnetic_field_covariance = [0.01, 0.0, 0.0,
                                                 0.0, 0.01, 0.0,
                                                 0.0, 0.0, 0.01]
            
            self.mag_pub.publish(mag_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = CsvImuPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка по запросу пользователя")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()