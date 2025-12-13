#!/usr/bin/env python3
"""
Конвертер данных Sensor Logger для ROS2 + OpenVINS
Преобразует CSV файлы в единый ROS2-совместимый CSV файл
"""

import pandas as pd
import numpy as np
from pathlib import Path
import sys

def convert_sensor_logger_to_ros(input_dir=".", output_file="ros_imu_data.csv"):
    """
    Основная функция конвертации
    """
    
    # Определяем имена файлов на основе скриншота
    files = {
        'accel': 'Accelerometer.csv',
        'gyro': 'Gyroscope.csv',
        'mag': 'Magnetometer.csv',
        'orientation': 'Orientation.csv'
    }
    
    # Словарь для хранения DataFrame
    dfs = {}
    
    print("Чтение файлов...")
    
    # Читаем все доступные файлы
    for key, filename in files.items():
        filepath = Path(input_dir) / filename
        if filepath.exists():
            dfs[key] = pd.read_csv(filepath)
            print(f"  Загружен: {filename} ({len(dfs[key])} записей)")
        else:
            print(f"  Файл не найден: {filename}")
    
    # Проверяем наличие обязательных файлов
    if 'accel' not in dfs or 'gyro' not in dfs:
        print("Ошибка: отсутствуют обязательные файлы Accelerometer.csv и/или Gyroscope.csv")
        sys.exit(1)
    
    # Синхронизируем данные по времени
    print("\nСинхронизация данных по времени...")
    
    # Используем акселерометр как базовый временной ряд
    base_time = dfs['accel']['seconds_elapsed'].values
    
    # Интерполируем гироскоп
    gyro_interp = {
        'x': np.interp(base_time, 
                      dfs['gyro']['seconds_elapsed'].values, 
                      dfs['gyro']['x'].values),
        'y': np.interp(base_time, 
                      dfs['gyro']['seconds_elapsed'].values, 
                      dfs['gyro']['y'].values),
        'z': np.interp(base_time, 
                      dfs['gyro']['seconds_elapsed'].values, 
                      dfs['gyro']['z'].values)
    }
    
    # Интерполируем магнитометр если есть
    mag_interp = None
    if 'mag' in dfs:
        mag_interp = {
            'x': np.interp(base_time, 
                          dfs['mag']['seconds_elapsed'].values, 
                          dfs['mag']['x'].values),
            'y': np.interp(base_time, 
                          dfs['mag']['seconds_elapsed'].values, 
                          dfs['mag']['y'].values),
            'z': np.interp(base_time, 
                          dfs['mag']['seconds_elapsed'].values, 
                          dfs['mag']['z'].values)
        }
    
    # Интерполируем ориентацию если есть
    orient_interp = None
    if 'orientation' in dfs:
        orient_interp = {
            'qx': np.interp(base_time, 
                           dfs['orientation']['seconds_elapsed'].values, 
                           dfs['orientation']['qx'].values),
            'qy': np.interp(base_time, 
                           dfs['orientation']['seconds_elapsed'].values, 
                           dfs['orientation']['qy'].values),
            'qz': np.interp(base_time, 
                           dfs['orientation']['seconds_elapsed'].values, 
                           dfs['orientation']['qz'].values),
            'qw': np.interp(base_time, 
                           dfs['orientation']['seconds_elapsed'].values, 
                           dfs['orientation']['qw'].values)
        }
    
    # Создаем DataFrame для ROS2
    print("\nСоздание ROS2-совместимого формата...")
    
    ros_data = {
        # Временная метка в наносекундах
        'timestamp': (base_time * 1e9).astype(np.int64),
        
        # Акселерометр (м/с²)
        'accel_x': dfs['accel']['x'].values,
        'accel_y': dfs['accel']['y'].values,
        'accel_z': dfs['accel']['z'].values,
        
        # Гироскоп (рад/с)
        'gyro_x': gyro_interp['x'],
        'gyro_y': gyro_interp['y'],
        'gyro_z': gyro_interp['z']
    }
    
    # Добавляем магнитометр если есть
    if mag_interp:
        ros_data['mag_x'] = mag_interp['x']
        ros_data['mag_y'] = mag_interp['y']
        ros_data['mag_z'] = mag_interp['z']
    
    # Добавляем ориентацию если есть
    if orient_interp:
        ros_data['orientation_qx'] = orient_interp['qx']
        ros_data['orientation_qy'] = orient_interp['qy']
        ros_data['orientation_qz'] = orient_interp['qz']
        ros_data['orientation_qw'] = orient_interp['qw']
    
    # Создаем финальный DataFrame
    df_ros = pd.DataFrame(ros_data)
    
    # Сохраняем результат
    output_path = Path(output_file)
    df_ros.to_csv(output_path, index=False)
    
    print(f"\nКонвертация завершена!")
    print(f"Сохранено: {output_file}")
    print(f"Количество записей: {len(df_ros)}")
    print(f"Поля данных: {', '.join(df_ros.columns)}")
    
    return df_ros

if __name__ == "__main__":
    # Укажите путь к папке с CSV файлами
    # По умолчанию используется текущая директория
    input_directory = "."  # Замените на путь к вашим файлам если нужно
    
    # Запускаем конвертацию
    convert_sensor_logger_to_ros(input_directory, "ros_imu_data.csv")