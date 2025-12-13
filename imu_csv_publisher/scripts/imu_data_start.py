import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

def quick_sensor_logger_analysis(folder_path):
    """
    Быстрый анализ всех файлов Sensor Logger в папке
    """
    print("🚀 БЫСТРЫЙ АНАЛИЗ SENSOR LOGGER")
    print("="*50)
    
    # Основные файлы для анализа
    main_files = [
        'Accelerometer.csv',
        'Gyroscope.csv', 
        'Magnetometer.csv',
        'Orientation.csv'
    ]
    
    for filename in main_files:
        filepath = os.path.join(folder_path, filename)
        
        if os.path.exists(filepath):
            print(f"\n📊 {filename}:")
            df = pd.read_csv(filepath)
            
            print(f"   Записей: {len(df)}")
            print(f"   Колонки: {list(df.columns)}")
            
            # Временные метки
            if 'time' in df.columns:
                duration = df['time'].max() - df['time'].min()
                avg_rate = len(df) / duration if duration > 0 else 0
                print(f"   Длительность: {duration:.2f} сек")
                print(f"   Средняя частота: {avg_rate:.1f} Hz")
            
            # Показать статистику для числовых колонок
            numeric_cols = df.select_dtypes(include=[np.number]).columns
            for col in numeric_cols[:3]:  # Первые 3 числовые колонки
                if col != 'time':
                    print(f"   {col}: μ={df[col].mean():.4f}, σ={df[col].std():.4f}")
    
    # Визуализация основных датчиков
    plot_sensors(folder_path)

def plot_sensors(folder_path):
    """
    Построение графиков для основных датчиков
    """
    sensors = {
        'Accelerometer.csv': ['x', 'y', 'z'],
        'Gyroscope.csv': ['x', 'y', 'z'],
        'Magnetometer.csv': ['x', 'y', 'z']
    }
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    for idx, (filename, axes_labels) in enumerate(sensors.items()):
        filepath = os.path.join(folder_path, filename)
        
        if os.path.exists(filepath):
            df = pd.read_csv(filepath)
            sensor_name = filename.replace('.csv', '')
            
            ax = axes[idx]
            
            # Ищем колонки с данными
            data_cols = []
            for col in df.columns:
                col_lower = col.lower()
                if any(label in col_lower for label in axes_labels) and df[col].dtype in [np.float64, np.float32]:
                    data_cols.append(col)
            
            # Строим график
            for col in data_cols[:3]:  # Берем первые 3
                ax.plot(df.index[:500], df[col].values[:500], label=col, alpha=0.8, linewidth=1)
            
            ax.set_title(f'{sensor_name} (первые 500 точек)')
            ax.set_xlabel('Номер измерения')
            ax.set_ylabel('Значение')
            ax.legend(loc='upper right', fontsize=8)
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('sensor_logger_quick_plot.png', dpi=120, bbox_inches='tight')
    plt.show()
    print("\n📈 График сохранен как 'sensor_logger_quick_plot.png'")

# Использование
folder_path = input("Введите путь к папке с данными: ").strip()
if os.path.exists(folder_path):
    quick_sensor_logger_analysis(folder_path)
else:
    print(f"Папка не найдена: {folder_path}")