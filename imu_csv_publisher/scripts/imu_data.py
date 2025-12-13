import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os
import glob
from datetime import datetime
import json

class SensorLoggerAnalyzer:
    def __init__(self, data_folder):
        """
        Инициализация анализатора для папки с данными Sensor Logger
        
        Args:
            data_folder: Путь к папке с CSV файлами
        """
        self.data_folder = data_folder
        self.data = {}
        self.metadata = {}
        self.sampling_rates = {}
        
        # Сопоставление файлов и их типов
        self.file_mapping = {
            'Accelerometer.csv': 'accelerometer',
            'AccelerometerUncalibrated.csv': 'accelerometer_uncalibrated',
            'Gyroscope.csv': 'gyroscope',
            'GyroscopeUncalibrated.csv': 'gyroscope_uncalibrated',
            'Magnetometer.csv': 'magnetometer',
            'MagnetometerUncalibrated.csv': 'magnetometer_uncalibrated',
            'Gravity.csv': 'gravity',
            'Orientation.csv': 'orientation',
            'Annotation.csv': 'annotations',
            'Metadata.csv': 'metadata'
        }
    
    def load_all_data(self):
        """
        Загрузка всех CSV файлов из папки
        """
        print(f"📁 Загрузка данных из папки: {self.data_folder}")
        print("="*60)
        
        loaded_files = 0
        for filename, sensor_type in self.file_mapping.items():
            filepath = os.path.join(self.data_folder, filename)
            
            if os.path.exists(filepath):
                try:
                    print(f"📄 Загрузка {filename}...", end=" ")
                    df = pd.read_csv(filepath)
                    
                    # Сохраняем данные
                    self.data[sensor_type] = df
                    
                    # Анализ временных меток для определения частоты дискретизации
                    if 'time' in df.columns:
                        self._analyze_timing(df, sensor_type)
                    
                    print(f"✓ Успешно ({len(df)} записей)")
                    loaded_files += 1
                    
                except Exception as e:
                    print(f"✗ Ошибка: {e}")
            else:
                print(f"📄 {filename} - файл не найден")
        
        print(f"\n✅ Загружено файлов: {loaded_files}/{len(self.file_mapping)}")
        
        # Загружаем метаданные если есть
        if 'metadata' in self.data:
            self._parse_metadata()
    
    def _analyze_timing(self, df, sensor_type):
        """
        Анализ временных меток для определения частоты дискретизации
        """
        if 'time' in df.columns and len(df) > 1:
            time_diff = np.diff(df['time'].values)
            if len(time_diff) > 0:
                self.sampling_rates[sensor_type] = {
                    'rate': 1.0 / np.mean(time_diff),
                    'min_interval': np.min(time_diff),
                    'max_interval': np.max(time_diff),
                    'std_interval': np.std(time_diff)
                }
    
    def _parse_metadata(self):
        """
        Парсинг метаданных
        """
        if 'metadata' in self.data:
            df = self.data['metadata']
            for _, row in df.iterrows():
                key = row.get('key', '')
                value = row.get('value', '')
                if key and pd.notna(value):
                    self.metadata[key] = value
    
    def print_summary(self):
        """
        Вывод сводной информации о загруженных данных
        """
        print("\n" + "="*60)
        print("📊 СВОДНАЯ ИНФОРМАЦИЯ О ДАННЫХ")
        print("="*60)
        
        print(f"\n📱 Метаданные устройства:")
        for key, value in self.metadata.items():
            print(f"  {key}: {value}")
        
        print(f"\n📈 Загруженные датчики:")
        for sensor_type, df in self.data.items():
            if sensor_type != 'metadata':
                print(f"\n  🎯 {sensor_type.upper()}:")
                print(f"    Количество записей: {len(df)}")
                
                if sensor_type in self.sampling_rates:
                    rate_info = self.sampling_rates[sensor_type]
                    print(f"    Частота дискретизации: {rate_info['rate']:.2f} Hz")
                    print(f"    Интервал: {rate_info['min_interval']:.4f} - {rate_info['max_interval']:.4f} сек")
                
                print(f"    Колонки: {list(df.columns)}")
                
                # Показываем диапазон значений для числовых колонок
                numeric_cols = df.select_dtypes(include=[np.number]).columns
                for col in numeric_cols[:3]:  # Первые 3 числовые колонки
                    if col not in ['time', 'seconds_elapsed']:
                        print(f"    {col}: [{df[col].min():.4f}, {df[col].max():.4f}]")
    
    def visualize_sensor_data(self, sensor_type, start_idx=0, num_points=1000):
        """
        Визуализация данных конкретного датчика
        
        Args:
            sensor_type: Тип датчика (например, 'accelerometer')
            start_idx: Начальный индекс
            num_points: Количество точек для отображения
        """
        if sensor_type not in self.data:
            print(f"Датчик {sensor_type} не найден")
            return
        
        df = self.data[sensor_type]
        
        # Определяем колонки для отображения
        data_cols = []
        for col in df.columns:
            if col not in ['time', 'seconds_elapsed'] and df[col].dtype in [np.float64, np.float32, np.int64]:
                data_cols.append(col)
        
        if not data_cols:
            print("Нет числовых данных для отображения")
            return
        
        # Создаем график
        fig, axes = plt.subplots(len(data_cols), 1, figsize=(12, 3*len(data_cols)))
        if len(data_cols) == 1:
            axes = [axes]
        
        # Временная ось
        if 'time' in df.columns:
            time_data = df['time'].values[start_idx:start_idx+num_points]
            xlabel = 'Время (секунды)'
        else:
            time_data = np.arange(start_idx, min(start_idx+num_points, len(df)))
            xlabel = 'Номер измерения'
        
        for idx, col in enumerate(data_cols[:4]):  # Максимум 4 графика
            ax = axes[idx]
            data = df[col].values[start_idx:start_idx+num_points]
            
            ax.plot(time_data, data, linewidth=1, alpha=0.8)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(col)
            ax.set_title(f'{sensor_type.upper()}: {col}')
            ax.grid(True, alpha=0.3)
            
            # Добавляем статистику на график
            stats_text = f'μ={np.mean(data):.4f}, σ={np.std(data):.4f}'
            ax.text(0.02, 0.95, stats_text, transform=ax.transAxes, 
                   fontsize=10, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        plt.savefig(f'{sensor_type}_plot.png', dpi=120, bbox_inches='tight')
        plt.show()
        print(f"График сохранен как '{sensor_type}_plot.png'")
    
    def compare_calibrated_vs_uncalibrated(self):
        """
        Сравнение калиброванных и некалиброванных данных
        """
        comparisons = [
            ('accelerometer', 'accelerometer_uncalibrated'),
            ('gyroscope', 'gyroscope_uncalibrated'),
            ('magnetometer', 'magnetometer_uncalibrated')
        ]
        
        for calibrated, uncalibrated in comparisons:
            if calibrated in self.data and uncalibrated in self.data:
                print(f"\n🔍 Сравнение {calibrated} vs {uncalibrated}:")
                
                df_cal = self.data[calibrated]
                df_uncal = self.data[uncalibrated]
                
                # Находим общие колонки (исключая временные)
                common_cols = []
                for col in df_cal.columns:
                    if col in df_uncal.columns and col not in ['time', 'seconds_elapsed']:
                        common_cols.append(col)
                
                if common_cols:
                    fig, axes = plt.subplots(len(common_cols), 2, figsize=(15, 3*len(common_cols)))
                    
                    for idx, col in enumerate(common_cols[:3]):  # Первые 3 колонки
                        # Гистограммы
                        ax1 = axes[idx, 0] if len(common_cols) > 1 else axes[0]
                        ax1.hist(df_cal[col].dropna(), bins=50, alpha=0.7, label='Калиброванный', density=True)
                        ax1.hist(df_uncal[col].dropna(), bins=50, alpha=0.7, label='Некалиброванный', density=True)
                        ax1.set_xlabel(col)
                        ax1.set_ylabel('Плотность')
                        ax1.set_title(f'Распределение: {col}')
                        ax1.legend()
                        ax1.grid(True, alpha=0.3)
                        
                        # Разность
                        ax2 = axes[idx, 1] if len(common_cols) > 1 else axes[1]
                        # Для сравнения нужно выровнять данные по времени
                        if len(df_cal) > 100 and len(df_uncal) > 100:
                            diff = df_cal[col].values[:100] - df_uncal[col].values[:100]
                            ax2.plot(diff, 'r-', alpha=0.7)
                            ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
                            ax2.set_xlabel('Номер измерения')
                            ax2.set_ylabel('Разность (кал - некал)')
                            ax2.set_title(f'Разность: {col}')
                            ax2.grid(True, alpha=0.3)
                    
                    plt.tight_layout()
                    plt.savefig(f'comparison_{calibrated}.png', dpi=120, bbox_inches='tight')
                    plt.show()
    
    def analyze_imu_triad(self):
        """
        Комплексный анализ данных акселерометра, гироскопа и магнитометра
        """
        print("\n" + "="*60)
        print("🎯 КОМПЛЕКСНЫЙ АНАЛИЗ IMU")
        print("="*60)
        
        # Проверяем наличие основных датчиков
        has_acc = 'accelerometer' in self.data
        has_gyro = 'gyroscope' in self.data
        has_mag = 'magnetometer' in self.data
        
        print(f"✓ Акселерометр: {'Да' if has_acc else 'Нет'}")
        print(f"✓ Гироскоп: {'Да' if has_gyro else 'Нет'}")
        print(f"✓ Магнитометр: {'Да' if has_mag else 'Нет'}")
        
        if not (has_acc and has_gyro):
            print("\n⚠️  Для полного анализа IMU необходимы акселерометр и гироскоп")
            return
        
        # Создаем комплексный график
        fig = make_subplots(
            rows=3, cols=1,
            subplot_titles=('Акселерометр (g)', 'Гироскоп (°/s)', 'Магнитометр (μT)'),
            vertical_spacing=0.1
        )
        
        # Акселерометр
        if has_acc:
            df_acc = self.data['accelerometer']
            acc_cols = [col for col in df_acc.columns if 'x' in col.lower() or 'y' in col.lower() or 'z' in col.lower()]
            
            for col in acc_cols[:3]:  # XYZ компоненты
                axis_label = col[-1].upper() if col[-1] in ['x', 'y', 'z'] else col
                fig.add_trace(
                    go.Scatter(
                        x=df_acc.index[:1000],
                        y=df_acc[col].values[:1000],
                        name=f'Acc {axis_label}',
                        line=dict(width=1)
                    ),
                    row=1, col=1
                )
        
        # Гироскоп
        if has_gyro:
            df_gyro = self.data['gyroscope']
            gyro_cols = [col for col in df_gyro.columns if 'x' in col.lower() or 'y' in col.lower() or 'z' in col.lower()]
            
            for col in gyro_cols[:3]:
                axis_label = col[-1].upper() if col[-1] in ['x', 'y', 'z'] else col
                fig.add_trace(
                    go.Scatter(
                        x=df_gyro.index[:1000],
                        y=df_gyro[col].values[:1000],
                        name=f'Gyro {axis_label}',
                        line=dict(width=1)
                    ),
                    row=2, col=1
                )
        
        # Магнитометр
        if has_mag:
            df_mag = self.data['magnetometer']
            mag_cols = [col for col in df_mag.columns if 'x' in col.lower() or 'y' in col.lower() or 'z' in col.lower()]
            
            for col in mag_cols[:3]:
                axis_label = col[-1].upper() if col[-1] in ['x', 'y', 'z'] else col
                fig.add_trace(
                    go.Scatter(
                        x=df_mag.index[:1000],
                        y=df_mag[col].values[:1000],
                        name=f'Mag {axis_label}',
                        line=dict(width=1)
                    ),
                    row=3, col=1
                )
        
        fig.update_layout(
            height=900,
            title_text="Комплексные данные IMU (первые 1000 точек)",
            showlegend=True
        )
        
        fig.write_html("imu_triad_analysis.html")
        print("\n✅ Интерактивный график сохранен как 'imu_triad_analysis.html'")
        fig.show()
    
    def calculate_basic_orientation(self):
        """
        Расчет базовой ориентации из данных IMU
        """
        if 'accelerometer' not in self.data or 'gyroscope' not in self.data:
            print("Для расчета ориентации нужны акселерометр и гироскоп")
            return
        
        df_acc = self.data['accelerometer']
        df_gyro = self.data['gyroscope']
        
        # Находим колонки XYZ
        acc_x_col = next((col for col in df_acc.columns if 'x' in col.lower()), None)
        acc_y_col = next((col for col in df_acc.columns if 'y' in col.lower()), None)
        acc_z_col = next((col for col in df_acc.columns if 'z' in col.lower()), None)
        
        gyro_x_col = next((col for col in df_gyro.columns if 'x' in col.lower()), None)
        gyro_y_col = next((col for col in df_gyro.columns if 'y' in col.lower()), None)
        gyro_z_col = next((col for col in df_gyro.columns if 'z' in col.lower()), None)
        
        if not all([acc_x_col, acc_y_col, acc_z_col, gyro_x_col, gyro_y_col, gyro_z_col]):
            print("Не все XYZ компоненты найдены в данных")
            return
        
        # Берем первые N точек для анализа
        n_points = min(500, len(df_acc), len(df_gyro))
        
        # Извлекаем данные
        acc_x = df_acc[acc_x_col].values[:n_points]
        acc_y = df_acc[acc_y_col].values[:n_points]
        acc_z = df_acc[acc_z_col].values[:n_points]
        
        gyro_x = df_gyro[gyro_x_col].values[:n_points]
        gyro_y = df_gyro[gyro_y_col].values[:n_points]
        gyro_z = df_gyro[gyro_z_col].values[:n_points]
        
        # Расчет углов из акселерометра (в радианах)
        pitch_acc = np.arctan2(-acc_x, np.sqrt(acc_y**2 + acc_z**2))
        roll_acc = np.arctan2(acc_y, acc_z)
        
        # Простая интеграция гироскопа
        if 'time' in df_acc.columns and 'time' in df_gyro.columns:
            # Используем временные метки акселерометра
            time_acc = df_acc['time'].values[:n_points]
            dt = np.diff(time_acc)
            dt = np.insert(dt, 0, dt[0] if len(dt) > 0 else 0.01)
            
            # Интегрируем гироскоп
            pitch_gyro = np.cumsum(gyro_y * dt)
            roll_gyro = np.cumsum(gyro_x * dt)
            
            # Комплементарный фильтр
            alpha = 0.98
            pitch = np.zeros_like(pitch_acc)
            roll = np.zeros_like(roll_acc)
            
            pitch[0] = pitch_acc[0]
            roll[0] = roll_acc[0]
            
            for i in range(1, n_points):
                pitch[i] = alpha * (pitch[i-1] + gyro_y[i] * dt[i]) + (1 - alpha) * pitch_acc[i]
                roll[i] = alpha * (roll[i-1] + gyro_x[i] * dt[i]) + (1 - alpha) * roll_acc[i]
            
            # Визуализация
            fig, axes = plt.subplots(2, 1, figsize=(12, 8))
            
            time_axis = time_acc
            
            # Pitch (тангаж)
            axes[0].plot(time_axis, np.degrees(pitch_acc), 'r--', alpha=0.5, label='Pitch (Acc)')
            axes[0].plot(time_axis, np.degrees(pitch_gyro), 'g--', alpha=0.5, label='Pitch (Gyro)')
            axes[0].plot(time_axis, np.degrees(pitch), 'b-', linewidth=2, label='Pitch (Filtered)')
            axes[0].set_xlabel('Время (с)')
            axes[0].set_ylabel('Угол (градусы)')
            axes[0].set_title('Pitch (Тангаж) - вращение вокруг оси X')
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)
            
            # Roll (крен)
            axes[1].plot(time_axis, np.degrees(roll_acc), 'r--', alpha=0.5, label='Roll (Acc)')
            axes[1].plot(time_axis, np.degrees(roll_gyro), 'g--', alpha=0.5, label='Roll (Gyro)')
            axes[1].plot(time_axis, np.degrees(roll), 'b-', linewidth=2, label='Roll (Filtered)')
            axes[1].set_xlabel('Время (с)')
            axes[1].set_ylabel('Угол (градусы)')
            axes[1].set_title('Roll (Крен) - вращение вокруг оси Y')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig('orientation_from_imu.png', dpi=150, bbox_inches='tight')
            plt.show()
            
            print("\n✅ Расчет ориентации завершен")
            print(f"   Pitch: {np.degrees(pitch[-1]):.1f}°")
            print(f"   Roll: {np.degrees(roll[-1]):.1f}°")
            print("   График сохранен как 'orientation_from_imu.png'")
    
    def export_for_openvins(self, output_file='openvins_input.csv'):
        """
        Экспорт данных в формат, подходящий для OpenVINS
        """
        print(f"\n📤 Подготовка данных для OpenVINS...")
        
        # Нам нужны временные метки, акселерометр и гироскоп
        required_sensors = ['accelerometer', 'gyroscope']
        
        for sensor in required_sensors:
            if sensor not in self.data:
                print(f"⚠️  Отсутствует {sensor}, экспорт невозможен")
                return
        
        df_acc = self.data['accelerometer']
        df_gyro = self.data['gyroscope']
        
        # Находим временную колонку
        time_col = 'time' if 'time' in df_acc.columns else 'seconds_elapsed'
        
        # Находим XYZ колонки
        def find_xyz_columns(df, sensor_name):
            cols = {'x': None, 'y': None, 'z': None}
            
            for col in df.columns:
                col_lower = col.lower()
                if 'x' in col_lower and not any(c in col_lower for c in ['time', 'seconds']):
                    cols['x'] = col
                elif 'y' in col_lower and not any(c in col_lower for c in ['time', 'seconds']):
                    cols['y'] = col
                elif 'z' in col_lower and not any(c in col_lower for c in ['time', 'seconds']):
                    cols['z'] = col
            
            # Если не нашли по имени, берем первые 3 числовые колонки
            if None in cols.values():
                numeric_cols = df.select_dtypes(include=[np.number]).columns.tolist()
                numeric_cols = [c for c in numeric_cols if c != time_col]
                for i, axis in enumerate(['x', 'y', 'z'][:len(numeric_cols)]):
                    cols[axis] = numeric_cols[i]
            
            print(f"{sensor_name} колонки: {cols}")
            return cols
        
        acc_cols = find_xyz_columns(df_acc, 'Акселерометр')
        gyro_cols = find_xyz_columns(df_gyro, 'Гироскоп')
        
        # Создаем объединенный DataFrame
        openvins_data = []
        
        # Определяем общее количество точек (берем минимум из двух датчиков)
        n_points = min(len(df_acc), len(df_gyro))
        
        for i in range(n_points):
            timestamp = df_acc.iloc[i][time_col] if i < len(df_acc) else df_gyro.iloc[i][time_col]
            
            row = {
                'timestamp': timestamp,
                'acc_x': df_acc.iloc[i][acc_cols['x']] if acc_cols['x'] else 0,
                'acc_y': df_acc.iloc[i][acc_cols['y']] if acc_cols['y'] else 0,
                'acc_z': df_acc.iloc[i][acc_cols['z']] if acc_cols['z'] else 0,
                'gyro_x': df_gyro.iloc[i][gyro_cols['x']] if gyro_cols['x'] else 0,
                'gyro_y': df_gyro.iloc[i][gyro_cols['y']] if gyro_cols['y'] else 0,
                'gyro_z': df_gyro.iloc[i][gyro_cols['z']] if gyro_cols['z'] else 0
            }
            openvins_data.append(row)
        
        df_output = pd.DataFrame(openvins_data)
        
        # Сохраняем в CSV
        df_output.to_csv(output_file, index=False)
        
        print(f"✅ Данные экспортированы в '{output_file}'")
        print(f"   Количество записей: {len(df_output)}")
        print(f"   Временной диапазон: {df_output['timestamp'].min():.2f} - {df_output['timestamp'].max():.2f} сек")
        print("\n   Структура файла:")
        print(df_output.head())
        
        return df_output

def main():
    """
    Основная функция для запуска анализа
    """
    print("🎯 АНАЛИЗАТОР ДАННЫХ SENSOR LOGGER")
    print("="*60)
    
    # Запрос пути к папке с данными
    data_folder = input("Введите путь к папке с данными Sensor Logger: ").strip()
    
    if not os.path.exists(data_folder):
        print(f"❌ Папка не найдена: {data_folder}")
        print("Пример: /Users/username/Downloads/sensor_logger_data/")
        return
    
    # Инициализация анализатора
    analyzer = SensorLoggerAnalyzer(data_folder)
    
    # Загрузка данных
    analyzer.load_all_data()
    
    # Вывод сводной информации
    analyzer.print_summary()
    
    # Меню выбора действий
    while True:
        print("\n" + "="*60)
        print("📋 МЕНЮ АНАЛИЗА")
        print("="*60)
        print("1. 📈 Визуализация данных акселерометра")
        print("2. 🌀 Визуализация данных гироскопа")
        print("3. 🧭 Визуализация данных магнитометра")
        print("4. 🔍 Сравнение калиброванных/некалиброванных данных")
        print("5. 🎯 Комплексный анализ IMU (все датчики)")
        print("6. 📐 Расчет ориентации из IMU")
        print("7. 📤 Экспорт данных для OpenVINS")
        print("8. 🚪 Выход")
        
        choice = input("\nВыберите действие (1-8): ").strip()
        
        if choice == '1':
            analyzer.visualize_sensor_data('accelerometer')
        elif choice == '2':
            analyzer.visualize_sensor_data('gyroscope')
        elif choice == '3':
            analyzer.visualize_sensor_data('magnetometer')
        elif choice == '4':
            analyzer.compare_calibrated_vs_uncalibrated()
        elif choice == '5':
            analyzer.analyze_imu_triad()
        elif choice == '6':
            analyzer.calculate_basic_orientation()
        elif choice == '7':
            analyzer.export_for_openvins()
        elif choice == '8':
            print("\n👋 Завершение работы...")
            break
        else:
            print("❌ Неверный выбор, попробуйте снова")

if __name__ == "__main__":
    main()