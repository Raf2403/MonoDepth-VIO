import rosbag
import pandas as pd
import rospy
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Vector3

# 1. Пути к файлам
CAMERA_BAG_PATH = 'realsense_data.bag'  # Записанный bag с камеры
IMU_CSV_PATH = 'Combined.csv'           # Файл из Sensor Logger
OUTPUT_BAG_PATH = 'final_dataset.bag'   # Итоговый файл

# 2. Топики
CAMERA_TOPIC = '/camera/color/image_raw' # Название топика камеры в исходном bag
NEW_IMU_TOPIC = '/imu/data'              # Пишем IMU

# 3. Синхронизация (САМОЕ ВАЖНОЕ)
# TIME_OFFSET = (Время события в ROS) - (Время события в CSV)
# Пример: Если удар в ROS был в 1700000010.0, а в CSV в 50.0, то offset = 1700000010.0 - 50.0
TIME_OFFSET = 0.0 

def create_imu_msg(row, offset):
    """Создает ROS сообщение Imu из строки pandas"""
    msg = Imu()
    
    # Расчет времени: Timestamp телефона + сдвиг
    # Sensor Logger обычно пишет seconds (float) или nanoseconds (int). Проверьте свой CSV!
    # Если в CSV время в секундах (например, 123.456):
    timestamp = row['seconds_elapsed'] + offset 
    
    # Если в CSV 'time' это epoch time (настоящее время), то offset может быть маленьким или 0
    # Но для Sensor Logger обычно используют 'seconds_elapsed' + смещение до ROS time.
    
    t = rospy.Time.from_sec(timestamp)
    msg.header.stamp = t
    msg.header.frame_id = "imu_link"
    
    # Гироскоп (rad/s)
    msg.angular_velocity.x = row['gyro_x']
    msg.angular_velocity.y = row['gyro_y']
    msg.angular_velocity.z = row['gyro_z']
    
    # Акселерометр (m/s^2) 
    # Sensor Logger может писать в g. Если значения около 1.0 (когда лежит), умножаем на 9.81
    # Если значения около 9.8, оставляем так.
    # Предположим, что данные в m/s^2 (проверьте заголовки CSV!)
    msg.linear_acceleration.x = row['accel_x']
    msg.linear_acceleration.y = row['accel_y']
    msg.linear_acceleration.z = row['accel_z']
    
    return msg, t

def main():
    print(f"Reading IMU data from {IMU_CSV_PATH}...")
    # Читаем CSV. Имена колонок зависят от Sensor Logger! 
    # Обычно это: time, seconds_elapsed, z, y, x (для accel и gyro)
    # Проверьте заголовки вашего файла и поправьте их тут при необходимости
    try:
        df = pd.read_csv(IMU_CSV_PATH)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    # Переименуем колонки для удобства (адаптируйте под свой CSV)
    # Sensor Logger 'Combined' обычно имеет такие колонки (пример):
    # 'seconds_elapsed', 'rotationRateX', 'rotationRateY', 'rotationRateZ', 'userAccelerationX', 'userAccelerationY', 'userAccelerationZ'
    # ВНИМАНИЕ: userAcceleration - это БЕЗ гравитации. Для VIO нужен RAW (Total) acceleration (с гравитацией)!
    # Если у вас 'Accelerometer.csv', там обычно 'x', 'y', 'z'.
    
    # Пример маппинга (поправьте под ваш файл):
    # df.rename(columns={
    #     'rotationRateX': 'gyro_x', 'rotationRateY': 'gyro_y', 'rotationRateZ': 'gyro_z',
    #     'accelerometerAccelerationX': 'accel_x', 'accelerometerAccelerationY': 'accel_y', 'accelerometerAccelerationZ': 'accel_z'
    # }, inplace=True)
    
    # Если у вас просто 'x', 'y', 'z' и два разных файла, придется сначала их объединить (merge) по времени в pandas.
    # Допустим, вы уже подготовили CSV с колонками accel_x... и gyro_x...
    
    print("Opening ROS bags...")
    with rosbag.Bag(OUTPUT_BAG_PATH, 'w') as out_bag:
        
        # 1. Записываем данные камеры из старого bag
        print("Writing camera data...")
        try:
            with rosbag.Bag(CAMERA_BAG_PATH, 'r') as in_bag:
                for topic, msg, t in in_bag.read_messages(topics=[CAMERA_TOPIC]):
                    # Пишем как есть
                    out_bag.write(topic, msg, t)
        except Exception as e:
            print(f"Error reading camera bag: {e}. Make sure path is correct.")

        # 2. Записываем данные IMU из CSV
        print("Writing IMU data...")
        imu_count = 0
        for index, row in df.iterrows():
            # Важно: В VIO критично, чтобы timestamp сообщения совпадал с timestamp записи
            msg, t = create_imu_msg(row, TIME_OFFSET)
            
            # Пишем в bag
            out_bag.write(NEW_IMU_TOPIC, msg, t)
            imu_count += 1
            
    print(f"Done! Created {OUTPUT_BAG_PATH}")
    print(f"Total IMU messages: {imu_count}")

if __name__ == "__main__":
    main()
