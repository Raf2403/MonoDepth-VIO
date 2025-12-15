#!/usr/bin/env python3
import rospy
import csv
import os
from sensor_msgs.msg import Imu

def talker():
    # Инициализация ROS узла
    rospy.init_node('imu_csv_publisher', anonymous=True)
    
    # Создаем Publisher: топик /imu, тип сообщения Imu
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    
    # Частота публикации. Можно менять (сейчас 200 Гц)
    rate = rospy.Rate(200) 

    # Путь к CSV: берем из параметра или ищем в текущей папке
    csv_path = rospy.get_param('~csv_path', 'ros_imu_data.csv')
    
    # Если путь не абсолютный, пробуем найти его относительно пакета (опционально)
    # Но проще положить CSV рядом, где запускаете roslaunch, или указать полный путь.

    if not os.path.exists(csv_path):
        rospy.logwarn(f"CSV file not found at: {csv_path}. Trying default package path...")
        # Попытка найти внутри пакета (если запускаем через roslaunch)
        import rospkg
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('imu_csv_publisher')
            csv_path = os.path.join(pkg_path, 'ros_imu_data.csv')
        except:
            pass

    rospy.loginfo(f"Reading IMU data from: {csv_path}")

    try:
        with open(csv_path, 'r') as csvfile:
            reader = csv.reader(csvfile)
            
            # Пропускаем заголовок (первая строка: timestamp,accel_x...)
            next(reader)
            
            for row in reader:
                if rospy.is_shutdown():
                    break
                
                try:
                    # Создаем сообщение
                    msg = Imu()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "imu_link"
                    
                    # Заполняем ориентацию (пока нули или единичная, так как у нас только Raw данные)
                    # Если нужно, можно брать кватернионы из колонок 10-13
                    msg.orientation.x = 0.0
                    msg.orientation.y = 0.0
                    msg.orientation.z = 0.0
                    msg.orientation.w = 1.0 # Единичный кватернион
                    
                    # --- Читаем данные из CSV ---
                    
                    # Акселерометр (колонки 1, 2, 3) -> м/с^2
                    msg.linear_acceleration.x = float(row[1])
                    msg.linear_acceleration.y = float(row[2])
                    msg.linear_acceleration.z = float(row[3])
                    
                    # Гироскоп (колонки 4, 5, 6) -> рад/с
                    msg.angular_velocity.x = float(row[4])
                    msg.angular_velocity.y = float(row[5])
                    msg.angular_velocity.z = float(row[6])
                    
                    # Публикуем
                    pub.publish(msg)
                    
                except (ValueError, IndexError) as e:
                    rospy.logwarn(f"Error parsing row: {e}")
                    continue
                
                # Ждем до следующего цикла
                rate.sleep()
                
    except FileNotFoundError:
        rospy.logerr(f"CRITICAL: File not found: {csv_path}")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

