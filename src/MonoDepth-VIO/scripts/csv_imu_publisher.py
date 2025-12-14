#!/usr/bin/env python3
import rospy
import csv
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

def publish_csv():
    rospy.init_node('csv_imu_player')
    
    csv_path = rospy.get_param('~csv_path', '/catkin_ws/src/MonoDepth-VIO/data/imu.csv')
    topic_name = rospy.get_param('~topic_name', '/iphone/imu')
    freq = rospy.get_param('~frequency', 500) # Гц
    
    pub = rospy.Publisher(topic_name, Imu, queue_size=100)
    rate = rospy.Rate(freq)
    
    rospy.loginfo(f"Opening CSV: {csv_path}")
    
    try:
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            # Если есть заголовок - пропустите его:
            # next(reader) 
            
            for row in reader:
                if rospy.is_shutdown(): break
                
                # Парсим строку (ЗАВИСИТ ОТ ФОРМАТА CSV!)
                # Пример: timestamp, ax, ay, az, gx, gy, gz
                # Проверьте порядок столбцов в вашем файле!
                
                msg = Imu()
                msg.header.stamp = rospy.Time.now() # Или берем время из CSV, если нужно синхронизировать с записью
                msg.header.frame_id = "imu_link"
                
                # Акселерометр (m/s^2)
                msg.linear_acceleration.x = float(row[1])
                msg.linear_acceleration.y = float(row[2])
                msg.linear_acceleration.z = float(row[3])
                
                # Гироскоп (rad/s)
                msg.angular_velocity.x = float(row[4])
                msg.angular_velocity.y = float(row[5])
                msg.angular_velocity.z = float(row[6])
                
                pub.publish(msg)
                rate.sleep()
                
    except Exception as e:
        rospy.logerr(f"Error reading CSV: {e}")

if __name__ == '__main__':
    publish_csv()
