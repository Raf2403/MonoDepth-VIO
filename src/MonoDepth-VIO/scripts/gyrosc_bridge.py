#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer

IP = "0.0.0.0"
PORT = 8000
TOPIC_NAME = "/iphone/imu"

class GyrOSCBridge:
    def __init__(self):
        rospy.init_node('gyrosc_bridge', anonymous=True)
        self.pub = rospy.Publisher(TOPIC_NAME, Imu, queue_size=10)
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

    def handle_accel(self, address, *args):
        self.accel = [args[0], args[1], args[2]]
        self.publish_imu()

    def handle_gyro(self, address, *args):
        self.gyro = [args[0], args[1], args[2]]
        self.publish_imu()

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"
        
        # Конвертация G -> m/s^2
        msg.linear_acceleration.x = self.accel[0] * 9.81
        msg.linear_acceleration.y = self.accel[1] * 9.81
        msg.linear_acceleration.z = self.accel[2] * 9.81
        
        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]
        
        msg.orientation_covariance[0] = -1
        self.pub.publish(msg)

def start_server(bridge):
    dispatcher = Dispatcher()
    # ВАЖНО: Проверьте в GyrOSC пути! Иногда там /gyrosc/accel, а иногда /accelerometer
    dispatcher.map("/gyrosc/accel", bridge.handle_accel)
    dispatcher.map("/gyrosc/gyro", bridge.handle_gyro)
    
    server = BlockingOSCUDPServer((IP, PORT), dispatcher)
    rospy.loginfo(f"Serving GyrOSC on {IP}:{PORT}")
    server.serve_forever()

if __name__ == '__main__':
    try:
        bridge = GyrOSCBridge()
        start_server(bridge)
    except rospy.ROSInterruptException:
        pass
