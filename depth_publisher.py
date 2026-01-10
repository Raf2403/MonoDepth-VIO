import sys
from pathlib import Path

# добавляем папку src в sys.path
sys.path.append(str(Path(__file__).resolve().parent / "src"))

import os
import cv2
import torch
import numpy as np
from depth_anything_3.api import DepthAnything3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

LOCAL_MODEL_DIR = os.path.join("src", "depth_anything_3", "weights", "DA3METRIC-LARGE")

class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__('depth_anything_node')

        # --- Параметры ---
        # Топик, на который подписываемся (картинка с OpenVINS или bag-файла)
        self.declare_parameter('input_topic', '/cam0/image_raw')
        # Топик, куда выдаем глубину
        self.declare_parameter('output_topic', '/depth/image_raw')
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Loading model from: {LOCAL_MODEL_DIR}')

        # --- Инициализация модели ---
        try:
            self.model = DepthAnything3.from_pretrained(LOCAL_MODEL_DIR)
            self.model.to(self.device)
            self.model.eval()
            self.get_logger().info('Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise e

        # --- ROS Setup ---
        self.bridge = CvBridge()
        
        # Подписка
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # Публикация
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.get_logger().info(f'Subscribed to {input_topic}, publishing to {output_topic}')

    def image_callback(self, msg):
        try:
            # 1. Конвертация ROS Image -> OpenCV
            if msg.encoding == 'mono8':
                # Если приходит черно-белая (инфракрасная), делаем RGB, так как модель хочет 3 канала
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            elif msg.encoding == 'bgr8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            else:
                self.get_logger().warn(f'Unknown encoding: {msg.encoding}, trying bgr8 conversion')
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # 2. Инференс
            # model.inference ожидает список картинок
            with torch.no_grad():
                prediction = self.model.inference([cv_image])
                depth = prediction.depth[0] # Это сырые метры (numpy array float)

            # 3. Публикация
            # Конвертируем numpy float32 в ROS Image (32FC1)
            depth_msg = self.bridge.cv2_to_imgmsg(depth.astype(np.float32), encoding="32FC1")
            
            # ВАЖНО: Сохраняем таймстемп!
            depth_msg.header = msg.header
            
            self.publisher.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
