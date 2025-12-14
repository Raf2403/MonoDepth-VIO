#!/usr/bin/env python3
import sys
import os
import torch
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Путь к весам 
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent / "src"))
from depth_anything_3.api import DepthAnything3

LOCAL_MODEL_DIR = os.path.join("src", "depth_anything_3", "weights", "DA3METRIC-LARGE")

class DepthNode:
    def __init__(self):
        rospy.init_node('depth_anything_node')
        
        # Настройка модели
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = DepthAnything3.from_pretrained(LOCAL_MODEL_DIR).to(self.device).eval()
        
        self.bridge = CvBridge()
        
        # Подписчик на реальную камеру
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback, queue_size=1)
        
        # Паблишер карты глубины
        self.pub_depth = rospy.Publisher('/depth_anything/depth', Image, queue_size=1)
        
        print("Depth Node Started. Waiting for images...")

    def callback(self, msg):
        try:
            # 1. Конвертация ROS -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # 2. Инференс
            with torch.no_grad():
                prediction = self.model.inference([cv_image])
                depth = prediction.depth[0] # numpy array (H, W) float32
            
            # 3. Публикация в ROS
            # Depth Anything выдает абстрактные единицы. Для теста умножаем на условный скейл или оставляем как есть.
            # OpenVINS будет ждать float32 (метры).
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
            depth_msg.header = msg.header 
            
            self.pub_depth.publish(depth_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing frame: {e}")

if __name__ == '__main__':
    node = DepthNode()
    rospy.spin()
