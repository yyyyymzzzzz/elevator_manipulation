#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import random
import glob
from cv_bridge import CvBridge

class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        
        # 初始化 CvBridge
        self.bridge = CvBridge()
        
        # 设置数据路径
        self.data_path = '/home/ymz/Workspace/elevator_manipulation/data'
        self.train_images = glob.glob(os.path.join(self.data_path, 'train/images/*.jpg'))
        self.test_images = glob.glob(os.path.join(self.data_path, 'test/images/*.jpg'))
        
        # 合并所有图像
        self.all_images = self.train_images + self.test_images
        
        if not self.all_images:
            self.get_logger().error('没有找到图像文件!')
            return
            
        self.get_logger().info(f'找到 {len(self.all_images)} 张图像')
        
        # 创建发布器
        self.image_publisher = self.create_publisher(Image, '/raw/image', 10)
        
        # 当前图像和发布计数器
        self.current_image = None
        self.current_image_path = None
        self.publish_count = 0
        self.max_publish_count = 5  # 1秒 / 0.2秒 = 5次
        
        # 创建定时器，每0.2秒发布一次图像（5Hz）
        self.publish_timer = self.create_timer(0.2, self.publish_current_image)
        
        # 创建定时器，每1秒切换图像
        self.switch_timer = self.create_timer(1.0, self.switch_to_next_image)
        
        # 初始化第一张图像
        self.switch_to_next_image()
        
        self.get_logger().info('相机模拟器已启动，以5Hz频率发布图像，每1秒切换一张图像到 /raw/image')
    
    def switch_to_next_image(self):
        """切换到下一张随机图像"""
        # 随机选择一张图像
        image_path = random.choice(self.all_images)
        
        # 读取图像
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            self.get_logger().warn(f'无法读取图像: {image_path}')
            return
        
        # 保存当前图像
        self.current_image = cv_image
        self.current_image_path = image_path
        self.publish_count = 0
        
        filename = os.path.basename(image_path)
        self.get_logger().info(f'切换到新图像: {filename}')
    
    def publish_current_image(self):
        """发布当前图像"""
        if self.current_image is None:
            return
        
        # 转换为 ROS Image 消息
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.current_image, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera_frame'
            
            # 发布图像
            self.image_publisher.publish(ros_image)
            
            self.publish_count += 1
            
            # 每秒记录一次日志（每5次发布）
            if self.publish_count % 5 == 1:
                filename = os.path.basename(self.current_image_path)
                self.get_logger().info(f'正在发布图像: {filename} (第{self.publish_count}次)')
            
        except Exception as e:
            self.get_logger().error(f'转换图像失败: {e}')
    
    def publish_random_image(self):
        # 这个方法已经不再使用，可以删除
        pass

def main(args=None):
    rclpy.init(args=args)
    
    node = CameraSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
