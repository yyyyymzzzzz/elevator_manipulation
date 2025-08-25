#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import math
import time

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        
        # 创建发布器
        self.pose_publisher = self.create_publisher(PoseStamped, '/arm/target_pose', 10)
        
        # 创建定时器，每2秒发布一个新的目标位姿
        self.timer = self.create_timer(20.0, self.publish_target_pose)
        
        # 预定义的目标位置列表
        self.target_positions = [
            # [x, y, z, roll, pitch, yaw] 相对于base_link
            [0.3, 0.0, 0.8, 0.0, 0.0, 0.0],      # 前方
            [0.2, 0.2, 0.9, 0.0, 0.0, 0.785],    # 右前方
            [0.2, -0.2, 0.9, 0.0, 0.0, -0.785],  # 左前方
            [0.4, 0.0, 0.7, 0.0, 0.785, 0.0],    # 前方向下
            [0.1, 0.0, 1.0, 0.0, -0.785, 0.0],   # 上方向前
        ]
        
        self.current_target_index = 0
        
        self.get_logger().info('目标位姿发布器已启动，将在 /arm/target_pose 上发布目标位姿')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q
    
    def publish_target_pose(self):
        """发布目标位姿"""
        # 获取当前目标位置
        target = self.target_positions[self.current_target_index]
        x, y, z, roll, pitch, yaw = target
        
        # 创建位姿消息
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        
        pose_msg.pose.position = Point(x=x, y=y, z=z)
        pose_msg.pose.orientation = self.euler_to_quaternion(roll, pitch, yaw)
        
        # 发布位姿
        self.pose_publisher.publish(pose_msg)
        
        self.get_logger().info(f'发布目标位姿 #{self.current_target_index + 1}: '
                              f'位置({x:.2f}, {y:.2f}, {z:.2f}), '
                              f'姿态({roll:.2f}, {pitch:.2f}, {yaw:.2f})')
        
        # 切换到下一个目标
        self.current_target_index = (self.current_target_index + 1) % len(self.target_positions)

def main(args=None):
    rclpy.init(args=args)
    
    node = TargetPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
