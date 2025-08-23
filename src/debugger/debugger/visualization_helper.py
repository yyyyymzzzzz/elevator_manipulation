#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import math

class VisualizationHelper(Node):
    def __init__(self):
        super().__init__('visualization_helper')
        
        # 订阅目标位姿
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/arm/target_pose',
            self.target_pose_callback,
            10
        )
        
        # 发布可视化标记
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # 存储历史目标位姿
        self.pose_history = []
        self.max_history = 10
        
        self.get_logger().info('可视化辅助节点已启动，将在RViz中显示目标位姿')
    
    def target_pose_callback(self, msg):
        """接收目标位姿并创建可视化标记"""
        # 添加到历史记录
        self.pose_history.append(msg)
        if len(self.pose_history) > self.max_history:
            self.pose_history.pop(0)
        
        # 创建标记数组
        marker_array = MarkerArray()
        
        # 当前目标位姿标记（红色箭头）
        current_marker = self.create_pose_marker(msg, 0, "current_target", ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        marker_array.markers.append(current_marker)
        
        # 历史位姿标记（逐渐透明的蓝色球体）
        for i, pose in enumerate(self.pose_history[:-1]):  # 除了最后一个（当前）
            alpha = 0.3 + 0.7 * (i / max(1, len(self.pose_history) - 1))
            history_marker = self.create_sphere_marker(pose, i + 1, f"history_{i}", 
                                                     ColorRGBA(r=0.0, g=0.0, b=1.0, a=alpha))
            marker_array.markers.append(history_marker)
        
        # 路径线条
        if len(self.pose_history) > 1:
            path_marker = self.create_path_marker()
            marker_array.markers.append(path_marker)
        
        # 发布标记
        self.marker_publisher.publish(marker_array)
        
        self.get_logger().info(f'发布可视化标记，当前目标: '
                              f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})')
    
    def create_pose_marker(self, pose_msg, marker_id, ns, color):
        """创建位姿标记（箭头）"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = pose_msg.header.frame_id
        
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        marker.pose = pose_msg.pose
        
        # 箭头尺寸
        marker.scale.x = 0.1  # 长度
        marker.scale.y = 0.02  # 宽度
        marker.scale.z = 0.02  # 高度
        
        marker.color = color
        
        return marker
    
    def create_sphere_marker(self, pose_msg, marker_id, ns, color):
        """创建球体标记"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = pose_msg.header.frame_id
        
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position = pose_msg.pose.position
        marker.pose.orientation.w = 1.0
        
        # 球体尺寸
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        
        marker.color = color
        
        return marker
    
    def create_path_marker(self):
        """创建路径线条标记"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        
        marker.ns = "path"
        marker.id = 1000
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 添加路径点
        for pose_msg in self.pose_history:
            marker.points.append(pose_msg.pose.position)
        
        # 线条尺寸和颜色
        marker.scale.x = 0.005  # 线条宽度
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # 绿色
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    
    node = VisualizationHelper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
