#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import String, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import re
import struct
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import rclpy.duration

class Button3DVisualizer(Node):
    def __init__(self):
        super().__init__('button_3d_visualizer')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_depth', 5.0),
                ('min_depth', 0.2),
                ('marker_scale', 0.03),
                ('marker_lifetime', 1.0),
                ('target_frame', 'world'),
                ('camera_frame', 'camera_depth_optical_frame')
            ]
        )
        
        self.max_depth = self.get_parameter('max_depth').get_parameter_value().double_value
        self.min_depth = self.get_parameter('min_depth').get_parameter_value().double_value
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.bridge = CvBridge()
        
        # 初始化TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅检测结果、深度图像和相机信息
        self.detection_subscriber = self.create_subscription(
            String,
            '/detector/result',
            self.detection_callback,
            10
        )
        
        self.depth_image_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 发布3D可视化标记
        self.marker_publisher = self.create_publisher(
            MarkerArray, 
            '/button_3d_markers', 
            10
        )
        
        # 存储最新的深度图像和相机信息
        self.latest_depth_image = None
        self.camera_info = None
        
        # 存储当前的检测结果和标记，用于持续发布
        self.current_detections = []
        self.last_marker_array = None
        
        # 创建定时器，定期重新发布标记以防止消失
        self.marker_republish_timer = self.create_timer(
            1.0,  # 每1秒重新发布一次
            self.republish_markers_callback
        )
        
        self.get_logger().info('按钮3D可视化节点已启动')
        self.get_logger().info(f'目标坐标系: {self.target_frame}')

    def republish_markers_callback(self):
        """定期重新发布标记以防止消失"""
        if self.last_marker_array is not None and self.current_detections:
            # 更新时间戳
            current_time = self.get_clock().now().to_msg()
            for marker in self.last_marker_array.markers:
                marker.header.stamp = current_time
            
            # 重新发布
            self.marker_publisher.publish(self.last_marker_array)
            self.get_logger().debug(f'重新发布了 {len(self.last_marker_array.markers)} 个标记')

    def camera_info_callback(self, msg):
        """接收相机内参信息"""
        self.camera_info = msg
        # self.get_logger().info(f'收到相机内参信息: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}')

    def depth_callback(self, msg):
        """接收深度图像"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            # self.get_logger().info(f'收到深度图像: {self.latest_depth_image.shape}, dtype: {self.latest_depth_image.dtype}')
        except Exception as e:
            self.get_logger().error(f'深度图像转换失败: {e}')

    def detection_callback(self, msg):
        """接收检测结果并生成3D可视化"""
        if self.latest_depth_image is None:
            self.get_logger().warn('没有可用的深度图像')
            return
            
        if self.camera_info is None:
            self.get_logger().warn('没有可用的相机内参信息')
            return
            
        detection_text = msg.data
        if detection_text == 'No detections':
            # 清空检测结果和标记
            self.current_detections = []
            self.last_marker_array = None
            self.publish_empty_markers()
            return
            
        # 解析检测结果
        detections = self.parse_detection_result(detection_text)
        self.current_detections = detections  # 保存当前检测结果
        
        self.get_logger().info(f'检测到 {len(detections)} 个按钮')

        # 生成3D标记
        markers = self.create_3d_markers(detections)
        
        # 发布标记
        marker_array = MarkerArray()
        marker_array.markers = markers
        self.last_marker_array = marker_array  # 保存标记数组用于重新发布
        self.marker_publisher.publish(marker_array)
        
        self.get_logger().info(f'发布了 {len(markers)} 个3D按钮标记')

    def parse_detection_result(self, detection_text):
        """解析检测结果文本"""
        detections = []
        lines = detection_text.strip().split('\n')
        
        for line in lines:
            if not line.strip():
                continue
                
            # 解析格式: "class: confidence, bbox:[x1,y1,x2,y2], center:[cx,cy], size:[w×h]"
            try:
                # 提取类别和置信度
                class_conf_match = re.search(r'([^:]+):\s*([\d.]+)', line)
                if not class_conf_match:
                    continue
                    
                class_name = class_conf_match.group(1).strip()
                confidence = float(class_conf_match.group(2))
                
                # 提取中心点坐标
                center_match = re.search(r'center:\[([\d.]+),([\d.]+)\]', line)
                if not center_match:
                    continue
                    
                center_x = float(center_match.group(1))
                center_y = float(center_match.group(2))
                
                # 提取边界框
                bbox_match = re.search(r'bbox:\[([\d.]+),([\d.]+),([\d.]+),([\d.]+)\]', line)
                if bbox_match:
                    x1, y1, x2, y2 = map(float, bbox_match.groups())
                    width = x2 - x1
                    height = y2 - y1
                else:
                    width, height = 50, 50  # 默认大小
                
                detections.append({
                    'class': class_name,
                    'confidence': confidence,
                    'center_x': center_x,
                    'center_y': center_y,
                    'width': width,
                    'height': height
                })
                
            except Exception as e:
                self.get_logger().warn(f'解析检测行失败: {line}, 错误: {e}')
                continue
                
        return detections

    def get_3d_point_from_depth(self, pixel_x, pixel_y, search_radius=3):
        """从深度图像中获取指定像素坐标对应的3D点"""
        if self.latest_depth_image is None:
            return None
            
        if self.camera_info is None:
            self.get_logger().warn('没有可用的相机内参信息')
            return None
            
        h, w = self.latest_depth_image.shape
        
        # 确保像素坐标在有效范围内
        pixel_x = max(0, min(int(pixel_x), w - 1))
        pixel_y = max(0, min(int(pixel_y), h - 1))
        
        # 在像素周围搜索有效深度值
        depth_values = []
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                x = pixel_x + dx
                y = pixel_y + dy
                
                if 0 <= x < w and 0 <= y < h:
                    depth_raw = self.latest_depth_image[y, x]
                    
                    # 检查深度值是否有效
                    if depth_raw > 0:
                        # 将深度值转换为米 (假设深度图单位是毫米)
                        depth_m = depth_raw / 1000.0
                        
                        if self.min_depth <= depth_m <= self.max_depth:
                            depth_values.append(depth_m)
        
        if not depth_values:
            return None
        
        # 使用中位数深度值
        depth_m = np.median(depth_values)
        
        # 使用实际的相机内参将像素坐标转换为3D坐标
        fx = self.camera_info.k[0]  # 焦距 x
        fy = self.camera_info.k[4]  # 焦距 y  
        cx = self.camera_info.k[2]  # 主点 x
        cy = self.camera_info.k[5]  # 主点 y
        
        # 计算相机光学坐标系下的3D点
        x_3d = (pixel_x - cx) * depth_m / fx
        y_3d = (pixel_y - cy) * depth_m / fy
        z_3d = depth_m
        
        return np.array([x_3d, y_3d, z_3d])

    def pixel_to_3d_point(self, u, v, depth_value=None):
        """将像素坐标转换为3D点 - 使用深度图像"""
        return self.get_3d_point_from_depth(u, v)

    def transform_point_to_target_frame(self, point_3d):
        """将相机坐标系下的点转换到目标坐标系"""
        # 如果目标坐标系和相机坐标系相同，直接返回原始点
        if self.target_frame == self.camera_frame:
            self.get_logger().info('目标坐标系与相机坐标系相同，跳过变换')
            return point_3d
            
        try:
            # 创建PointStamped消息
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = float(point_3d[0])
            point_stamped.point.y = float(point_3d[1])
            point_stamped.point.z = float(point_3d[2])
            
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 应用变换
            transformed_point = do_transform_point(point_stamped, transform)
            
            self.get_logger().debug(f'成功变换点从 {self.camera_frame} 到 {self.target_frame}')
            
            return np.array([
                transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z
            ])
            
        except Exception as e:
            self.get_logger().warn(f'坐标变换从 {self.camera_frame} 到 {self.target_frame} 失败: {e}')
            return None  # 变换失败时返回None而不是原始点

    def create_3d_markers(self, detections):
        """根据检测结果创建3D标记"""
        markers = []
        
        for i, detection in enumerate(detections):
            # 直接从深度图获取3D点（已经在camera_depth_optical_frame坐标系下）
            point_3d = self.get_3d_point_from_depth(
                detection['center_x'], 
                detection['center_y']
            )
            
            if point_3d is None:
                self.get_logger().warn(f'检测 {detection["class"]} 在位置({detection["center_x"]:.1f}, {detection["center_y"]:.1f})无法获取3D坐标')
                continue
            
            # 将点从camera_depth_optical_frame变换到目标坐标系（world）
            if self.target_frame != self.camera_frame:
                final_point = self.transform_point_to_target_frame(point_3d)
                if final_point is None:
                    self.get_logger().warn(f'检测 {detection["class"]} 坐标变换失败，跳过')
                    continue
                used_frame = self.target_frame
            else:
                final_point = point_3d
                used_frame = self.camera_frame
            
            self.get_logger().debug(f'使用坐标系: {used_frame}')
                
            # 创建标记
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = used_frame
            
            marker.ns = "button_detections"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 设置位置
            marker.pose.position.x = float(final_point[0])
            marker.pose.position.y = float(final_point[1]) 
            marker.pose.position.z = float(final_point[2])
            marker.pose.orientation.w = 1.0
            
            # 设置大小
            marker.scale.x = self.marker_scale
            marker.scale.y = self.marker_scale
            marker.scale.z = self.marker_scale
            
            # 根据类别设置颜色
            if 'up' in detection['class'].lower():
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # 绿色
            elif 'down' in detection['class'].lower():
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # 红色
            else:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # 蓝色
                
            # 设置生存时间 
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            
            markers.append(marker)
            
            # 添加文字标记
            text_marker = self.create_text_marker(detection, final_point, i + 100, used_frame)
            markers.append(text_marker)
            
            self.get_logger().debug(
                f'按钮 {detection["class"]} 在像素({detection["center_x"]:.1f}, {detection["center_y"]:.1f}) '
                f'{used_frame}坐标: ({final_point[0]:.3f}, {final_point[1]:.3f}, {final_point[2]:.3f})'
            )
        
        return markers

    def create_text_marker(self, detection, point_3d, marker_id, frame_id=None):
        """创建文字标记"""
        if frame_id is None:
            frame_id = self.target_frame
            
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame_id
        
        marker.ns = "button_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 文字位置（稍微偏移）
        marker.pose.position.x = float(point_3d[0])
        marker.pose.position.y = float(point_3d[1])
        marker.pose.position.z = float(point_3d[2]) + 0.05  
        marker.pose.orientation.w = 1.0
        marker.text = f"{detection['class']}\n{detection['confidence']:.2f}"
        marker.scale.z = 0.03
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # 红色

        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        return marker

    def publish_empty_markers(self):
        """发布空标记数组以清空显示"""
        marker_array = MarkerArray()
        
        # 创建删除标记 
        for frame_id in [self.target_frame, self.camera_frame]:
            for i in range(20):  # 假设最多20个标记
                for ns in ["button_detections", "button_labels"]:
                    marker = Marker()
                    marker.header = Header()
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.header.frame_id = frame_id
                    marker.ns = ns
                    marker.id = i
                    marker.action = Marker.DELETE
                    marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Button3DVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
