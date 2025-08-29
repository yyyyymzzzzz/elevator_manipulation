#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, CameraInfo
from std_msgs.msg import String, Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import re
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import rclpy.duration

class ButtonTargetPlanner(Node):
    def __init__(self):
        super().__init__('button_target_planner')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_distance', 0.10),  # 目标位置距离按钮平面的距离 (m)
                ('plane_radius', 0.05),     # 用于拟合平面的点云半径 (m)
                ('min_points_for_plane', 30), # 拟合平面所需的最少点数
                ('target_frame', 'world'),
                ('camera_frame', 'camera_color_optical_frame'),
                ('marker_scale', 0.02),
                ('marker_lifetime', 3.0),
                ('button_selection_method', 'first'),  # 'first', 'closest', 'manual'
                ('manual_button_index', 0)
            ]
        )
        
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.plane_radius = self.get_parameter('plane_radius').get_parameter_value().double_value
        self.min_points_for_plane = self.get_parameter('min_points_for_plane').get_parameter_value().integer_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        self.button_selection_method = self.get_parameter('button_selection_method').get_parameter_value().string_value
        self.manual_button_index = self.get_parameter('manual_button_index').get_parameter_value().integer_value
        
        self.bridge = CvBridge()
        
        # 初始化TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅按钮3D位置信息（从button_3d_visualizer获取）
        self.button_markers_subscriber = self.create_subscription(
            MarkerArray,
            '/button_3d_markers',
            self.button_markers_callback,
            10
        )
        
        # 订阅点云数据
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )
        
        # 订阅相机信息
        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 发布目标位置可视化标记
        self.target_marker_publisher = self.create_publisher(
            MarkerArray,
            '/button_target_markers',
            10
        )
        
        # 发布目标位置姿态（后续用于机械臂控制）
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/button_target_pose',
            10
        )
        
        # 存储数据
        self.latest_pointcloud = None
        self.camera_info = None
        self.current_button_positions = []
        self.selected_button_index = 0
        self.current_target_pose = None
        
        # 创建定时器，定期重新发布标记
        self.marker_republish_timer = self.create_timer(
            1.0,
            self.republish_markers_callback
        )
        
        self.get_logger().info('按钮目标规划节点已启动')
        self.get_logger().info(f'目标距离: {self.target_distance}m, 按钮选择方法: {self.button_selection_method}')

    def republish_markers_callback(self):
        """定期重新发布标记以防止消失"""
        if self.current_target_pose is not None:
            self.publish_target_visualization()

    def camera_info_callback(self, msg):
        """接收相机内参信息"""
        self.camera_info = msg

    def pointcloud_callback(self, msg):
        """接收点云数据"""
        self.latest_pointcloud = msg

    def button_markers_callback(self, msg):
        """接收按钮3D位置标记"""
        # 提取按钮位置信息
        button_positions = []
        
        for marker in msg.markers:
            if marker.ns == "button_detections" and marker.action == Marker.ADD:
                position = np.array([
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ])
                button_positions.append({
                    'id': marker.id,
                    'position': position,
                    'frame_id': marker.header.frame_id
                })
        
        if not button_positions:
            self.get_logger().debug('没有检测到按钮位置')
            self.current_button_positions = []
            self.current_target_pose = None
            return
        
        self.current_button_positions = button_positions
        self.get_logger().info(f'收到 {len(button_positions)} 个按钮位置')
        
        # 选择目标按钮
        selected_button = self.select_target_button(button_positions)
        if selected_button is None:
            self.get_logger().warn('未能选择目标按钮')
            return
            
        self.get_logger().info(f'选择按钮 ID: {selected_button["id"]}, '
                              f'位置: ({selected_button["position"][0]:.3f}, '
                              f'{selected_button["position"][1]:.3f}, '
                              f'{selected_button["position"][2]:.3f})')
        
        # 计算目标位置和姿态
        target_pose = self.calculate_target_pose(selected_button)
        if target_pose is not None:
            self.current_target_pose = target_pose
            self.publish_target_visualization()
            self.publish_target_pose()

    def select_target_button(self, button_positions):
        """选择目标按钮"""
        if not button_positions:
            return None
            
        if self.button_selection_method == 'first':
            return button_positions[0]
        elif self.button_selection_method == 'closest':
            # 选择最近的按钮（z值最小）
            closest_button = min(button_positions, key=lambda b: b['position'][2])
            return closest_button
        elif self.button_selection_method == 'manual':
            # 手动选择指定索引的按钮
            if 0 <= self.manual_button_index < len(button_positions):
                return button_positions[self.manual_button_index]
            else:
                self.get_logger().warn(f'手动索引 {self.manual_button_index} 超出范围，使用第一个按钮')
                return button_positions[0]
        else:
            return button_positions[0]

    def calculate_target_pose(self, selected_button):
        """计算目标位置和姿态"""
        if self.latest_pointcloud is None:
            self.get_logger().warn('没有可用的点云数据')
            return None
        
        button_position = selected_button['position']
        button_frame = selected_button['frame_id']
        
        self.get_logger().info(f'按钮位置坐标系: {button_frame}')
        self.get_logger().info(f'点云坐标系: {self.latest_pointcloud.header.frame_id}')
        
        # 如果按钮位置和点云不在同一坐标系，需要进行转换
        if button_frame != self.latest_pointcloud.header.frame_id:
            # 将按钮位置转换到点云坐标系
            try:
                # 创建PointStamped消息
                point_stamped = PointStamped()
                point_stamped.header.frame_id = button_frame
                point_stamped.header.stamp = self.get_clock().now().to_msg()
                point_stamped.point.x = float(button_position[0])
                point_stamped.point.y = float(button_position[1])
                point_stamped.point.z = float(button_position[2])
                
                # 获取变换
                transform = self.tf_buffer.lookup_transform(
                    self.latest_pointcloud.header.frame_id,
                    button_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # 应用变换
                transformed_point = do_transform_point(point_stamped, transform)
                
                button_position_in_cloud_frame = np.array([
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z
                ])
                
                self.get_logger().info(f'按钮位置转换到点云坐标系: ({button_position_in_cloud_frame[0]:.3f}, {button_position_in_cloud_frame[1]:.3f}, {button_position_in_cloud_frame[2]:.3f})')
                
            except Exception as e:
                self.get_logger().warn(f'坐标转换失败: {e}，使用原始按钮位置')
                button_position_in_cloud_frame = button_position
        else:
            button_position_in_cloud_frame = button_position
        
        # 从点云中提取按钮周围的点
        local_points = self.extract_local_points(button_position_in_cloud_frame)
        if len(local_points) < self.min_points_for_plane:
            self.get_logger().warn(f'按钮周围点数不足: {len(local_points)} < {self.min_points_for_plane}')
            # 尝试增大搜索半径
            original_radius = self.plane_radius
            self.plane_radius *= 3  # 临时增大到3倍
            self.get_logger().info(f'尝试增大搜索半径到 {self.plane_radius:.3f}m')
            local_points = self.extract_local_points(button_position_in_cloud_frame)
            self.plane_radius = original_radius  # 恢复原始半径
            
            if len(local_points) < self.min_points_for_plane:
                # 如果还是找不到足够的点，使用默认法向量进行测试
                self.get_logger().warn('使用默认法向量进行测试 (假设电梯面板垂直)')
                # 假设电梯面板是垂直的，法向量大约是 (0, 0, -1) 在相机坐标系中
                plane_normal = np.array([0.0, 0.0, -1.0])
                use_default_normal = True
            else:
                use_default_normal = False
        else:
            use_default_normal = False
        
        if not use_default_normal:
            # 拟合平面
            plane_normal = self.fit_plane(local_points)
            if plane_normal is None:
                self.get_logger().warn('平面拟合失败，使用默认法向量')
                plane_normal = np.array([0.0, 0.0, -1.0])
                use_default_normal = True
        
        # 确保法向量指向相机方向（向我们）
        # 在相机坐标系中，相机位置是原点，z轴指向前方
        # 如果法向量指向相机，那么从按钮位置到相机的向量与法向量的点积应该为正
        
        if not use_default_normal:
            # 计算从按钮位置到相机原点的向量（在点云坐标系中）
            camera_direction = -button_position_in_cloud_frame  # 指向相机原点
            camera_direction = camera_direction / np.linalg.norm(camera_direction)
            
            # 检查法向量方向，确保指向相机
            dot_product = np.dot(plane_normal, camera_direction)
            self.get_logger().info(f'法向量与相机方向点积: {dot_product:.3f}')
            
            if dot_product < 0:
                # 如果点积为负，说明法向量指向远离相机的方向，需要翻转
                plane_normal = -plane_normal
                self.get_logger().info('翻转法向量方向以指向相机')
        else:
            self.get_logger().info('使用默认法向量，假设垂直向外')
        
        self.get_logger().info(f'最终法向量: ({plane_normal[0]:.3f}, {plane_normal[1]:.3f}, {plane_normal[2]:.3f})')
        
        # 计算目标位置（在按钮位置基础上，沿法向量向外移动目标距离）
        # 注意：这里我们要沿着法向量的反方向移动，因为我们希望位置在按钮外侧
        target_position_in_cloud_frame = button_position_in_cloud_frame + plane_normal * self.target_distance
        
        # 如果需要，将目标位置转换回原始坐标系
        if button_frame != self.latest_pointcloud.header.frame_id:
            try:
                # 创建PointStamped消息
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.latest_pointcloud.header.frame_id
                point_stamped.header.stamp = self.get_clock().now().to_msg()
                point_stamped.point.x = float(target_position_in_cloud_frame[0])
                point_stamped.point.y = float(target_position_in_cloud_frame[1])
                point_stamped.point.z = float(target_position_in_cloud_frame[2])
                
                # 获取变换
                transform = self.tf_buffer.lookup_transform(
                    button_frame,
                    self.latest_pointcloud.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                # 应用变换
                transformed_point = do_transform_point(point_stamped, transform)
                
                target_position = np.array([
                    transformed_point.point.x,
                    transformed_point.point.y,
                    transformed_point.point.z
                ])
                
                self.get_logger().info(f'目标位置转换回原坐标系: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})')
                
            except Exception as e:
                self.get_logger().warn(f'目标位置坐标转换失败: {e}')
                target_position = target_position_in_cloud_frame
        else:
            target_position = target_position_in_cloud_frame
        
        # 计算目标姿态（使z轴与法向量对齐）
        target_orientation = self.calculate_orientation_from_normal(plane_normal)
        
        self.get_logger().info(f'计算目标位置: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})')
        self.get_logger().info(f'平面法向量: ({plane_normal[0]:.3f}, {plane_normal[1]:.3f}, {plane_normal[2]:.3f})')
        
        return {
            'position': target_position,
            'orientation': target_orientation,
            'frame_id': selected_button['frame_id'],
            'normal': plane_normal,
            'button_position': button_position
        }

    def extract_local_points(self, center_position):
        """提取按钮周围的点云点"""
        if self.latest_pointcloud is None:
            self.get_logger().warn('没有可用的点云数据')
            return []
        
        self.get_logger().info(f'点云坐标系: {self.latest_pointcloud.header.frame_id}')
        self.get_logger().info(f'按钮中心位置: ({center_position[0]:.3f}, {center_position[1]:.3f}, {center_position[2]:.3f})')
        self.get_logger().info(f'搜索半径: {self.plane_radius}m')
        
        points = []
        total_points = 0
        valid_points = 0
        
        try:
            # 读取点云数据
            for point in pc2.read_points(self.latest_pointcloud, skip_nans=True):
                total_points += 1
                point_pos = np.array([point[0], point[1], point[2]])
                
                # 检查点是否有效（不是nan或inf）
                if not (np.isfinite(point_pos).all()):
                    continue
                    
                valid_points += 1
                
                # 计算与按钮中心的距离
                distance = np.linalg.norm(point_pos - center_position)
                
                if distance <= self.plane_radius:
                    points.append(point_pos)
                    
                # 记录前几个点的信息用于调试
                if total_points <= 5:
                    self.get_logger().info(f'点 {total_points}: ({point_pos[0]:.3f}, {point_pos[1]:.3f}, {point_pos[2]:.3f}), 距离: {distance:.3f}')
                    
        except Exception as e:
            self.get_logger().error(f'读取点云失败: {e}')
            return []
        
        self.get_logger().info(f'总点数: {total_points}, 有效点数: {valid_points}, 在半径 {self.plane_radius}m 内找到 {len(points)} 个点')
        
        if len(points) == 0:
            self.get_logger().warn('未找到任何近邻点，可能的原因：')
            self.get_logger().warn('1. 坐标系不匹配')
            self.get_logger().warn('2. 按钮位置与点云不在同一尺度')
            self.get_logger().warn('3. 搜索半径太小')
        
        return np.array(points)

    def fit_plane(self, points):
        """使用简单的最小二乘法拟合平面"""
        if len(points) < 3:
            return None
            
        try:
            # 使用PCA进行平面拟合
            # 计算点云的质心
            centroid = np.mean(points, axis=0)
            
            # 将点云移到原点
            centered_points = points - centroid
            
            # 计算协方差矩阵
            cov_matrix = np.cov(centered_points.T)
            
            # 计算特征值和特征向量
            eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
            
            # 最小特征值对应的特征向量就是平面的法向量
            normal = eigenvectors[:, np.argmin(eigenvalues)]
            
            # 归一化法向量
            normal = normal / np.linalg.norm(normal)
            
            self.get_logger().info(f'原始法向量: ({normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f})')
            self.get_logger().info(f'点云质心: ({centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f})')
            self.get_logger().debug(f'特征值: {eigenvalues}')
            
            return normal
            
        except Exception as e:
            self.get_logger().error(f'平面拟合失败: {e}')
            return None

    def calculate_orientation_from_normal(self, normal):
        """根据法向量计算四元数姿态"""
        # 目标：让机械臂的z轴（工具方向）与法向量对齐
        
        # 默认的z轴方向
        z_axis = np.array([0, 0, 1])
        
        # 如果法向量已经与z轴对齐，返回单位四元数
        if np.allclose(normal, z_axis, atol=1e-6):
            return np.array([0, 0, 0, 1])  # 无旋转
        elif np.allclose(normal, -z_axis, atol=1e-6):
            return np.array([1, 0, 0, 0])  # 180度绕x轴旋转
        
        # 计算旋转轴（叉积）
        rotation_axis = np.cross(z_axis, normal)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        
        # 计算旋转角度
        cos_angle = np.dot(z_axis, normal)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        
        # 手动创建四元数 (使用轴角表示)
        sin_half_angle = np.sin(angle / 2)
        cos_half_angle = np.cos(angle / 2)
        
        quat = np.array([
            rotation_axis[0] * sin_half_angle,  # x
            rotation_axis[1] * sin_half_angle,  # y  
            rotation_axis[2] * sin_half_angle,  # z
            cos_half_angle                      # w
        ])
        
        return quat

    def publish_target_visualization(self):
        """发布目标位置可视化"""
        if self.current_target_pose is None:
            return
        
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        target_pos = self.current_target_pose['position']
        target_quat = self.current_target_pose['orientation']
        frame_id = self.current_target_pose['frame_id']
        normal = self.current_target_pose['normal']
        button_pos = self.current_target_pose['button_position']
        
        # 1. 目标位置标记（球体）
        target_marker = Marker()
        target_marker.header.stamp = current_time
        target_marker.header.frame_id = frame_id
        target_marker.ns = "target_position"
        target_marker.id = 0
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        
        target_marker.pose.position.x = float(target_pos[0])
        target_marker.pose.position.y = float(target_pos[1])
        target_marker.pose.position.z = float(target_pos[2])
        target_marker.pose.orientation.x = float(target_quat[0])
        target_marker.pose.orientation.y = float(target_quat[1])
        target_marker.pose.orientation.z = float(target_quat[2])
        target_marker.pose.orientation.w = float(target_quat[3])
        
        target_marker.scale.x = self.marker_scale * 2
        target_marker.scale.y = self.marker_scale * 2
        target_marker.scale.z = self.marker_scale * 2
        target_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)  # 橙色
        
        target_marker.lifetime.sec = int(self.marker_lifetime)
        target_marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        marker_array.markers.append(target_marker)
        
        # 2. 姿态坐标轴标记
        axis_length = 0.05
        axis_markers = self.create_axis_markers(target_pos, target_quat, frame_id, current_time, axis_length)
        marker_array.markers.extend(axis_markers)
        
        # 3. 从按钮到目标的连线
        line_marker = Marker()
        line_marker.header.stamp = current_time
        line_marker.header.frame_id = frame_id
        line_marker.ns = "target_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        line_marker.scale.x = 0.005  # 线宽
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)  # 青色
        
        # 添加线的端点
        button_point = Point()
        button_point.x = float(button_pos[0])
        button_point.y = float(button_pos[1])
        button_point.z = float(button_pos[2])
        line_marker.points.append(button_point)
        
        target_point = Point()
        target_point.x = float(target_pos[0])
        target_point.y = float(target_pos[1])
        target_point.z = float(target_pos[2])
        line_marker.points.append(target_point)
        
        line_marker.lifetime.sec = int(self.marker_lifetime)
        line_marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        marker_array.markers.append(line_marker)
        
        # 4. 法向量箭头
        arrow_marker = Marker()
        arrow_marker.header.stamp = current_time
        arrow_marker.header.frame_id = frame_id
        arrow_marker.ns = "normal_arrow"
        arrow_marker.id = 0
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        
        # 箭头从按钮位置开始，沿法向量方向指向目标位置
        start_point = Point()
        start_point.x = float(button_pos[0])
        start_point.y = float(button_pos[1])
        start_point.z = float(button_pos[2])
        arrow_marker.points.append(start_point)
        
        end_point = Point()
        end_point.x = float(target_pos[0])
        end_point.y = float(target_pos[1])
        end_point.z = float(target_pos[2])
        arrow_marker.points.append(end_point)
        
        arrow_marker.scale.x = 0.005  # 箭头轴直径
        arrow_marker.scale.y = 0.015  # 箭头头部直径
        arrow_marker.scale.z = 0.02   # 箭头头部长度
        arrow_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)  # 紫色
        
        arrow_marker.lifetime.sec = int(self.marker_lifetime)
        arrow_marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        marker_array.markers.append(arrow_marker)
        
        # 发布标记
        self.target_marker_publisher.publish(marker_array)
        self.get_logger().debug(f'发布了目标可视化标记: {len(marker_array.markers)} 个')

    def create_axis_markers(self, position, orientation, frame_id, timestamp, axis_length):
        """创建坐标轴标记"""
        markers = []
        
        # 将四元数转换为旋转矩阵（手动实现）
        qx, qy, qz, qw = orientation
        
        # 四元数到旋转矩阵的转换
        rotation_matrix = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # 定义坐标轴颜色
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),  # X轴 - 红色
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8),  # Y轴 - 绿色  
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)   # Z轴 - 蓝色
        ]
        
        axis_names = ['x', 'y', 'z']
        
        for i in range(3):
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = frame_id
            marker.ns = f"target_axis_{axis_names[i]}"
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # 起点
            start_point = Point()
            start_point.x = float(position[0])
            start_point.y = float(position[1])
            start_point.z = float(position[2])
            marker.points.append(start_point)
            
            # 终点（沿各个轴方向）
            axis_direction = rotation_matrix[:, i]
            end_pos = position + axis_direction * axis_length
            end_point = Point()
            end_point.x = float(end_pos[0])
            end_point.y = float(end_pos[1])
            end_point.z = float(end_pos[2])
            marker.points.append(end_point)
            
            marker.scale.x = 0.005  # 箭头轴直径
            marker.scale.y = 0.01   # 箭头头部直径
            marker.scale.z = 0.01   # 箭头头部长度
            marker.color = colors[i]
            
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            
            markers.append(marker)
        
        return markers

    def publish_target_pose(self):
        """发布目标姿态用于机械臂控制"""
        if self.current_target_pose is None:
            return
        
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.current_target_pose['frame_id']
        
        target_pos = self.current_target_pose['position']
        target_quat = self.current_target_pose['orientation']
        
        pose_stamped.pose.position.x = float(target_pos[0])
        pose_stamped.pose.position.y = float(target_pos[1])
        pose_stamped.pose.position.z = float(target_pos[2])
        pose_stamped.pose.orientation.x = float(target_quat[0])
        pose_stamped.pose.orientation.y = float(target_quat[1])
        pose_stamped.pose.orientation.z = float(target_quat[2])
        pose_stamped.pose.orientation.w = float(target_quat[3])
        
        self.target_pose_publisher.publish(pose_stamped)
        self.get_logger().debug('发布目标姿态')


def main(args=None):
    rclpy.init(args=args)
    node = ButtonTargetPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
