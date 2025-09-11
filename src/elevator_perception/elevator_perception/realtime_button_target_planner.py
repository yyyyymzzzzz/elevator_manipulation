#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import json
import copy
from collections import defaultdict
import time

from geometry_msgs.msg import PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class RealtimeButtonTargetPlanner(Node):
    def __init__(self):
        super().__init__('realtime_button_target_planner')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_distance', 0.20),  # 目标位置距离按钮平面的距离 (m)
                ('use_fixed_normal', True),  # 是否使用固定法向量
                ('fixed_normal_x', 0.0),
                ('fixed_normal_y', 0.0), 
                ('fixed_normal_z', 1.0),
                ('target_frame', 'world'),
                ('camera_frame', 'camera_color_optical_frame'),
                ('marker_scale', 0.02),
                ('marker_lifetime', 0.5),  # 更短的标记生命周期以提高响应性
                ('enable_smoothing', True),  # 启用位置平滑
                ('smoothing_alpha', 0.3),  # 平滑系数
                ('max_position_change', 0.05),  # 最大位置变化阈值 (m)
                ('publish_frequency', 30.0),  # 发布频率 (Hz)
                ('button_selection_method', 'sequential'),  # 'sequential', 'all', 'closest'
                ('sequential_duration', 20.0),  # 顺序模式下每个按钮的持续时间
                ('target_cleanup_interval', 2.0),  # 清理检查间隔 (s)
                ('target_expiry_time', 5.0),  # 目标过期时间 (s)
            ]
        )
        
        # 获取参数值
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.use_fixed_normal = self.get_parameter('use_fixed_normal').get_parameter_value().bool_value
        self.fixed_normal = np.array([
            self.get_parameter('fixed_normal_x').get_parameter_value().double_value,
            self.get_parameter('fixed_normal_y').get_parameter_value().double_value,
            self.get_parameter('fixed_normal_z').get_parameter_value().double_value
        ])
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        self.enable_smoothing = self.get_parameter('enable_smoothing').get_parameter_value().bool_value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').get_parameter_value().double_value
        self.max_position_change = self.get_parameter('max_position_change').get_parameter_value().double_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.button_selection_method = self.get_parameter('button_selection_method').get_parameter_value().string_value
        self.sequential_duration = self.get_parameter('sequential_duration').get_parameter_value().double_value
        
        # 清理相关参数
        self.target_cleanup_interval = self.get_parameter('target_cleanup_interval').get_parameter_value().double_value
        self.target_expiry_time = self.get_parameter('target_expiry_time').get_parameter_value().double_value
        
        # TF2 缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 订阅者
        self.button_markers_subscription = self.create_subscription(
            MarkerArray,
            '/button_3d_markers',
            self.button_markers_callback,
            10
        )
        
        self.press_completion_subscriber = self.create_subscription(
            String,
            '/button_press/completed',
            self.press_completion_callback,
            10
        )
        
        # 订阅决策节点发送的目标按钮信息
        self.target_button_subscriber = self.create_subscription(
            String,
            '/decision/target_button',
            self.target_button_callback,
            10
        )

        # 订阅当前机器人状态
        self.robot_status_subscriber = self.create_subscription(
            String,
            '/decision_maker/status',
            self.robot_status_callback,
            10
        )

        # 发布者
        self.target_marker_publisher = self.create_publisher(
            MarkerArray,
            '/realtime_target_markers',
            10
        )
        
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/button_target_pose',
            10
        )
        
        self.all_targets_publisher = self.create_publisher(
            String,
            '/button_targets_json',
            10
        )
        
        # 发布当前选中按钮的位置信息（给AGV控制器使用）
        self.current_button_info_publisher = self.create_publisher(
            String,
            '/current_button_info',
            10
        )
        
        # 数据存储
        self.current_button_positions = {}  # {class_name: button_data}
        self.target_poses = {}  # {class_name: target_pose}
        self.last_update_times = {}  # {class_name: timestamp}
        
        # 历史按钮类别管理
        self.historical_button_classes = []  # 按出现顺序存储的所有类别
        self.current_detected_classes = set()  # 当前检测到的类别
        self.last_seen_times = {}  # {class_name: timestamp} 每个类别最后被看到的时间
        
        # 顺序发布相关
        self.current_selected_class = None  # 当前选中的按钮类别
        self.current_class_index = 0  # 在历史列表中的索引
        self.last_button_switch_time = None
        self.waiting_for_missing_button = False  # 是否在等待缺失的按钮
        self.missing_button_wait_time = 3.0  # 等待缺失按钮的时间（秒）
        self.missing_button_start_time = None
        
        # AGV控制的目标按钮
        self.agv_target_button = None  # AGV指定的目标按钮
        self.agv_target_location = None  # AGV当前地点
        self.use_agv_control = False  # 是否使用AGV控制模式
        
        # AGV状态相关
        self.agv_is_moving = False
        self.agv_stop_time = None
        self.planning_delay_after_stop = 2.0  # AGV停止后等待2秒再开始目标规划
        
        # 清理过时目标的相关参数
        self.last_cleanup_time = time.time()
        
        # 预定义的类别排序（用于稳定排序）
        self.class_name_priority = {
            '1': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10,
            '11': 11, '12': 12, '13': 13, '14': 14, '15': 15, '16': 16, '17': 17, '18': 18, '19': 19, '20': 20,
            'Open': 100, 'Close': 101, 'Alarm': 102, 'Up': 103, 'Down': 104
        }
        
        # 创建高频发布定时器
        self.publish_timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_targets_callback
        )
        
        # 顺序切换定时器
        if self.button_selection_method == 'sequential':
            self.switch_timer = self.create_timer(
                1.0,  # 每秒检查一次是否需要切换
                self.check_button_switch_callback
            )
        elif self.button_selection_method == 'all':
            # 在all模式下，也需要切换当前高亮的按钮用于可视化
            self.switch_timer = self.create_timer(
                1.0,
                self.check_button_switch_callback
            )
        
        self.get_logger().info('实时按钮目标规划节点已启动')
        self.get_logger().info(f'目标距离: {self.target_distance}m')
        self.get_logger().info(f'使用固定法向量: {self.use_fixed_normal}')
        if self.use_fixed_normal:
            self.get_logger().info(f'固定法向量: ({self.fixed_normal[0]}, {self.fixed_normal[1]}, {self.fixed_normal[2]})')
        self.get_logger().info(f'发布频率: {self.publish_frequency}Hz')
        self.get_logger().info(f'按钮选择方法: {self.button_selection_method}')
        self.get_logger().info(f'目标清理: 检查间隔={self.target_cleanup_interval}s, 过期时间={self.target_expiry_time}s')
        self.get_logger().info('位置滤波: 已禁用，直接使用检测位置确保准确性')

    def press_completion_callback(self, msg):
        # 修改：收到任何成功信号都停止发布目标，等待AGV移动和发送新目标
        if msg.data == "success" or msg.data == "cycle_complete":
            self.get_logger().info(f"收到 {msg.data} 信号，按钮操作完成，停止发布目标等待AGV移动并发送新目标")
            # 停止发布目标，等待AGV移动到新位置并发送新的目标按钮
            self.use_agv_control = False
            self.agv_target_button = None
            self.agv_target_location = None
    
    def target_button_callback(self, msg):
        """接收目标按钮信息"""
        try:
            target_data = json.loads(msg.data)
            target_button = target_data.get('target_button')
            location = target_data.get('location')
            command = target_data.get('command')
            
            if command == 'press_button' and target_button:
                self.get_logger().info(f"🎯 接收到新目标按钮指令: {target_button} (地点: {location})，开始发布目标位置")
                self.agv_target_button = target_button
                self.agv_target_location = location
                self.use_agv_control = True
                
                # 立即切换到指定的按钮
                self.switch_to_agv_target_button(target_button)
                
        except json.JSONDecodeError:
            self.get_logger().error("无法解析AGV目标按钮数据")

    def robot_status_callback(self, msg):
        """接收机器人状态并清理历史数据"""
        try:
            data = json.loads(msg.data)
            is_moving = data['is_moving']
            
            if is_moving:
                if not self.agv_is_moving:
                    self.get_logger().info("AGV开始移动，暂停目标规划并清理按钮检测历史数据")
                self.agv_is_moving = True
                self.agv_stop_time = None
                # 清理历史数据
                self.current_button_positions.clear()
                self.target_poses.clear()
                self.last_update_times.clear()
                self.current_detected_classes.clear()
                self.last_seen_times.clear()
                
                # 停止发布目标
                self.use_agv_control = False
                self.agv_target_button = None
                
                # 发布空的标记数组清空显示
                empty_marker_array = MarkerArray()
                self.target_marker_publisher.publish(empty_marker_array)   
            else:
                if self.agv_is_moving:
                    self.get_logger().info(f"AGV停止移动，等待新的目标指令（不自动恢复目标规划）")
                    self.agv_stop_time = self.get_clock().now()
                    self._first_planning_after_stop = True
                self.agv_is_moving = False
                
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid AGV motion status message")

    def button_markers_callback(self, msg):
        """接收按钮3D位置标记并实时处理"""
        # 检查AGV状态，如果正在移动则跳过处理
        if self.agv_is_moving:
            return
            
        # 如果AGV刚停止，检查是否已经等待足够时间
        if self.agv_stop_time is not None:
            current_time_ros = self.get_clock().now()
            time_since_stop = (current_time_ros - self.agv_stop_time).nanoseconds / 1e9
            if time_since_stop < self.planning_delay_after_stop:
                return  # 还未到规划时间
            else:
                # 首次恢复规划时的日志
                if hasattr(self, '_first_planning_after_stop') and self._first_planning_after_stop:
                    self.get_logger().info("恢复目标规划")
                    self._first_planning_after_stop = False
        
        current_time = time.time()
        
        # 提取按钮位置信息和类别名称
        detected_buttons = {}
        current_classes = set()
        
        for marker in msg.markers:
            if marker.ns == "button_detections" and marker.action == Marker.ADD:
                # 从marker.text字段获取类别名称
                class_name = marker.text.strip() if marker.text else f"未知{marker.id}"
                
                # 跳过无效的类别名称
                if not class_name or class_name.startswith('未知'):
                    continue
                
                position = np.array([
                    marker.pose.position.x,
                    marker.pose.position.y,
                    marker.pose.position.z
                ])
                
                detected_buttons[class_name] = {
                    'class_name': class_name,
                    'position': position,
                    'frame_id': marker.header.frame_id,
                    'timestamp': current_time
                }
                
                current_classes.add(class_name)
        
        if not detected_buttons:
            self.get_logger().debug('没有检测到有效的按钮位置')
            return
        
        # 更新历史类别列表和当前检测状态
        self.update_historical_classes(current_classes, current_time)
        
        # 更新按钮位置
        positions_changed = self.update_button_positions(detected_buttons)
        
        # 清理过时的目标位置
        self.cleanup_expired_targets()
        
        # 只在位置有明显变化时重新计算目标位置
        if positions_changed:
            self.calculate_targets_fast()
            self.get_logger().debug('按钮位置发生变化，重新计算目标位置')

    def update_historical_classes(self, current_classes, current_time):
        """更新历史类别列表和检测状态"""
        self.current_detected_classes = current_classes
        
        # 更新所有检测到的类别的最后见到时间
        for class_name in current_classes:
            self.last_seen_times[class_name] = current_time
            
            # 如果是新类别，按优先级顺序插入历史列表
            if class_name not in self.historical_button_classes:
                self.insert_class_by_priority(class_name)
                self.get_logger().info(f'新检测到按钮类别: {class_name}')
        
        # 显示当前历史类别列表
        if len(self.historical_button_classes) > 0:
            detected_list = [f"{cls}{'✓' if cls in current_classes else '✗'}" 
                           for cls in self.historical_button_classes]
            self.get_logger().debug(f'历史按钮类别: {detected_list}')

    def insert_class_by_priority(self, new_class):
        """将新类别按优先级插入历史列表的正确位置"""
        new_priority = self.class_name_priority.get(new_class, 999)
        
        # 找到插入位置
        insert_pos = len(self.historical_button_classes)
        for i, existing_class in enumerate(self.historical_button_classes):
            existing_priority = self.class_name_priority.get(existing_class, 999)
            if new_priority < existing_priority or (new_priority == existing_priority and new_class < existing_class):
                insert_pos = i
                break
        
        self.historical_button_classes.insert(insert_pos, new_class)
        self.get_logger().info(f'历史按钮列表更新: {self.historical_button_classes}')

    def update_button_positions(self, detected_buttons):
        """更新按钮位置数据，按类别名称存储"""
        positions_changed = False
        
        for class_name, button_data in detected_buttons.items():
            new_position = button_data['position']
            
            # 检查位置是否有明显变化
            if class_name in self.current_button_positions:
                old_position = self.current_button_positions[class_name]['position']
                position_change = np.linalg.norm(new_position - old_position)
                
                # 如果位置变化小于阈值（2mm），不更新
                if position_change < 0.002:
                    continue
                
                self.get_logger().debug(f'按钮 {class_name} 位置变化: {position_change*1000:.1f}mm')
            
            # 更新位置数据
            self.current_button_positions[class_name] = button_data
            self.last_update_times[class_name] = button_data['timestamp']
            positions_changed = True
            
        return positions_changed

    def cleanup_expired_targets(self):
        """清理长时间未检测到的按钮的目标位置"""
        current_time = time.time()
        
        # 检查是否需要执行清理
        if current_time - self.last_cleanup_time < self.target_cleanup_interval:
            return
        
        self.last_cleanup_time = current_time
        
        # 找出需要清理的类别
        expired_classes = []
        for class_name in list(self.target_poses.keys()):
            if class_name not in self.last_seen_times:
                # 如果从来没有记录过见到时间，直接删除
                expired_classes.append(class_name)
                continue
                
            time_since_last_seen = current_time - self.last_seen_times[class_name]
            if time_since_last_seen > self.target_expiry_time:
                expired_classes.append(class_name)
        
        # 清理过期的目标和相关数据
        for class_name in expired_classes:
            self.get_logger().info(f'清理过期按钮目标: {class_name} (未见到 {current_time - self.last_seen_times.get(class_name, current_time):.1f}s)')
            
            # 移除相关数据
            if class_name in self.target_poses:
                del self.target_poses[class_name]
            if class_name in self.current_button_positions:
                del self.current_button_positions[class_name]
            if class_name in self.last_update_times:
                del self.last_update_times[class_name]
            if class_name in self.last_seen_times:
                del self.last_seen_times[class_name]
            
            # 从历史类别列表中移除
            if class_name in self.historical_button_classes:
                self.historical_button_classes.remove(class_name)
            
            # 如果当前选中的类别被清理了，需要重新选择
            if self.current_selected_class == class_name:
                self.reset_current_selection()
        
        # 如果清理了类别，更新历史列表
        if expired_classes:
            self.get_logger().info(f'清理完成，剩余按钮类别: {self.historical_button_classes}')

    def reset_current_selection(self):
        """重置当前选中的按钮类别"""
        self.current_selected_class = None
        self.current_class_index = 0
        self.last_button_switch_time = None
        self.waiting_for_missing_button = False
        self.missing_button_start_time = None
        
        # 如果还有其他按钮，选择第一个
        if self.historical_button_classes:
            self.current_selected_class = self.historical_button_classes[0]
            self.current_class_index = 0
            self.last_button_switch_time = self.get_clock().now()
            self.get_logger().info(f'重新选择按钮: {self.current_selected_class}')

    def calculate_targets_fast(self):
        """快速计算目标位置（避免复杂的点云处理）"""
        for class_name, button_data in self.current_button_positions.items():
            # 使用简化的目标位置计算
            button_position = button_data['position']
            frame_id = button_data['frame_id']
            
            # 使用固定法向量或简单估计
            if self.use_fixed_normal:
                # 将固定法向量转换到按钮坐标系
                normal = self.transform_normal_to_frame(self.fixed_normal, frame_id)
            else:
                # 使用简单的默认法向量（假设按钮面向Z轴负方向）
                normal = np.array([0.0, 0.0, -1.0])
            
            # 确保法向量归一化
            normal = normal / np.linalg.norm(normal)
            
            # 计算目标位置（沿法向量方向外移）
            target_position = button_position + normal * self.target_distance
            
            # 计算目标姿态（参照原始代码逻辑：z轴从目标指向按钮）
            target_orientation = self.calculate_simple_orientation(normal, target_position, button_position)
            
            # 存储目标位置
            self.target_poses[class_name] = {
                'position': target_position,
                'orientation': target_orientation,
                'frame_id': frame_id,
                'normal': normal,
                'button_position': button_position,
                'class_name': class_name
            }

    def transform_normal_to_frame(self, normal, target_frame):
        """将法向量转换到目标坐标系"""
        try:
            # 获取从world到target_frame的变换
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                'world',  # 假设固定法向量在world坐标系
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # 提取旋转部分并应用到法向量
            q = transform.transform.rotation
            # 简化处理：这里可以添加四元数旋转计算
            # 暂时返回原始法向量
            return normal
            
        except Exception as e:
            self.get_logger().debug(f'法向量坐标转换失败: {e}，使用原始法向量')
            return normal

    def calculate_simple_orientation(self, normal, target_position, button_position):
        """基于法向量、目标位置和按钮位置计算目标姿态
        
        参照原始代码的逻辑：
        - z轴从目标位置指向按钮位置（垂直墙面向内）
        - x轴在垂直于z轴的平面内
        - y轴由右手法则确定
        """
        # z轴：从目标位置指向按钮位置（这是正确的方向）
        z_axis = button_position - target_position
        z_axis = z_axis / np.linalg.norm(z_axis)
        
        # 确保法向量归一化
        normal = normal / np.linalg.norm(normal)
        
        # x轴：在垂直于z轴的平面内，优先选择与法向量垂直的方向
        # 使用法向量与z轴的叉积来构造x轴
        x_direction = np.cross(normal, z_axis)
        x_axis_norm = np.linalg.norm(x_direction)
        
        if x_axis_norm < 1e-6:
            # 如果法向量与z轴平行，选择一个垂直的方向作为x轴
            if abs(z_axis[0]) < 0.9:
                x_axis = np.array([1.0, 0.0, 0.0])
            else:
                x_axis = np.array([0.0, 1.0, 0.0])
            # 确保x轴与z轴垂直
            dot_product = np.dot(x_axis, z_axis)
            x_axis = x_axis - dot_product * z_axis
            x_axis = x_axis / np.linalg.norm(x_axis)
        else:
            x_axis = x_direction / x_axis_norm
        
        # y轴：由右手法则确定 (z × x = y)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # 构建旋转矩阵
        rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
        
        # 转换为四元数
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        self.get_logger().debug(f'目标姿态 - X轴: ({x_axis[0]:.3f}, {x_axis[1]:.3f}, {x_axis[2]:.3f})')
        self.get_logger().debug(f'目标姿态 - Y轴: ({y_axis[0]:.3f}, {y_axis[1]:.3f}, {y_axis[2]:.3f})')
        self.get_logger().debug(f'目标姿态 - Z轴(指向按钮): ({z_axis[0]:.3f}, {z_axis[1]:.3f}, {z_axis[2]:.3f})')
        
        return quaternion

    def rotation_matrix_to_quaternion(self, R):
        """将旋转矩阵转换为四元数 [x, y, z, w]"""
        trace = np.trace(R)
        
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        return np.array([x, y, z, w])

    def check_button_switch_callback(self):
        """基于历史类别列表进行顺序切换"""
        if self.button_selection_method not in ['sequential', 'all'] or not self.historical_button_classes:
            return
        
        current_time = self.get_clock().now()
        current_timestamp = time.time()
        
        # 如果没有当前选中的按钮类别，选择第一个
        if self.current_selected_class is None:
            self.current_selected_class = self.historical_button_classes[0]
            self.current_class_index = 0
            self.last_button_switch_time = current_time
            self.waiting_for_missing_button = False
            
            self.get_logger().info(f'选择第一个按钮: {self.current_selected_class} '
                                  f'[1/{len(self.historical_button_classes)}]')
            return
        
        # 检查当前选中的按钮是否存在
        current_exists = self.current_selected_class in self.current_detected_classes
        
        if not current_exists and not self.waiting_for_missing_button:
            # 当前按钮消失了，开始等待
            self.waiting_for_missing_button = True
            self.missing_button_start_time = current_timestamp
            self.get_logger().info(f'按钮 {self.current_selected_class} 消失，等待 {self.missing_button_wait_time}s...')
            return
        
        if self.waiting_for_missing_button:
            if current_exists:
                # 按钮重新出现了
                self.waiting_for_missing_button = False
                self.missing_button_start_time = None
                self.get_logger().info(f'按钮 {self.current_selected_class} 重新出现')
                return
            else:
                # 检查等待时间是否已过
                if current_timestamp - self.missing_button_start_time >= self.missing_button_wait_time:
                    self.get_logger().info(f'按钮 {self.current_selected_class} 等待超时，切换到下一个')
                    self.switch_to_next_button(current_time)
                    return
                else:
                    # 继续等待
                    remaining_time = self.missing_button_wait_time - (current_timestamp - self.missing_button_start_time)
                    self.get_logger().debug(f'等待按钮 {self.current_selected_class} 重新出现，剩余: {remaining_time:.1f}s')
                    return
        
        # 正常的时间切换逻辑
        # 仅当模式为 'all' 时（用于可视化高亮切换），才执行基于时间的切换
        if self.button_selection_method == 'all' and self.last_button_switch_time is not None:
            elapsed_time = (current_time - self.last_button_switch_time).nanoseconds / 1e9

            if elapsed_time >= self.sequential_duration:
                self.switch_to_next_button(current_time)

    def switch_to_next_button(self, current_time):
        """切换到下一个按钮"""
        if not self.historical_button_classes:
            return
        
        # 找到下一个存在的按钮
        start_index = self.current_class_index
        next_index = start_index
        attempts = 0
        max_attempts = len(self.historical_button_classes)
        
        while attempts < max_attempts:
            next_index = (next_index + 1) % len(self.historical_button_classes)
            next_class = self.historical_button_classes[next_index]
            
            # 检查这个按钮是否存在
            if next_class in self.current_detected_classes:
                # 找到了存在的按钮
                self.current_selected_class = next_class
                self.current_class_index = next_index
                self.last_button_switch_time = current_time
                self.waiting_for_missing_button = False
                self.missing_button_start_time = None
                
                if self.button_selection_method == 'sequential':
                    self.get_logger().info(f'切换到按钮 {self.current_selected_class} '
                                          f'[{next_index + 1}/{len(self.historical_button_classes)}]')
                else:  # all mode
                    self.get_logger().info(f'高亮切换到按钮 {self.current_selected_class} '
                                          f'[{next_index + 1}/{len(self.historical_button_classes)}] - 继续发布所有目标位置')
                return
            
            attempts += 1
        
        # 如果所有按钮都不存在，保持当前选择但记录警告
        self.get_logger().warn('所有历史按钮都不存在，保持当前选择')
        self.waiting_for_missing_button = True
        self.missing_button_start_time = time.time()
    
    def switch_to_agv_target_button(self, target_button: str):
        """切换到AGV指定的目标按钮"""
        # 检查目标按钮是否在历史按钮列表中
        if target_button in self.historical_button_classes:
            # 直接切换到指定的按钮
            self.current_selected_class = target_button
            self.current_class_index = self.historical_button_classes.index(target_button)
            self.last_button_switch_time = self.get_clock().now()
            self.waiting_for_missing_button = False
            self.missing_button_start_time = None
            
            # 检查按钮是否存在
            button_exists = target_button in self.current_detected_classes
            status = "✓存在" if button_exists else "✗缺失"
            
            self.get_logger().info(f'🎯 AGV指定目标按钮: {target_button} ({status})')
            
            if not button_exists:
                self.get_logger().warn(f'AGV指定的按钮 {target_button} 当前不可见，等待检测...')
                self.waiting_for_missing_button = True
                self.missing_button_start_time = time.time()
        else:
            self.get_logger().error(f'AGV指定的按钮 {target_button} 不在历史按钮列表中: {self.historical_button_classes}')
            # 如果不在列表中，将其添加到列表末尾
            self.historical_button_classes.append(target_button)
            self.current_selected_class = target_button
            self.current_class_index = len(self.historical_button_classes) - 1
            self.waiting_for_missing_button = True
            self.missing_button_start_time = time.time()
            self.get_logger().info(f'已将 {target_button} 添加到历史按钮列表，等待检测...')

    def publish_targets_callback(self):
        """高频发布目标位置"""
        # 检查AGV状态，如果正在移动则跳过发布
        if self.agv_is_moving:
            return
        
        # 如果没有AGV控制目标，则不发布任何目标
        if not self.use_agv_control or self.agv_target_button is None:
            return
            
        # 如果AGV刚停止，检查是否已经等待足够时间
        if self.agv_stop_time is not None:
            current_time = self.get_clock().now()
            time_since_stop = (current_time - self.agv_stop_time).nanoseconds / 1e9
            if time_since_stop < self.planning_delay_after_stop:
                return  # 还未到发布时间
        
        if not self.target_poses:
            return
        
        # 只发布AGV控制模式的目标
        self.publish_agv_controlled_target()
        
        # 发布可视化标记
        self.publish_visualization()

    def publish_sequential_target(self):
        """发布当前选中按钮的目标位置"""
        if (self.current_selected_class is None or 
            self.current_selected_class not in self.target_poses):
            return
        
        target_pose = self.target_poses[self.current_selected_class]
        self.publish_single_target_pose(target_pose)
        
        # 发布当前选中按钮的详细信息给AGV控制器
        self.publish_current_button_info()
        
        # 每隔一定时间输出当前选中按钮的详细信息
        current_time = time.time()
        if not hasattr(self, 'last_info_time'):
            self.last_info_time = current_time
        
        if current_time - self.last_info_time > 2.0:  # 每2秒输出一次详细信息
            current_index = self.current_class_index
            button_exists = self.current_selected_class in self.current_detected_classes
            status = "✓存在" if button_exists else "✗缺失"
            
            if self.waiting_for_missing_button and self.missing_button_start_time is not None:
                remaining_wait = max(0, self.missing_button_wait_time - (current_time - self.missing_button_start_time))
                status += f" (等待{remaining_wait:.1f}s)"
            elif self.last_button_switch_time is not None:
                elapsed_time = (self.get_clock().now() - self.last_button_switch_time).nanoseconds / 1e9
                remaining_time = max(0, self.sequential_duration - elapsed_time)
                status += f" (剩余{remaining_time:.1f}s)"
            
            self.get_logger().info(f'🎯 当前目标: {self.current_selected_class} '
                                  f'[{current_index + 1}/{len(self.historical_button_classes)}] {status}')
                                  
            if button_exists:
                target_pos = target_pose['position']
                button_pos = target_pose['button_position']
                self.get_logger().info(f'📍 按钮位置: ({button_pos[0]:.3f}, {button_pos[1]:.3f}, {button_pos[2]:.3f})')
                self.get_logger().info(f'🎯 目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})')
                
            self.last_info_time = current_time
    
    def publish_agv_controlled_target(self):
        """发布AGV控制指定的按钮目标位置"""
        if (self.agv_target_button is None or 
            self.agv_target_button not in self.target_poses):
            # 如果指定的按钮不存在目标位置，记录警告但不发布
            if self.agv_target_button:
                button_exists = self.agv_target_button in self.current_detected_classes
                if not button_exists:
                    self.get_logger().debug(f'AGV指定按钮 {self.agv_target_button} 未检测到，等待中...')
                else:
                    self.get_logger().warn(f'AGV指定按钮 {self.agv_target_button} 已检测到但无目标位置计算')
            return
        
        target_pose = self.target_poses[self.agv_target_button]
        self.publish_single_target_pose(target_pose)
        
        # 发布当前按钮信息
        self.publish_current_button_info()
        
        # 定期输出AGV控制模式的状态信息
        current_time = time.time()
        if not hasattr(self, 'last_agv_info_time'):
            self.last_agv_info_time = current_time
        
        if current_time - self.last_agv_info_time > 2.0:  # 每2秒输出一次
            button_exists = self.agv_target_button in self.current_detected_classes
            status = "✓存在" if button_exists else "✗缺失"
            
            self.get_logger().info(f'🎯 AGV控制模式 - 目标按钮: {self.agv_target_button} '
                                  f'(地点: {self.agv_target_location}) {status}')
            
            if button_exists:
                target_pos = target_pose['position']
                button_pos = target_pose['button_position']
                self.get_logger().info(f'📍 按钮位置: ({button_pos[0]:.3f}, {button_pos[1]:.3f}, {button_pos[2]:.3f})')
                self.get_logger().info(f'🎯 目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})')
            
            self.last_agv_info_time = current_time

    def publish_closest_target(self):
        """发布距离机器人最近的按钮目标位置"""
        if not self.target_poses:
            return
        
        # 简化处理：选择z值最小的按钮（假设距离相机最近）
        closest_class = min(self.target_poses.keys(), 
                           key=lambda cls: self.current_button_positions[cls]['position'][2])
        
        target_pose = self.target_poses[closest_class]
        self.publish_single_target_pose(target_pose)

    def publish_all_targets(self):
        """发布所有按钮的目标位置JSON"""
        targets_data = {
            'timestamp': time.time(),
            'frame_id': self.target_frame,
            'count': len(self.target_poses),
            'targets': []
        }
        
        for class_name, target_pose in self.target_poses.items():
            target_info = {
                'class_name': class_name,
                'position': {
                    'x': float(target_pose['position'][0]),
                    'y': float(target_pose['position'][1]),
                    'z': float(target_pose['position'][2])
                },
                'orientation': {
                    'x': float(target_pose['orientation'][0]),
                    'y': float(target_pose['orientation'][1]),
                    'z': float(target_pose['orientation'][2]),
                    'w': float(target_pose['orientation'][3])
                },
                'button_position': {
                    'x': float(target_pose['button_position'][0]),
                    'y': float(target_pose['button_position'][1]),
                    'z': float(target_pose['button_position'][2])
                },
                'frame_id': target_pose['frame_id']
            }
            targets_data['targets'].append(target_info)
        
        # 发布JSON
        json_msg = String()
        json_msg.data = json.dumps(targets_data, indent=2)
        self.all_targets_publisher.publish(json_msg)

    def publish_single_target_pose(self, target_pose):
        """发布单个目标位置"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = target_pose['frame_id']
        
        pose_msg.pose.position.x = float(target_pose['position'][0])
        pose_msg.pose.position.y = float(target_pose['position'][1])
        pose_msg.pose.position.z = float(target_pose['position'][2])
        
        pose_msg.pose.orientation.x = float(target_pose['orientation'][0])
        pose_msg.pose.orientation.y = float(target_pose['orientation'][1])
        pose_msg.pose.orientation.z = float(target_pose['orientation'][2])
        pose_msg.pose.orientation.w = float(target_pose['orientation'][3])
        
        self.target_pose_publisher.publish(pose_msg)
    
    def publish_current_button_info(self):
        """发布当前选中按钮的详细信息给AGV控制器"""
        if (self.current_selected_class is None or 
            self.current_selected_class not in self.target_poses or
            self.current_selected_class not in self.current_button_positions):
            return
        
        target_pose = self.target_poses[self.current_selected_class]
        button_data = self.current_button_positions[self.current_selected_class]
        
        # 构建当前按钮信息
        button_info = {
            'timestamp': time.time(),
            'button_name': self.current_selected_class,
            'button_index': self.current_class_index + 1,
            'total_buttons': len(self.historical_button_classes),
            'button_exists': self.current_selected_class in self.current_detected_classes,
            'button_position': {
                'x': float(button_data['position'][0]),
                'y': float(button_data['position'][1]),
                'z': float(button_data['position'][2])
            },
            'target_position': {
                'x': float(target_pose['position'][0]),
                'y': float(target_pose['position'][1]),
                'z': float(target_pose['position'][2])
            },
            'target_orientation': {
                'x': float(target_pose['orientation'][0]),
                'y': float(target_pose['orientation'][1]),
                'z': float(target_pose['orientation'][2]),
                'w': float(target_pose['orientation'][3])
            },
            'frame_id': target_pose['frame_id'],
            'status': 'available' if self.current_selected_class in self.current_detected_classes else 'missing',
            'selection_method': 'agv_controlled' if self.use_agv_control else self.button_selection_method,
            'agv_controlled': self.use_agv_control,
            'agv_target_location': self.agv_target_location if self.use_agv_control else None
        }
        
        # 添加等待状态信息
        if self.waiting_for_missing_button and self.missing_button_start_time is not None:
            remaining_wait = max(0, self.missing_button_wait_time - (time.time() - self.missing_button_start_time))
            button_info['waiting_for_button'] = True
            button_info['remaining_wait_time'] = remaining_wait
        else:
            button_info['waiting_for_button'] = False
            button_info['remaining_wait_time'] = 0.0
        
        # 发布信息
        info_msg = String()
        info_msg.data = json.dumps(button_info, indent=2)
        self.current_button_info_publisher.publish(info_msg)

    def publish_visualization(self):
        """发布可视化标记"""
        marker_array = MarkerArray()
        current_time = self.get_clock().now().to_msg()
        
        # 为每个类别分配一个唯一的ID用于marker
        class_to_id = {cls: i for i, cls in enumerate(self.historical_button_classes)}
        
        # 记录当前活跃的marker ID，用于清理
        active_marker_ids = set()
        
        for class_name, target_pose in self.target_poses.items():
            marker_id = class_to_id.get(class_name, hash(class_name) % 1000)
            active_marker_ids.add(marker_id)
            active_marker_ids.add(marker_id + 1000)  # 连线marker的ID
            
            # 根据选择方法决定颜色
            if self.button_selection_method == 'sequential':
                is_selected = (class_name == self.current_selected_class)
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8) if is_selected else ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)
            else:
                is_highlighted = (class_name == self.current_selected_class)
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8) if is_highlighted else ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            
            # 目标位置标记
            target_marker = Marker()
            target_marker.header.stamp = current_time
            target_marker.header.frame_id = target_pose['frame_id']
            target_marker.ns = f"realtime_target_{class_name}"
            target_marker.id = marker_id
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            
            target_marker.pose.position.x = float(target_pose['position'][0])
            target_marker.pose.position.y = float(target_pose['position'][1])
            target_marker.pose.position.z = float(target_pose['position'][2])
            target_marker.pose.orientation.x = float(target_pose['orientation'][0])
            target_marker.pose.orientation.y = float(target_pose['orientation'][1])
            target_marker.pose.orientation.z = float(target_pose['orientation'][2])
            target_marker.pose.orientation.w = float(target_pose['orientation'][3])
            
            target_marker.scale.x = self.marker_scale * 2
            target_marker.scale.y = self.marker_scale * 2
            target_marker.scale.z = self.marker_scale * 2
            target_marker.color = color
            
            target_marker.lifetime.sec = int(self.marker_lifetime)
            target_marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            
            marker_array.markers.append(target_marker)
            
            # 连线标记
            line_marker = Marker()
            line_marker.header.stamp = current_time
            line_marker.header.frame_id = target_pose['frame_id']
            line_marker.ns = f"realtime_line_{class_name}"
            line_marker.id = marker_id + 1000  # 避免ID冲突
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            line_marker.scale.x = 0.002  # 线宽
            line_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.6)  # 青色
            
            # 从按钮到目标的连线
            button_point = Point()
            button_point.x = float(target_pose['button_position'][0])
            button_point.y = float(target_pose['button_position'][1])
            button_point.z = float(target_pose['button_position'][2])
            line_marker.points.append(button_point)
            
            target_point = Point()
            target_point.x = float(target_pose['position'][0])
            target_point.y = float(target_pose['position'][1])
            target_point.z = float(target_pose['position'][2])
            line_marker.points.append(target_point)
            
            line_marker.lifetime.sec = int(self.marker_lifetime)
            line_marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            
            marker_array.markers.append(line_marker)
        
        # 添加删除过期marker的标记 (通过发布DELETE动作的marker来清理可能残留的旧marker)
        # 这里使用一个更大的范围来确保清理所有可能的旧marker
        if not hasattr(self, 'last_cleanup_marker_time'):
            self.last_cleanup_marker_time = time.time()
            
        current_timestamp = time.time()
        if current_timestamp - self.last_cleanup_marker_time > 5.0:  # 每5秒执行一次marker清理
            self.last_cleanup_marker_time = current_timestamp
            
            # 发布DELETE标记来清理可能残留的marker
            for cleanup_id in range(2000):  # 清理可能的ID范围
                if cleanup_id not in active_marker_ids and cleanup_id not in [id + 1000 for id in active_marker_ids]:
                    # 清理目标marker
                    delete_marker = Marker()
                    delete_marker.header.stamp = current_time
                    delete_marker.header.frame_id = self.target_frame
                    delete_marker.ns = "realtime_target_cleanup"
                    delete_marker.id = cleanup_id
                    delete_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_marker)
                    
                    # 清理连线marker
                    delete_line_marker = Marker()
                    delete_line_marker.header.stamp = current_time
                    delete_line_marker.header.frame_id = self.target_frame
                    delete_line_marker.ns = "realtime_line_cleanup"
                    delete_line_marker.id = cleanup_id
                    delete_line_marker.action = Marker.DELETE
                    marker_array.markers.append(delete_line_marker)
        
        self.target_marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = RealtimeButtonTargetPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
