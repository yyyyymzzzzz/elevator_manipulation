import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import torch
import json
import os
import time
from datetime import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ButtonDetector(Node):
    def __init__(self, name='button_detector'):
        super().__init__(name)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', '/home/nvidia/Workspace/elevator_manipulation/model/result/AdamW/weights/best.pt'),
                ('enable_roi_crop', True),  # 启用ROI裁剪，但用于多窗口检测
                ('roi_x_ratio', 0.2), 
                ('roi_y_ratio', 0.1), 
                ('roi_width_ratio', 0.6),  
                ('roi_height_ratio', 0.8), 
                ('enable_resize', True),
                ('resize_scale', 2.0), 
                ('confidence_threshold', 0.5),
                ('enable_multi_window', True),  # 启用多窗口检测
                ('window_overlap_ratio', 0.5),  # 增加窗口重叠比例，确保边缘覆盖
                ('enable_recording', False), # 控制是否启用内录功能
                ('recording_save_path', '/home/nvidia/Workspace/elevator_manipulation/data/dataset'), # 内录图像保存路径
                ('recording_fps', 5.0) # 内录采集帧率
            ]
        )
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        # 获取参数
        self.enable_roi_crop = self.get_parameter('enable_roi_crop').get_parameter_value().bool_value
        self.roi_x_ratio = self.get_parameter('roi_x_ratio').get_parameter_value().double_value
        self.roi_y_ratio = self.get_parameter('roi_y_ratio').get_parameter_value().double_value
        self.roi_width_ratio = self.get_parameter('roi_width_ratio').get_parameter_value().double_value
        self.roi_height_ratio = self.get_parameter('roi_height_ratio').get_parameter_value().double_value
        self.enable_resize = self.get_parameter('enable_resize').get_parameter_value().bool_value
        self.resize_scale = self.get_parameter('resize_scale').get_parameter_value().double_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.enable_multi_window = self.get_parameter('enable_multi_window').get_parameter_value().bool_value
        self.window_overlap_ratio = self.get_parameter('window_overlap_ratio').get_parameter_value().double_value
        
        # 内录功能参数
        self.enable_recording = self.get_parameter('enable_recording').get_parameter_value().bool_value
        self.recording_save_path = self.get_parameter('recording_save_path').get_parameter_value().string_value
        self.recording_fps = self.get_parameter('recording_fps').get_parameter_value().double_value

        # 内录状态变量
        self.run_session_path = ""
        self.last_capture_time = 0.0

        self.get_logger().info(f'Loading model from: {model_path}')
        self.get_logger().info(f'ROI crop enabled: {self.enable_roi_crop}')
        self.get_logger().info(f'Multi-window detection: {self.enable_multi_window}')
        self.get_logger().info(f'Window overlap ratio: {self.window_overlap_ratio}')
        self.get_logger().info(f'Resize enabled: {self.enable_resize}, scale: {self.resize_scale}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        
        if self.enable_recording:
            session_name = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.run_session_path = os.path.join(self.recording_save_path, session_name)
            os.makedirs(self.run_session_path, exist_ok=True)
            self.get_logger().info(f"Data recording is enabled. Saving to: {self.run_session_path}")
            self.last_capture_time = self.get_clock().now().nanoseconds / 1e9

        # 初始化检测历史用于置信度稳定化
        self.detection_history = []
        self.max_history_length = 5  # 保持最近5帧的检测历史
        
        # AGV状态相关
        self.agv_is_moving = False
        self.agv_stop_time = None
        self.detection_delay_after_stop = 1.0  # AGV停止后等待2秒再开始识别
        
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # 订阅原始图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # 订阅当前机器人状态
        self.robot_status_subscriber = self.create_subscription(
            String,
            '/decision_maker/status',
            self.robot_status_callback,
            10
        )

        # 发布识别结果图像
        self.image_publisher = self.create_publisher(Image, '/detector/image', 10)
        # 发布识别结果（文本）
        self.result_publisher = self.create_publisher(String, '/detector/result', 10)

        # 初始化CvBridge
        self.bridge = CvBridge()

    def robot_status_callback(self, msg):
        """接收机器人状态并清理历史数据"""
        try:
            data = json.loads(msg.data)
            is_moving = data.get('is_moving', False)

            if is_moving:
                if not self.agv_is_moving:
                    self.get_logger().info("AGV开始移动，暂停按钮识别并清理历史数据")
                self.agv_is_moving = True
                self.agv_stop_time = None
                # 清理检测历史
                self.detection_history.clear()
                # 发布空检测结果
                empty_result = String(data='No detections')
                self.result_publisher.publish(empty_result)
            else:
                if self.agv_is_moving:
                    self.get_logger().info(f"AGV停止移动，{self.detection_delay_after_stop}秒后恢复按钮识别")
                    self.agv_stop_time = self.get_clock().now()
                    self._first_detection_after_stop = True  # 标记需要在恢复时打印日志
                self.agv_is_moving = False
                
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid AGV motion status message")

    def image_callback(self, msg):
        # 内录功能
        if self.enable_recording:
            self.handle_recording(msg)

        # 检查AGV状态，如果正在移动则跳过识别
        if self.agv_is_moving:
            # 在移动时也发布空结果，确保下游节点知道没有检测目标
            empty_result = String(data=json.dumps({'detections': []}))
            self.result_publisher.publish(empty_result)
            return
            
        # 如果AGV刚停止，检查是否已经等待足够时间
        if self.agv_stop_time is not None:
            current_time = self.get_clock().now()
            time_since_stop = (current_time - self.agv_stop_time).nanoseconds / 1e9
            if time_since_stop < self.detection_delay_after_stop:
                return  # 还未到识别时间
            else:
                # 首次恢复识别时的日志
                if hasattr(self, '_first_detection_after_stop') and self._first_detection_after_stop:
                    self.get_logger().info("恢复按钮识别")
                    self._first_detection_after_stop = False
                elif not hasattr(self, '_first_detection_after_stop'):
                    self._first_detection_after_stop = False
        
        # ROS Image -> OpenCV
        img = self.ros_img_to_cv2(msg)
        if img is None:
            self.get_logger().warn("图像转换失败，跳过当前帧")
            return
        
        # --- 核心修改：直接对完整图像进行推理 ---
        # YOLOv8模型会自动处理缩放、填充和归一化，与训练时保持一致
        results = self.model(img, verbose=False)
        
        # 从返回的列表中获取第一个结果对象
        yolo_result = results[0]

        # --- 简化结果绘制和发布 ---
        # 使用YOLOv8内置的plot()功能在原始图像上绘制所有结果
        original_annotated = yolo_result.plot()
        
        # 发布标注后的图像
        annotated_msg = self.cv2_to_ros_img(original_annotated, msg.header)
        if annotated_msg:
            self.image_publisher.publish(annotated_msg)
        
        # 将检测结果格式化为JSON字符串并发布
        result_str = self.format_results_to_json(yolo_result)
        self.result_publisher.publish(String(data=result_str))


    def handle_recording(self, msg):
        """处理图像采集逻辑"""
        current_time_ns = self.get_clock().now().nanoseconds
        current_time_sec = current_time_ns / 1e9
        
        # 检查是否达到了采集帧率要求的时间间隔
        if (current_time_sec - self.last_capture_time) >= (1.0 / self.recording_fps):
            try:
                # 使用相同的ros_img_to_cv2函数确保保存的图像与推理的图像来源一致
                img_to_save = self.ros_img_to_cv2(msg)
                if img_to_save is not None:
                    filename = os.path.join(self.run_session_path, f"frame_{current_time_ns}.png")
                    cv2.imwrite(filename, img_to_save)
                    self.last_capture_time = current_time_sec # 更新上次采集时间
                    self.get_logger().debug(f"Saved image to {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save recording image: {e}")

    def format_results_to_json(self, result):
        out = []
        
        if result.boxes is not None:
            for box in result.boxes:
                conf = float(box.conf[0])
                
                # 使用可配置的置信度阈值
                if conf <= self.confidence_threshold:
                    continue
                
                # 获取类别名称
                cls_id = int(box.cls[0])
                class_name = result.names[cls_id]
                
                # 获取边界框坐标 (V2使用浮点数)
                # 注意：我们直接从YOLO结果中获取，V1的推理逻辑未变
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                
                # 计算中心点坐标 (V2格式)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # 计算宽度和高度 (V2格式)
                width = x2 - x1
                height = y2 - y1
                
                # 格式化输出 (V2格式)
                result_line = f'{class_name}: {conf:.2f}, bbox:[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}], center:[{center_x:.1f},{center_y:.1f}], size:[{width:.1f}x{height:.1f}]'
                out.append(result_line)
        
        # 返回V2的多行纯文本格式，如果列表为空则返回 'No detections'
        return '\n'.join(out) if out else 'No detections'

    def ros_img_to_cv2(self, ros_img):
        """使用cv_bridge将ROS Image消息转换为OpenCV图像"""
        try:
            # 将ROS Image消息转换为OpenCV图像（BGR格式）
            return self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return None

    def cv2_to_ros_img(self, cv2_img, header):
        """使用cv_bridge将OpenCV图像转换为ROS Image消息"""
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv2_img, "bgr8")
            ros_img.header = header
            return ros_img
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ButtonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()