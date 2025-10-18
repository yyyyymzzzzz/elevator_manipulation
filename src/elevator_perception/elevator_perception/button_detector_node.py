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
        import numpy as np
        img = self.ros_img_to_cv2(msg)
        
        # 保存原始图像尺寸用于坐标转换
        self.original_height, self.original_width = img.shape[:2]
        
        # 如果启用多窗口检测
        if self.enable_multi_window:
            all_results = self.multi_window_detection(img)
        else:
            # 原始单窗口检测
            processed_img, self.roi_offset, self.scale_factor = self.preprocess_image(img)
            results = self.model(processed_img, verbose=False)
            all_results = self.transform_results_to_original(results[0])
        
        # 在原始图像上绘制所有结果
        original_annotated = self.draw_all_results_on_original(img, all_results)
        
        # 发布标注图像
        annotated_msg = self.cv2_to_ros_img(original_annotated, msg.header)
        self.image_publisher.publish(annotated_msg)
        
        # 发布识别结果（文本）- 使用原始坐标
        result_str = self.format_all_results_original_coords(all_results)
        self.result_publisher.publish(String(data=result_str))

    def handle_recording(self, msg):
        """处理图像采集逻辑"""
        current_time_ns = self.get_clock().now().nanoseconds
        current_time_sec = current_time_ns / 1e9
        
        # 检查是否达到了采集帧率要求的时间间隔
        if (current_time_sec - self.last_capture_time) >= (1.0 / self.recording_fps):
            try:
                img_to_save = self.ros_img_to_cv2(msg)
                filename = os.path.join(self.run_session_path, f"frame_{current_time_ns}.png")
                cv2.imwrite(filename, img_to_save)
                self.last_capture_time = current_time_sec # 更新上次采集时间
                self.get_logger().debug(f"Saved image to {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save recording image: {e}")

    def draw_filtered_results(self, result, img):
        """绘制置信度高于阈值的检测结果"""
        import numpy as np
        annotated_img = img.copy()
        
        if result.boxes is not None:
            for box in result.boxes:
                conf = float(box.conf)
                # 使用可配置的置信度阈值
                if conf > self.confidence_threshold:
                    cls = result.names[int(box.cls)] if hasattr(result, 'names') else str(int(box.cls))
                    
                    # 获取边界框坐标
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    
                    # 绘制边界框 (BGR格式: 绿色)
                    cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # 绘制标签和置信度
                    label = f'{cls}: {conf:.2f}'
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    # 标签背景 (BGR格式: 绿色)
                    cv2.rectangle(annotated_img, (x1, y1 - label_size[1] - 10), 
                                (x1 + label_size[0], y1), (0, 255, 0), -1)
                    # 标签文字 (BGR格式: 黑色)
                    cv2.putText(annotated_img, label, (x1, y1 - 5), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return annotated_img

    def ros_img_to_cv2(self, msg):
        import numpy as np
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            # 将RGB转换为BGR (OpenCV格式)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        else:
            raise ValueError(f'Unsupported encoding: {msg.encoding}')
        return img

    def cv2_to_ros_img(self, img, header):
        msg = Image()
        msg.header = header
        msg.height, msg.width = img.shape[:2]
        msg.encoding = 'bgr8'
        # 确保图像是BGR格式（OpenCV默认格式）
        if len(img.shape) == 3 and img.shape[2] == 3:
            msg.data = img.tobytes()
        else:
            raise ValueError(f'Unsupported image shape: {img.shape}')
        msg.step = msg.width * 3
        return msg

    def format_result(self, results):
        # 输出类别、置信度和坐标信息
        res = results[0]
        out = []
        for box in res.boxes:
            cls = res.names[int(box.cls)] if hasattr(res, 'names') else str(int(box.cls))
            conf = float(box.conf)
            
            # 使用可配置的置信度阈值
            if conf <= self.confidence_threshold:
                continue
            
            # 获取边界框坐标 (x1, y1, x2, y2)
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            
            # 计算中心点坐标
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # 计算宽度和高度
            width = x2 - x1
            height = y2 - y1
            
            # 格式化输出
            result_line = f'{cls}: {conf:.2f}, bbox:[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}], center:[{center_x:.1f},{center_y:.1f}], size:[{width:.1f}x{height:.1f}]'
            out.append(result_line)
        
        return '\n'.join(out) if out else 'No detections'
    
    def preprocess_image(self, img):
        """图像预处理：裁剪ROI和缩放"""
        import numpy as np
        
        roi_offset = (0, 0)
        scale_factor = 1.0
        processed_img = img.copy()
        
        # ROI裁剪
        if self.enable_roi_crop:
            h, w = img.shape[:2]
            x1 = int(w * self.roi_x_ratio)
            y1 = int(h * self.roi_y_ratio)
            x2 = int(x1 + w * self.roi_width_ratio)
            y2 = int(y1 + h * self.roi_height_ratio)
            
            # 确保坐标在有效范围内
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            
            processed_img = img[y1:y2, x1:x2]
            roi_offset = (x1, y1)
            
            self.get_logger().debug(f'ROI crop: ({x1},{y1}) to ({x2},{y2})')
        
        # 图像缩放
        if self.enable_resize and self.resize_scale != 1.0:
            h, w = processed_img.shape[:2]
            new_w = int(w * self.resize_scale)
            new_h = int(h * self.resize_scale)
            processed_img = cv2.resize(processed_img, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
            scale_factor = self.resize_scale
            
            self.get_logger().debug(f'Resize: {w}x{h} -> {new_w}x{new_h}')
        
        return processed_img, roi_offset, scale_factor
    
    def transform_results_to_original(self, result):
        """将检测结果坐标转换回原始图像坐标系"""
        import numpy as np
        
        if result.boxes is None:
            return result
        
        # 创建转换后的坐标列表
        self.transformed_boxes = []
        
        for i, box in enumerate(result.boxes):
            conf = float(box.conf)
            if conf <= self.confidence_threshold:
                continue
                
            # 获取处理后图像中的坐标
            xyxy = box.xyxy[0].cpu().numpy().copy()
            
            # 反向缩放
            if self.enable_resize and self.resize_scale != 1.0:
                xyxy = xyxy / self.resize_scale
            
            # 反向ROI偏移
            if self.enable_roi_crop:
                xyxy[0] += self.roi_offset[0]  # x1
                xyxy[1] += self.roi_offset[1]  # y1
                xyxy[2] += self.roi_offset[0]  # x2
                xyxy[3] += self.roi_offset[1]  # y2
            
            # 保存转换后的坐标和其他信息
            self.transformed_boxes.append({
                'xyxy': xyxy,
                'conf': conf,
                'cls': int(box.cls),
                'names': result.names if hasattr(result, 'names') else None
            })
        
        return result
    
    def draw_results_on_original(self, img, result):
        """在原始图像上绘制检测结果"""
        import numpy as np
        annotated_img = img.copy()
        
        # 使用转换后的坐标
        if hasattr(self, 'transformed_boxes') and self.transformed_boxes:
            for box_info in self.transformed_boxes:
                conf = box_info['conf']
                if conf > self.confidence_threshold:
                    cls = box_info['names'][box_info['cls']] if box_info['names'] else str(box_info['cls'])
                    
                    # 获取边界框坐标
                    xyxy = box_info['xyxy']
                    x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    
                    # 绘制边界框 (BGR格式: 绿色)
                    cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # 绘制标签和置信度
                    label = f'{cls}: {conf:.2f}'
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    # 标签背景 (BGR格式: 绿色)
                    cv2.rectangle(annotated_img, (x1, y1 - label_size[1] - 10), 
                                (x1 + label_size[0], y1), (0, 255, 0), -1)
                    # 标签文字 (BGR格式: 黑色)
                    cv2.putText(annotated_img, label, (x1, y1 - 5), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        if self.enable_roi_crop:
            h, w = self.original_height, self.original_width
            x1 = int(w * self.roi_x_ratio)
            y1 = int(h * self.roi_y_ratio)
            x2 = int(x1 + w * self.roi_width_ratio)
            y2 = int(y1 + h * self.roi_height_ratio)
            
            # 绘制ROI框 (BGR格式: 蓝色虚线)
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(annotated_img, 'ROI', (x1, y1-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        return annotated_img
    
    def multi_window_detection(self, img):
        """多窗口滑动检测，覆盖整个画面"""
        import numpy as np
        
        h, w = img.shape[:2]
        
        # 计算窗口尺寸（保持与原始ROI相同的尺寸）
        window_w = int(w * self.roi_width_ratio)
        window_h = int(h * self.roi_height_ratio)
        
        # 计算步长（考虑重叠）
        step_w = int(window_w * (1 - self.window_overlap_ratio))
        step_h = int(window_h * (1 - self.window_overlap_ratio))
        
        # 确保步长不为0
        step_w = max(step_w, 1)
        step_h = max(step_h, 1)
        
        all_detections = []
        
        # 计算网格位置，确保完全覆盖
        y_positions = []
        x_positions = []
        
        # Y方向的位置
        y = 0
        while y < h:
            y_positions.append(y)
            if y + window_h >= h:  # 如果下一个窗口会超出边界，添加最后一个位置
                if y + window_h != h and h - window_h >= 0:
                    y_positions.append(h - window_h)
                break
            y += step_h
        
        # X方向的位置
        x = 0
        while x < w:
            x_positions.append(x)
            if x + window_w >= w:  # 如果下一个窗口会超出边界，添加最后一个位置
                if x + window_w != w and w - window_w >= 0:
                    x_positions.append(w - window_w)
                break
            x += step_w
        
        # 去重并排序
        y_positions = sorted(list(set(y_positions)))
        x_positions = sorted(list(set(x_positions)))
        
        self.get_logger().debug(f'Window positions - Y: {y_positions}, X: {x_positions}')
        
        # 滑动窗口遍历
        for y_start in y_positions:
            for x_start in x_positions:
                # 确保窗口不超出边界
                y_end = min(y_start + window_h, h)
                x_end = min(x_start + window_w, w)
                
                # 如果窗口太小，跳过
                if (y_end - y_start) < window_h * 0.5 or (x_end - x_start) < window_w * 0.5:
                    continue
                
                # 裁剪当前窗口
                window_img = img[y_start:y_end, x_start:x_end]
                
                # 如果窗口尺寸不匹配，进行填充到标准尺寸
                if window_img.shape[:2] != (window_h, window_w):
                    # 创建标准尺寸的黑色背景
                    padded_window = np.zeros((window_h, window_w, 3), dtype=np.uint8)
                    # 将实际窗口放在左上角
                    actual_h, actual_w = window_img.shape[:2]
                    padded_window[:actual_h, :actual_w] = window_img
                    window_img = padded_window
                
                # 对当前窗口进行缩放
                if self.enable_resize and self.resize_scale != 1.0:
                    new_h = int(window_h * self.resize_scale)
                    new_w = int(window_w * self.resize_scale)
                    processed_window = cv2.resize(window_img, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
                else:
                    processed_window = window_img.copy()
                
                # YOLO 推理
                results = self.model(processed_window, verbose=False)
                
                # 转换检测结果坐标到原始图像坐标系
                if results[0].boxes is not None:
                    for box in results[0].boxes:
                        conf = float(box.conf)
                        if conf > self.confidence_threshold:
                            # 获取边界框坐标
                            xyxy = box.xyxy[0].cpu().numpy().copy()
                            
                            # 反向缩放
                            if self.enable_resize and self.resize_scale != 1.0:
                                xyxy = xyxy / self.resize_scale
                            
                            # 检查检测框是否在有效区域内（避免填充区域的误检）
                            if (xyxy[0] < x_end - x_start and xyxy[1] < y_end - y_start and
                                xyxy[2] < x_end - x_start and xyxy[3] < y_end - y_start):
                                
                                # 转换到原始图像坐标系
                                xyxy[0] += x_start  # x1
                                xyxy[1] += y_start  # y1
                                xyxy[2] += x_start  # x2
                                xyxy[3] += y_start  # y2
                                
                                # 保存检测结果，包含窗口信息用于调试
                                detection = {
                                    'xyxy': xyxy,
                                    'conf': conf,
                                    'cls': int(box.cls),
                                    'names': results[0].names if hasattr(results[0], 'names') else None,
                                    'window_pos': (x_start, y_start)  # 添加窗口位置信息
                                }
                                all_detections.append(detection)
        
        self.get_logger().debug(f'Total detections before NMS: {len(all_detections)}')
        
        # 应用非极大值抑制去除重复检测
        filtered_detections = self.apply_nms(all_detections, iou_threshold=0.3)  # 降低IoU阈值，减少误删
        
        # 步骤2：合并相邻的【相同类别】检测框
        merged_detections = self.merge_adjacent_boxes(filtered_detections)
        
        # 新增步骤3：对相邻的【不同类别】的模糊对进行重新识别
        re_detected_detections = self.re_detect_ambiguous_pairs(merged_detections, img, window_w, window_h)
        
        # 步骤4：应用置信度稳定化
        stabilized_detections = self.stabilize_detections(re_detected_detections)
        
        self.get_logger().debug(f'Total detections after NMS: {len(filtered_detections)}')
        self.get_logger().debug(f'Total detections after merging: {len(merged_detections)}')
        self.get_logger().debug(f'Total detections after re-detection: {len(re_detected_detections)}') 
        self.get_logger().debug(f'Total detections after stabilization: {len(stabilized_detections)}')
        
        return stabilized_detections
    
    def apply_nms(self, detections, iou_threshold=0.3):
        """应用非极大值抑制去除重复检测"""
        import numpy as np
        
        if not detections:
            return []
        
        # 按置信度排序（降序）
        detections = sorted(detections, key=lambda x: x['conf'], reverse=True)
        
        filtered = []
        
        while detections:
            # 取出置信度最高的检测
            best = detections.pop(0)
            filtered.append(best)
            
            # 计算与剩余检测的IoU，移除重叠度高的检测
            remaining = []
            for det in detections:
                iou = self.calculate_iou(best['xyxy'], det['xyxy'])
                
                # 如果IoU较高，说明是重复检测，选择置信度更高的
                if iou >= iou_threshold:
                    # 如果当前检测的置信度更高，替换best（这通常不会发生，因为已经按置信度排序）
                    if det['conf'] > best['conf']:
                        # 移除已添加的best，添加当前det
                        filtered.pop()
                        filtered.append(det)
                        # 将原best重新加入待处理列表
                        remaining.append(best)
                    # 否则丢弃当前det
                else:
                    # IoU低，保留这个检测
                    remaining.append(det)
            
            detections = remaining
        
        # 按置信度重新排序最终结果
        filtered = sorted(filtered, key=lambda x: x['conf'], reverse=True)
        
        return filtered

    def merge_adjacent_boxes(self, detections, max_gap_ratio=0.55, min_overlap_ratio=0.15):
        """
        合并相邻且属于同一类别的检测框。
        主要用于解决滑动窗口导致的单个目标被分割成多个部分的问题。
        """
        import numpy as np

        if len(detections) < 2:
            return detections

        while True:
            merged_something = False
            i = 0
            while i < len(detections):
                j = i + 1
                while j < len(detections):
                    det1 = detections[i]
                    det2 = detections[j]

                    # 检查是否可以合并
                    can_merge = False
                    if det1['cls'] == det2['cls']:
                        # 获取坐标和尺寸
                        xyxy1 = det1['xyxy']
                        xyxy2 = det2['xyxy']
                        x1_1, y1_1, x2_1, y2_1 = xyxy1
                        x1_2, y1_2, x2_2, y2_2 = xyxy2
                        w1, h1 = x2_1 - x1_1, y2_1 - y1_1
                        w2, h2 = x2_2 - x1_2, y2_2 - y1_2

                        # 计算水平间距和垂直重叠率
                        horizontal_gap = max(0, max(x1_1, x1_2) - min(x2_1, x2_2))
                        vertical_overlap = max(0, min(y2_1, y2_2) - max(y1_1, y1_2))
                        vertical_overlap_ratio = vertical_overlap / min(h1, h2) if min(h1, h2) > 0 else 0
                        
                        # 合并条件：垂直重叠度足够高，且水平间距足够小
                        if (vertical_overlap_ratio > min_overlap_ratio and 
                            horizontal_gap < min(w1, w2) * max_gap_ratio):
                            can_merge = True

                    if can_merge:
                        # 合并框
                        new_x1 = min(x1_1, x1_2)
                        new_y1 = min(y1_1, y1_2)
                        new_x2 = max(x2_1, x2_2)
                        new_y2 = max(y2_1, y2_2)
                        
                        # 创建新的合并后的检测结果
                        merged_detection = {
                            'xyxy': np.array([new_x1, new_y1, new_x2, new_y2]),
                            'conf': max(det1['conf'], det2['conf']),
                            'cls': det1['cls'],
                            'names': det1['names'],
                            'window_pos': det1.get('window_pos', (0,0))
                        }
                        
                        # 移除旧的框，添加新的合并框
                        detections.pop(j)
                        detections.pop(i)
                        detections.append(merged_detection)
                        
                        merged_something = True
                        break # 跳出内层 j 循环，重新开始扫描
                    
                    # *** BUG修复：确保 j 总是递增 ***
                    j += 1
                
                if merged_something:
                    break # 跳出外层 i 循环，从头开始
                
                i += 1
            
            if not merged_something:
                break

        return detections

    def re_detect_ambiguous_pairs(self, detections, original_image, window_w, window_h):
        """
        寻找相邻的不同类别检测框。
        对该区域创建一个与初次识别相同大小的窗口进行重新识别，以消除歧义。
        """
        import numpy as np

        if len(detections) < 2:
            return detections

        adjacency_iou_threshold = 0.05
        max_re_detections = 3  # 最大重新检测次数，防止无限循环
        re_detection_count = 0

        while re_detection_count < max_re_detections:
            re_detected_something = False
            i = 0
            while i < len(detections):
                j = i + 1
                while j < len(detections):
                    det1 = detections[i]
                    det2 = detections[j]

                    if det1['cls'] == det2['cls']:
                        j += 1
                        continue

                    xyxy1 = det1['xyxy']
                    xyxy2 = det2['xyxy']
                    
                    union_x1 = min(xyxy1[0], xyxy2[0])
                    union_y1 = min(xyxy1[1], xyxy2[1])
                    union_x2 = max(xyxy1[2], xyxy2[2])
                    union_y2 = max(xyxy1[3], xyxy2[3])
                    
                    area1 = (xyxy1[2] - xyxy1[0]) * (xyxy1[3] - xyxy1[1])
                    area2 = (xyxy2[2] - xyxy2[0]) * (xyxy2[3] - xyxy2[1])
                    union_area = (union_x2 - union_x1) * (union_y2 - union_y1)
                    
                    if union_area > 0 and (area1 + area2 - self.calculate_iou(xyxy1, xyxy2) * union_area) / union_area < (1 - adjacency_iou_threshold):
                        j += 1
                        continue

                    self.get_logger().info(f"Ambiguous pair found. Re-detecting with standard window size.")
                    
                    # 1. 计算标准尺寸ROI的新位置
                    h, w = original_image.shape[:2]
                    
                    # 计算两个模糊框合并后的中心点
                    center_x = (union_x1 + union_x2) / 2
                    center_y = (union_y1 + union_y2) / 2
                    
                    # 以该中心点为中心，计算标准尺寸窗口的坐标
                    roi_x1 = int(center_x - window_w / 2)
                    roi_y1 = int(center_y - window_h / 2)
                    roi_x2 = int(roi_x1 + window_w)
                    roi_y2 = int(roi_y1 + window_h)
                    
                    # 2. 修正ROI，确保它不超出画面边界
                    # 如果左边越界，将整个框向右移动
                    if roi_x1 < 0:
                        roi_x2 -= roi_x1  # roi_x2 = roi_x2 + (-roi_x1)
                        roi_x1 = 0
                    # 如果右边越界，将整个框向左移动
                    if roi_x2 > w:
                        roi_x1 -= (roi_x2 - w)
                        roi_x2 = w
                    # 如果上边越界，将整个框向下移动
                    if roi_y1 < 0:
                        roi_y2 -= roi_y1
                        roi_y1 = 0
                    # 如果下边越界，将整个框向上移动
                    if roi_y2 > h:
                        roi_y1 -= (roi_y2 - h)
                        roi_y2 = h

                    # 3. 裁剪图像并进行推理
                    roi_image = original_image[roi_y1:roi_y2, roi_x1:roi_x2]
                    
                    if roi_image.size == 0 or roi_image.shape[0] != window_h or roi_image.shape[1] != window_w:
                        self.get_logger().warn(f"Re-detection ROI is not standard size after boundary correction. Skipping.")
                        j += 1
                        continue

                    results = self.model(roi_image, verbose=False)
                    
                    # 4. 处理新结果并替换旧结果
                    new_detections = []
                    if results[0].boxes is not None:
                        for box in results[0].boxes:
                            conf = float(box.conf)
                            if conf > self.confidence_threshold:
                                xyxy_roi = box.xyxy[0].cpu().numpy().copy()
                                
                                xyxy_original = [
                                    xyxy_roi[0] + roi_x1,
                                    xyxy_roi[1] + roi_y1,
                                    xyxy_roi[2] + roi_x1,
                                    xyxy_roi[3] + roi_y1
                                ]
                                
                                new_detections.append({
                                    'xyxy': np.array(xyxy_original),
                                    'conf': conf,
                                    'cls': int(box.cls),
                                    'names': results[0].names
                                })

                    detections.pop(j)
                    detections.pop(i)
                    detections.extend(new_detections)
                    
                    re_detected_something = True
                    re_detection_count += 1  # 增加重新检测计数
                    self.get_logger().info(f"Re-detection completed. Count: {re_detection_count}/{max_re_detections}")
                    break
                
                if re_detected_something:
                    break
                
                i += 1
            
            if not re_detected_something:
                break
        
        if re_detection_count >= max_re_detections:
            self.get_logger().warn(f"Maximum re-detection limit ({max_re_detections}) reached. Stopping to prevent infinite loop.")

        return detections
    
    def stabilize_detections(self, current_detections):
        """使用历史检测结果稳定置信度"""
        import numpy as np
        
        # 添加当前检测到历史记录
        self.detection_history.append(current_detections)
        
        # 保持历史记录长度
        if len(self.detection_history) > self.max_history_length:
            self.detection_history.pop(0)
        
        # 如果历史记录不足，直接返回当前检测
        if len(self.detection_history) < 2:
            return current_detections
        
        stabilized = []
        
        for current_det in current_detections:
            # 寻找历史检测中的匹配项
            matching_history = []
            
            for hist_frame in self.detection_history[:-1]:  # 排除当前帧
                for hist_det in hist_frame:
                    # 检查是否为同一目标（位置相近且类别相同）
                    if (hist_det['cls'] == current_det['cls'] and 
                        self.calculate_iou(hist_det['xyxy'], current_det['xyxy']) > 0.3):
                        matching_history.append(hist_det['conf'])
            
            # 计算稳定化的置信度
            if matching_history:
                # 使用指数移动平均来平滑置信度
                alpha = 0.7  # 当前帧权重
                avg_hist_conf = np.mean(matching_history)
                stabilized_conf = alpha * current_det['conf'] + (1 - alpha) * avg_hist_conf
                
                # 创建稳定化的检测结果
                stabilized_det = current_det.copy()
                stabilized_det['conf'] = stabilized_conf
                stabilized.append(stabilized_det)
            else:
                # 如果没有历史匹配，直接使用当前检测
                stabilized.append(current_det)
        
        return stabilized
    
    def calculate_iou(self, box1, box2):
        """计算两个边界框的IoU"""
        import numpy as np
        
        # 计算交集
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        if x2 <= x1 or y2 <= y1:
            return 0.0
        
        intersection = (x2 - x1) * (y2 - y1)
        
        # 计算并集
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def draw_all_results_on_original(self, img, detections):
        """在原始图像上绘制所有检测结果"""
        import numpy as np
        annotated_img = img.copy()
        
        for detection in detections:
            conf = detection['conf']
            if conf > self.confidence_threshold:
                cls = detection['names'][detection['cls']] if detection['names'] else str(detection['cls'])
                
                # 获取边界框坐标
                xyxy = detection['xyxy']
                x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                
                # 绘制边界框 (BGR格式: 绿色)
                cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # 绘制标签和置信度
                label = f'{cls}: {conf:.2f}'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                # 标签背景 (BGR格式: 绿色)
                cv2.rectangle(annotated_img, (x1, y1 - label_size[1] - 10), 
                            (x1 + label_size[0], y1), (0, 255, 0), -1)
                # 标签文字 (BGR格式: 黑色)
                cv2.putText(annotated_img, label, (x1, y1 - 5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return annotated_img
    
    def format_all_results_original_coords(self, detections):
        """格式化所有检测结果"""
        out = []
        
        for detection in detections:
            cls = detection['names'][detection['cls']] if detection['names'] else str(detection['cls'])
            conf = detection['conf']
            
            # 筛选置信度高于阈值的结果
            if conf <= self.confidence_threshold:
                continue
            
            # 获取边界框坐标 (x1, y1, x2, y2)
            xyxy = detection['xyxy']
            x1, y1, x2, y2 = xyxy
            
            # 计算中心点坐标
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # 计算宽度和高度
            width = x2 - x1
            height = y2 - y1
            
            # 格式化输出
            result_line = f'{cls}: {conf:.2f}, bbox:[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}], center:[{center_x:.1f},{center_y:.1f}], size:[{width:.1f}x{height:.1f}]'
            out.append(result_line)
        
        return '\n'.join(out) if out else 'No detections'
    
    def format_result_original_coords(self, result):
        """格式化原始坐标系下的检测结果"""
        out = []
        
        # 使用转换后的坐标
        if hasattr(self, 'transformed_boxes') and self.transformed_boxes:
            for box_info in self.transformed_boxes:
                cls = box_info['names'][box_info['cls']] if box_info['names'] else str(box_info['cls'])
                conf = box_info['conf']
                
                # 筛选置信度高于阈值的结果
                if conf <= self.confidence_threshold:
                    continue
                
                # 获取边界框坐标 (x1, y1, x2, y2)
                xyxy = box_info['xyxy']
                x1, y1, x2, y2 = xyxy
                
                # 计算中心点坐标
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # 计算宽度和高度
                width = x2 - x1
                height = y2 - y1
                
                # 格式化输出
                result_line = f'{cls}: {conf:.2f}, bbox:[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}], center:[{center_x:.1f},{center_y:.1f}], size:[{width:.1f}x{height:.1f}]'
                out.append(result_line)
        
        return '\n'.join(out) if out else 'No detections'

def main(args=None):
    rclpy.init(args=args)
    node = ButtonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()