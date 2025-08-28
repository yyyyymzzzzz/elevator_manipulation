import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ButtonDetector(Node):
    def __init__(self, name='button_detector'):
        super().__init__(name)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', '/home/ymz/Workspace/elevator_manipulation/model/result/AdamW/weights/best.pt'),
                ('enable_roi_crop', True),
                ('roi_x_ratio', 0.2), 
                ('roi_y_ratio', 0.1), 
                ('roi_width_ratio', 0.6),  
                ('roi_height_ratio', 0.8), 
                ('enable_resize', True),
                ('resize_scale', 2.0), 
                ('confidence_threshold', 0.5)  
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
        
        self.get_logger().info(f'Loading model from: {model_path}')
        self.get_logger().info(f'ROI crop enabled: {self.enable_roi_crop}')
        self.get_logger().info(f'Resize enabled: {self.enable_resize}, scale: {self.resize_scale}')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # 订阅原始图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # 发布识别结果图像
        self.image_publisher = self.create_publisher(Image, '/detector/image', 10)
        # 发布识别结果（文本）
        self.result_publisher = self.create_publisher(String, '/detector/result', 10)

    def image_callback(self, msg):
        # ROS Image -> OpenCV
        import numpy as np
        img = self.ros_img_to_cv2(msg)
        
        # 保存原始图像尺寸用于坐标转换
        self.original_height, self.original_width = img.shape[:2]
        
        # 图像预处理
        processed_img, self.roi_offset, self.scale_factor = self.preprocess_image(img)
        
        # YOLO 推理
        results = self.model(processed_img)
        
        # 绘制检测框并筛选高置信度结果
        annotated_img = self.draw_filtered_results(results[0], processed_img)
        
        # 将结果坐标转换回原始图像坐标系
        original_results = self.transform_results_to_original(results[0])
        
        # 在原始图像上绘制结果
        original_annotated = self.draw_results_on_original(img, original_results)
        
        # 发布标注图像
        annotated_msg = self.cv2_to_ros_img(original_annotated, msg.header)
        self.image_publisher.publish(annotated_msg)
        
        # 发布识别结果（文本）- 使用原始坐标
        result_str = self.format_result_original_coords(original_results)
        self.result_publisher.publish(String(data=result_str))

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