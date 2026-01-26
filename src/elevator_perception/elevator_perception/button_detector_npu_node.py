#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import json
import os
import time
import numpy as np
from datetime import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# 导入 RKNN Lite
from rknnlite.api import RKNNLite

class NpuButtonDetector(Node):
    def __init__(self, name='button_detector'):
        super().__init__(name)
        
        # 需要与训练时 dataset.yaml 一致，请在 yaml 文件中设置
        # self.CLASSES = ['button_6', 'button_5', 'button_4', 'button_3', 'button_2', 'button_1', 'button_b1', 'button_open', 'button_close', 'button_alarm', 'display_1', 'display_2', 'display_3', 'display_4', 'display_5', 'display_6', 'button_up', 'button_down'] 
        # self.CLASSES = ['6', '5', '4', '3', '2', '1', 'B1', 'open', 'close', 'up', 'down']

        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', '/home/cat/Workspace/elevator_manipulation/model/best.rknn'), # 指向 .rknn 文件
                ('class_names', ['button_6', 'button_5', 'button_4', 'button_3', 'button_2', 'button_1', 'button_b1', 'button_open', 'button_close', 'button_alarm', 'display_1', 'display_2', 'display_3', 'display_4', 'display_5', 'display_6', 'button_up', 'button_down']),
                ('confidence_threshold', 0.5),
                ('nms_threshold', 0.45),      # NMS 阈值
                ('enable_recording', False),
                ('recording_save_path', '/home/cat/Workspace/elevator_manipulation/data/dataset'),
                ('recording_fps', 5.0)
            ]
        )
        
        # 获取参数
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.CLASSES = self.get_parameter('class_names').get_parameter_value().string_array_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.nms_thres = self.get_parameter('nms_threshold').get_parameter_value().double_value
        
        # 定义类别颜色 (随机生成或固定)，用于可视化区分
        np.random.seed(42)
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        # 内录参数
        self.enable_recording = self.get_parameter('enable_recording').get_parameter_value().bool_value
        self.recording_save_path = self.get_parameter('recording_save_path').get_parameter_value().string_value
        self.recording_fps = self.get_parameter('recording_fps').get_parameter_value().double_value

        self.get_logger().info(f'Loading NPU Model from: {model_path}')

        # --- NPU 初始化 ---
        self.rknn_lite = RKNNLite(verbose=False)
        
        # 1. 加载模型
        ret = self.rknn_lite.load_rknn(model_path)
        if ret != 0:
            self.get_logger().error('RKNN Load failed!')
            exit(ret)
            
        # 2. 初始化运行时 (使用 NPU Core 0)
        ret = self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            self.get_logger().error('RKNN Init Runtime failed!')
            exit(ret)
        
        self.get_logger().info('RKNN Runtime initialized. Ready to run on NPU.')
        self.get_logger().info(f'Configured Classes ({len(self.CLASSES)}): {self.CLASSES}')
        
        # 模型固定输入尺寸
        self.IMG_SIZE = (640, 640)

        # 内录初始化
        self.run_session_path = ""
        self.last_capture_time = 0.0
        if self.enable_recording:
            session_name = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.run_session_path = os.path.join(self.recording_save_path, session_name)
            os.makedirs(self.run_session_path, exist_ok=True)
            self.get_logger().info(f"Recording enabled: {self.run_session_path}")

        # AGV 状态控制变量
        self.agv_is_moving = False
        self.agv_stop_time = None
        self.detection_delay_after_stop = 1.0

        # ROS 订阅与发布
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        
        self.robot_status_subscriber = self.create_subscription(
            String, '/decision_maker/status', self.robot_status_callback, 10)

        self.image_publisher = self.create_publisher(Image, '/detector/image', 10)
        self.result_publisher = self.create_publisher(String, '/detector/result', 10)

        self.bridge = CvBridge()

    def robot_status_callback(self, msg):
        """处理 AGV 移动状态"""
        try:
            data = json.loads(msg.data)
            is_moving = data.get('is_moving', False)

            if is_moving:
                if not self.agv_is_moving:
                    self.get_logger().info("AGV moving -> Pausing detection")
                self.agv_is_moving = True
                self.agv_stop_time = None
                self.result_publisher.publish(String(data='No detections'))
            else:
                if self.agv_is_moving:
                    self.get_logger().info(f"AGV stopped -> Resuming in {self.detection_delay_after_stop}s")
                    self.agv_stop_time = self.get_clock().now()
                    self._first_detection_after_stop = True # 标记首次恢复
                self.agv_is_moving = False
        except json.JSONDecodeError:
            pass

    def image_callback(self, msg):
        # 1. 内录
        if self.enable_recording:
            self.handle_recording(msg)

        # 2. 状态检查
        if self.agv_is_moving:
            self.publish_empty_result()
            return

        if self.agv_stop_time is not None:
            time_since_stop = (self.get_clock().now() - self.agv_stop_time).nanoseconds / 1e9
            if time_since_stop < self.detection_delay_after_stop:
                return
            else:
                # 首次恢复识别日志
                if hasattr(self, '_first_detection_after_stop') and self._first_detection_after_stop:
                    self.get_logger().info("Resuming button detection")
                    self._first_detection_after_stop = False

        # 3. 图像转换 ROS -> CV2
        cv_image = self.ros_img_to_cv2(msg)
        if cv_image is None: return

        # 记录原始分辨率
        self.orig_h, self.orig_w = cv_image.shape[:2]

        # 4. 预处理 (Pre-process) - 使用 Letterbox 保持长宽比
        # NPU 需要 RGB 格式，且尺寸必须严格匹配 640x640
        input_img, self.ratio, self.pad = self.letterbox(cv_image, new_shape=self.IMG_SIZE)
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        input_img = np.expand_dims(input_img, 0) # 增加 Batch 维度: (1, 640, 640, 3)

        # 5. 推理 (Inference)
        # outputs 是 list，YOLOv8 只有一个输出节点
        outputs = self.rknn_lite.inference(inputs=[input_img])

        # 6. 后处理 (Post-process)
        # 解析 NPU 的原始输出矩阵
        boxes, classes, scores = self.post_process(outputs[0])

        # 7. 绘图与发布
        if boxes is not None:
            # 绘制结果
            annotated_img = self.draw_detections(cv_image, boxes, scores, classes)
            # 生成 JSON 字符串
            result_str = self.format_json(boxes, scores, classes)
        else:
            annotated_img = cv_image
            result_str = 'No detections'

        # 发布图像
        annotated_msg = self.cv2_to_ros_img(annotated_img, msg.header)
        if annotated_msg:
            self.image_publisher.publish(annotated_msg)
        
        # 发布结果文本
        self.result_publisher.publish(String(data=result_str))

    def post_process(self, input_data):
        """
        解析 YOLOv8 输出。
        输入 shape 通常为: (1, 4 + num_classes, 8400)
        """
        # 1. 维度处理
        input_data = np.squeeze(input_data) # 去掉 batch -> (4+nc, 8400)
        input_data = input_data.transpose() # 转置 -> (8400, 4+nc)

        # 2. 分割 坐标 和 分数
        # 前4列是 cx, cy, w, h
        boxes = input_data[:, :4] 
        # 后面的列是各类别分数
        scores = input_data[:, 4:] 

        # 3. 获取最高分数的类别
        class_ids = np.argmax(scores, axis=1)
        max_scores = np.max(scores, axis=1) # 获取最大值

        # 4. 阈值过滤
        mask = max_scores > self.conf_thres
        boxes = boxes[mask]
        max_scores = max_scores[mask]
        class_ids = class_ids[mask]

        if len(boxes) == 0:
            return None, None, None

        # 5. 坐标转换: (cx, cy, w, h) -> (x1, y1, x2, y2)
        # 注意：这里还是 640x640 (Letterboxed) 尺度
        boxes_xyxy = np.zeros_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

        # 6. NMS (非极大值抑制)
        # OpenCV 的 NMSBoxes 需要 xywh 格式，我们构建一个临时变量
        nms_boxes_xywh = boxes.copy()
        nms_boxes_xywh[:, 0] = boxes_xyxy[:, 0] # x1
        nms_boxes_xywh[:, 1] = boxes_xyxy[:, 1] # y1
        # w, h 保持不变
        
        indices = cv2.dnn.NMSBoxes(
            bboxes=nms_boxes_xywh.tolist(),
            scores=max_scores.tolist(),
            score_threshold=self.conf_thres,
            nms_threshold=self.nms_thres
        )

        if len(indices) > 0:
            indices = indices.flatten()
            final_boxes = boxes_xyxy[indices]
            final_scores = max_scores[indices]
            final_classes = class_ids[indices]

            # 7. 映射回原图尺寸 (Letterbox 逆变换)
            # 先减去 padding，再除以 ratio
            final_boxes[:, 0] = (final_boxes[:, 0] - self.pad[0]) / self.ratio[0]
            final_boxes[:, 2] = (final_boxes[:, 2] - self.pad[0]) / self.ratio[0]
            final_boxes[:, 1] = (final_boxes[:, 1] - self.pad[1]) / self.ratio[1]
            final_boxes[:, 3] = (final_boxes[:, 3] - self.pad[1]) / self.ratio[1]

            # 边界截断，防止超出原图
            final_boxes[:, 0] = np.clip(final_boxes[:, 0], 0, self.orig_w)
            final_boxes[:, 2] = np.clip(final_boxes[:, 2], 0, self.orig_w)
            final_boxes[:, 1] = np.clip(final_boxes[:, 1], 0, self.orig_h)
            final_boxes[:, 3] = np.clip(final_boxes[:, 3], 0, self.orig_h)

            return final_boxes, final_classes, final_scores
        else:
            return None, None, None

    def draw_detections(self, img, boxes, scores, classes):
        img_copy = img.copy()
        for box, score, cls_id in zip(boxes, scores, classes):
            x1, y1, x2, y2 = map(int, box)
            
            # 边界保护
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(self.orig_w, x2), min(self.orig_h, y2)

            class_name = self.CLASSES[cls_id] if cls_id < len(self.CLASSES) else str(cls_id)
            label = f"{class_name} {score:.2f}"
            
            # 获取该类别的颜色
            color = self.COLORS[cls_id % len(self.COLORS)]

            # 绘图
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), color, 2)
            t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
            cv2.rectangle(img_copy, (x1, y1), (x1+t_size[0], y1-t_size[1]-3), color, -1)
            cv2.putText(img_copy, label, (x1, y1-2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        return img_copy

    def letterbox(self, im, new_shape=(640, 640), color=(114, 114, 114)):
        """
        Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
        """
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        return im, ratio, (dw, dh)

    def format_json(self, boxes, scores, classes):
        out = []
        for box, score, cls_id in zip(boxes, scores, classes):
            x1, y1, x2, y2 = box
            class_name = self.CLASSES[cls_id] if cls_id < len(self.CLASSES) else str(cls_id)
            
            # 计算中心点和宽高
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            w = x2 - x1
            h = y2 - y1
            
            line = f'{class_name}: {score:.2f}, bbox:[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}], center:[{cx:.1f},{cy:.1f}], size:[{w:.1f}x{h:.1f}]'
            out.append(line)
        return '\n'.join(out)

    def publish_empty_result(self):
        empty_result = String(data=json.dumps({'detections': []}))
        self.result_publisher.publish(empty_result)

    def handle_recording(self, msg):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        if (current_time_sec - self.last_capture_time) >= (1.0 / self.recording_fps):
            img = self.ros_img_to_cv2(msg)
            if img is not None:
                fname = os.path.join(self.run_session_path, f"frame_{int(current_time_sec*1e9)}.png")
                cv2.imwrite(fname, img)
                self.last_capture_time = current_time_sec

    def ros_img_to_cv2(self, ros_img):
        try: return self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return None

    def cv2_to_ros_img(self, cv2_img, header):
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv2_img, "bgr8")
            ros_img.header = header
            return ros_img
        except CvBridgeError: return None

    def destroy_node(self):
        self.rknn_lite.release() # 释放 NPU
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NpuButtonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()