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
                ('model_path', '/home/ymz/Workspace/elevator_manipulation/model/result/AdamW/weights/best.pt')
            ]
        )
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # 订阅原始图像
        self.subscription = self.create_subscription(
            Image,
            '/raw/image',
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
        # YOLO 推理
        results = self.model(img)
        # 绘制检测框
        annotated_img = results[0].plot()
        # 发布标注图像
        annotated_msg = self.cv2_to_ros_img(annotated_img, msg.header)
        self.image_publisher.publish(annotated_msg)
        # 发布识别结果（文本）
        result_str = self.format_result(results)
        self.result_publisher.publish(String(data=result_str))

    def ros_img_to_cv2(self, msg):
        import numpy as np
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
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
        msg.data = img.tobytes()
        msg.step = msg.width * 3
        return msg

    def format_result(self, results):
        # 输出类别、置信度和坐标信息
        res = results[0]
        out = []
        for box in res.boxes:
            cls = res.names[int(box.cls)] if hasattr(res, 'names') else str(int(box.cls))
            conf = float(box.conf)
            
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

def main(args=None):
    rclpy.init(args=args)
    node = ButtonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()