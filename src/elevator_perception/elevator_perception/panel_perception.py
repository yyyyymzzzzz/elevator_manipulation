#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from collections import deque
from datetime import datetime

class FloorDetectionNode(Node):
    def __init__(self):
        super().__init__('floor_detection_node')
        
        # 参数配置
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_class', 'display_2'),        # 要检测的目标类别
                ('detection_window', 2.0),           # 检测时间窗口（秒）
                ('detection_threshold', 2),          # 触发阈值（次数）
                ('cooldown_time', 3.0),              # 冷却时间（秒）
                ('enable_terminal_output', True),    # 终端输出开关
                ('confidence_filter', 0.3),          # 置信度过滤阈值
            ]
        )
        
        # 获取参数
        self.target_class = self.get_parameter('target_class').get_parameter_value().string_value
        self.detection_window = self.get_parameter('detection_window').get_parameter_value().double_value
        self.detection_threshold = self.get_parameter('detection_threshold').get_parameter_value().integer_value
        self.cooldown_time = self.get_parameter('cooldown_time').get_parameter_value().double_value
        self.enable_terminal_output = self.get_parameter('enable_terminal_output').get_parameter_value().bool_value
        self.confidence_filter = self.get_parameter('confidence_filter').get_parameter_value().double_value
        
        # 状态变量初始化
        self.detection_records = deque()  # 存储检测记录 (时间戳, 置信度)
        self.last_trigger_time = 0.0      # 上次触发时间
        self.is_triggered = False         # 是否已触发
        self.total_frames = 0             # 总处理帧数
        self.target_frames = 0            # 检测到目标的帧数
        self.start_time = time.time()     # 开始时间
        
        # 订阅ButtonDetector的检测结果
        self.detection_sub = self.create_subscription(
            String,
            '/detector/result',  # ButtonDetector发布的话题
            self.detection_callback,
            10
        )
        
        # 发布楼层状态
        self.status_pub = self.create_publisher(
            String,
            '/look_floor/completed',
            10
        )
        
        # 定时器：清理过期记录
        self.cleanup_timer = self.create_timer(0.5, self.cleanup_old_records)
        
        # 终端输出初始化
        if self.enable_terminal_output:
            self.print_init_info()
            
        self.get_logger().info(f"楼层检测节点已启动，监听 /detector/result 话题")
        self.get_logger().info(f"目标类别: {self.target_class}, 时间窗口: {self.detection_window}秒, 阈值: {self.detection_threshold}次")

    def print_init_info(self):
        """打印初始化信息"""
        print("\n" + "="*70)
        print("🎯 楼层检测节点")
        print("="*70)
        print(f"目标类别: {self.target_class}")
        print(f"检测窗口: {self.detection_window}秒")
        print(f"触发阈值: {self.detection_threshold}次")
        print(f"置信度过滤: >{self.confidence_filter}")
        print(f"冷却时间: {self.cooldown_time}秒")
        print("="*70 + "\n")
        print("等待检测数据...")

    def detection_callback(self, msg):
        """处理ButtonDetector的检测结果"""
        current_time = time.time()
        self.total_frames += 1
        
        # 检查是否在冷却期内
        if current_time - self.last_trigger_time < self.cooldown_time:
            return
        
        # 解析检测结果
        result_text = msg.data
        
        # 检查是否包含目标类别
        if self.target_class in result_text and result_text != 'No detections':
            # 解析置信度
            confidence = self.parse_confidence(result_text, self.target_class)
            
            # 置信度过滤
            if confidence < self.confidence_filter:
                return
                
            # 记录检测
            self.detection_records.append((current_time, confidence))
            self.target_frames += 1
            
            # 实时输出检测信息
            if self.enable_terminal_output:
                self.print_detection_info(current_time, confidence)
                
            # 检查是否达到触发条件
            self.check_trigger_condition(current_time)

    def parse_confidence(self, result_text, target_class):
        """从检测结果中解析置信度"""
        lines = result_text.strip().split('\n')
        
        for line in lines:
            if target_class in line:
                try:
                    # 解析格式: "display_2: 0.85, bbox:[...]"
                    parts = line.split(':')
                    if len(parts) >= 2:
                        conf_part = parts[1].split(',')[0].strip()
                        return float(conf_part)
                except (ValueError, IndexError):
                    return 0.0
        
        return 0.0

    def print_detection_info(self, current_time, confidence):
        """在终端输出检测信息"""
        # 清空当前行，重新输出
        print("\r", end="")
        
        # 计算统计信息
        detection_count = len(self.detection_records)
        time_elapsed = current_time - self.start_time
        
        # 计算检测频率
        if time_elapsed > 0:
            freq = self.target_frames / time_elapsed
        else:
            freq = 0.0
        
        # 输出实时信息
        time_str = datetime.fromtimestamp(current_time).strftime('%H:%M:%S.%f')[:-3]
        progress = min(detection_count / self.detection_threshold, 1.0)
        bar_length = 20
        filled = int(bar_length * progress)
        bar = '█' * filled + '░' * (bar_length - filled)
        
        print(f"[{time_str}] 检测到 {self.target_class} | "
              f"置信度: {confidence:.3f} | "
              f"窗口内次数: {detection_count}/{self.detection_threshold} | "
              f"[{bar}] | "
              f"频率: {freq:.1f} Hz", end="", flush=True)

    def check_trigger_condition(self, current_time):
        """检查是否满足触发条件"""
        # 先清理过期记录
        self.cleanup_old_records(current_time)
        
        # 检查检测次数
        detection_count = len(self.detection_records)
        
        if detection_count >= self.detection_threshold:
            self.trigger_floor_detection(current_time)

    def trigger_floor_detection(self, trigger_time):
        """触发楼层检测"""
        self.last_trigger_time = trigger_time
        self.is_triggered = True
        
        # 计算平均置信度
        confidences = [conf for _, conf in self.detection_records]
        avg_confidence = sum(confidences) / len(confidences) if confidences else 0.0
        
        # 创建状态消息 - 修改为TaskAssignment期望的格式
        status_data = {
            'status': 'completed',  # 重要：改为'completed'以匹配TaskAssignment的期望
            'identified_floor': '2',  # 改为'identified_floor'以匹配TaskAssignment的期望
            'target_floor': '2',     # 添加target_floor字段
            'detected_class': self.target_class,
            'detection_count': len(self.detection_records),
            'average_confidence': avg_confidence,
            'time_window': self.detection_window,
            'timestamp': trigger_time,
            'human_readable_time': datetime.fromtimestamp(trigger_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'message': f'到达2楼，检测到{self.target_class} {len(self.detection_records)}次'
        }
        
        # 发布状态
        status_msg = String(data=json.dumps(status_data, ensure_ascii=False))
        self.status_pub.publish(status_msg)
        
        # 终端输出成功信息
        if self.enable_terminal_output:
            self.print_success_message(trigger_time, avg_confidence)
        
        # ROS日志
        self.get_logger().info(f"✅ 到达2楼！检测到{self.target_class} {len(self.detection_records)}次，平均置信度: {avg_confidence:.3f}")
        
        # 清空检测记录（重新开始计数）
        self.detection_records.clear()

    def print_success_message(self, trigger_time, avg_confidence):
        """打印成功检测信息"""
        print("\n" + "="*70)
        print("\033[92m" + "🎉🎉🎉 成功到达 2楼！ 🎉🎉🎉" + "\033[0m")
        print("-"*70)
        print(f"📊 检测统计:")
        print(f"  目标类别: {self.target_class}")
        print(f"  检测次数: {len(self.detection_records)}次")
        print(f"  时间窗口: {self.detection_window}秒")
        print(f"  平均置信度: {avg_confidence:.3f}")
        print(f"⏰ 触发时间: {datetime.fromtimestamp(trigger_time).strftime('%H:%M:%S.%f')[:-3]}")
        print("📡 状态已发布到: /floor/detection_status")
        print("="*70)
        print("\n继续检测...")

    def cleanup_old_records(self, current_time=None):
        """清理过期的检测记录"""
        if current_time is None:
            current_time = time.time()
        
        cutoff_time = current_time - self.detection_window
        
        # 移除时间窗口之外的记录
        while self.detection_records and self.detection_records[0][0] < cutoff_time:
            self.detection_records.popleft()
        
        # 重置触发状态（冷却期结束）
        if self.is_triggered and current_time - self.last_trigger_time >= self.cooldown_time:
            self.is_triggered = False
            if self.enable_terminal_output:
                print(f"\n\033[93m[信息] 冷却期结束，准备下一次检测\033[0m")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FloorDetectionNode()
        
        print("\n楼层检测节点正在运行...")
        print("按 Ctrl+C 停止\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\n\033[93m正在关闭楼层检测节点...\033[0m")
        
        # 输出最终统计
        if node.enable_terminal_output:
            print("\n" + "="*70)
            print("📊 最终统计:")
            print(f"  总处理帧数: {node.total_frames}")
            print(f"  检测到目标的帧数: {node.target_frames}")
            print(f"  检测率: {node.target_frames/max(node.total_frames, 1)*100:.1f}%")
            print(f"  运行时间: {time.time() - node.start_time:.1f}秒")
            print("="*70)
        
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()