#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        
        # 订阅AGV状态
        self.agv_status_subscriber = self.create_subscription(
            String,
            '/agv/status',
            self.agv_status_callback,
            10
        )
        
        # 订阅当前任务信息
        self.task_info_subscriber = self.create_subscription(
            String,
            '/agv/current_task',
            self.task_info_callback,
            10
        )
        
        # 订阅按钮按压完成信号
        self.button_completion_subscriber = self.create_subscription(
            String,
            '/button_press/completed',
            self.button_completion_callback,
            10
        )
        
        # 订阅当前按钮信息
        self.current_button_info_subscriber = self.create_subscription(
            String,
            '/current_button_info',
            self.current_button_info_callback,
            10
        )
        
        self.get_logger().info("系统监控器已启动")
        
        # 状态存储
        self.last_agv_status = {}
        self.last_task_info = {}
        self.last_button_info = {}
    
    def agv_status_callback(self, msg: String):
        """AGV状态回调"""
        try:
            status_data = json.loads(msg.data)
            self.last_agv_status = status_data
            
            timestamp = datetime.now().strftime("%H:%M:%S")
            state = status_data.get('state', 'unknown')
            target = status_data.get('current_target_location', 'none')
            waiting = status_data.get('waiting_for_button', False)
            
            status_str = f"[{timestamp}] AGV状态: {state}"
            if target != 'none':
                status_str += f" | 目标地点: {target}"
            if waiting:
                status_str += f" | 等待按钮完成"
            
            self.get_logger().info(status_str)
            
        except json.JSONDecodeError:
            self.get_logger().error("无法解析AGV状态数据")
    
    def task_info_callback(self, msg: String):
        """任务信息回调"""
        try:
            task_data = json.loads(msg.data)
            self.last_task_info = task_data
            
            location = task_data.get('location', 'unknown')
            next_button = task_data.get('next_button', 'none')
            completed = task_data.get('completed_buttons', [])
            total = task_data.get('total_buttons', [])
            
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            progress_str = f"[{timestamp}] 任务进度 - 地点: {location}"
            if next_button != 'none':
                progress_str += f" | 下一个按钮: {next_button}"
            progress_str += f" | 已完成: {completed} | 总共: {total}"
            
            self.get_logger().info(progress_str)
            
        except json.JSONDecodeError:
            self.get_logger().error("无法解析任务信息数据")
    
    def button_completion_callback(self, msg: String):
        """按钮完成回调"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        if msg.data == "success":
            self.get_logger().info(f"[{timestamp}] ✅ 按钮按压成功完成！")
        elif msg.data == "cycle_complete":
            self.get_logger().info(f"[{timestamp}] 🔄 按钮操作周期完全完成！（包括复位）")
        else:
            self.get_logger().info(f"[{timestamp}] ❌ 按钮按压结果: {msg.data}")
    
    def current_button_info_callback(self, msg: String):
        """当前按钮信息回调"""
        try:
            button_data = json.loads(msg.data)
            self.last_button_info = button_data
            
            timestamp = datetime.now().strftime("%H:%M:%S")
            button_name = button_data.get('button_name', 'unknown')
            button_index = button_data.get('button_index', 0)
            total_buttons = button_data.get('total_buttons', 0)
            status = button_data.get('status', 'unknown')
            button_exists = button_data.get('button_exists', False)
            
            info_str = f"[{timestamp}] 🎯 当前目标按钮: {button_name} [{button_index}/{total_buttons}]"
            
            if button_exists:
                button_pos = button_data.get('button_position', {})
                info_str += f" | ✓可见 位置:({button_pos.get('x', 0):.3f}, {button_pos.get('y', 0):.3f}, {button_pos.get('z', 0):.3f})"
            else:
                if button_data.get('waiting_for_button', False):
                    remaining = button_data.get('remaining_wait_time', 0)
                    info_str += f" | ✗缺失 等待中 ({remaining:.1f}s)"
                else:
                    info_str += f" | ✗缺失"
            
            self.get_logger().info(info_str)
            
        except json.JSONDecodeError:
            self.get_logger().error("无法解析当前按钮信息数据")


def main(args=None):
    rclpy.init(args=args)
    
    node = SystemMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
