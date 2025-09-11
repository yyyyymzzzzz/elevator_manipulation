#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import yaml
import os
from typing import List, Dict


class TaskAssignment(Node):
    def __init__(self):
        super().__init__('task_assignment')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('task_config_file', 'tasks.yaml'),
                ('enable_continuous_loop', True),
            ]
        )
        
        # 获取参数
        self.task_config_file = self.get_parameter('task_config_file').get_parameter_value().string_value
        self.enable_continuous_loop = self.get_parameter('enable_continuous_loop').get_parameter_value().bool_value
        
        # 任务清单
        self.task_list: List[Dict] = []
        self.current_task_index = 0
        
        # 状态跟踪
        self.waiting_for_navigation = False
        self.waiting_for_button = False
        self.is_moving = False
        
        # 设置ROS接口
        self.setup_ros_interfaces()
        
        # 加载任务配置
        self.load_tasks()
        
        # 主循环定时器 - 每秒检查一次状态
        self.main_timer = self.create_timer(1.0, self.main_loop)
        
        self.get_logger().info(f"任务分配器已启动，共加载 {len(self.task_list)} 个任务")
    
    def setup_ros_interfaces(self):
        """设置ROS接口"""
        # 发布器 - 发送导航目标给AGV控制器
        self.navigation_publisher = self.create_publisher(
            String,
            '/decision/navigation_target',
            10
        )
        
        # 发布器 - 发送按钮目标给感知系统
        self.button_target_publisher = self.create_publisher(
            String,
            '/decision/target_button',
            10
        )
        
        # 发布器 - 发布任务状态
        self.task_status_publisher = self.create_publisher(
            String,
            '/decision_maker/status',
            10
        )
        
        # 订阅器 - AGV状态（检测导航完成）
        self.agv_status_subscriber = self.create_subscription(
            String,
            '/agv/status',
            self.agv_status_callback,
            10
        )
        
        # 订阅器 - 按钮按压完成信号
        self.button_completion_subscriber = self.create_subscription(
            String,
            '/button_press/completed',
            self.button_completion_callback,
            10
        )
    
    def load_tasks(self):
        """从YAML文件加载任务清单"""
        try:
            config_path = os.path.join('/home/nvidia/Workspace/elevator_manipulation/config', self.task_config_file)
            
            if not os.path.exists(config_path):
                self.get_logger().warn(f"配置文件 {config_path} 不存在，创建默认配置")
                self.create_default_config(config_path)
            
            with open(config_path, 'r', encoding='utf-8') as file:
                config_data = yaml.safe_load(file)
            
            # 提取任务列表
            for task_data in config_data.get('tasks', []):
                for step in task_data.get('steps', []):
                    self.task_list.append({
                        'type': step.get('task_type'),
                        'location': step.get('location'),
                        'button': step.get('button'),
                        'description': f"{task_data.get('name', '')} - {step.get('step_id', '')}"
                    })
            
            self.get_logger().info(f"成功加载任务配置文件: {config_path}")
            
        except Exception as e:
            self.get_logger().error(f"加载任务配置失败: {e}")
            self.create_fallback_tasks()
    
    def create_default_config(self, config_path: str):
        """创建默认配置文件"""
        default_config = {
            'tasks': [
                {
                    'name': '电梯按钮操作任务',
                    'steps': [
                        {'task_type': 'navigation', 'location': 'A', 'step_id': '导航到A点'},
                        {'task_type': 'button_press', 'location': 'A', 'button': '1', 'step_id': '按压按钮1'},
                        {'task_type': 'button_press', 'location': 'A', 'button': '2', 'step_id': '按压按钮2'},
                        {'task_type': 'button_press', 'location': 'A', 'button': '3', 'step_id': '按压按钮3'},
                        {'task_type': 'navigation', 'location': 'B', 'step_id': '导航到B点'},
                        {'task_type': 'button_press', 'location': 'B', 'button': '2', 'step_id': '按压按钮2'},
                        {'task_type': 'button_press', 'location': 'B', 'button': '4', 'step_id': '按压按钮4'},
                        {'task_type': 'button_press', 'location': 'B', 'button': 'Open', 'step_id': '按压开门按钮'},
                    ]
                }
            ]
        }
        
        try:
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with open(config_path, 'w', encoding='utf-8') as file:
                yaml.dump(default_config, file, default_flow_style=False, allow_unicode=True)
        except Exception as e:
            self.get_logger().error(f"无法创建默认配置文件: {e}")
    
    def create_fallback_tasks(self):
        """创建后备任务"""
        self.task_list = [
            {'type': 'navigation', 'location': 'A', 'description': '导航到A点'},
            {'type': 'button_press', 'location': 'A', 'button': '1', 'description': '按压按钮1'},
            {'type': 'navigation', 'location': 'B', 'description': '导航到B点'},
            {'type': 'button_press', 'location': 'B', 'button': 'Open', 'description': '按压开门按钮'},
        ]
    
    def main_loop(self):
        # 发布状态信息
        self.publish_status()

        """主循环 - 检查状态并执行下一个任务"""
        # 如果正在等待导航或按钮完成，则不执行新任务
        if self.waiting_for_navigation or self.waiting_for_button:
            return
        
        # 检查是否还有任务要执行
        if self.current_task_index >= len(self.task_list):
            if self.enable_continuous_loop:
                self.get_logger().info("所有任务完成，开始新的循环")
                self.current_task_index = 0
            else:
                self.get_logger().info("所有任务已完成")
                return
        
        # 执行当前任务
        current_task = self.task_list[self.current_task_index]
        self.execute_task(current_task)
    
    def execute_task(self, task: Dict):
        """执行单个任务"""
        task_type = task.get('type')
        description = task.get('description', '未知任务')
        
        self.get_logger().info(f"执行任务 [{self.current_task_index + 1}/{len(self.task_list)}]: {description}")
        
        if task_type == 'navigation':
            self.execute_navigation_task(task)
        elif task_type == 'button_press':
            self.execute_button_press_task(task)
        else:
            self.get_logger().error(f"未知任务类型: {task_type}")
            self.move_to_next_task()
    
    def execute_navigation_task(self, task: Dict):
        """执行导航任务"""
        location = task.get('location')
        if not location:
            self.get_logger().error("导航任务缺少目标位置")
            self.move_to_next_task()
            return
        
        self.get_logger().info(f"🚗 开始导航到: {location}")
        
        # 发送导航指令（适配现有的AGV控制器接口）
        nav_msg = String()
        nav_msg.data = json.dumps({
            'target_location': location,
            'command': 'navigate_to',
            'timestamp': time.time()
        })
        self.navigation_publisher.publish(nav_msg)
        
        # 设置等待导航完成的标志
        self.waiting_for_navigation = True
    
    def execute_button_press_task(self, task: Dict):
        """执行按钮按压任务"""
        button = task.get('button')
        location = task.get('location', '')
        
        if not button:
            self.get_logger().error("按钮任务缺少目标按钮")
            self.move_to_next_task()
            return
        
        self.get_logger().info(f"👆 开始按压按钮: {button} (位置: {location})")
        
        # 发送按钮目标指令（使用现有接口）
        button_msg = String()
        button_msg.data = json.dumps({
            'target_button': button,
            'location': location,
            'command': 'press_button',
            'timestamp': time.time()
        })
        self.button_target_publisher.publish(button_msg)
        
        # 设置等待按钮完成的标志
        self.waiting_for_button = True
    
    def agv_status_callback(self, msg: String):
        """AGV状态回调"""
        if not self.waiting_for_navigation:
            return
        
        try:
            status_data = json.loads(msg.data)
            state = status_data.get('state', 'unknown')
            
            # 检查导航是否完成
            motion_status = status_data.get('motion_status', '')
            if state in ['idle', 'completed'] and motion_status == 'stopped':  # 导航完成
                self.get_logger().info("✅ 导航完成")
                self.waiting_for_navigation = False
                self.is_moving = False
                self.move_to_next_task()
            elif state == 'navigating':
                self.get_logger().info("🚗 AGV正在导航中...")
                self.is_moving = True
            elif state == 'failed':
                self.get_logger().error("❌ 导航失败")
                self.waiting_for_navigation = False
                self.is_moving = False
                # 导航失败，重试当前任务
                self.get_logger().info("正在重试导航任务")
                self.execute_navigation_task(self.task_list[self.current_task_index])
                
        except json.JSONDecodeError:
            self.get_logger().error("无法解析AGV状态数据")
    
    def button_completion_callback(self, msg: String):
        """按钮完成回调"""
        if not self.waiting_for_button:
            return
        
        if msg.data == "cycle_complete":
            self.get_logger().info("✅ 按钮操作完成")
            self.waiting_for_button = False
            self.move_to_next_task()
        elif msg.data == "failed":
            self.get_logger().error("❌ 按钮操作失败")
            self.waiting_for_button = False
            # 可以选择重试或跳过
            self.move_to_next_task()
    
    def move_to_next_task(self):
        """移动到下一个任务"""
        self.current_task_index += 1
    
    def publish_status(self):
        """发布当前状态（可选，用于监控）"""
        status_data = {
            'current_task_index': self.current_task_index,
            'total_tasks': len(self.task_list),
            'waiting_for_navigation': self.waiting_for_navigation,
            'waiting_for_button': self.waiting_for_button,
            'is_moving': self.is_moving,
            'timestamp': time.time()
        }
        
        if self.current_task_index < len(self.task_list):
            status_data['current_task'] = self.task_list[self.current_task_index]
        
        msg = String()
        msg.data = json.dumps(status_data)
        self.task_status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = TaskAssignment()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
