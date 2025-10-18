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
        self.waiting_for_door = False
        self.is_moving = False

        # 新增：用于管理多步任务的状态
        self.button_press_step = 'idle'  # 'idle', 'lowering_head', 'pressing_button', 'resetting_head'
        
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

        # 发布器 - 发送头部控制目标
        self.head_control_publisher = self.create_publisher(
            String,
            '/decision/body_head_goal',
            10
        )

        # 发布器 - 发送等待开门指令
        self.door_wait_publisher = self.create_publisher(
            String,
            '/decision/wait_for_door',
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
        
        # 订阅器 - 头部控制完成信号
        self.head_control_completion_subscriber = self.create_subscription(
            String,
            '/decision/body_head_completed',
            self.head_control_completion_callback,
            10
        )

        # 订阅器 - 按钮按压完成信号
        self.button_completion_subscriber = self.create_subscription(
            String,
            '/button_press/completed',
            self.button_completion_callback,
            10
        )

        # 订阅器 - 电梯门状态信号
        self.door_status_subscriber = self.create_subscription(
            String,
            '/elevator/door_status',
            self.door_status_callback,
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
            
            # 提取任务列表 (更灵活的方式)
            for task_data in config_data.get('tasks', []):
                for step in task_data.get('steps', []):
                    # 复制YAML中的所有任务参数
                    new_task = step.copy()
                    
                    # 为了内部代码一致性，将'task_type'重命名为'type'
                    new_task['type'] = new_task.pop('task_type', None)
                    
                    # 添加自动生成的描述信息
                    new_task['description'] = f"{task_data.get('name', '')} - {step.get('step_id', '')}"
                    
                    self.task_list.append(new_task)
            
            self.get_logger().info(f"成功加载任务配置文件: {config_path}")
            
        except Exception as e:
            self.get_logger().error(f"加载任务配置失败: {e}")
            self.create_fallback_tasks()
    
    def create_default_config(self, config_path: str):
        """创建默认配置文件"""
        default_config = {
            'tasks': [
                {
                    'name': '完整电梯操作流程示例',
                    'steps': [
                        {'task_type': 'navigation', 'location': 'Elevator_Door_Outside_F1', 'step_id': '导航到1楼电梯门外'},
                        {'task_type': 'button_press', 'button': 'Up', 'step_id': '按压向上按钮'},
                        {'task_type': 'wait_for_door', 'step_id': '等待电梯门开启'},
                        {'task_type': 'motion_sequence', 'sequence': 'enter_elevator', 'step_id': '进入电梯并转向'},
                        {'task_type': 'button_press', 'button': '5', 'step_id': '按压楼层按钮5'},
                        {'task_type': 'motion_sequence', 'sequence': 'panel_to_door', 'step_id': '电梯内转向门口'},
                        {'task_type': 'wait_for_door', 'step_id': '等待电梯门开启'},
                        {
                            'task_type': 'exit_and_update_pose', 
                            'map_name': 'my_building_map',  # 机器人中地图的名称
                            'floor': '5',                  # 目标楼层
                            'location': 'F5_Lobby',        # 目标楼层的标定点
                            'step_id': '驶出电梯并更新位姿到5楼大厅'
                        },
                        {'task_type': 'navigation', 'location': 'Destination_On_F5', 'step_id': '导航到5楼的目的地'},
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
        if self.waiting_for_navigation or self.waiting_for_button or self.waiting_for_door:
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
        elif task_type == 'wait_for_door':
            self.execute_wait_for_door_task(task)
        elif task_type == 'motion_sequence':
            self.execute_motion_sequence_task(task)
        elif task_type == 'exit_and_update_pose':
            self.execute_exit_and_update_pose_task(task)
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
        """执行按钮按压任务，这是一个多步骤过程"""
        self.get_logger().info("开始执行按按钮序列...")
        self.waiting_for_button = True  # 启动整个序列的等待标志
        self.button_press_step = 'lowering_head'
        
        # 步骤1: 低头
        self.get_logger().info("步骤 1/3: 低头以便观察按钮")
        self._send_head_command(30.0)

    def _send_head_command(self, pitch: float):
        """发送头部控制指令的辅助函数"""
        self.get_logger().info("头部开始运动，is_moving设置为True")
        self.is_moving = True
        msg = String()
        msg.data = json.dumps({
            'command': 'set_pitch',
            'pitch': pitch,
            'timestamp': time.time()
        })
        self.head_control_publisher.publish(msg)

    def execute_wait_for_door_task(self, task: Dict):
        """执行等待开门任务"""
        self.get_logger().info("⏳ 等待电梯门开启...")
        
        # 发送指令给AGV控制器，令其开始检测门的状态
        door_msg = String()
        door_msg.data = json.dumps({
            'command': 'wait_for_door_open',
            'timestamp': time.time()
        })
        self.door_wait_publisher.publish(door_msg)
        
        # 设置等待标志
        self.waiting_for_door = True

    def execute_motion_sequence_task(self, task: Dict):
        """执行预定义的运动序列任务（进入电梯、转向等）"""
        sequence = task.get('sequence')
        if not sequence:
            self.get_logger().error("运动序列任务缺少 'sequence' 参数")
            self.move_to_next_task()
            return
        
        self.get_logger().info(f"🚗 开始执行运动序列: {sequence}")
        
        # 复用导航话题，发送一个包含新命令的JSON消息
        msg = String()
        msg.data = json.dumps({
            'command': 'execute_sequence',
            'sequence_name': sequence,
            'timestamp': time.time()
        })
        self.navigation_publisher.publish(msg)
        
        # 复用导航等待标志，因为完成信号将通过 /agv/status topic 发来
        self.waiting_for_navigation = True

    def execute_exit_and_update_pose_task(self, task: Dict):
        """执行驶出电梯并更新地图位姿的任务"""
        map_name = task.get('map_name')
        target_floor = task.get('floor')
        target_location = task.get('location')

        if not all([map_name, target_floor, target_location]):
            self.get_logger().error("驶出并更新位姿任务缺少 'map_name', 'floor', 或 'location' 参数")
            self.move_to_next_task()
            return

        self.get_logger().info(
            f"🚀 开始驶出电梯，完成后将更新位姿到地图 '{map_name}' 的楼层 '{target_floor}' 的 '{target_location}' 点"
        )
        
        msg = String()
        msg.data = json.dumps({
            'command': 'exit_and_update_pose',
            'map_name': map_name,
            'target_floor': str(target_floor),
            'target_location': target_location,
            'timestamp': time.time()
        })
        self.navigation_publisher.publish(msg)
        
        # 同样复用导航等待标志
        self.waiting_for_navigation = True

    def head_control_completion_callback(self, msg: String):
        """头部控制完成回调"""
        if not self.waiting_for_button or self.button_press_step not in ['lowering_head', 'resetting_head']:
            return

        try:
            data = json.loads(msg.data)
            if data.get('status') != 'completed':
                self.get_logger().error("头部姿态调整失败，中止按钮按压序列")
                self.is_moving = False # 运动结束
                self.reset_button_sequence()
                self.move_to_next_task()
                return

            if self.button_press_step == 'lowering_head':
                self.get_logger().info("✅ 头部已放低，is_moving设置为False，准备按按钮")
                self.is_moving = False # 运动结束
                self.button_press_step = 'pressing_button'
                
                # 步骤2: 按按钮
                current_task = self.task_list[self.current_task_index]
                button = current_task.get('button')
                location = current_task.get('location', '')
                self.get_logger().info(f"步骤 2/3: 按压按钮 '{button}'")
                
                button_msg = String()
                button_msg.data = json.dumps({
                    'target_button': button,
                    'location': location,
                    'command': 'press_button',
                    'timestamp': time.time()
                })
                self.button_target_publisher.publish(button_msg)

            elif self.button_press_step == 'resetting_head':
                self.get_logger().info("✅ 头部已恢复姿态，按钮按压序列完成")
                self.is_moving = False # 运动结束
                self.reset_button_sequence()
                self.move_to_next_task()

        except (json.JSONDecodeError, AttributeError):
            self.get_logger().warn("无法解析头部控制完成消息")

    def door_status_callback(self, msg: String):
        """电梯门状态回调"""
        if not self.waiting_for_door:
            return
        
        try:
            data = json.loads(msg.data)
            if data.get('status') == 'opened':
                self.get_logger().info("✅ 电梯门已开启")
                self.waiting_for_door = False
                self.move_to_next_task()
        except (json.JSONDecodeError, AttributeError):
            self.get_logger().warn("无法解析电梯门状态消息")

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
        if not self.waiting_for_button or self.button_press_step != 'pressing_button':
            return
        
        if msg.data == "cycle_complete":
            self.get_logger().info("✅ 按钮操作完成")
            self.button_press_step = 'resetting_head'
            
            # 步骤3: 恢复头部姿态
            self.get_logger().info("步骤 3/3: 恢复头部姿态")
            self._send_head_command(0.0)

        elif msg.data == "failed":
            self.get_logger().error("❌ 按钮操作失败，将尝试恢复头部姿态并继续")
            self.button_press_step = 'resetting_head'
            self._send_head_command(0.0) # 即使失败也要抬头
    
    def reset_button_sequence(self):
        """重置按钮按压序列的状态"""
        self.waiting_for_button = False
        self.button_press_step = 'idle'

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
            'waiting_for_door': self.waiting_for_door,
            'button_press_step': self.button_press_step,
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
