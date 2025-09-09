#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
import socket
import json
import time
import uuid
import threading
from enum import Enum
from typing import List, Dict, Optional


class TaskState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    BUTTON_DETECTING = "button_detecting"
    BUTTON_PRESSING = "button_pressing"
    COMPLETED = "completed"
    FAILED = "failed"


class LocationButtonTracker:
    """跟踪每个地点的按钮按压状态"""
    def __init__(self, location_name: str, button_sequence: List[str]):
        self.location_name = location_name
        self.button_sequence = button_sequence  # 按钮按压顺序，例如 ['1', '2', 'Close']
        self.current_button_index = 0
        self.completed_buttons = []
    
    def get_next_button(self) -> Optional[str]:
        """获取下一个要按的按钮"""
        if self.current_button_index < len(self.button_sequence):
            return self.button_sequence[self.current_button_index]
        return None
    
    def mark_button_completed(self, button_name: str) -> bool:
        """标记按钮完成，返回是否成功"""
        expected_button = self.get_next_button()
        if expected_button == button_name:
            self.completed_buttons.append(button_name)
            self.current_button_index += 1
            return True
        return False
    
    def is_location_completed(self) -> bool:
        """检查该地点是否所有按钮都已完成"""
        return self.current_button_index >= len(self.button_sequence)
    
    def reset(self):
        """重置按钮状态，用于开始新的周期"""
        self.current_button_index = 0
        self.completed_buttons = []


class RobotAPI:
    """从demo脚本移植的机器人API类"""
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.buffer = b""

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1)
            self.sock.connect((self.ip, self.port))
            return True
        except socket.error:
            self.sock = None
            return False

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def _receive_json(self):
        try:
            end_brace_pos = self.buffer.find(b'}{')
            if end_brace_pos != -1:
                json_str = self.buffer[:end_brace_pos + 1]
                self.buffer = self.buffer[end_brace_pos + 1:]
            else:
                json_str = self.buffer
                self.buffer = b""
            return json.loads(json_str.decode('utf-8'))
        except (json.JSONDecodeError, IndexError):
            self.buffer = json_str + self.buffer
            return None

    def send_command(self, command, silent=False):
        if not self.sock:
            return None
        request_uuid = str(uuid.uuid4())
        if '?' in command:
            command_with_uuid = f"{command}&uuid={request_uuid}"
        else:
            command_with_uuid = f"{command}?uuid={request_uuid}"
        
        try:
            self.sock.sendall(command_with_uuid.encode('utf-8'))
        except socket.error:
            return None
            
        start_time = time.time()
        while time.time() - start_time < 10:
            try:
                self.buffer += self.sock.recv(4096)
            except socket.timeout:
                pass
            while True:
                response_json = self._receive_json()
                if response_json:
                    if response_json.get("type") == "notification":
                        continue
                    if response_json.get("uuid") == request_uuid:
                        return response_json
                else:
                    break
        return None


class AGVTargetController(Node):
    def __init__(self):
        super().__init__('agv_target_controller')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', '192.168.10.10'),
                ('robot_port', 31001),
                ('locations', ['A', 'B']),  # 导航目标点列表
                ('wait_time_after_arrival', 5.0),  # 到达后额外等待时间，确保AGV完全稳定后再发送按钮目标
                ('button_press_timeout', 90.0),  # 按钮按压超时时间
                ('navigation_timeout', 60.0),  # 导航超时时间
                ('enable_continuous_loop', True),  # 是否启用连续循环
            ]
        )
        
        # 获取参数
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        self.locations = self.get_parameter('locations').get_parameter_value().string_array_value
        
        # 硬编码按钮序列（可以通过YAML配置文件在未来扩展）
        self.location_button_sequences = {
            'A': ['1', '2', '3'],
            'B': ['2', '4', 'Open']
        }
        
        self.wait_time_after_arrival = self.get_parameter('wait_time_after_arrival').get_parameter_value().double_value
        self.button_press_timeout = self.get_parameter('button_press_timeout').get_parameter_value().double_value
        self.navigation_timeout = self.get_parameter('navigation_timeout').get_parameter_value().double_value
        self.enable_continuous_loop = self.get_parameter('enable_continuous_loop').get_parameter_value().bool_value
        
        # 初始化机器人API
        self.robot_api = RobotAPI(self.robot_ip, self.robot_port)
        
        # 状态跟踪
        self.current_state = TaskState.IDLE
        self.current_location_index = 0
        self.location_trackers: Dict[str, LocationButtonTracker] = {}
        
        # 初始化每个地点的按钮跟踪器
        for location in self.locations:
            if location in self.location_button_sequences:
                button_sequence = self.location_button_sequences[location]
                self.location_trackers[location] = LocationButtonTracker(location, button_sequence)
        
        # 任务控制变量
        self.current_target_location = None
        self.task_start_time = None
        self.waiting_for_button_completion = False
        self.navigation_start_time = None
        
        # ROS2 订阅和发布
        self.setup_ros_interfaces()
        
        # 启动连接
        self.connect_to_robot()
        
        # 主任务循环定时器
        self.main_loop_timer = self.create_timer(1.0, self.main_loop_callback)
        
        self.get_logger().info(f"AGV目标控制器已启动")
        self.get_logger().info(f"配置的地点: {self.locations}")
        self.get_logger().info(f"按钮序列: {self.location_button_sequences}")
    
    def setup_ros_interfaces(self):
        """设置ROS2接口"""
        # 订阅按钮按压完成信号
        self.button_completion_subscriber = self.create_subscription(
            String,
            '/button_press/completed',
            self.button_completion_callback,
            10
        )
        
        # 发布AGV状态信息
        self.status_publisher = self.create_publisher(
            String,
            '/agv/status',
            10
        )
        
        # 发布当前任务信息
        self.task_info_publisher = self.create_publisher(
            String,
            '/agv/current_task',
            10
        )
        
        # 发布当前需要按压的按钮编号（给实时按钮目标规划器）
        self.target_button_publisher = self.create_publisher(
            String,
            '/agv/target_button',
            10
        )
        
        # 发布AGV移动状态（用于禁止机械臂运动）
        self.agv_motion_status_publisher = self.create_publisher(
            String,
            '/agv/motion_status',
            10
        )
    
    def connect_to_robot(self):
        """连接到机器人"""
        if self.robot_api.connect():
            self.get_logger().info(f"成功连接到机器人 {self.robot_ip}:{self.robot_port}")
            return True
        else:
            self.get_logger().error(f"无法连接到机器人 {self.robot_ip}:{self.robot_port}")
            return False
    
    def button_completion_callback(self, msg: String):
        """处理按钮完成回调，只响应cycle_complete信号"""
        if not self.waiting_for_button_completion:
            return
        
        if msg.data != "cycle_complete":
            return
            
        self.get_logger().info("收到cycle_complete信号，按钮操作周期完成")
            
        current_location = self.locations[self.current_location_index]
        tracker = self.location_trackers.get(current_location)
        
        if tracker:
            expected_button = tracker.get_next_button()
            if expected_button:
                if tracker.mark_button_completed(expected_button):
                    self.get_logger().info(f"地点 {current_location} 的按钮 '{expected_button}' 已完成，准备前往下一个地点")
                    # 【修改点 1】按下一个按钮后，立即标记为完成状态，以便移动到下一个地点
                    self.waiting_for_button_completion = False
                    self.current_state = TaskState.COMPLETED
    
    def get_robot_status(self) -> Optional[Dict]:
        """获取机器人状态"""
        response = self.robot_api.send_command("/api/robot_status", silent=True)
        if response and "results" in response:
            return response["results"]
        return None
    
    def navigate_to_location(self, location: str) -> bool:
        """导航到指定地点"""
        self.get_logger().info(f"开始导航到地点: {location}")
        response = self.robot_api.send_command(f"/api/move?marker={location}")
        if response:
            self.navigation_start_time = time.time()
            return True
        return False
    
    def wait_for_navigation_complete(self, location: str) -> Optional[bool]:
        """等待导航完成，返回True表示成功，False表示失败，None表示仍在进行中"""
        status = self.get_robot_status()
        if not status:
            return False
        
        move_status = status.get("move_status")
        
        if time.time() - self.navigation_start_time > self.navigation_timeout:
            self.get_logger().error(f"导航到 {location} 超时")
            return False
        
        if move_status == "succeeded":
            self.get_logger().info(f"成功到达地点: {location}")
            return True
        elif move_status in ["failed", "canceled"]:
            self.get_logger().error(f"导航到 {location} 失败，状态: {move_status}")
            return False
        
        return None
    
    def start_button_detection_and_pressing(self, location: str):
        """开始按钮检测和按压流程"""
        tracker = self.location_trackers.get(location)
        if not tracker:
            self.get_logger().error(f"地点 {location} 没有配置按钮序列")
            self.current_state = TaskState.FAILED
            return
        
        next_button = tracker.get_next_button()
        if not next_button:
            self.get_logger().info(f"地点 {location} 的所有按钮都已按完，将前往下一个地点继续任务")
            # 如果当前地点没按钮了，也直接进入COMPLETED状态去切换地点
            self.current_state = TaskState.COMPLETED
            return
        
        self.get_logger().info(f"在地点 {location} 开始按压按钮: {next_button}")
        self.get_logger().info(f"发送目标按钮 '{next_button}' 给实时按钮目标规划器")
        
        self.publish_target_button(next_button, location)
        
        self.current_state = TaskState.BUTTON_PRESSING
        self.waiting_for_button_completion = True
        self.task_start_time = time.time()
    
    def main_loop_callback(self):
        """主任务循环"""
        try:
            self.publish_status()
            
            if self.current_state == TaskState.IDLE:
                self.handle_idle_state()
            elif self.current_state == TaskState.NAVIGATING:
                self.handle_navigating_state()
            elif self.current_state == TaskState.BUTTON_DETECTING:
                self.handle_button_detecting_state()
            elif self.current_state == TaskState.BUTTON_PRESSING:
                self.handle_button_pressing_state()
            elif self.current_state == TaskState.COMPLETED:
                self.handle_completed_state()
            elif self.current_state == TaskState.FAILED:
                self.handle_failed_state()
                
        except Exception as e:
            self.get_logger().error(f"主循环异常: {e}")
            self.current_state = TaskState.FAILED
    
    def handle_idle_state(self):
        """处理空闲状态"""
        if not self.robot_api.sock:
            if not self.connect_to_robot():
                return
        
        # 发布AGV开始移动状态
        self.publish_agv_motion_status("moving")
        
        self.current_target_location = self.locations[self.current_location_index]
        self.get_logger().info(f"开始新任务：前往地点 {self.current_target_location}")
        
        if self.navigate_to_location(self.current_target_location):
            self.current_state = TaskState.NAVIGATING
        else:
            self.current_state = TaskState.FAILED
    
    def handle_navigating_state(self):
        """处理导航状态"""
        result = self.wait_for_navigation_complete(self.current_target_location)
        if result is True:
            # 发布AGV停止移动状态
            self.publish_agv_motion_status("stopped")
            self.get_logger().info(f"到达地点 {self.current_target_location}，等待 {self.wait_time_after_arrival} 秒确保完全稳定后开始按钮检测")
            self.current_state = TaskState.BUTTON_DETECTING
            self.task_start_time = time.time()
        elif result is False:
            self.current_state = TaskState.FAILED
    
    def handle_button_detecting_state(self):
        """处理按钮检测状态"""
        if time.time() - self.task_start_time >= self.wait_time_after_arrival:
            self.start_button_detection_and_pressing(self.current_target_location)
    
    def handle_button_pressing_state(self):
        """处理按钮按压状态"""
        if time.time() - self.task_start_time > self.button_press_timeout:
            self.get_logger().error(f"按钮按压超时")
            self.current_state = TaskState.FAILED
            return
    
    def handle_completed_state(self):
        """【修改点 2】处理完成状态（现在代表单个按钮按压完成，需要切换地点）"""
        # 移动到下一个地点
        self.current_location_index = (self.current_location_index + 1) % len(self.locations)
        next_location = self.locations[self.current_location_index]
        
        # 检查是否完成了一整轮循环（即回到了起点并且起点所有按钮都已完成）
        # 这是判断整个任务是否结束或需要重置的逻辑
        start_location_tracker = self.location_trackers.get(self.locations[0])
        if self.current_location_index == 0 and start_location_tracker and start_location_tracker.is_location_completed():
            if not self.enable_continuous_loop:
                self.get_logger().info("所有地点的所有按钮都已按顺序完成，任务结束。")
                # 可以在这里添加停止逻辑，例如销毁定时器
                # self.main_loop_timer.cancel()
                return

            self.get_logger().info("完成一整轮按钮循环，重置所有按钮状态以开始新循环。")
            for tracker in self.location_trackers.values():
                tracker.reset()

        self.get_logger().info(f"准备前往下一个地点: {next_location}")
        self.current_state = TaskState.IDLE

    def handle_failed_state(self):
        """处理失败状态"""
        self.get_logger().error("任务失败，5秒后重试")
        time.sleep(5)
        self.current_state = TaskState.IDLE
        self.robot_api.disconnect()
    
    def publish_target_button(self, button_name: str, location: str):
        """发布当前需要按压的按钮编号给实时按钮目标规划器"""
        target_button_data = {
            "timestamp": time.time(),
            "target_button": button_name,
            "location": location,
            "command": "press_button"
        }
        
        target_msg = String()
        target_msg.data = json.dumps(target_button_data)
        self.target_button_publisher.publish(target_msg)
        
        self.get_logger().info(f"发布目标按钮: {button_name} (地点: {location})")
    
    def publish_agv_motion_status(self, status: str):
        """发布AGV移动状态"""
        motion_data = {
            "timestamp": time.time(),
            "status": status,  # "moving" 或 "stopped"
            "location": self.current_target_location
        }
        
        motion_msg = String()
        motion_msg.data = json.dumps(motion_data)
        self.agv_motion_status_publisher.publish(motion_msg)
        
        self.get_logger().info(f"发布AGV移动状态: {status}")
    
    def publish_status(self):
        """发布状态信息"""
        current_time = time.time()
        status_data = {
            "timestamp": current_time,
            "state": self.current_state.value,
            "current_location_index": self.current_location_index,
            "current_target_location": self.current_target_location,
            "waiting_for_button": self.waiting_for_button_completion,
            "total_locations": len(self.locations),
        }
        
        if self.current_target_location:
            tracker = self.location_trackers.get(self.current_target_location)
            if tracker:
                status_data["location_progress"] = {
                    "location": self.current_target_location,
                    "completed_count": len(tracker.completed_buttons),
                    "total_count": len(tracker.button_sequence),
                    "progress_percentage": len(tracker.completed_buttons) / len(tracker.button_sequence) * 100,
                    "next_button": tracker.get_next_button(),
                    "completed_buttons": tracker.completed_buttons,
                }
        
        if self.current_state == TaskState.NAVIGATING and self.navigation_start_time:
            elapsed_time = current_time - self.navigation_start_time
            status_data["navigation_elapsed"] = elapsed_time
            status_data["navigation_timeout"] = self.navigation_timeout
            
        elif self.current_state == TaskState.BUTTON_PRESSING and self.task_start_time:
            elapsed_time = current_time - self.task_start_time
            status_data["button_press_elapsed"] = elapsed_time
            status_data["button_press_timeout"] = self.button_press_timeout
        
        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.status_publisher.publish(status_msg)
        
        if self.current_target_location:
            tracker = self.location_trackers.get(self.current_target_location)
            if tracker:
                next_button = tracker.get_next_button()
                task_info = {
                    "timestamp": current_time,
                    "location": self.current_target_location,
                    "next_button": next_button,
                    "completed_buttons": tracker.completed_buttons,
                    "total_buttons": tracker.button_sequence,
                    "state": self.current_state.value
                }
                
                task_msg = String()
                task_msg.data = json.dumps(task_info)
                self.task_info_publisher.publish(task_msg)
    
    def cleanup(self):
        """清理资源"""
        self.robot_api.disconnect()


def main(args=None):
    rclpy.init(args=args)
    
    node = AGVTargetController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()