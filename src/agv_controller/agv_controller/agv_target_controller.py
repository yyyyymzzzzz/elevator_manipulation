#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import time
import uuid
from typing import Optional, Dict


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
                ('navigation_timeout', 60.0),  # 导航超时时间
            ]
        )
        # 获取参数
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        self.navigation_timeout = self.get_parameter('navigation_timeout').get_parameter_value().double_value
        # 初始化机器人API
        self.robot_api = RobotAPI(self.robot_ip, self.robot_port)
        # 状态
        self.state = 'idle'  # idle | navigating | completed | failed
        self.current_target_location: Optional[str] = None
        self.navigation_start_time: Optional[float] = None
        self.motion_status = 'stopped'  # moving | stopped
        # ROS2 接口
        self.setup_ros_interfaces()
        # 连接机器人
        self.connect_to_robot()
        # 轮询状态定时器
        self.timer = self.create_timer(0.5, self.poll_status)
        self.get_logger().info("AGV目标控制器已启动（简化版：仅导航 + 统一状态发布）")
    
    def setup_ros_interfaces(self):
        """设置ROS2接口"""
        self.nav_target_sub = self.create_subscription(
            String,
            '/decision/navigation_target',
            self.navigation_target_callback,
            10
        )
        self.status_pub = self.create_publisher(String, '/agv/status', 10)
    
    def connect_to_robot(self):
        """连接到机器人"""
        if self.robot_api.connect():
            self.get_logger().info(f"成功连接到机器人 {self.robot_ip}:{self.robot_port}")
            return True
        else:
            self.get_logger().error(f"无法连接到机器人 {self.robot_ip}:{self.robot_port}")
            return False

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
            self.state = 'navigating'
            self.motion_status = 'moving'
            self.publish_state('navigating')
            return True
        return False
    
    def navigation_target_callback(self, msg: String):
        """接收导航目标并触发移动"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("导航目标消息解析失败（非JSON）")
            return
        
        if data.get('command') != 'navigate_to':
            # 忽略非导航指令
            return
        
        target = data.get('target_location')
        if not target:
            self.get_logger().error("导航目标缺少 target_location")
            return
        
        # 若未连接尝试重连
        if not self.robot_api.sock and not self.connect_to_robot():
            self.publish_state('failed')
            return
        
        self.current_target_location = target
        ok = self.navigate_to_location(target)
        if not ok:
            self.get_logger().error(f"发送导航到 {target} 指令失败")
            self.publish_state('failed')
            return
        
        self.publish_state('navigating')
    
    def poll_status(self):
        """周期性查询导航状态，发布完成/失败"""
        # 仅在导航中才轮询
        if self.state != 'navigating' or not self.current_target_location:
            return
        
        status = self.get_robot_status()
        if not status:
            # 获取不到状态，先忽略一次
            return
        
        # 超时处理
        if self.navigation_start_time and time.time() - self.navigation_start_time > self.navigation_timeout:
            self.get_logger().error(f"导航到 {self.current_target_location} 超时")
            self.motion_status = 'stopped'
            self.publish_state('failed')
            # 重置
            self.state = 'idle'
            self.current_target_location = None
            self.navigation_start_time = None
            return
        
        move_status = status.get('move_status')
        if move_status == 'succeeded':
            self.get_logger().info(f"成功到达地点: {self.current_target_location}")
            self.motion_status = 'stopped'
            self.publish_state('completed')
            self.state = 'idle'
            self.current_target_location = None
            self.navigation_start_time = None
        elif move_status in ['failed', 'canceled']:
            self.get_logger().error(f"导航失败，状态: {move_status}")
            self.motion_status = 'stopped'
            self.publish_state('failed')
            self.state = 'idle'
            self.current_target_location = None
            self.navigation_start_time = None
        else:
            # 仍在进行中，更新状态
            self.publish_state('navigating')
    
    def publish_state(self, state: str):
        """发布 /agv/status"""
        self.state = state
        payload = {
            'timestamp': time.time(),
            'state': state,
            'target_location': self.current_target_location,
            'motion_status': self.motion_status,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
    
    def cleanup(self):
        """清理资源"""
        self.robot_api.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = AGVTargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()