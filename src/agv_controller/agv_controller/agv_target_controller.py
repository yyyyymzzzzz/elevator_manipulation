#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import time
import uuid
from typing import Optional, Dict
import math 


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

    def get_current_pose(self) -> Optional[Dict]:
        """从 robot_status 中提取当前位姿 (x, y, theta)。失败返回 None。"""
        status = self.send_command("/api/robot_status", silent=True)
        if status and status.get("status") == "OK":
            return status.get("results", {}).get("current_pose")
        return None

    def get_distance_probe(self, x, y):
        command = f"/api/map/distance_probe?x={x}&y={y}"
        response = self.send_command(command)
        if response and response.get("status") == "OK" and "results" in response:
            return response["results"]["env_dist"].get("obstacle", -1)
        return -1

    def fused_forward_distance(self, near_threshold, far_probe_distance):
        """
        使用融合策略估计“正前方”障碍物距离。
        返回: (final_distance, method_used)
        失败: ( -1, "原因" )
        """
        pose = self.get_current_pose()
        if not pose:
            return -1, "位姿获取失败"

        dist_center = self.get_distance_probe(pose['x'], pose['y'])

        if dist_center >= 0 and dist_center < near_threshold:
            return dist_center, "近距离模式(中心点)"

        if dist_center >= near_threshold:
            probe_x = pose['x'] + far_probe_distance * math.cos(pose['theta'])
            probe_y = pose['y'] + far_probe_distance * math.sin(pose['theta'])
            dist_probe = self.get_distance_probe(probe_x, probe_y)
            if dist_probe >= 0:
                return far_probe_distance + dist_probe, "远距离模式(前方点)"
            return -1, "前方点测距失败"

        return -1, "中心点测距失败"


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
                ('move_speed', 0.2),         # 盲走线速度 (m/s)
                ('turn_speed', 0.5),         # 盲走角速度 (rad/s)
                ('send_rate_hz', 10),        # joy_control发送频率
                # 新增：等待开门相关参数
                ('door_open_threshold', 0.45), # 门开启距离阈值 (m)
                ('near_obstacle_threshold', 0.1), # 近距离探测阈值 (m)
                ('far_probe_distance', 0.3),      # 远距离探测点 (m)
            ]
        )
        # 获取参数
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value
        self.navigation_timeout = self.get_parameter('navigation_timeout').get_parameter_value().double_value
        self.move_speed = self.get_parameter('move_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.send_rate_hz = self.get_parameter('send_rate_hz').get_parameter_value().integer_value
        self.door_open_threshold = self.get_parameter('door_open_threshold').get_parameter_value().double_value
        self.near_obstacle_threshold = self.get_parameter('near_obstacle_threshold').get_parameter_value().double_value
        self.far_probe_distance = self.get_parameter('far_probe_distance').get_parameter_value().double_value
        
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
        self.door_wait_sub = self.create_subscription(
            String,
            '/decision/wait_for_door',
            self.wait_for_door_callback,
            10
        )
        self.status_pub = self.create_publisher(String, '/agv/status', 10)
        self.door_status_pub = self.create_publisher(String, '/elevator/door_status', 10)

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

    def get_current_pose(self) -> Optional[Dict]:
        """从 robot_status 中提取当前位姿 (x, y, theta)。失败返回 None。"""
        status = self.robot_api.send_command("/api/robot_status", silent=True)
        if status and status.get("status") == "OK":
            return status.get("results", {}).get("current_pose")
        return None

    def get_distance_probe(self, x, y):
        command = f"/api/map/distance_probe?x={x}&y={y}"
        response = self.robot_api.send_command(command)
        if response and response.get("status") == "OK" and "results" in response:
            return response["results"]["env_dist"].get("obstacle", -1)
        return -1

    def fused_forward_distance(self, near_threshold, far_probe_distance):
        """
        使用融合策略估计“正前方”障碍物距离。
        返回: (final_distance, method_used)
        失败: ( -1, "原因" )
        """
        pose = self.get_current_pose()
        if not pose:
            return -1, "位姿获取失败"

        dist_center = self.get_distance_probe(pose['x'], pose['y'])

        if dist_center >= 0 and dist_center < near_threshold:
            return dist_center, "近距离模式(中心点)"

        if dist_center >= near_threshold:
            probe_x = pose['x'] + far_probe_distance * math.cos(pose['theta'])
            probe_y = pose['y'] + far_probe_distance * math.sin(pose['theta'])
            dist_probe = self.get_distance_probe(probe_x, probe_y)
            if dist_probe >= 0:
                return far_probe_distance + dist_probe, "远距离模式(前方点)"
            return -1, "前方点测距失败"

        return -1, "中心点测距失败"


    def wait_for_door_callback(self, msg: String):
        """接收等待开门的指令"""
        try:
            data = json.loads(msg.data)
            if data.get('command') == 'wait_for_door_open':
                self.get_logger().info("接收到'等待电梯门开启'指令，开始使用融合测距进行检测...")
                self.execute_wait_for_door()
        except (json.JSONDecodeError, AttributeError):
            self.get_logger().error("等待开门指令解析失败")

    def execute_wait_for_door(self):
        """执行等待开门任务，使用融合测距"""
        # 确保机器人连接正常
        if not self.robot_api.sock and not self.connect_to_robot():
            self.get_logger().error("无法连接机器人，无法执行开门检测")
            return

        # 1. 获取初始距离
        init_dist, init_mode = self.robot_api.fused_forward_distance(
            self.near_obstacle_threshold, self.far_probe_distance
        )
        if init_dist < 0:
            self.get_logger().error(f"无法获取初始距离 ({init_mode})，开门检测失败。")
            return
        
        self.get_logger().info(f"  [开门检测] 初始距离: {init_dist:.2f} m (模式: {init_mode})")
        self.get_logger().info(f"  [开门检测] 等待距离增加超过 {self.door_open_threshold:.2f} m...")

        # 2. 循环检测距离变化
        start_time = time.time()
        timeout = 60.0 # 设置一个60秒的超时
        while time.time() - start_time < timeout:
            cur_dist, mode = self.robot_api.fused_forward_distance(
                self.near_obstacle_threshold, self.far_probe_distance
            )
            
            if cur_dist >= 0:
                self.get_logger().info(f"\r  [开门检测] 当前距离: {cur_dist:.3f} m (模式: {mode})    ", throttle_duration_sec=1.0)
                # 判断距离是否显著增加
                if cur_dist > self.door_open_threshold:
                    self.get_logger().info(f"\n  [开门检测] 成功: 距离由 {init_dist:.2f}m 增至 {cur_dist:.2f}m，判定门已开启。")
                    # 发布门已开启的消息
                    door_msg = String()
                    door_msg.data = json.dumps({"status": "opened"})
                    self.door_status_pub.publish(door_msg)
                    return # 任务完成
            else:
                self.get_logger().warn(f"\r  [开门检测] 距离探测失败 (原因: {mode})    ", throttle_duration_sec=1.0)

            time.sleep(0.5) # 检测频率

        self.get_logger().warn("  [开门检测] 等待超时，仍未检测到门开启。")

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
        """接收任务目标并根据指令类型分发"""
        try:
            data = json.loads(msg.data)
            command = data.get('command')
        except (json.JSONDecodeError, AttributeError):
            self.get_logger().error("任务消息解析失败")
            return

        # 若未连接则尝试重连
        if not self.robot_api.sock and not self.connect_to_robot():
            self.publish_state('failed')
            return
        
        # --- 根据 command 分发任务 ---
        if command == 'navigate_to':
            target = data.get('target_location')
            if not target:
                self.get_logger().error("导航任务缺少 target_location")
                self.publish_state('failed')
                return
            self.current_target_location = target
            if not self.navigate_to_location(target):
                self.get_logger().error(f"发送导航到 {target} 指令失败")
                self.publish_state('failed')

        elif command == 'execute_sequence':
            sequence_name = data.get('sequence_name')
            if not sequence_name:
                self.get_logger().error("运动序列任务缺少 sequence_name")
                self.publish_state('failed')
                return
            self.get_logger().info(f"开始执行运动序列: {sequence_name}")
            # 开始执行序列，这是一个阻塞过程
            success = self.execute_motion_sequence(sequence_name)
            if success:
                self.get_logger().info(f"运动序列 {sequence_name} 执行成功")
                self.publish_state('completed') # <--- 完成后立即发布状态
            else:
                self.get_logger().error(f"运动序列 {sequence_name} 执行失败")
                self.publish_state('failed')    # <--- 失败后立即发布状态

        elif command == 'exit_and_update_pose':
            map_name = data.get('map_name')
            floor = data.get('target_floor')
            location = data.get('target_location')
            
            if not all([map_name, floor, location]):
                self.get_logger().error("退出并更新位姿任务缺少 map_name, target_floor, 或 target_location")
                self.publish_state('failed')
                return

            self.get_logger().info(f"开始执行'退出、切换楼层并更新位姿'任务: 地图={map_name}, 楼层={floor}, 点位={location}")
            success = self.execute_exit_and_update_pose(map_name, str(floor), location) # 调用更新后的函数
            
            if success:
                self.get_logger().info(f"已成功退出并更新位姿到 {location}")
                self.publish_state('completed')
            else:
                self.get_logger().error(f"执行'退出并更新位姿'失败")
                self.publish_state('failed')
                
        else:
            self.get_logger().warn(f"接收到未知指令: {command}")
    
    def execute_timed_move(self, duration: float, linear_v: float = 0.0, angular_v: float = 0.0):
        """以指定频率，在指定时间内持续发送joy_control指令"""
        period = 1.0 / self.send_rate_hz
        self.get_logger().info(f"  [动作] 持续 {duration:.2f}s, 线速度={linear_v}, 角速度={angular_v}")
        end_time = time.time() + duration
        while time.time() < end_time:
            # 使用joy_control实现盲走，不期待回复以提高频率
            self.robot_api.send_command(
                f"/api/joy_control?linear_velocity={linear_v}&angular_velocity={angular_v}", 
                silent=True
            )
            time.sleep(period)
        # 动作结束后发送停止指令
        self.robot_api.send_command("/api/joy_control?linear_velocity=0.0&angular_velocity=0.0", silent=True)
        self.get_logger().info("  [动作] 定时移动完成，已发送停止指令。")
        time.sleep(0.5) # 等待机器人完全停止

    def move_blind(self, distance: float, speed: float):
        """以固定速度前进/后退指定距离"""
        if speed == 0: return
        duration = abs(distance / speed)
        direction = 1.0 if distance > 0 else -1.0
        self.execute_timed_move(duration, linear_v=speed * direction)

    def turn_blind(self, angle_rad: float, turn_speed: float):
        """以固定角速度原地旋转指定角度（弧度）"""
        if turn_speed == 0: return
        duration = abs(angle_rad / turn_speed)
        direction = 1.0 if angle_rad > 0 else -1.0
        self.execute_timed_move(duration, angular_v=turn_speed * direction)

    def execute_motion_sequence(self, sequence_name: str) -> bool:
        """根据序列名称，执行一系列预设的盲走动作"""
        self.state = 'navigating' # 执行期间，将状态设为导航中
        self.motion_status = 'moving'
        self.publish_state('navigating')
        
        try:
            if sequence_name == 'enter_elevator':
                self.get_logger().info("  [序列] 正在执行 'enter_elevator'...")
                self.move_blind(distance=1.0, speed=self.move_speed) # 前进1.0米
                self.turn_blind(angle_rad=math.pi / 2, turn_speed=self.turn_speed) # 左转90度
                self.move_blind(distance=0.8, speed=self.move_speed) # 前进0.8米
                self.turn_blind(angle_rad=math.pi / 2, turn_speed=self.turn_speed) # 左转90度
                self.get_logger().info("  [序列] 'enter_elevator' 完成")
                return True

            elif sequence_name == 'panel_to_door':
                self.get_logger().info("  [序列] 正在执行 'panel_to_door'...")
                self.turn_blind(angle_rad=math.pi / 2, turn_speed=self.turn_speed) # 左转90度
                self.move_blind(distance=0.8, speed=self.move_speed) # 前进0.8米
                self.turn_blind(angle_rad=-math.pi / 2, turn_speed=self.turn_speed) # 右转90度
                self.get_logger().info("  [序列] 'panel_to_door' 完成")
                return True
            
            else:
                self.get_logger().error(f"未知的运动序列名称: {sequence_name}")
                return False
        except Exception as e:
            self.get_logger().error(f"执行运动序列时发生异常: {e}")
            return False
        finally:
            # 序列结束后，无论成功与否，都重置状态
            self.state = 'idle'
            self.motion_status = 'stopped'

    def execute_exit_and_update_pose(self, map_name: str, floor: str, location_marker: str) -> bool:
        """执行驶出电梯，切换楼层，并校正位置的完整流程"""
        self.state = 'navigating'
        self.motion_status = 'moving'
        self.publish_state('navigating')

        try:
            # 步骤 1: 盲走一小段距离以完全离开电梯
            self.get_logger().info("  [动作] 正在驶出电梯...")
            self.move_blind(distance=1.5, speed=self.move_speed)
            
            # 步骤 2: 调用API设置当前地图和楼层
            self.get_logger().info(f"  [API] 正在切换地图到 '{map_name}' 的楼层 '{floor}'...")
            # 注意：根据手册说明，此服务重启后可能收不到response，因此我们发送后等待一小段时间
            set_map_cmd = f"/api/map/set_current_map?map_name={map_name}&floor={floor}"
            self.robot_api.send_command(set_map_cmd, silent=True) # silent=True表示不严格等待uuid匹配的返回
            time.sleep(5) # 等待机器人服务重启和地图加载

            # 重新连接，因为设置地图会重启服务
            self.get_logger().info("  [系统] 尝试重新连接机器人...")
            if not self.connect_to_robot():
                self.get_logger().error("  [系统] 切换楼层后重新连接机器人失败！")
                return False

            # 步骤 3: 调用API校正当前位置到指定的marker
            self.get_logger().info(f"  [API] 在新楼层地图上，发送位置校正指令到 marker: {location_marker}")
            adjust_cmd = f"/api/position_adjust?marker={location_marker}"
            response = self.robot_api.send_command(adjust_cmd)
            
            if response and response.get("status") == "OK":
                self.get_logger().info("  [API] 位置校正成功！")
                return True
            else:
                error_msg = response.get('error_message', '无详细错误信息') if response else '无响应'
                self.get_logger().error(f"  [API] 位置校正失败: {error_msg}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"执行退出并更新位姿时发生异常: {e}")
            return False
        finally:
            self.state = 'idle'
            self.motion_status = 'stopped'

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