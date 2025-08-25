#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64MultiArray, Bool
from std_srvs.srv import SetBool
import math
import jkrc
import requests
import json
import time
import threading

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 机械臂和Body控制配置
        self.arm_ip = "192.168.10.90"
        self.body_api_url = "http://192.168.10.90:5000/api/extaxis"
        
        # JAKA机械臂连接
        self.arm_robot = None
        self.arm_connected = False
        self.arm_enabled = False
        
        # Body控制URLs
        self.body_urls = {
            'enable': self.body_api_url + "/enable",
            'reset': self.body_api_url + "/reset", 
            'moveto': self.body_api_url + "/moveto",
            'status': self.body_api_url + "/status",
            'sysinfo': self.body_api_url + "/sysinfo"
        }
        
        # 订阅关节状态（来自arm_pose_calculator）
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/arm/calculated_joints',
            self.joint_state_callback,
            10
        )
        
        # # 发布最终关节状态（给RViz等工具使用）
        # self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # 订阅机械臂关节控制命令（从ROS2话题接收6DOF机械臂控制）
        self.arm_joint_subscriber = self.create_subscription(
            Float64MultiArray,
            '/arm/joint_command',
            self.arm_joint_callback,
            10
        )
        
        # 订阅Body控制命令
        self.body_command_subscriber = self.create_subscription(
            Float64MultiArray,
            '/body/command',
            self.body_command_callback,
            10
        )
        
        # 发布机械臂状态
        self.arm_status_publisher = self.create_publisher(Bool, '/arm/status', 10)
        
        # 发布Body状态
        self.body_status_publisher = self.create_publisher(Float64MultiArray, '/body/status', 10)
        
        # 服务：机械臂使能/禁用
        self.arm_enable_service = self.create_service(
            SetBool,
            '/arm/enable',
            self.arm_enable_callback
        )
        
        # 服务：Body使能/禁用
        self.body_enable_service = self.create_service(
            SetBool,
            '/body/enable', 
            self.body_enable_callback
        )
        
        # 机械臂关节名称（基于URDF文件分析）
        self.joint_names = [
            'l_1',    # 升降关节 (prismatic)
            'l_2',    # 机械臂底座旋转 (revolute)
            'l_3',    # 机械臂第二关节 (revolute) 
            'l_4',    # 机械臂第三关节 (revolute) - 这个控制头部，需要小心控制
            'l_a1',   # 6自由度机械臂关节1 (revolute)
            'l_a2',   # 6自由度机械臂关节2 (revolute)
            'l_a3',   # 6自由度机械臂关节3 (revolute)
            'l_a4',   # 6自由度机械臂关节4 (revolute)
            'l_a5',   # 6自由度机械臂关节5 (revolute)
            'l_a6'    # 6自由度机械臂关节6 (revolute)
        ]
        
        # 当前关节位置
        self.current_joint_positions = [0.0] * len(self.joint_names)
        
        # 目标关节位置
        self.target_joint_positions = [0.0] * len(self.joint_names)
        
        # Body当前位置 [升降, 腰部旋转, 头部旋转, 头部俯仰]
        self.current_body_positions = [0.0, 0.0, 0.0, 0.0]
        
        # Body控制状态
        self.body_enabled = False
        self.body_last_command_time = 0
        self.body_command_interval = 0.5  # Body命令最小间隔(秒)
        
        # 机械臂控制状态
        self.last_arm_command_time = 0
        self.arm_command_interval = 0.1  # 机械臂命令最小间隔(秒)
        self.last_arm_positions = [0.0] * 6  # 上次发送给机械臂的关节位置
        
        # 创建定时器，以10Hz频率发布关节状态
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # 创建定时器，以1Hz频率更新状态
        self.status_timer = self.create_timer(1.0, self.status_update_loop)
        
        # 运动规划参数
        self.motion_speed = 0.5  # 关节运动速度 (rad/s 或 m/s)
        
        # 自动控制参数
        self.declare_parameter('auto_control_robot', True)  # 是否自动控制实际机器人
        self.auto_control = self.get_parameter('auto_control_robot').value
        
        # 控制模式参数
        self.declare_parameter('enable_interactive', False)
        self.interactive_mode = self.get_parameter('enable_interactive').value
        
        # 初始化机械臂连接
        self.init_arm_connection()
        
        # 如果启用自动控制，尝试使能机器人
        if self.auto_control and self.arm_connected:
            self.auto_enable_robot()
            # 测试Body连接并尝试使能
            if self.test_body_connection():
                self.auto_enable_body()
            else:
                self.get_logger().warn('Body系统连接失败，Body控制将被禁用')
        
        if self.interactive_mode:
            self.get_logger().info('机械臂控制器已启动 - 交互模式')
            # 在单独线程中启动交互控制
            import threading
            self.control_thread = threading.Thread(target=self.interactive_control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
        else:
            mode_info = "自动控制模式" if self.auto_control else "服务模式"
            self.get_logger().info(f'机械臂控制器已启动 - {mode_info}')
    
    def init_arm_connection(self):
        """初始化机械臂连接"""
        try:
            self.arm_robot = jkrc.RC(self.arm_ip)
            self.arm_robot.login()
            self.arm_connected = True
            self.get_logger().info(f'机械臂连接成功: {self.arm_ip}')
        except Exception as e:
            self.get_logger().error(f'机械臂连接失败: {e}')
            self.arm_connected = False
    
    def auto_enable_robot(self):
        """自动使能机器人"""
        try:
            if not self.arm_connected:
                return
            
            self.arm_robot.power_on()
            time.sleep(0.5)
            self.arm_robot.enable_robot()
            self.arm_enabled = True
            self.get_logger().info('机器人已自动使能，准备接收控制命令')
        except Exception as e:
            self.get_logger().warn(f'自动使能失败: {e}，请手动使能机器人')
    
    def auto_enable_body(self):
        """自动使能Body"""
        try:
            # 先复位Body
            reset_response = requests.post(self.body_urls['reset'], json={}, timeout=3)
            if reset_response.status_code == 200:
                self.get_logger().info('Body系统已复位')
            
            time.sleep(0.5)
            
            # 使能Body
            enable_data = {"enable": 1}
            enable_response = requests.post(self.body_urls['enable'], json=enable_data, timeout=3)
            
            if enable_response.status_code == 200:
                self.body_enabled = True
                self.get_logger().info('Body系统已自动使能')
            else:
                self.get_logger().warn(f'Body自动使能失败: HTTP {enable_response.status_code}')
                
        except Exception as e:
            self.get_logger().warn(f'Body自动使能失败: {e}')
    
    def test_body_connection(self):
        """测试Body连接"""
        try:
            response = requests.get(self.body_urls['sysinfo'], timeout=2)
            if response.status_code == 200:
                self.get_logger().info('Body系统连接正常')
                return True
            else:
                self.get_logger().warn(f'Body系统响应异常: HTTP {response.status_code}')
                return False
        except Exception as e:
            self.get_logger().error(f'Body系统连接失败: {e}')
            return False
    
    def arm_enable_callback(self, request, response):
        """机械臂使能/禁用服务回调"""
        try:
            if not self.arm_connected:
                self.init_arm_connection()
            
            if not self.arm_connected:
                response.success = False
                response.message = "机械臂未连接"
                return response
            
            if request.data:  # 使能
                self.arm_robot.power_on()
                time.sleep(0.5)
                self.arm_robot.enable_robot()
                self.arm_enabled = True
                response.message = "机械臂已使能"
            else:  # 禁用
                self.arm_robot.disable_robot()
                time.sleep(0.5) 
                self.arm_robot.power_off()
                self.arm_enabled = False
                response.message = "机械臂已禁用"
            
            response.success = True
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"机械臂操作失败: {e}"
            self.get_logger().error(response.message)
        
        return response
    
    def joint_state_callback(self, msg):
        """接收来自arm_pose_calculator的关节状态"""
        if len(msg.position) == len(self.joint_names):
            self.current_joint_positions = list(msg.position)
            self.target_joint_positions = list(msg.position)  # 更新目标位置
            
            # 发布关节状态给RViz等工具
            # self.publish_joint_states()
            
            # 如果启用自动控制，立即更新Body部分（升降、旋转等）
            if self.auto_control:
                self.update_body_immediately(self.current_joint_positions)
    
    # def publish_joint_states(self):
    #     """发布当前关节状态给RViz等工具"""
    #     joint_state = JointState()
    #     joint_state.header = Header()
    #     joint_state.header.stamp = self.get_clock().now().to_msg()
    #     joint_state.header.frame_id = ''
        
    #     joint_state.name = self.joint_names
    #     joint_state.position = self.current_joint_positions
    #     joint_state.velocity = [0.0] * len(self.joint_names)
    #     joint_state.effort = [0.0] * len(self.joint_names)
        
    #     self.joint_publisher.publish(joint_state)
    
    def arm_joint_callback(self, msg):
        """接收机械臂关节控制命令"""
        if not self.arm_enabled:
            self.get_logger().warn("机械臂未使能，忽略关节命令")
            return
        
        if len(msg.data) != 6:
            self.get_logger().error("机械臂关节命令必须包含6个关节角度")
            return
        
        try:
            # 运动模式：ABS = 0（绝对位置）
            joint_positions = list(msg.data).reverse()
            self.arm_robot.joint_move(joint_positions, 0, True, 1.0)
            self.get_logger().info(f'机械臂关节运动命令: {[f"{j:.3f}" for j in joint_positions]}')
        except Exception as e:
            self.get_logger().error(f'机械臂关节运动失败: {e}')
    
    def body_enable_callback(self, request, response):
        """Body使能/禁用服务回调"""
        try:
            if request.data:
                # 先复位再使能
                reset_response = requests.post(self.body_urls['reset'], json={}, timeout=3)
                time.sleep(0.2)
            
            enable_data = {"enable": 1 if request.data else 0}
            response_req = requests.post(self.body_urls['enable'], json=enable_data, timeout=3)
            
            if response_req.status_code == 200:
                self.body_enabled = request.data
                response.success = True
                response.message = f"Body{'已使能' if request.data else '已禁用'}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"Body操作失败: HTTP {response_req.status_code}"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Body操作失败: {e}"
            self.get_logger().error(response.message)
        
        return response
    
    def body_command_callback(self, msg):
        """接收Body控制命令 [升降, 腰部旋转, 头部旋转, 头部俯仰]"""
        if len(msg.data) != 4:
            self.get_logger().error("Body命令必须包含4个值: [升降, 腰部旋转, 头部旋转, 头部俯仰]")
            return
        
        try:
            # 限制范围
            lift = max(0, min(300, msg.data[0]))      # 升降范围[0,300]mm
            waist = max(-140, min(140, msg.data[1]))  # 腰部旋转[-140,140]度
            head_yaw = max(-180, min(180, msg.data[2]))  # 头部旋转[-180,180]度
            head_pitch = max(-5, min(35, msg.data[3]))   # 头部俯仰[-5,35]度
            
            move_data = {
                "pos": [lift, waist, head_yaw, head_pitch],
                "vel": 100,
                "acc": 100
            }
            
            response = requests.post(self.body_urls['moveto'], json=move_data, timeout=5)
            
            if response.status_code == 200:
                self.get_logger().info(f'Body运动命令: 升降={lift}, 腰部={waist}, 头部yaw={head_yaw}, 头部pitch={head_pitch}')
            else:
                self.get_logger().error(f'Body运动失败: HTTP {response.status_code}')
                
        except Exception as e:
            self.get_logger().error(f'Body运动失败: {e}')
    
    def status_update_loop(self):
        """状态更新循环"""
        # 发布机械臂状态
        arm_status = Bool()
        arm_status.data = self.arm_enabled
        self.arm_status_publisher.publish(arm_status)
        
        # 获取并发布Body状态
        try:
            response = requests.get(self.body_urls['status'], timeout=2)
            if response.status_code == 200:
                status_data = json.loads(response.text)
                if len(status_data) >= 4:
                    body_status = Float64MultiArray()
                    body_status.data = [
                        float(status_data[0].get('pos', 0)),  # 升降
                        float(status_data[1].get('pos', 0)),  # 腰部旋转
                        float(status_data[2].get('pos', 0)),  # 头部旋转
                        float(status_data[3].get('pos', 0))   # 头部俯仰
                    ]
                    self.current_body_positions = body_status.data
                    self.body_status_publisher.publish(body_status)
        except Exception as e:
            if hasattr(self, '_last_body_error_time'):
                if time.time() - self._last_body_error_time > 10:  # 每10秒记录一次错误
                    self.get_logger().warn(f'获取Body状态失败: {e}')
                    self._last_body_error_time = time.time()
            else:
                self._last_body_error_time = time.time()
    
    def interactive_control_loop(self):
        """交互式控制循环"""
        time.sleep(2)  # 等待系统初始化
        self.print_interactive_help()
        
        while rclpy.ok():
            try:
                command = input("\n请输入命令 (help获取帮助): ").strip().split()
                if not command:
                    continue
                
                cmd = command[0].lower()
                
                if cmd == 'quit' or cmd == 'exit':
                    self.get_logger().info('退出交互模式...')
                    break
                elif cmd == 'help':
                    self.print_interactive_help()
                elif cmd == 'arm_enable':
                    self.interactive_enable_arm(True)
                elif cmd == 'arm_disable':
                    self.interactive_enable_arm(False)
                elif cmd == 'body_enable':
                    self.interactive_enable_body(True)
                elif cmd == 'body_disable':
                    self.interactive_enable_body(False)
                elif cmd == 'arm':
                    if len(command) == 7:
                        self.interactive_arm_move(command[1:])
                    else:
                        print('错误: arm命令需要6个关节角度')
                elif cmd == 'body':
                    if len(command) == 5:
                        self.interactive_body_move(command[1:])
                    else:
                        print('错误: body命令需要4个位置值')
                elif cmd == 'preset':
                    if len(command) == 2:
                        self.interactive_preset(command[1])
                    else:
                        print('错误: preset命令需要指定预设类型')
                elif cmd == 'status':
                    self.interactive_show_status()
                else:
                    print(f'错误: 未知命令 {cmd}，输入help获取帮助')
                    
            except KeyboardInterrupt:
                print("\n收到中断信号，退出交互模式...")
                break
            except EOFError:
                print("\n输入结束，退出交互模式...")
                break
            except Exception as e:
                print(f'错误: {e}')
    
    def print_interactive_help(self):
        """打印交互模式帮助信息"""
        print("\n=== 机械臂控制器 - 交互模式 ===")
        print("命令格式:")
        print("  arm_enable     - 使能机械臂")
        print("  arm_disable    - 禁用机械臂")
        print("  body_enable    - 使能Body")
        print("  body_disable   - 禁用Body")
        print("  arm j1 j2 j3 j4 j5 j6  - 机械臂关节运动 (弧度)")
        print("  body lift waist head_yaw head_pitch  - Body运动")
        print("    lift: [0,300]mm, waist: [-140,140]°")
        print("    head_yaw: [-180,180]°, head_pitch: [-5,35]°")
        print("  preset arm_home  - 机械臂回到初始位置")
        print("  preset body_home - Body回到初始位置")
        print("  status         - 显示当前状态")
        print("  help           - 显示此帮助")
        print("  quit/exit      - 退出交互模式")
        print("============================")
        print("注意: 交互模式下ROS2话题和服务仍然正常工作")
    
    def interactive_enable_arm(self, enable):
        """交互模式：使能/禁用机械臂"""
        try:
            if not self.arm_connected:
                self.init_arm_connection()
            
            if not self.arm_connected:
                print("错误: 机械臂未连接")
                return
            
            if enable:
                self.arm_robot.power_on()
                time.sleep(0.5)
                self.arm_robot.enable_robot()
                self.arm_enabled = True
                print("✓ 机械臂已使能")
            else:
                self.arm_robot.disable_robot()
                time.sleep(0.5)
                self.arm_robot.power_off()
                self.arm_enabled = False
                print("✓ 机械臂已禁用")
                
        except Exception as e:
            print(f"错误: 机械臂操作失败 - {e}")
    
    def interactive_enable_body(self, enable):
        """交互模式：使能/禁用Body"""
        try:
            enable_data = {"enable": 1 if enable else 0}
            response = requests.post(self.body_urls['enable'], json=enable_data, timeout=5)
            
            if response.status_code == 200:
                print(f"✓ Body{'已使能' if enable else '已禁用'}")
            else:
                print(f"错误: Body操作失败 - HTTP {response.status_code}")
                
        except Exception as e:
            print(f"错误: Body操作失败 - {e}")
    
    def interactive_arm_move(self, joint_angles):
        """交互模式：机械臂关节运动"""
        if not self.arm_enabled:
            print("错误: 机械臂未使能")
            return
        
        try:
            angles = [float(a) for a in joint_angles]
            self.arm_robot.joint_move(angles, 0, True, 1.0)
            print(f"✓ 机械臂运动命令已发送: {[f'{j:.3f}' for j in angles]}")
        except ValueError:
            print("错误: 关节角度必须是数字")
        except Exception as e:
            print(f"错误: 机械臂运动失败 - {e}")
    
    def interactive_body_move(self, positions):
        """交互模式：Body运动"""
        try:
            pos = [float(p) for p in positions]
            
            # 检查范围
            lift = max(0, min(300, pos[0]))
            waist = max(-140, min(140, pos[1]))
            head_yaw = max(-180, min(180, pos[2]))
            head_pitch = max(-5, min(35, pos[3]))
            
            if pos[0] != lift or pos[1] != waist or pos[2] != head_yaw or pos[3] != head_pitch:
                print("警告: 部分值超出范围，已自动限制")
            
            move_data = {
                "pos": [lift, waist, head_yaw, head_pitch],
                "vel": 100,
                "acc": 100
            }
            
            response = requests.post(self.body_urls['moveto'], json=move_data, timeout=5)
            
            if response.status_code == 200:
                print(f"✓ Body运动命令已发送: 升降={lift}, 腰部={waist}, 头部yaw={head_yaw}, 头部pitch={head_pitch}")
            else:
                print(f"错误: Body运动失败 - HTTP {response.status_code}")
                
        except ValueError:
            print("错误: 位置值必须是数字")
        except Exception as e:
            print(f"错误: Body运动失败 - {e}")
    
    def interactive_preset(self, preset_type):
        """交互模式：预设动作"""
        if preset_type == 'arm_home':
            if not self.arm_enabled:
                print("错误: 机械臂未使能")
                return
            
            try:
                joint_home = [-math.pi/2.0, math.pi/2.0, 0, 0, 0, 0]
                self.arm_robot.joint_move(joint_home, 0, True, 1.0)
                print("✓ 机械臂回到初始位置")
            except Exception as e:
                print(f"错误: 机械臂回初始位置失败 - {e}")
                
        elif preset_type == 'body_home':
            try:
                move_data = {
                    "pos": [100, 0, 0, 0],
                    "vel": 100,
                    "acc": 100
                }
                response = requests.post(self.body_urls['moveto'], json=move_data, timeout=5)
                
                if response.status_code == 200:
                    print("✓ Body回到初始位置")
                else:
                    print(f"错误: Body回初始位置失败 - HTTP {response.status_code}")
            except Exception as e:
                print(f"错误: Body回初始位置失败 - {e}")
        else:
            print(f"错误: 未知预设 {preset_type}")
    
    def interactive_show_status(self):
        """交互模式：显示状态"""
        print(f"\n=== 当前状态 ===")
        print(f"机械臂连接: {'✓' if self.arm_connected else '✗'}")
        print(f"机械臂使能: {'✓' if self.arm_enabled else '✗'}")
        print(f"Body状态: {self.current_body_positions}")
        print(f"机械臂关节: {[f'{j:.3f}' for j in self.current_joint_positions]}")
        print("================\n")
    
    def update_body_immediately(self, joint_positions):
        """立即更新Body部分的位置"""
        # 检查Body是否使能
        if not self.body_enabled:
            self.get_logger().debug('Body未使能，跳过Body控制')
            return
        
        # 限制调用频率，避免频繁发送命令
        current_time = time.time()
        if current_time - self.body_last_command_time < self.body_command_interval:
            return
        
        try:
            # 提取Body相关的关节 (l_1升降, l_2底座旋转)
            lift_position = joint_positions[0] * 1000  # 转换为mm
            waist_rotation = math.degrees(joint_positions[1])  # 转换为度
            
            # 限制范围
            lift = max(0, min(300, lift_position))
            waist = max(-140, min(140, waist_rotation))
            
            # 检查变化量，避免微小变化导致的频繁调用
            lift_change = abs(lift - self.current_body_positions[0])
            waist_change = abs(waist - self.current_body_positions[1])
            
            if lift_change < 5 and waist_change < 2:  # 升降<5mm, 旋转<2度则跳过
                return
            
            # 发送Body控制命令，使用较短的超时和较慢的速度
            move_data = {
                "pos": [lift, waist, 0, 0],  # 头部保持中性
                "vel": 50,   # 较慢的速度，避免过快运动
                "acc": 50
            }
            
            response = requests.post(self.body_urls['moveto'], json=move_data, timeout=3)
            if response.status_code == 200:
                self.body_last_command_time = current_time
                self.get_logger().info(f'Body位置更新: 升降={lift:.1f}mm, 腰部旋转={waist:.1f}°')
            else:
                self.get_logger().warn(f'Body位置更新失败: HTTP {response.status_code}')
            
        except requests.exceptions.Timeout:
            self.get_logger().warn('Body控制超时，可能Body系统响应慢')
        except requests.exceptions.ConnectionError:
            self.get_logger().warn('Body连接失败，检查网络连接到192.168.10.90:5000')
            self.body_enabled = False  # 标记为未使能，需要重新使能
        except Exception as e:
            self.get_logger().warn(f'Body位置更新失败: {e}')
    
    def control_loop(self):
        """控制循环，主要负责实际机器人控制"""
        # 如果启用自动控制且机械臂使能，控制实际机器人
        if self.auto_control and self.arm_enabled:
            self.control_real_robot()
    
    def control_real_robot(self):
        """控制实际机器人，使其与关节状态一致"""
        try:
            if not self.arm_connected or not self.arm_enabled:
                return
            
            # 限制调用频率，避免频繁发送命令
            current_time = time.time()
            if current_time - self.last_arm_command_time < self.arm_command_interval:
                return
            
            # 提取6DOF机械臂关节角度 (l_a1 到 l_a6)
            if len(self.current_joint_positions) >= 10:
                arm_joints = self.current_joint_positions[4:10]  # 索引4-9对应l_a1到l_a6
                
                # 检查关节角度是否有效
                if len(arm_joints) == 6:
                    # 检查与上次位置的差异，避免发送相同的命令
                    max_change = max(abs(arm_joints[i] - self.last_arm_positions[i]) for i in range(6))
                    
                    if max_change > 0.01:  # 只有变化超过阈值才发送命令
                        # 发送关节角度给实际机器人
                        self.arm_robot.joint_move(arm_joints, 0, False, 0.8)  # 非阻塞模式，速度0.8
                        self.last_arm_positions = arm_joints.copy()
                        self.last_arm_command_time = current_time
                        self.get_logger().debug(f'实际机器人关节控制: {[f"{j:.3f}" for j in arm_joints]}')
                
        except Exception as e:
            self.get_logger().error(f'控制实际机器人失败: {e}')
    
    def control_body_from_joints(self):
        """根据关节状态控制Body部分"""
        try:
            # 从关节状态中提取Body控制信息
            lift_position = self.current_joint_positions[0] * 1000  # l_1转换为mm
            waist_rotation = math.degrees(self.current_joint_positions[1])  # l_2转换为度
            head_rotation = math.degrees(self.current_joint_positions[3])   # l_4转换为度
            
            # 限制范围
            lift = max(0, min(300, lift_position))
            waist = max(-140, min(140, waist_rotation))
            head_yaw = max(-180, min(180, head_rotation))
            head_pitch = 0  # 头部俯仰保持中性
            
            # 发送Body控制命令
            move_data = {
                "pos": [lift, waist, head_yaw, head_pitch],
                "vel": 50,   # 较慢的速度
                "acc": 50
            }
            
            response = requests.post(self.body_urls['moveto'], json=move_data, timeout=2)
            if response.status_code == 200:
                self.get_logger().debug(f'Body同步控制: 升降={lift:.1f}, 腰部={waist:.1f}°')
            
        except Exception as e:
            self.get_logger().debug(f'Body同步控制失败: {e}')
    
    def cleanup(self):
        """清理资源"""
        try:
            if self.arm_connected and self.arm_robot:
                if self.arm_enabled:
                    self.arm_robot.disable_robot()
                    self.arm_robot.power_off()
                self.arm_robot.logout()
                self.get_logger().info('机械臂连接已断开')
        except Exception as e:
            self.get_logger().error(f'机械臂断开失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = ArmController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在关闭...')
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
