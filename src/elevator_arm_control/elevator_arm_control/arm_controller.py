#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Header, Float64MultiArray
from std_srvs.srv import SetBool
import math
import jkrc
import requests
import json
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # 机械臂和Body控制配置
        self.arm_ip = "192.168.10.90"
        
        # JAKA机械臂连接
        self.arm_robot = None
        self.arm_connected = False
        self.arm_enabled = False
        
        # Body控制配置
        self.body_api_url = "http://192.168.10.90:5000/api/extaxis"
        self.body_enabled = False
        
        # 订阅由MoveIt2生成的轨迹
        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory,
            '/jaka_arm_controller/joint_trajectory', 
            self.execute_trajectory_callback,
            10
        )
        
        # 订阅body控制器的轨迹
        self.body_trajectory_subscriber = self.create_subscription(
            JointTrajectory,
            '/body_controller/joint_trajectory',
            self.execute_body_trajectory_callback,
            10
        )
        
        # 订阅来自trajectory_controller的关节命令
        self.joint_command_subscriber = self.create_subscription(
            Float64MultiArray,
            '/arm/joint_command',
            self.joint_command_callback,
            10
        )
        
        # 发布最终关节状态 (给RViz和MoveIt使用)
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # 服务：机械臂使能/禁用
        self.arm_enable_service = self.create_service(
            SetBool,
            '/arm/enable',
            self.arm_enable_callback
        )
        
        # 机械臂关节名称
        self.joint_names = [
            'w_r', 'w_l', 'l_1', 'l_2', 'l_3', 'l_4', 'cm3_l', 'l_a1', 
            'l_a2', 'l_a3', 'l_a4', 'l_a5', 'l_a6', 'l_at', 'cm2_l', 
            'radar_l', 'cm1_l', 'pw1_1', 'pw1_2', 'pw2_1', 'pw2_2', 
            'pw3_1', 'pw3_2', 'pw4_1', 'pw4_2'
        ]
        
        # 完整的当前关节位置
        self.current_joint_positions = [0.0] * len(self.joint_names)
        
        # 记录上次发送给机械臂的命令
        self.last_joint_command = [0.0] * 6
        self.last_command_time = 0.0
        
        # 添加位置稳定性检查
        self.position_history = []  # 存储最近几次的位置读取
        self.stable_position = [0.0] * 6  # 稳定的位置
        self.position_read_errors = 0  # 连续读取错误计数
        
        # 初始化机械臂连接
        self.init_arm_connection()
        self.auto_enable_robot()

        # 初始化body控制
        self.init_body_control()

        # 创建一个定时器，高频更新和发布关节状态
        self.status_timer = self.create_timer(0.1, self.update_and_publish_status)
        
        self.get_logger().info('机械臂控制器已启动 - MoveIt2接口模式 (带状态发布)')

    def init_arm_connection(self):
        try:
            self.arm_robot = jkrc.RC(self.arm_ip)
            self.arm_robot.login()
            self.arm_connected = True
            self.get_logger().info(f'机械臂连接成功: {self.arm_ip}')
        except Exception as e:
            self.get_logger().error(f'机械臂连接失败: {e}')
            self.arm_connected = False
    
    def auto_enable_robot(self):
        if not self.arm_connected: return
        try:
            self.arm_robot.power_on()
            time.sleep(0.5)
            self.arm_robot.enable_robot()
            
            # 停止任何正在进行的运动
            try:
                self.arm_robot.stop_move()
                time.sleep(0.5)
                self.get_logger().info('已停止机械臂运动')
            except:
                pass  # 如果没有运动在进行，这个命令可能会失败，忽略错误
            
            self.arm_enabled = True
            self.get_logger().info('机器人已自动使能')
        except Exception as e:
            self.get_logger().warn(f'自动使能失败: {e}')
    
    def init_body_control(self):
        """初始化body控制"""
        try:
            # 先检查body服务是否可用
            status_url = f"{self.body_api_url}/status"
            response = requests.get(status_url, timeout=2)
            if response.status_code == 200:
                self.get_logger().info('Body服务连接成功')
                
                # 启用body控制
                enable_url = f"{self.body_api_url}/enable"
                response = requests.post(enable_url, json={"enable": 1}, timeout=10)
                if response.status_code == 200:
                    self.body_enabled = True
                    self.get_logger().info('Body控制已启用')
                else:
                    self.get_logger().warn(f'Body启用失败: HTTP {response.status_code}')
            else:
                self.get_logger().warn(f'Body服务不可用: HTTP {response.status_code}')
        except Exception as e:
            self.get_logger().warn(f'Body控制初始化失败: {e}，将跳过body控制')
            self.body_enabled = False
    
    def send_body_command(self, l_1_pos, l_2_pos):
        """发送body关节命令"""
        if not self.body_enabled:
            self.get_logger().warn('Body未启用，忽略命令')
            return False
        
        try:
            # l_1: prismatic关节，输入是米，需要转换为mm
            l_1_mm = l_1_pos * 1000.0  # 米转换为毫米
            l_1_mm = max(0, min(400, l_1_mm))  # 限制范围0-400mm (对应URDF的0-0.4m)
            
            # l_2: 腰部旋转，弧度转度数，直接映射
            l_2_deg = l_2_pos * 180.0 / math.pi
            l_2_deg = max(-160, min(160, l_2_deg))  # 限制范围（约对应±2.79弧度）
            
            moveto_url = f"{self.body_api_url}/moveto"
            # 发送4个关节位置：[升降, 腰部旋转, 头部旋转, 头部俯仰]
            # 只控制前两个，后两个保持0
            payload = {
                "pos": [l_1_mm, l_2_deg, 0.0, 0.0],
                "vel": 50,  # 降低速度避免超时
                "acc": 50   # 降低加速度
            }
            
            self.get_logger().info(f'发送Body命令: l_1={l_1_pos:.3f}m->{l_1_mm:.1f}mm, l_2={l_2_pos:.3f}rad->{l_2_deg:.1f}°')
            
            # 增加超时时间，并添加重试机制
            max_retries = 2
            for attempt in range(max_retries):
                try:
                    response = requests.post(moveto_url, json=payload, timeout=15)  # 增加到15秒
                    if response.status_code == 200:
                        self.get_logger().info(f'Body命令发送成功 (尝试 {attempt+1})')
                        return True
                    else:
                        self.get_logger().warn(f'Body命令HTTP错误 {response.status_code} (尝试 {attempt+1})')
                        if attempt < max_retries - 1:
                            time.sleep(1)  # 重试前等待
                except requests.exceptions.Timeout:
                    self.get_logger().warn(f'Body命令超时 (尝试 {attempt+1}/{max_retries})')
                    if attempt < max_retries - 1:
                        time.sleep(1)  # 重试前等待
                except Exception as e:
                    self.get_logger().warn(f'Body命令异常 (尝试 {attempt+1}): {e}')
                    if attempt < max_retries - 1:
                        time.sleep(1)
            
            return False
                
        except Exception as e:
            self.get_logger().error(f'Body命令发送异常: {e}')
            return False
    
    def reconnect_and_enable(self):
        """重新连接并使能机械臂"""
        try:
            # 清理现有连接
            if self.arm_robot:
                try:
                    self.arm_robot.logout()
                except:
                    pass
            
            # 重新连接
            self.arm_robot = jkrc.RC(self.arm_ip)
            self.arm_robot.login()
            self.arm_connected = True
            
            # 重新使能
            self.arm_robot.power_on()
            time.sleep(0.5)
            self.arm_robot.enable_robot()
            self.arm_enabled = True
            
            self.get_logger().info('机械臂重新连接并使能成功')
            
        except Exception as e:
            self.get_logger().error(f'机械臂重新连接失败: {e}')
            self.arm_connected = False
            self.arm_enabled = False
    
    def arm_enable_callback(self, request, response):
        response.success = True
        return response

    def execute_trajectory_callback(self, msg):
        if not self.arm_enabled:
            self.get_logger().warn("机械臂未使能，忽略轨迹命令")
            return

        self.get_logger().info(f"收到新的轨迹，包含 {len(msg.points)} 个路径点")
        # 简单的轨迹执行：直接运动到最后一个点
        if msg.points:
            final_point = msg.points[-1]
            arm_joint_names = [name for name in msg.joint_names if name.startswith('l_a')]
            
            if len(arm_joint_names) == 6:
                try:
                    target_positions = final_point.positions
                    self.get_logger().info(f"正在移动到目标关节位置: {target_positions}")
                    self.arm_robot.joint_move(list(target_positions), 0, True, 1.0) # 阻塞模式
                except Exception as e:
                    self.get_logger().error(f"机械臂运动失败: {e}")
            else:
                self.get_logger().error("轨迹中的关节数量不是6个！")
        self.get_logger().info("轨迹执行完成")

    def execute_body_trajectory_callback(self, msg):
        """执行body轨迹"""
        self.get_logger().info(f"收到body轨迹，包含 {len(msg.points)} 个路径点")
        
        if not msg.points:
            self.get_logger().warn("Body轨迹为空")
            return
            
        # 检查关节名称
        if 'l_1' not in msg.joint_names or 'l_2' not in msg.joint_names:
            self.get_logger().error("Body轨迹必须包含l_1和l_2关节")
            return
            
        try:
            l_1_idx = msg.joint_names.index('l_1')
            l_2_idx = msg.joint_names.index('l_2')
            
            # 执行轨迹中的每个点
            for i, point in enumerate(msg.points):
                l_1_pos = point.positions[l_1_idx]
                l_2_pos = point.positions[l_2_idx]
                
                self.get_logger().info(f"Body轨迹点 {i+1}/{len(msg.points)}: l_1={l_1_pos:.3f}m, l_2={l_2_pos:.3f}rad")
                
                # 发送body命令
                success = self.send_body_command(l_1_pos, l_2_pos)
                if not success:
                    self.get_logger().warn(f"Body轨迹点 {i+1} 执行失败")
                
                # 更新关节状态
                l_1_full_idx = self.joint_names.index('l_1')
                l_2_full_idx = self.joint_names.index('l_2')
                self.current_joint_positions[l_1_full_idx] = l_1_pos
                self.current_joint_positions[l_2_full_idx] = l_2_pos
                
                # 轨迹点间等待
                if i < len(msg.points) - 1:
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f"Body轨迹执行失败: {e}")
            
        self.get_logger().info("Body轨迹执行完成")

    def update_and_publish_status(self):
        """获取机器人状态并发布 /joint_states"""
        if self.arm_connected and self.arm_enabled:
            # 使用改进的关节位置获取方法，带重试机制
            arm_positions = self.get_current_joint_positions()
            
            # 转换实际机器人角度到URDF坐标系角度
            urdf_positions = self.convert_robot_to_urdf_angles(arm_positions)
            
            # 更新完整关节列表中的对应部分
            # 假设l_a1到l_a6在self.joint_names中的索引是7到12
            for i in range(6):
                self.current_joint_positions[7 + i] = urdf_positions[i]

        # 更新body状态
        if self.body_enabled:
            self.update_and_publish_body_status()

        # 发布完整的关节状态
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_joint_positions
        self.joint_publisher.publish(joint_state_msg)
    
    def update_and_publish_body_status(self):
        """更新body状态"""
        if not self.body_enabled:
            return
            
        try:
            # 获取body状态
            status_url = f"{self.body_api_url}/status"
            response = requests.get(status_url, timeout=2)
            if response.status_code != 200:
                self.get_logger().warn(f"获取Body状态失败: HTTP {response.status_code}")
                return

            parsed_json = response.json()
            self.get_logger().debug(f"Body状态响应: {parsed_json}")
            
            # 根据实际API响应格式解析
            # 假设返回格式为: {"pos": [高度_mm, 旋转_deg, ...]} 或 [{"pos": 高度_mm}, {"pos": 旋转_deg}, ...]
            if isinstance(parsed_json, dict) and "pos" in parsed_json:
                # 格式1: {"pos": [value1, value2, ...]}
                positions = parsed_json["pos"]
                l_1_mm = positions[0] if len(positions) > 0 else 0.0
                l_2_deg = positions[1] if len(positions) > 1 else 0.0
            elif isinstance(parsed_json, list) and len(parsed_json) >= 2:
                # 格式2: [{"pos": value1}, {"pos": value2}, ...]
                l_1_mm = parsed_json[0].get("pos", 0.0) if isinstance(parsed_json[0], dict) else 0.0
                l_2_deg = parsed_json[1].get("pos", 0.0) if isinstance(parsed_json[1], dict) else 0.0
            else:
                self.get_logger().warn(f"Body状态格式不正确: {parsed_json}")
                return

            # 转换为关节空间
            # l_1: prismatic关节，API返回mm，URDF需要米
            l_1_rad = l_1_mm / 1000.0  # mm转换为米
            l_2_rad = l_2_deg * math.pi / 180.0  # deg转rad

            # 限制关节值在URDF定义的范围内
            l_1_rad = max(0.0, min(0.4, l_1_rad))  # l_1范围: 0-0.4米
            l_2_rad = max(-2.7925, min(2.7925, l_2_rad))  # l_2范围: ±2.7925弧度

            # 更新关节状态
            l_1_idx = self.joint_names.index('l_1')
            l_2_idx = self.joint_names.index('l_2')
            self.current_joint_positions[l_1_idx] = l_1_rad
            self.current_joint_positions[l_2_idx] = l_2_rad

            self.get_logger().debug(f"Body状态更新: l_1={l_1_mm}mm->({l_1_rad:.3f}m), l_2={l_2_deg}deg->({l_2_rad:.3f}rad)")

        except requests.exceptions.Timeout:
            self.get_logger().warn("获取Body状态超时")
        except Exception as e:
            self.get_logger().warn(f"获取Body状态异常: {e}")

    def get_current_joint_positions(self):
        """获取当前关节位置，带重试和错误处理以及稳定性过滤"""
        if not self.arm_connected or not self.arm_enabled:
            self.get_logger().warn('机械臂未连接或未使能，返回默认位置')
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        max_retries = 3
        for attempt in range(max_retries):
            try:
                ret = self.arm_robot.get_joint_position()
                
                # 详细日志记录
                self.get_logger().debug(f'尝试 {attempt + 1}: get_joint_position() 返回: {ret}')
                
                # 检查返回值格式
                if ret is None:
                    self.get_logger().warn(f'尝试 {attempt + 1}: get_joint_position() 返回 None')
                elif isinstance(ret, (list, tuple)) and len(ret) >= 2:
                    # JAKA API 通常返回 (error_code, joint_positions) 格式
                    error_code = ret[0]
                    if error_code == 0 and len(ret[1]) == 6:
                        # 成功获取关节位置，应用稳定性过滤
                        current_pos = list(ret[1])
                        return self._apply_position_stability_filter(current_pos)
                    else:
                        self.get_logger().warn(f'尝试 {attempt + 1}: API错误码: {error_code}, 数据: {ret[1] if len(ret) > 1 else "无数据"}')
                else:
                    self.get_logger().warn(f'尝试 {attempt + 1}: 意外的返回格式: {ret}')
                
                # 如果不是最后一次尝试，短暂等待后重试
                if attempt < max_retries - 1:
                    time.sleep(0.1)
                    
            except Exception as e:
                self.get_logger().warn(f'尝试 {attempt + 1}: 获取关节位置异常: {e}')
                if attempt < max_retries - 1:
                    time.sleep(0.1)
        
        # 所有重试都失败，尝试重新连接
        self.get_logger().error('所有重试失败，尝试重新连接机械臂')
        self.reconnect_and_enable()
        
        # 如果有稳定的历史位置，返回它
        if self.stable_position is not None:
            return self.stable_position
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def convert_robot_to_urdf_angles(self, robot_angles):
        if len(robot_angles) != 6:
            self.get_logger().error(f"期望6个关节角度，但收到{len(robot_angles)}个")
            return [0.0] * 6
        
        urdf_angles = [0.0] * 6
        
        # 基于URDF中的rpy偏移调整关节角度
        # 这些偏移量根据URDF定义计算得出
        
        # l_a1: 有180°和90°的旋转偏移，可能需要反向旋转
        urdf_angles[0] = robot_angles[0]  
        urdf_angles[1] = robot_angles[1]  
        urdf_angles[2] = robot_angles[2]
        urdf_angles[3] = robot_angles[3] - math.pi 
        urdf_angles[4] = robot_angles[4] 
        urdf_angles[5] = robot_angles[5]  
        self.get_logger().debug(f"关节角度转换: 机器人 {[f'{j:.3f}' for j in robot_angles]} -> URDF {[f'{j:.3f}' for j in urdf_angles]}")
        return urdf_angles

    def convert_urdf_to_robot_angles(self, urdf_angles):
        """
        将URDF坐标系中的角度转换为实际机器人的关节角度
        这是convert_robot_to_urdf_angles的逆变换
        """
        if len(urdf_angles) != 6:
            self.get_logger().error(f"期望6个关节角度，但收到{len(urdf_angles)}个")
            return [0.0] * 6
        
        robot_angles = [0.0] * 6
        
        # 应用与convert_robot_to_urdf_angles相反的变换
        robot_angles[0] = urdf_angles[0] 
        robot_angles[1] = urdf_angles[1]  
        robot_angles[2] = urdf_angles[2]   
        robot_angles[3] = urdf_angles[3] + math.pi  # l_a4 + pi
        robot_angles[4] = urdf_angles[4]  
        robot_angles[5] = urdf_angles[5]  
        
        self.get_logger().debug(f"关节角度逆转换: URDF {[f'{j:.3f}' for j in urdf_angles]} -> 机器人 {[f'{j:.3f}' for j in robot_angles]}")
        return robot_angles

    def _apply_position_stability_filter(self, current_pos):
        """应用位置稳定性过滤"""
        import numpy as np
        
        # 添加到历史记录
        self.position_history.append(current_pos.copy())
        if len(self.position_history) > 5:
            self.position_history.pop(0)
        
        # 如果历史记录足够，检查位置稳定性
        if len(self.position_history) >= 3:
            # 计算最近几次读取的方差
            pos_array = np.array(self.position_history[-3:])
            variance = np.var(pos_array, axis=0)
            max_variance = np.max(variance)
            
            # 如果方差小于阈值，认为位置稳定
            if max_variance < 0.01:  # 0.01弧度约等于0.57度
                self.stable_position = current_pos.copy()
                self.position_read_errors = 0
                self.get_logger().debug(f'位置稳定，方差: {max_variance:.4f}')
                return current_pos
            else:
                self.position_read_errors += 1
                self.get_logger().warn(f'位置不稳定，方差: {max_variance:.4f}，错误计数: {self.position_read_errors}')
                
                # 如果连续多次不稳定，但有历史稳定位置，返回稳定位置
                if self.position_read_errors > 3 and self.stable_position is not None:
                    self.get_logger().warn(f'连续{self.position_read_errors}次位置不稳定，使用历史稳定位置')
                    return self.stable_position
                else:
                    # 位置不稳定但错误次数不多，返回当前位置
                    return current_pos
        else:
            # 历史记录不足，直接返回当前位置
            self.get_logger().debug(f'位置历史记录不足({len(self.position_history)}/3)，直接返回当前位置')
            return current_pos

    def reconnect_and_enable(self):
        """重新连接并启用机械臂"""
        try:
            self.get_logger().warn('尝试重新连接机械臂...')
            
            # 重置状态
            self.arm_connected = False
            self.arm_enabled = False
            self.position_history.clear()
            self.stable_position = None
            self.position_read_errors = 0
            
            # 重新连接
            time.sleep(1.0)  # 等待一下再重连
            self.arm_robot.login_in(self.robot_ip)
            self.arm_connected = True
            
            # 重新启用
            self.auto_enable_robot()
            
            self.get_logger().info('机械臂重新连接成功')
            
        except Exception as e:
            self.get_logger().error(f'重新连接失败: {e}')
            self.arm_connected = False
            self.arm_enabled = False
    
    def joint_command_callback(self, msg):
        """接收来自trajectory_controller的关节命令"""
        if not self.arm_enabled:
            self.get_logger().warn("机械臂未使能，忽略关节命令")
            return
        
        if len(msg.data) != 6:
            self.get_logger().error("关节命令必须包含6个关节角度")
            return
        
        # 检查是否与上次命令相同，避免重复执行
        urdf_joint_positions = list(msg.data)
        current_time = time.time()
        
        # 将URDF坐标系的角度转换为实际机器人的角度
        robot_joint_positions = self.convert_urdf_to_robot_angles(urdf_joint_positions)
        
        # 计算与上次命令的差异
        max_diff = max(abs(robot_joint_positions[i] - self.last_joint_command[i]) for i in range(6))
        
        # 如果变化很小且时间间隔很短，跳过这次命令
        if max_diff < 0.01 and (current_time - self.last_command_time) < 0.3:
            return
        
        try:
            # 发送关节角度给实际机器人
            self.arm_robot.joint_move(robot_joint_positions, 0, False, 0.8)  # 非阻塞模式，速度0.8
            self.get_logger().info(f'执行关节命令 (URDF): {[f"{j:.3f}" for j in urdf_joint_positions]}')
            self.get_logger().info(f'执行关节命令 (机器人): {[f"{j:.3f}" for j in robot_joint_positions]}')
            
            # 更新当前关节位置（6DOF机械臂部分）- 使用URDF角度
            self.current_joint_positions[7:13] = urdf_joint_positions  # l_a1到l_a6的位置
            
            # 记录这次命令 - 使用机器人角度
            self.last_joint_command = robot_joint_positions.copy()
            self.last_command_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'机械臂关节运动失败: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()