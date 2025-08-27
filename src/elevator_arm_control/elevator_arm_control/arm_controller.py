#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
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
        
        # 订阅由MoveIt2生成的轨迹
        self.trajectory_subscriber = self.create_subscription(
            JointTrajectory,
            '/jaka_arm_controller/joint_trajectory', 
            self.execute_trajectory_callback,
            10
        )
        
        # **新增**: 订阅来自trajectory_controller的关节命令
        self.joint_command_subscriber = self.create_subscription(
            Float64MultiArray,
            '/arm/joint_command',
            self.joint_command_callback,
            10
        )
        
        # **新增**: 发布最终关节状态 (给RViz和MoveIt使用)
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

        # 发布完整的关节状态
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_joint_positions
        self.joint_publisher.publish(joint_state_msg)
    
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
        """
        将实际机器人的关节角度转换为URDF坐标系中的角度
        
        根据URDF中每个关节的rpy偏移，需要进行相应的角度调整：
        - l_a1: rpy="3.1416 1.5708 0" (180°绕X轴 + 90°绕Y轴)
        - l_a2: rpy="1.5708 -1.5708 0" (90°绕X轴 - 90°绕Y轴) 
        - l_a3: rpy="0 0 0" (无偏移)
        - l_a4: rpy="-1.5708 0 -1.5708" (-90°绕X轴 - 90°绕Z轴)
        - l_a5: rpy="-1.5708 1.5708 0" (-90°绕X轴 + 90°绕Y轴)
        - l_a6: rpy="1.5708 0 -1.5708" (90°绕X轴 - 90°绕Z轴)
        """
        if len(robot_angles) != 6:
            self.get_logger().error(f"期望6个关节角度，但收到{len(robot_angles)}个")
            return [0.0] * 6
        
        urdf_angles = [0.0] * 6
        
        # 基于URDF中的rpy偏移调整关节角度
        # 这些偏移量根据URDF定义计算得出
        
        # l_a1: 有180°和90°的旋转偏移，可能需要反向旋转
        urdf_angles[0] = robot_angles[0]  # 先尝试反向
        
        # l_a2: 有90°偏移组合
        urdf_angles[1] = robot_angles[1]  # 先保持原值，稍后根据测试调整
        
        # l_a3: 无偏移，保持原值
        urdf_angles[2] = robot_angles[2]
        
        # l_a4: 有-90°和-90°的偏移
        urdf_angles[3] = robot_angles[3] - math.pi   # 先尝试反向
        
        # l_a5: 有-90°和90°的偏移
        urdf_angles[4] = robot_angles[4]  # 先保持原值
        
        # l_a6: 有90°和-90°的偏移
        urdf_angles[5] = robot_angles[5]  # 先尝试反向
        
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
        robot_angles[0] = urdf_angles[0]  # l_a1反向
        robot_angles[1] = urdf_angles[1]   # l_a2保持
        robot_angles[2] = urdf_angles[2]   # l_a3保持 
        robot_angles[3] = urdf_angles[3] + math.pi  # l_a4反向
        robot_angles[4] = urdf_angles[4]   # l_a5保持
        robot_angles[5] = urdf_angles[5]  # l_a6反向
        
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