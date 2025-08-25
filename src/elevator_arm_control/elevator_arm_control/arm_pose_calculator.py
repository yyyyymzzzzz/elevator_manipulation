#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math

class ArmPoseCalculator(Node):
    def __init__(self):
        super().__init__('arm_pose_calculator')
        
        # 订阅目标位姿
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/arm/target_pose',
            self.target_pose_callback,
            10
        )
        
        # 发布计算后的关节状态
        self.calculation_publisher = self.create_publisher(JointState, '/arm/calculated_joints', 10)

        # 发布关节状态（给RViz等工具使用）
        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        
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
        
        # 创建定时器，以10Hz频率发布关节状态
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # 运动规划参数
        self.motion_speed = 0.5  # 关节运动速度 (rad/s 或 m/s)
        
        # 运动模式参数
        self.declare_parameter('smooth_motion', False)  # 是否启用平滑运动
        self.smooth_motion = self.get_parameter('smooth_motion').value
        
        self.get_logger().info(f'机械臂位姿计算器已启动 - {"平滑运动模式" if self.smooth_motion else "直接运动模式"}')
    
    def target_pose_callback(self, msg):
        """接收目标位姿并进行逆运动学计算"""
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_z = msg.pose.position.z
        
        # 提取四元数并转换为欧拉角
        quat = msg.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
        
        self.get_logger().info(f'收到目标位姿: 位置({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), '
                              f'姿态({roll:.2f}, {pitch:.2f}, {yaw:.2f})')
        
        # 进行逆运动学计算
        target_joints = self.inverse_kinematics(target_x, target_y, target_z, roll, pitch, yaw)
        
        if target_joints is not None:
            self.target_joint_positions = target_joints
            self.get_logger().info(f'计算得到目标关节角度: {[f"{j:.3f}" for j in target_joints]}')
            
            # 根据运动模式选择是否立即更新位置
            if not self.smooth_motion:
                # 直接模式：立即跳转到目标位置
                self.current_joint_positions = target_joints.copy()
        else:
            self.get_logger().warn('逆运动学求解失败，目标位置可能不可达')
    
    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def inverse_kinematics(self, x, y, z, roll, pitch, yaw):
        """修正后的逆运动学计算 - 避免控制头部，专注于机械臂末端"""
        try:
            target_joints = [0.0] * len(self.joint_names)
            
            # 升降关节 (l_1) - 根据z坐标调整
            base_height = 0.565
            if z > base_height:
                target_joints[0] = min(0.4, z - base_height)  # 升降范围0-0.4m
            else:
                target_joints[0] = 0.0
            
            # 底座旋转 (l_2) - 根据x,y坐标计算水平角度
            target_joints[1] = math.atan2(y, x)
            
            # 计算到目标点的水平距离
            horizontal_distance = math.sqrt(x*x + y*y)
            
            # 机械臂逆运动学 - 专注于6DOF机械臂，不控制头部
            arm_reach = 0.5  # 6DOF机械臂的实际范围
            
            if horizontal_distance > arm_reach:
                self.get_logger().warn(f'目标距离{horizontal_distance:.2f}m超出6DOF机械臂范围{arm_reach:.2f}m')
                horizontal_distance = arm_reach
            
            # 调整后的Z高度（考虑升降和头部位置）
            effective_z = z - base_height - target_joints[0] - 0.3  # 减去头部高度偏移
            
            # 第二关节 (l_3) - 控制整体机械臂俯仰，但保持温和
            if horizontal_distance > 0.01:  # 避免除零
                target_joints[2] = math.atan2(effective_z, horizontal_distance) * 0.3  # 减小影响
            else:
                target_joints[2] = 0.0
            
            # l_4 关节控制头部 - 保持中性位置，避免旋转
            target_joints[3] = 0.0  # 头部保持不动
            
            # 6自由度机械臂关节 - 这是主要的末端执行器控制
            # l_a1: 基座旋转（补偿底座旋转，实现精确定位）
            target_joints[4] = 0.0  # 基础旋转已由l_2处理
            
            # l_a2: 主要俯仰控制
            target_joints[5] = pitch * 0.8  # 俯仰控制
            
            # l_a3: 肘部角度（根据距离和高度调整）
            elbow_factor = horizontal_distance / arm_reach if arm_reach > 0 else 0
            target_joints[6] = math.pi/6 * elbow_factor  # 适度的肘部弯曲
            
            # l_a4: 腕部俯仰（精细调整末端姿态）
            target_joints[7] = pitch * 0.5  # 腕部俯仰
            
            # l_a5: 腕部翻滚
            target_joints[8] = roll  # 翻滚控制
            
            # l_a6: 最终旋转（yaw控制）
            target_joints[9] = yaw  # 最终朝向
            
            # 检查关节限制（基于URDF中的限制）
            joint_limits = [
                [0.0, 0.4],         # l_1: 升降限制
                [-2.7925, 2.7925],  # l_2: 旋转限制
                [-3.1415, 3.1415],  # l_3: 旋转限制
                [-0.0872, 0.6108],  # l_4: 旋转限制 (头部 - 保持中性)
                [-6.2831, 6.2831],  # l_a1: 旋转限制
                [-2.1816, 2.1816],  # l_a2: 旋转限制
                [-2.2689, 2.2689],  # l_a3: 旋转限制
                [-6.2831, 6.2831],  # l_a4: 旋转限制
                [-2.0943, 2.0943],  # l_a5: 旋转限制
                [-6.2831, 6.2831],  # l_a6: 旋转限制
            ]
            
            # 应用关节限制
            for i, (lower, upper) in enumerate(joint_limits):
                target_joints[i] = max(lower, min(upper, target_joints[i]))
            
            # 输出调试信息
            self.get_logger().info(f'逆运动学结果: 距离={horizontal_distance:.3f}, 升降={target_joints[0]:.3f}, '
                                  f'底座旋转={target_joints[1]:.3f}, 机械臂俯仰={target_joints[2]:.3f}, '
                                  f'头部={target_joints[3]:.3f}(固定)')
            
            return target_joints
            
        except Exception as e:
            self.get_logger().error(f'逆运动学计算错误: {e}')
            return None
    
    def control_loop(self):
        """控制循环，根据模式选择是否平滑移动到目标位置"""
        if self.smooth_motion:
            # 平滑模式：逐步移动到目标位置
            position_changed = False
            for i in range(len(self.current_joint_positions)):
                error = self.target_joint_positions[i] - self.current_joint_positions[i]
                
                # 简单的比例控制
                if abs(error) > 0.001:  # 死区
                    step = self.motion_speed * 0.1  # 0.1秒的时间步长
                    if abs(error) < step:
                        self.current_joint_positions[i] = self.target_joint_positions[i]
                    else:
                        self.current_joint_positions[i] += step if error > 0 else -step
                    position_changed = True
            
            # 只有在位置变化时才发布
            if position_changed:
                self.publish_calculation_states()
                self.publish_joint_states()
        else:
            # 直接模式：立即发布当前状态
            self.publish_calculation_states()
            self.publish_joint_states()
    
    def publish_calculation_states(self):
        """发布关节控制信息"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)
        
        self.calculation_publisher.publish(joint_state)

    def publish_joint_states(self):
        """发布当前关节状态"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)
        
        self.joint_publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    
    node = ArmPoseCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
