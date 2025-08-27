#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import threading
import time

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('jaka_arm_controller')
        
        # 动作服务器 - 接收MoveIt的轨迹执行请求
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/jaka_arm_controller/follow_joint_trajectory',
            self.execute_trajectory_callback
        )

        # Body动作服务器 - 接收MoveIt的body轨迹执行请求
        self._body_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/body_controller/follow_joint_trajectory',
            self.execute_body_action_callback
        )
        
        # 发布器 - 发送关节命令给arm_controller
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/arm/joint_command',
            10
        )
        
        # 发布器 - 发送body轨迹给arm_controller
        self.body_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/body_controller/joint_trajectory',
            10
        )
        
        # 订阅器 - 接收当前关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 当前关节状态
        self.current_joint_positions = [0.0] * 6
        self.joint_names = ['l_a1', 'l_a2', 'l_a3', 'l_a4', 'l_a5', 'l_a6']
        
        # 轨迹执行状态
        self.trajectory_executing = False
        
        # 定时发布关节状态 - 注释掉以避免与arm_controller冲突
        # self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info('JAKA轨迹控制器已启动')
    
    def joint_state_callback(self, msg):
        """接收来自arm_controller的关节状态"""
        if 'l_a1' in msg.name:
            # 提取6DOF机械臂关节状态
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    if idx < len(msg.position):
                        self.current_joint_positions[i] = msg.position[idx]
                        
    
    def execute_trajectory_callback(self, goal_handle):
        """执行轨迹动作回调"""
        self.get_logger().info('收到轨迹执行请求')
        
        trajectory = goal_handle.request.trajectory
        
        # 检查关节名称是否匹配
        if not all(name in self.joint_names for name in trajectory.joint_names):
            self.get_logger().error('轨迹关节名称不匹配')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        self.trajectory_executing = True
        
        try:
            # 执行轨迹中的每个点
            for i, point in enumerate(trajectory.points):
                if not goal_handle.is_active:
                    self.get_logger().info('轨迹执行被取消')
                    break
                
                # 重新排列关节位置以匹配我们的关节顺序
                joint_positions = [0.0] * 6
                for j, joint_name in enumerate(trajectory.joint_names):
                    if joint_name in self.joint_names:
                        idx = self.joint_names.index(joint_name)
                        joint_positions[idx] = point.positions[j]
                
                # 发送关节命令
                command_msg = Float64MultiArray()
                command_msg.data = joint_positions
                self.joint_command_publisher.publish(command_msg)
                
                self.get_logger().info(f'执行轨迹点 {i+1}/{len(trajectory.points)}: {[f"{p:.3f}" for p in joint_positions]}')
                
                # 发布反馈
                feedback_msg = FollowJointTrajectory.Feedback()
                feedback_msg.joint_names = self.joint_names
                feedback_msg.actual = JointTrajectoryPoint()
                feedback_msg.actual.positions = self.current_joint_positions
                feedback_msg.desired = point
                goal_handle.publish_feedback(feedback_msg) 
            
            # 轨迹执行完成
            self.trajectory_executing = False
            
            if goal_handle.is_active:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
                self.get_logger().info('轨迹执行成功完成')
                return result
            
        except Exception as e:
            self.get_logger().error(f'轨迹执行失败: {e}')
            self.trajectory_executing = False
            if goal_handle.is_active:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                goal_handle.abort(result)
                return result
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def execute_body_action_callback(self, goal_handle):
        """执行body动作服务器的回调 - 处理MoveIt的FollowJointTrajectory请求"""
        self.get_logger().info('收到Body轨迹动作执行请求')
        
        request = goal_handle.request
        trajectory = request.trajectory
        
        # 验证关节名称
        expected_joints = {'l_1', 'l_2'}
        received_joints = set(trajectory.joint_names)
        
        if not expected_joints.issubset(received_joints):
            self.get_logger().error(f'Body轨迹包含意外的关节: 期望 {expected_joints}, 收到 {received_joints}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result

        self.get_logger().info(f'开始执行Body动作轨迹，包含 {len(trajectory.points)} 个点')
        
        try:
            l_1_idx = trajectory.joint_names.index('l_1')
            l_2_idx = trajectory.joint_names.index('l_2')
            
            # 直接执行最后一个点，避免过多的中间点导致超时
            if trajectory.points:
                final_point = trajectory.points[-1]
                l_1_pos = final_point.positions[l_1_idx]
                l_2_pos = final_point.positions[l_2_idx]
                
                self.get_logger().info(f"执行Body最终目标: l_1={l_1_pos:.3f}m, l_2={l_2_pos:.3f}rad")
                
                # 发布开始反馈
                feedback_msg = FollowJointTrajectory.Feedback()
                feedback_msg.joint_names = ['l_1', 'l_2']
                feedback_msg.actual = JointTrajectoryPoint()
                feedback_msg.actual.positions = [0.0, 0.0]  # 当前位置
                feedback_msg.desired = final_point
                goal_handle.publish_feedback(feedback_msg)
                
                # 创建body轨迹消息并发布给arm_controller
                body_traj_msg = JointTrajectory()
                body_traj_msg.header.stamp = self.get_clock().now().to_msg()
                body_traj_msg.joint_names = ['l_1', 'l_2']
                
                # 创建轨迹点
                traj_point = JointTrajectoryPoint()
                traj_point.positions = [l_1_pos, l_2_pos]
                traj_point.time_from_start.sec = 2  # 2秒内完成运动
                body_traj_msg.points = [traj_point]
                
                # 发布body轨迹
                self.body_trajectory_publisher.publish(body_traj_msg)
                self.get_logger().info(f"已发布Body轨迹到arm_controller: l_1={l_1_pos:.3f}m, l_2={l_2_pos:.3f}rad")
                
                # 等待执行完成
                time.sleep(3.0)  # 给arm_controller时间执行body命令
                
                # 发布最终反馈
                feedback_msg.actual.positions = [l_1_pos, l_2_pos]
                goal_handle.publish_feedback(feedback_msg)
                
                # 动作执行完成
                goal_handle.succeed()
                self.get_logger().info('Body动作轨迹执行完成')
                
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            else:
                self.get_logger().warn('Body轨迹为空')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result
            
        except Exception as e:
            self.get_logger().error(f'Body动作轨迹执行失败: {e}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

def main(args=None):
    rclpy.init(args=args)
    
    trajectory_controller = TrajectoryController()
    
    try:
        rclpy.spin(trajectory_controller)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
