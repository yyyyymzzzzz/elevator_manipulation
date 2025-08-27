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
        
        # 发布器 - 发送关节命令给arm_controller
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/arm/joint_command',
            10
        )
        
        # 订阅器 - 接收当前关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 发布关节状态给MoveIt - 注释掉以避免与arm_controller的/joint_states冲突
        # self.joint_state_publisher = self.create_publisher(
        #     JointState,
        #     '/joint_states',
        #     10
        # )
        
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
                
                # 等待到达目标时间或使用固定间隔
                # if i < len(trajectory.points) - 1:
                #     sleep_time = 0.5  # 每个点间隔0.5秒
                #     time.sleep(sleep_time)
                # else:
                #     # 最后一个点，等待更长时间确保到达
                #     time.sleep(1.0)   
            
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
