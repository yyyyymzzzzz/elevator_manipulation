#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
import threading
import time
import math

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
        
        # 声明路径优化参数
        self.declare_parameter('min_joint_distance', 0.15,  # 增大默认阈值到0.15弧度
                             ParameterDescriptor(description='关节空间中跳过中间点的最小距离阈值（弧度）'))
        self.declare_parameter('enable_path_optimization', True,
                             ParameterDescriptor(description='是否启用路径优化'))
        self.declare_parameter('point_execution_delay', 0.05,  # 轨迹点间执行延迟减小到0.05秒
                             ParameterDescriptor(description='轨迹点之间的最小执行间隔（秒）'))
        self.declare_parameter('max_trajectory_points', 10,  # 最大保留的轨迹点数
                             ParameterDescriptor(description='优化后最大保留的轨迹点数量'))
        
        # 获取参数值
        self.min_joint_distance = self.get_parameter('min_joint_distance').value
        self.enable_path_optimization = self.get_parameter('enable_path_optimization').value
        self.point_execution_delay = self.get_parameter('point_execution_delay').value
        self.max_trajectory_points = self.get_parameter('max_trajectory_points').value
        
        # 参数变化回调
        self.add_on_set_parameters_callback(self.parameter_callback)
        
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
    
    def parameter_callback(self, params):
        """参数变化回调函数"""
        for param in params:
            if param.name == 'min_joint_distance':
                self.min_joint_distance = param.value
                self.get_logger().info(f'更新路径优化距离阈值: {self.min_joint_distance:.4f}')
            elif param.name == 'enable_path_optimization':
                self.enable_path_optimization = param.value
                self.get_logger().info(f'路径优化状态: {"启用" if self.enable_path_optimization else "禁用"}')
            elif param.name == 'point_execution_delay':
                self.point_execution_delay = param.value
                self.get_logger().info(f'更新轨迹点最小执行间隔: {self.point_execution_delay:.3f}秒')
            elif param.name == 'max_trajectory_points':
                self.max_trajectory_points = param.value
                self.get_logger().info(f'更新最大轨迹点数: {self.max_trajectory_points}')
        
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)
                        
    def calculate_joint_distance(self, positions1, positions2):
        """计算两个关节位置之间的欧几里得距离"""
        if len(positions1) != len(positions2):
            return float('inf')
        
        distance = 0.0
        for i in range(len(positions1)):
            distance += (positions1[i] - positions2[i]) ** 2
        return math.sqrt(distance)
    
    def optimize_trajectory_points(self, trajectory_points, joint_names):
        """优化轨迹点，跳过距离很近的中间点"""
        if not self.enable_path_optimization or len(trajectory_points) <= 2:
            return trajectory_points
        
        # 总是保留第一个和最后一个点
        optimized_points = [trajectory_points[0]]
        
        # 如果轨迹点太少，不需要优化
        if len(trajectory_points) <= 3:
            return trajectory_points
        
        # 使用自适应采样策略
        total_points = len(trajectory_points)
        
        # 计算理想的采样间隔
        if total_points > self.max_trajectory_points:
            # 如果点数过多，使用等间隔采样
            step = max(1, total_points // self.max_trajectory_points)
            for i in range(step, total_points - 1, step):
                optimized_points.append(trajectory_points[i])
        else:
            # 使用距离阈值过滤
            for i in range(1, len(trajectory_points) - 1):
                current_point = trajectory_points[i]
                last_kept_point = optimized_points[-1]
                
                # 重新排列关节位置以匹配我们的关节顺序
                current_positions = [0.0] * 6
                last_positions = [0.0] * 6
                
                for j, joint_name in enumerate(joint_names):
                    if joint_name in self.joint_names:
                        idx = self.joint_names.index(joint_name)
                        current_positions[idx] = current_point.positions[j]
                        last_positions[idx] = last_kept_point.positions[j]
                
                # 计算当前点与上一个保留点的距离
                distance = self.calculate_joint_distance(current_positions, last_positions)
                
                # 如果距离大于阈值，保留这个点
                if distance >= self.min_joint_distance:
                    optimized_points.append(current_point)
                    self.get_logger().debug(f'保留轨迹点 {i+1}, 距离: {distance:.4f}')
                else:
                    self.get_logger().debug(f'跳过轨迹点 {i+1}, 距离太小: {distance:.4f}')
        
        # 总是保留最后一个点
        if optimized_points[-1] != trajectory_points[-1]:
            optimized_points.append(trajectory_points[-1])
        
        self.get_logger().info(f'轨迹优化完成: 原始点数 {len(trajectory_points)}, 优化后点数 {len(optimized_points)}')
        return optimized_points
    
    def optimize_body_trajectory_points(self, trajectory_points, joint_names):
        """优化body轨迹点"""
        if not self.enable_path_optimization or len(trajectory_points) <= 2:
            return trajectory_points
        
        # Body轨迹优化使用稍大的阈值，因为body运动更慢
        body_distance_threshold = self.min_joint_distance * 2.0
        
        optimized_points = [trajectory_points[0]]  # 总是保留第一个点
        
        try:
            l_1_idx = joint_names.index('l_1')
            l_2_idx = joint_names.index('l_2')
            
            for i in range(1, len(trajectory_points) - 1):
                current_point = trajectory_points[i]
                last_kept_point = optimized_points[-1]
                
                # 提取body关节位置
                current_l1 = current_point.positions[l_1_idx]
                current_l2 = current_point.positions[l_2_idx]
                last_l1 = last_kept_point.positions[l_1_idx]
                last_l2 = last_kept_point.positions[l_2_idx]
                
                # 计算body关节距离
                distance = self.calculate_joint_distance([current_l1, current_l2], [last_l1, last_l2])
                
                if distance >= body_distance_threshold:
                    optimized_points.append(current_point)
                    self.get_logger().debug(f'保留body轨迹点 {i+1}, 距离: {distance:.4f}')
                else:
                    self.get_logger().debug(f'跳过body轨迹点 {i+1}, 距离太小: {distance:.4f}')
            
            # 总是保留最后一个点
            optimized_points.append(trajectory_points[-1])
            
            self.get_logger().info(f'Body轨迹优化完成: 原始点数 {len(trajectory_points)}, 优化后点数 {len(optimized_points)}')
            
        except ValueError as e:
            self.get_logger().warn(f'Body轨迹优化失败: {e}，使用原始轨迹')
            return trajectory_points
        
        return optimized_points
                        
    
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
        
        # 优化轨迹点，跳过距离很近的中间点
        original_points = trajectory.points
        optimized_points = self.optimize_trajectory_points(original_points, trajectory.joint_names)
        
        try:
            # 计算优化轨迹的总执行时间
            if optimized_points:
                last_point = optimized_points[-1]
                # 使用原始轨迹最后一个点的时间作为预期执行时间
                original_last_point = original_points[-1]
                expected_duration = original_last_point.time_from_start.sec + original_last_point.time_from_start.nanosec / 1e9
                
                # 如果优化后的点数减少了，按比例缩放时间间隔
                if len(optimized_points) > 1:
                    time_per_point = expected_duration / (len(optimized_points) - 1)
                else:
                    time_per_point = expected_duration
                
                self.get_logger().info(f'预期轨迹执行时间: {expected_duration:.2f}秒, 每点间隔: {time_per_point:.2f}秒')
            
            # 执行优化后轨迹中的每个点
            start_time = time.time()
            for i, point in enumerate(optimized_points):
                if not goal_handle.is_active:
                    self.get_logger().info('轨迹执行被取消')
                    break
                
                # 计算当前点的预期执行时间
                if i == 0:
                    target_time = 0.0
                else:
                    target_time = i * time_per_point
                
                # 等待到达目标时间
                elapsed_time = time.time() - start_time
                if elapsed_time < target_time:
                    sleep_time = target_time - elapsed_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                
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
                
                current_elapsed = time.time() - start_time
                self.get_logger().info(f'执行优化轨迹点 {i+1}/{len(optimized_points)} (t={current_elapsed:.2f}s): {[f"{p:.3f}" for p in joint_positions]}')
                
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
        
        # 优化body轨迹点
        original_points = trajectory.points
        optimized_points = self.optimize_body_trajectory_points(original_points, trajectory.joint_names)
        
        try:
            l_1_idx = trajectory.joint_names.index('l_1')
            l_2_idx = trajectory.joint_names.index('l_2')
            
            # 如果启用了路径优化，执行优化后的轨迹；否则直接跳到最后一个点
            if self.enable_path_optimization and len(optimized_points) > 1:
                # 执行优化后的所有轨迹点
                body_traj_msg = JointTrajectory()
                body_traj_msg.header.stamp = self.get_clock().now().to_msg()
                body_traj_msg.joint_names = ['l_1', 'l_2']
                
                for i, point in enumerate(optimized_points):
                    traj_point = JointTrajectoryPoint()
                    traj_point.positions = [point.positions[l_1_idx], point.positions[l_2_idx]]
                    traj_point.time_from_start.sec = (i + 1) * 2  # 每个点2秒间隔
                    body_traj_msg.points.append(traj_point)
                
                self.body_trajectory_publisher.publish(body_traj_msg)
                self.get_logger().info(f"已发布优化的Body轨迹，包含 {len(optimized_points)} 个点")
                
                # 等待轨迹执行完成
                wait_time = len(optimized_points) * 2 + 1
                time.sleep(wait_time)
            elif trajectory.points:
                # 直接执行最后一个点
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
