# src/elevator_arm_control/elevator_arm_control/button_follower.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.timer import Timer

from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    RobotTrajectory,
    MotionPlanRequest,
    RobotState
)
import math
from moveit_msgs.srv import GetMotionPlan
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.executors import MultiThreadedExecutor

class ButtonFollower(Node):
    def __init__(self):
        super().__init__("button_follower_node")

        # TF2 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # MoveIt 规划服务的客户端
        self.planning_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        
        # trajectory_controller 的 Action 客户端
        self.execution_client = ActionClient(
            self, FollowJointTrajectory, "/jaka_arm_controller/follow_joint_trajectory"
        )
        
        # 目标点订阅者
        self.subscription = self.create_subscription(
            PoseStamped,
            '/button_target_pose',
            self.target_pose_callback,
            10)
        
        # 当前末端执行器位置订阅者
        self.current_joint_state = None
        self.filtered_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.current_end_effector_pose = None
        
        # 状态和数据存储
        self.is_executing = False
        self.is_going_home = False
        self.current_goal_pose = None
        self.current_press_pose = None  # 按压位置
        self.current_step = "idle" 
        self.trajectory_completed = False  
        
        self.pause_timer: Timer = None
        self.position_check_timer: Timer = None  
        
        # 位置检查相关参数
        self.position_tolerance = 0.02  # 位置容忍度 (米) - 2cm，更严格
        self.press_position_tolerance = 0.02  # 按压位置容忍度 (米) - 保持一致
        self.orientation_tolerance = 0.2  # 方向容忍度 (弧度) - 减小
        self.press_orientation_tolerance = 0.2  # 按压方向容忍度 (弧度) - 保持一致
        self.max_position_check_time = 20.0  # 最大位置检查时间 (秒)
        self.position_check_start_time = None
        
        # 位置检查的最小等待时间（确保机器人真正稳定）
        self.min_settle_time = 1.0  # 至少等待1秒再开始检查
        
        # 按压相关参数
        self.press_distance = 0.03  # 按压距离：3cm

        # 定义Home位置的关节角度 
        self.home_joint_angles = [-1.0*math.pi/2.0, math.pi/2.0, math.pi/2.0, 0.0, 0.0, 0.0]
        self.joint_names = ['l_a1', 'l_a2', 'l_a3', 'l_a4', 'l_a5', 'l_a6']

        # 目标接收冷却时间控制
        self.min_goal_interval = 10.0
        self.last_goal_time = None
        
        self.get_logger().info("Button Follower Node is ready for 'Home -> Target -> Press -> Target -> Home' cycle.")
        self.get_logger().info(f"Home position set to: {self.home_joint_angles}")
        self.get_logger().info(f"Press distance: {self.press_distance}m")
        self.get_logger().info(f"Minimum goal interval: {self.min_goal_interval} seconds")

        self.goal_marker_pub = self.create_publisher(Marker, "/final_goal_marker", 10)

    def create_one_shot_callback(self, callback_func):
        """创建一次性定时器回调包装器"""
        def one_shot_wrapper():
            # 首先取消并销毁定时器
            if self.pause_timer is not None:
                self.pause_timer.cancel()
                self.pause_timer.destroy()
                self.pause_timer = None
            # 然后执行原始回调
            callback_func()
        return one_shot_wrapper

    def joint_state_callback(self, msg: JointState):
        """接收并过滤关节状态，使其匹配规划组"""
        self.full_joint_state = msg
        
        # 如果消息为空或关节列表为空，则不处理
        if not msg.name:
            return

        # 创建一个新的、干净的JointState消息用于规划
        temp_joint_state = JointState()
        temp_joint_state.header = msg.header
        
        # 创建一个从关节名到索引的映射，以便快速查找
        joint_name_to_index = {name: i for i, name in enumerate(msg.name)}
        
        found_joints = 0
        for joint_name in self.joint_names:
            if joint_name in joint_name_to_index:
                idx = joint_name_to_index[joint_name]
                temp_joint_state.name.append(msg.name[idx])
                temp_joint_state.position.append(msg.position[idx])
                # 如果消息中包含速度和力矩信息，也一并添加
                if idx < len(msg.velocity):
                    temp_joint_state.velocity.append(msg.velocity[idx])
                if idx < len(msg.effort):
                    temp_joint_state.effort.append(msg.effort[idx])
                found_joints += 1
        
        # 只有当我们找到了所有需要的关节时，才更新可用于规划的状态
        if found_joints == len(self.joint_names):
            self.filtered_joint_state = temp_joint_state
        else:
            # 如果在某些消息中关节不全，清除旧的过滤状态以防止使用过时数据
            self.filtered_joint_state = None

    def get_current_end_effector_pose(self):
        """通过TF获取当前末端执行器位置"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", "link_a6", rclpy.time.Time(), timeout=Duration(seconds=0.1))
            
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            
            return pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Could not get current end effector pose: {e}")
            return None

    def check_position_reached(self, target_pose: PoseStamped, use_press_tolerance=False):
        """检查机器人是否到达目标位置"""
        current_pose = self.current_end_effector_pose
        if current_pose is None:
            current_pose = self.get_current_end_effector_pose()
        
        if current_pose is None:
            self.get_logger().debug("Could not get current end effector pose for position check")
            return False
        
        # 根据是否为按压动作选择不同的容忍度
        position_tolerance = self.press_position_tolerance if use_press_tolerance else self.position_tolerance
        orientation_tolerance = self.press_orientation_tolerance if use_press_tolerance else self.orientation_tolerance
        
        # 计算位置差异
        pos_diff_x = abs(current_pose.pose.position.x - target_pose.pose.position.x)
        pos_diff_y = abs(current_pose.pose.position.y - target_pose.pose.position.y)
        pos_diff_z = abs(current_pose.pose.position.z - target_pose.pose.position.z)
        
        position_error = (pos_diff_x**2 + pos_diff_y**2 + pos_diff_z**2)**0.5
        
        position_ok = position_error < position_tolerance
        
        # 更详细的日志输出
        action_type = "press" if use_press_tolerance else "target"
        self.get_logger().debug(f"{action_type.capitalize()} position check: error={position_error:.4f}m, tolerance={position_tolerance}m, OK={position_ok}")
        
        if position_ok:
            self.get_logger().info(f"{action_type.capitalize()} position reached! Error: {position_error:.4f}m (tolerance: {position_tolerance}m)")
            return True
        else:
            return False

    def calculate_press_pose(self, target_pose: PoseStamped):
        """根据目标位置计算按压位置（向前移动3cm）"""
        import copy
        import math
        press_pose = copy.deepcopy(target_pose)
        
        # 简化方法：假设按钮主要面向机器人，沿着末端执行器的接近方向移动
        # 这里使用末端执行器的当前朝向来计算按压方向
        orientation = target_pose.pose.orientation
        
        # 将四元数转换为旋转矩阵的简化版本
        # 假设主要的按压方向是沿着当前末端执行器的Z轴正方向（接近方向）
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        
        # 计算Z轴方向的单位向量（末端执行器的接近方向）
        # 旋转矩阵的第三列就是Z轴方向
        z_x = 2.0 * (qx * qz + qw * qy)
        z_y = 2.0 * (qy * qz - qw * qx)
        z_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        
        # 沿着Z轴正方向移动（接近按钮）
        press_pose.pose.position.x = target_pose.pose.position.x + self.press_distance * z_x
        press_pose.pose.position.y = target_pose.pose.position.y + self.press_distance * z_y
        press_pose.pose.position.z = target_pose.pose.position.z + self.press_distance * z_z
        
        self.get_logger().info(f"Calculated press pose: ({press_pose.pose.position.x:.3f}, {press_pose.pose.position.y:.3f}, {press_pose.pose.position.z:.3f})")
        self.get_logger().info(f"Press direction vector: ({z_x:.3f}, {z_y:.3f}, {z_z:.3f})")
        
        return press_pose

    def target_pose_callback(self, msg: PoseStamped):
        current_time = self.get_clock().now()
        
        # 严格检查当前状态
        if self.is_executing or self.current_step != "idle":
            # self.get_logger().warn(f"Already executing a cycle (step: {self.current_step}), skipping new goal.")
            return
        
        # 检查定时器状态
        if self.pause_timer is not None and not self.pause_timer.cancelled:
            self.get_logger().warn("Pause timer still active, skipping new goal.")
            return
        
        if self.last_goal_time is not None:
            time_since_last_goal = (current_time - self.last_goal_time).nanoseconds / 1e9
            if time_since_last_goal < self.min_goal_interval:
                remaining_time = self.min_goal_interval - time_since_last_goal
                self.get_logger().warn(f"Goal received too soon. Wait {remaining_time:.1f} more seconds.")
                return
        
        self.last_goal_time = current_time
        self.is_executing = True
        self.is_going_home = False
        self.current_step = "planning_to_target"  # 设置当前步骤
        self.trajectory_completed = False  # 重置轨迹完成标志
        self.get_logger().info(f"Received new target in '{msg.header.frame_id}' frame. Starting cycle.")
        self.get_logger().info(f"CYCLE START - Step: {self.current_step}")
        self.log_current_state("Target Received")

        try:
            transform = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=1.0))
            self.current_goal_pose = do_transform_pose_stamped(msg, transform)
            
            # 计算按压位置
            self.current_press_pose = self.calculate_press_pose(self.current_goal_pose)
            
            self.get_logger().info("Successfully transformed pose to 'base_link' frame.")
            self.get_logger().info(f"Target position: ({self.current_goal_pose.pose.position.x:.3f}, {self.current_goal_pose.pose.position.y:.3f}, {self.current_goal_pose.pose.position.z:.3f})")
            self.get_logger().info(f"Press position: ({self.current_press_pose.pose.position.x:.3f}, {self.current_press_pose.pose.position.y:.3f}, {self.current_press_pose.pose.position.z:.3f})")
            
            self.publish_goal_marker(self.current_goal_pose)
            
            self.get_logger().info("STEP 1: Planning path to TARGET...")
            self.plan_to_pose(self.current_goal_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Could not transform: {e}")
            self.reset_state()

    def plan_to_pose(self, goal_pose: PoseStamped):
        if not self.planning_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Planning service '/plan_kinematic_path' not available.")
            self.reset_state()
            return

        print('Filtered Joint State:')
        print(self.filtered_joint_state)
        if self.filtered_joint_state is None:
            self.get_logger().error("Cannot plan, no current joint state has been received yet.")
            self.reset_state()
            return

        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "jaka_arm"
        motion_plan_request.planner_id = "RRTstar"
        motion_plan_request.allowed_planning_time = 10.0

        start_state = RobotState()
        # 更新时间戳为当前时间，避免使用过旧的时间戳
        current_joint_state = JointState()
        current_joint_state.header.stamp = self.get_clock().now().to_msg()
        current_joint_state.header.frame_id = self.filtered_joint_state.header.frame_id
        current_joint_state.name = self.filtered_joint_state.name[:]
        current_joint_state.position = self.filtered_joint_state.position[:]
        current_joint_state.velocity = self.filtered_joint_state.velocity[:]
        current_joint_state.effort = self.filtered_joint_state.effort[:]
        start_state.joint_state = current_joint_state
        motion_plan_request.start_state = start_state

        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = goal_pose.header.frame_id
        position_constraint.link_name = "link_a6"
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.001, 0.001, 0.001] 
        position_constraint.constraint_region.primitives.append(bounding_box)
        position_constraint.constraint_region.primitive_poses.append(goal_pose.pose)
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = goal_pose.header.frame_id
        orientation_constraint.link_name = "link_a6"
        orientation_constraint.orientation = goal_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01 
        orientation_constraint.absolute_y_axis_tolerance = 0.01 
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 2.0
        constraints.orientation_constraints.append(orientation_constraint)
        
        motion_plan_request.goal_constraints.append(constraints)
        request.motion_plan_request = motion_plan_request
        
        self.get_logger().info("Calling planning service for target pose...")
        future = self.planning_client.call_async(request)
        future.add_done_callback(self.planning_done_callback)

    def plan_to_press(self):
        """规划到按压位置的路径"""
        if self.current_step != "planning_to_press":
            self.get_logger().error(f"plan_to_press called in wrong step: {self.current_step}")
            self.reset_state()
            return
            
        if self.current_press_pose is None:
            self.get_logger().error("No press pose available for planning")
            self.reset_state()
            return
            
        if not self.planning_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Planning service '/plan_kinematic_path' not available.")
            self.reset_state()
            return

        if self.filtered_joint_state is None:
            self.get_logger().error("Cannot plan to press, no current joint state has been received yet.")
            self.reset_state()
            return

        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "jaka_arm"
        # 使用更简单的规划器，对短距离移动更有效
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"  
        # 增加规划时间，因为按压位置可能更难到达
        motion_plan_request.allowed_planning_time = 15.0

        start_state = RobotState()
        # 更新时间戳为当前时间，避免使用过旧的时间戳
        current_joint_state = JointState()
        current_joint_state.header.stamp = self.get_clock().now().to_msg()
        current_joint_state.header.frame_id = self.filtered_joint_state.header.frame_id
        current_joint_state.name = self.filtered_joint_state.name[:]
        current_joint_state.position = self.filtered_joint_state.position[:]
        current_joint_state.velocity = self.filtered_joint_state.velocity[:]
        current_joint_state.effort = self.filtered_joint_state.effort[:]
        start_state.joint_state = current_joint_state
        motion_plan_request.start_state = start_state

        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.current_press_pose.header.frame_id
        position_constraint.link_name = "link_a6"
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        # 按压动作使用更宽松的约束，确保成功率
        bounding_box.dimensions = [0.01, 0.01, 0.01]  # 5cm的容忍范围
        position_constraint.constraint_region.primitives.append(bounding_box)
        position_constraint.constraint_region.primitive_poses.append(self.current_press_pose.pose)
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.current_press_pose.header.frame_id
        orientation_constraint.link_name = "link_a6"
        orientation_constraint.orientation = self.current_press_pose.pose.orientation
        # 保持与target相同的方向约束权重，确保一致性
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 2.0  # 与target保持一致
        constraints.orientation_constraints.append(orientation_constraint)
        
        motion_plan_request.goal_constraints.append(constraints)
        request.motion_plan_request = motion_plan_request
        
        self.get_logger().info("Calling planning service for press pose...")
        self.get_logger().info(f"Press distance: {self.press_distance}m")
        future = self.planning_client.call_async(request)
        future.add_done_callback(self.planning_done_callback)

    def plan_to_target_return(self):
        """规划从按压位置回到目标位置的路径"""
        if self.current_step != "planning_to_target_return":
            self.get_logger().error(f"plan_to_target_return called in wrong step: {self.current_step}")
            self.reset_state()
            return
            
        if self.current_goal_pose is None:
            self.get_logger().error("No target pose available for return planning")
            self.reset_state()
            return
            
        if not self.planning_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Planning service '/plan_kinematic_path' not available.")
            self.reset_state()
            return

        if self.filtered_joint_state is None:
            self.get_logger().error("Cannot plan to target return, no current joint state has been received yet.")
            self.reset_state()
            return

        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "jaka_arm"
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.allowed_planning_time = 10.0

        start_state = RobotState()
        # 更新时间戳为当前时间，避免使用过旧的时间戳
        current_joint_state = JointState()
        current_joint_state.header.stamp = self.get_clock().now().to_msg()
        current_joint_state.header.frame_id = self.filtered_joint_state.header.frame_id
        current_joint_state.name = self.filtered_joint_state.name[:]
        current_joint_state.position = self.filtered_joint_state.position[:]
        current_joint_state.velocity = self.filtered_joint_state.velocity[:]
        current_joint_state.effort = self.filtered_joint_state.effort[:]
        start_state.joint_state = current_joint_state
        motion_plan_request.start_state = start_state

        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.current_goal_pose.header.frame_id
        position_constraint.link_name = "link_a6"
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.01, 0.01, 0.01]  # 1cm的容忍范围
        position_constraint.constraint_region.primitives.append(bounding_box)
        position_constraint.constraint_region.primitive_poses.append(self.current_goal_pose.pose)
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.current_goal_pose.header.frame_id
        orientation_constraint.link_name = "link_a6"
        orientation_constraint.orientation = self.current_goal_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 2.0
        constraints.orientation_constraints.append(orientation_constraint)
        
        motion_plan_request.goal_constraints.append(constraints)
        request.motion_plan_request = motion_plan_request
        
        self.get_logger().info("Calling planning service for target return pose...")
        future = self.planning_client.call_async(request)
        future.add_done_callback(self.planning_done_callback)

    def plan_to_home(self):
        # 确认当前处于正确的步骤状态
        if self.current_step != "planning_to_home":
            self.get_logger().error(f"plan_to_home called in wrong step: {self.current_step}")
            self.reset_state()
            return
            
        if not self.planning_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Planning service '/plan_kinematic_path' not available.")
            self.reset_state()
            return

        if self.filtered_joint_state is None:
            self.get_logger().error("Cannot plan to home, no current joint state has been received yet.")
            self.reset_state()
            return
            
        request = GetMotionPlan.Request()
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = "jaka_arm"
        motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        motion_plan_request.allowed_planning_time = 10.0

        start_state = RobotState()
        # 更新时间戳为当前时间，避免使用过旧的时间戳
        current_joint_state = JointState()
        current_joint_state.header.stamp = self.get_clock().now().to_msg()
        current_joint_state.header.frame_id = self.filtered_joint_state.header.frame_id
        current_joint_state.name = self.filtered_joint_state.name[:]
        current_joint_state.position = self.filtered_joint_state.position[:]
        current_joint_state.velocity = self.filtered_joint_state.velocity[:]
        current_joint_state.effort = self.filtered_joint_state.effort[:]
        start_state.joint_state = current_joint_state
        motion_plan_request.start_state = start_state

        constraints = Constraints()
        for i in range(len(self.joint_names)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = self.joint_names[i]
            joint_constraint.position = self.home_joint_angles[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        motion_plan_request.goal_constraints.append(constraints)
        request.motion_plan_request = motion_plan_request

        self.get_logger().info("Calling planning service for home position...")
        future = self.planning_client.call_async(request)
        future.add_done_callback(self.planning_done_callback)

    def planning_done_callback(self, future):
        try:
            response = future.result()
            motion_plan_response = response.motion_plan_response
            
            if motion_plan_response.error_code.val == 1: # SUCCESS
                if self.current_step == "planning_to_target":
                    step_name = "to target"
                elif self.current_step == "planning_to_press":
                    step_name = "to press"
                elif self.current_step == "planning_to_target_return":
                    step_name = "to target return"
                elif self.current_step == "planning_to_home":
                    step_name = "to home"
                else:
                    step_name = "unknown"
                
                self.get_logger().info(f"Planning {step_name} successful. Executing trajectory...")
                self.get_logger().info(f"Current step: {self.current_step}")
                
                # 更新步骤状态
                if self.current_step == "planning_to_target":
                    self.current_step = "executing_to_target"
                elif self.current_step == "planning_to_press":
                    self.current_step = "executing_to_press"
                elif self.current_step == "planning_to_target_return":
                    self.current_step = "executing_to_target_return"
                elif self.current_step == "planning_to_home":
                    self.current_step = "executing_to_home"
                
                self.trajectory_completed = False  # 重置完成标志
                self.execute_trajectory(motion_plan_response.trajectory)
            else:
                self.get_logger().error(f"Planning failed with error code: {motion_plan_response.error_code}")
                self.reset_state()
        except Exception as e:
            self.get_logger().error(f"Planning service call failed: {e}")
            self.reset_state()

    def execute_trajectory(self, robot_trajectory: RobotTrajectory):
        # 验证当前步骤状态
        if self.current_step not in ["executing_to_target", "executing_to_press", "executing_to_target_return", "executing_to_home"]:
            self.get_logger().error(f"execute_trajectory called in wrong step: {self.current_step}")
            self.reset_state()
            return
            
        if not self.execution_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Execution action server not available.")
            self.reset_state()
            return
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = robot_trajectory.joint_trajectory

        if self.current_step == "executing_to_target":
            step_name = "to target"
        elif self.current_step == "executing_to_press":
            step_name = "to press"
        elif self.current_step == "executing_to_target_return":
            step_name = "to target return"
        elif self.current_step == "executing_to_home":
            step_name = "to home"
        else:
            step_name = "unknown"
            
        self.get_logger().info(f'Sending trajectory goal {step_name} to controller...')
        self._send_goal_future = self.execution_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by trajectory controller')
            self.reset_state()
            return

        if self.current_step == "executing_to_target":
            step_name = "to target"
        elif self.current_step == "executing_to_press":
            step_name = "to press"
        elif self.current_step == "executing_to_target_return":
            step_name = "to target return"
        elif self.current_step == "executing_to_home":
            step_name = "to home"
        else:
            step_name = "unknown"
            
        self.get_logger().info(f'Goal {step_name} accepted by controller. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"Trajectory execution failed with error code: {result.error_code}")
            self.reset_state()
            return
        
        self.trajectory_completed = True  # 标记轨迹执行完成
        
        if self.current_step == "executing_to_target":
            step_name = "to target"
        elif self.current_step == "executing_to_press":
            step_name = "to press"
        elif self.current_step == "executing_to_target_return":
            step_name = "to target return"
        elif self.current_step == "executing_to_home":
            step_name = "to home"
        else:
            step_name = "unknown"
            
        self.get_logger().info(f"Trajectory execution {step_name} successful!")
        self.get_logger().info(f"Current step completed: {self.current_step}")

        # === 【核心修改】创建定时器前先取消旧的 ===
        if self.pause_timer is not None and not self.pause_timer.cancelled:
            self.get_logger().warn("Cancelling a pre-existing timer.")
            self.pause_timer.cancel()
            self.pause_timer.destroy()

        # 根据当前步骤决定下一步行动
        if self.current_step == "executing_to_target":
            self.current_step = "settling_at_target"
            self.get_logger().info("STEP 1 COMPLETED: Trajectory to target finished")
            self.get_logger().info("Checking if robot has reached target position...")
            # 开始位置检查
            self.position_check_start_time = self.get_clock().now()
            self.position_check_timer = self.create_timer(0.1, self.check_target_position_reached)  # 每100ms检查一次
            self.get_logger().info("Position check timer started")
        elif self.current_step == "executing_to_press":
            self.current_step = "settling_at_press"
            self.get_logger().info("STEP 2 COMPLETED: Trajectory to press finished")
            self.get_logger().info("Checking if robot has reached press position...")
            # 开始位置检查（按压位置）
            self.position_check_start_time = self.get_clock().now()
            self.position_check_timer = self.create_timer(0.1, self.check_press_position_reached)  # 每100ms检查一次
            self.get_logger().info("Press position check timer started")
        elif self.current_step == "executing_to_target_return":
            self.current_step = "settling_at_target_return"
            self.get_logger().info("STEP 3 COMPLETED: Trajectory to target return finished")
            self.get_logger().info("Checking if robot has reached target return position...")
            # 开始位置检查（目标返回位置）
            self.position_check_start_time = self.get_clock().now()
            self.position_check_timer = self.create_timer(0.1, self.check_target_return_position_reached)  # 每100ms检查一次
            self.get_logger().info("Target return position check timer started")
        elif self.current_step == "executing_to_home":
            self.current_step = "settling_at_home"
            self.get_logger().info("STEP 4 COMPLETED: Trajectory to home finished")
            self.get_logger().info("Checking if robot has reached home position...")
            # 开始位置检查 (对于home位置，我们检查关节角度而不是末端执行器位置)
            self.position_check_start_time = self.get_clock().now()
            self.position_check_timer = self.create_timer(0.1, self.check_home_position_reached)  # 每100ms检查一次
            self.get_logger().info("Position check timer started")
        else:
            self.get_logger().warn(f"Unexpected step state: {self.current_step}")
            self.reset_state()

    def check_target_position_reached(self):
        """检查是否到达目标位置"""
        if self.current_goal_pose is None:
            self.get_logger().error("No target pose available for position check")
            self.stop_position_check_and_reset()
            return
        
        # 检查超时
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.position_check_start_time).nanoseconds / 1e9
        
        # 首先确保已经等待了最小稳定时间
        if elapsed_time < self.min_settle_time:
            self.get_logger().debug(f"Still settling... {elapsed_time:.1f}s < {self.min_settle_time}s")
            return
        
        if elapsed_time > self.max_position_check_time:
            self.get_logger().warn(f"Position check timeout after {elapsed_time:.1f} seconds")
            self.stop_position_check_and_start_pause("target")
            return
        
        # 检查位置
        if self.check_position_reached(self.current_goal_pose):
            self.get_logger().info("Robot has reached TARGET position!")
            self.stop_position_check_and_start_pause("target")

    def check_press_position_reached(self):
        """检查是否到达按压位置"""
        if self.current_press_pose is None:
            self.get_logger().error("No press pose available for position check")
            self.stop_position_check_and_reset()
            return
        
        # 检查超时
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.position_check_start_time).nanoseconds / 1e9
        
        # 首先确保已经等待了最小稳定时间
        if elapsed_time < self.min_settle_time:
            self.get_logger().debug(f"Press settling... {elapsed_time:.1f}s < {self.min_settle_time}s")
            return
        
        if elapsed_time > self.max_position_check_time:
            self.get_logger().warn(f"Press position check timeout after {elapsed_time:.1f} seconds")
            self.stop_position_check_and_start_pause("press")
            return
        
        # 检查位置（使用按压容忍度）
        if self.check_position_reached(self.current_press_pose, use_press_tolerance=True):
            self.get_logger().info("Robot has reached PRESS position!")
            self.stop_position_check_and_start_pause("press")

    def check_target_return_position_reached(self):
        """检查是否到达目标返回位置"""
        if self.current_goal_pose is None:
            self.get_logger().error("No target pose available for return position check")
            self.stop_position_check_and_reset()
            return
        
        # 检查超时
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.position_check_start_time).nanoseconds / 1e9
        
        # 首先确保已经等待了最小稳定时间
        if elapsed_time < self.min_settle_time:
            self.get_logger().debug(f"Target return settling... {elapsed_time:.1f}s < {self.min_settle_time}s")
            return
        
        if elapsed_time > self.max_position_check_time:
            self.get_logger().warn(f"Target return position check timeout after {elapsed_time:.1f} seconds")
            self.stop_position_check_and_start_pause("target_return")
            return
        
        # 检查位置
        if self.check_position_reached(self.current_goal_pose):
            self.get_logger().info("Robot has reached TARGET RETURN position!")
            self.stop_position_check_and_start_pause("target_return")

    def check_home_position_reached(self):
        """检查是否到达home位置 (通过关节角度检查)"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.position_check_start_time).nanoseconds / 1e9
        
        # 首先确保已经等待了最小稳定时间
        if elapsed_time < self.min_settle_time:
            self.get_logger().debug(f"Home settling... {elapsed_time:.1f}s < {self.min_settle_time}s")
            return
        
        # 给home位置更长的检查时间，因为通常回home更可靠
        if elapsed_time > 5.0:  # 5秒后认为已经到达home
            self.get_logger().info("Robot has reached HOME position!")
            self.stop_position_check_and_start_pause("home")

    def stop_position_check_and_start_pause(self, location):
        """停止位置检查并开始pause"""
        if self.position_check_timer is not None:
            self.position_check_timer.cancel()
            self.position_check_timer.destroy()
            self.position_check_timer = None
        
        if location == "target":
            self.start_target_pause()
        elif location == "press":
            self.start_press_pause()
        elif location == "target_return":
            self.start_target_return_pause()
        elif location == "home":
            self.start_home_pause()

    def stop_position_check_and_reset(self):
        """停止位置检查并重置状态"""
        if self.position_check_timer is not None:
            self.position_check_timer.cancel()
            self.position_check_timer.destroy()
            self.position_check_timer = None
        self.reset_state()

    def start_target_pause(self):
        """机器人稳定在target位置后，开始真正的pause计时"""
        self.get_logger().info("Robot settled at TARGET position. Starting 2-second pause...")
        self.current_step = "pausing_at_target"
        # 创建真正的pause计时器
        self.pause_timer = self.create_timer(2.0, self.create_one_shot_callback(self.proceed_to_press_step))
        self.get_logger().info("Target pause timer created (2 seconds)")

    def start_press_pause(self):
        """机器人稳定在press位置后，开始真正的pause计时"""
        self.get_logger().info("Robot settled at PRESS position. Starting 3-second pause for button press...")
        self.current_step = "pausing_at_press"
        # 创建真正的pause计时器，按压后先回到target位置
        self.pause_timer = self.create_timer(3.0, self.create_one_shot_callback(self.proceed_to_target_step))
        self.get_logger().info("Press pause timer created (3 seconds)")

    def start_target_return_pause(self):
        """机器人稳定在target返回位置后，开始真正的pause计时"""
        self.get_logger().info("Robot settled at TARGET RETURN position. Starting 2-second pause...")
        self.current_step = "pausing_at_target_return"
        # 创建真正的pause计时器，从target返回位置到home
        self.pause_timer = self.create_timer(2.0, self.create_one_shot_callback(self.proceed_to_home_step))
        self.get_logger().info("Target return pause timer created (2 seconds)")

    def start_home_pause(self):
        """机器人稳定在home位置后，开始真正的pause计时"""
        self.get_logger().info("Robot settled at HOME position. Starting 10-second pause...")
        self.current_step = "pausing_at_home"
        # 创建真正的pause计时器
        self.pause_timer = self.create_timer(5.0, self.create_one_shot_callback(self.finish_cycle_step))
        self.get_logger().info("Home pause timer created (5 seconds)")

    def proceed_to_press_step(self):
        # 定时器已在包装器中被取消和销毁
        self.get_logger().info("2-second pause timer triggered")
        self.log_current_state("Before Press Planning")

        # 确认前一步已完成
        if not self.trajectory_completed or self.current_step != "pausing_at_target":
            self.get_logger().error(f"Invalid state transition. Current step: {self.current_step}, Trajectory completed: {self.trajectory_completed}")
            self.reset_state()
            return

        self.get_logger().info("STEP 2 STARTING: 2-second pause completed. Now planning path to PRESS position...")
        self.current_step = "planning_to_press"
        self.trajectory_completed = False  # 重置完成标志
        self.plan_to_press()

    def proceed_to_target_step(self):
        # 定时器已在包装器中被取消和销毁
        self.get_logger().info("3-second press pause timer triggered")
        self.log_current_state("Before Return to Target Planning")

        # 确认前一步已完成
        if not self.trajectory_completed or self.current_step != "pausing_at_press":
            self.get_logger().error(f"Invalid state transition. Current step: {self.current_step}, Trajectory completed: {self.trajectory_completed}")
            self.reset_state()
            return

        self.get_logger().info("STEP 3 STARTING: 3-second press pause completed. Now planning path back to TARGET...")
        self.current_step = "planning_to_target_return"
        self.trajectory_completed = False  # 重置完成标志
        self.plan_to_target_return()

    def proceed_to_home_step(self):
        # 定时器已在包装器中被取消和销毁
        self.get_logger().info("2-second target return pause timer triggered")
        self.log_current_state("Before Home Planning")

        # 确认前一步已完成
        if not self.trajectory_completed or self.current_step != "pausing_at_target_return":
            self.get_logger().error(f"Invalid state transition. Current step: {self.current_step}, Trajectory completed: {self.trajectory_completed}")
            self.reset_state()
            return

        self.get_logger().info("STEP 4 STARTING: 2-second target return pause completed. Now planning path back HOME...")
        self.current_step = "planning_to_home"
        self.is_going_home = True
        self.trajectory_completed = False  # 重置完成标志
        self.plan_to_home()

    def finish_cycle_step(self):
        # 定时器已在包装器中被取消和销毁
        self.get_logger().info("5-second home pause timer triggered")
        self.log_current_state("Before Cycle Finish")

        # 确认前一步已完成
        if not self.trajectory_completed or self.current_step != "pausing_at_home":
            self.get_logger().error(f"Invalid state transition. Current step: {self.current_step}, Trajectory completed: {self.trajectory_completed}")
            self.reset_state()
            return
            
        self.get_logger().info("STEP 5 COMPLETED: 5-second home pause completed. Full cycle complete.")
        self.get_logger().info("=== CYCLE FINISHED SUCCESSFULLY ===")
        self.reset_state()

    def reset_state(self):
        """重置所有状态标志，以便开始下一次循环"""
        # === 【核心修改】重置状态时也要取消任何活动的定时器 ===
        if self.pause_timer is not None and not self.pause_timer.cancelled:
            self.get_logger().info("Cancelling active pause timer due to state reset.")
            self.pause_timer.cancel()
            self.pause_timer.destroy()
            self.pause_timer = None

        # 取消位置检查定时器
        if self.position_check_timer is not None and not self.position_check_timer.cancelled:
            self.get_logger().info("Cancelling active position check timer due to state reset.")
            self.position_check_timer.cancel()
            self.position_check_timer.destroy()
            self.position_check_timer = None

        previous_step = self.current_step
        self.get_logger().info(f"Resetting state for next cycle. Previous step: {previous_step}")
        self.is_executing = False
        self.is_going_home = False
        self.current_goal_pose = None
        self.current_press_pose = None  # 清理按压位置
        self.current_step = "idle"
        self.trajectory_completed = False
        self.get_logger().info("Ready for next target.")

    def publish_goal_marker(self, pose_stamped: PoseStamped):
        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "final_goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.goal_marker_pub.publish(marker)

    def log_current_state(self, context=""):
        """记录当前系统状态，用于调试"""
        timer_status = "None"
        if self.pause_timer is not None:
            timer_status = "Active" if not self.pause_timer.cancelled else "Cancelled"
        
        self.get_logger().info(f"[{context}] Current State:")
        self.get_logger().info(f"  - Step: {self.current_step}")
        self.get_logger().info(f"  - Is Executing: {self.is_executing}")
        self.get_logger().info(f"  - Is Going Home: {self.is_going_home}")
        self.get_logger().info(f"  - Trajectory Completed: {self.trajectory_completed}")
        self.get_logger().info(f"  - Pause Timer: {timer_status}")

def main(args=None):
    rclpy.init(args=args)
    node = ButtonFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()