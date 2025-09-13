# 自主乘梯机器人

这是一个用于电梯操作任务的机器人ROS 2仓库。

## 简介

该仓库包含了一系列用于控制机器人在电梯环境中执行任务的软件包。这些任务包括使用机械臂进行精确操作（例如按下电梯按钮），以及通过移动底盘在环境中导航。该系统主要使用深度相机进行环境感知，并使用MoveIt 2进行运动规划。



## 软件包

该仓库包含以下软件包：

* **agv_controller**: 控制AGV移动底盘。
* **decision_maker**: 顶层决策包，用于管理电梯操作任务的分配和执行。
* **debugger**: 包含用于调试和可视化的工具节点。
* **elevator_arm_control**: 控制用于电梯任务的机械臂。
* **elevator_perception**: 处理电梯场景的感知，包括按钮检测与定位。
* **jaka_lumi_moveit_config**: JAKA-Lumi机械臂的MoveIt 2配置文件。
* **orbbec_camera**: Orbbec相机的ROS 2驱动程序。
* **robot_description**: 包含机器人的URDF和其他描述文件。

## 主要节点目录及功能

以下是本仓库中主要节点的功能和接口信息。

### 1. `elevator_perception`

* **路径**: `src/elevator_perception/`
* **功能**: 负责环境感知，特别是电梯按钮的检测和三维空间定位。

| 节点                        | 功能描述                                                                               | 订阅的Topic                                                                                                                                                     | 发布的Topic                                                                                         |
| :-------------------------- | :------------------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------- |
| **`button_detector_node`**  | 订阅摄像头图像流，使用YOLO模型检测图像中的电梯按钮，并发布2D检测框结果。               | `/camera/color/image_raw` (sensor\_msgs/Image)                                                                                                                  | `/detection_results` (vision\_msgs/Detection2DArray)                                                |
| **`button_target_planner`** | 结合2D检测结果和深度相机信息，计算出目标按钮在三维空间中的精确坐标，并发布为目标位姿。 | `/detection_results` (vision\_msgs/Detection2DArray)<br>`/camera/depth/image_raw` (sensor\_msgs/Image)<br>`/camera/depth/camera_info` (sensor\_msgs/CameraInfo) | `/target_pose` (geometry\_msgs/PoseStamped)<br>`/visualization_marker` (visualization\_msgs/Marker) |
| **`button_3d_visualizer`**  | 订阅3D检测结果，并在RViz中将其可视化，方便调试。                                       | `/target_pose` (geometry\_msgs/PoseStamped)                                                                                                                     | `/visualization_marker` (visualization\_msgs/Marker)                                                |

### 2. `elevator_arm_control`

* **路径**: `src/elevator_arm_control/`
* **功能**: 负责控制机械臂完成指定的动作，如移动到目标点、执行按压。

| 节点                  | 功能描述                                                                                         | 订阅的Topic                                                                              | 提供的Service                      |
| :-------------------- | :----------------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------- | :--------------------------------- |
| **`arm_controller`**  | 接收目标位姿，控制机械臂移动到指定位置。它通常是MoveIt的上层封装，用于执行具体的轨迹规划和运动。 | `/target_pose` (geometry\_msgs/PoseStamped)                                              | `press_button` (std\_srvs/Trigger) |
| **`button_follower`** | 实现基于视觉伺服的精细控制，用于在最后阶段精确对准并按下按钮。                            | `/target_pose` (geometry\_msgs/PoseStamped)<br>`/joint_states` (sensor\_msgs/JointState) | -                                  |

### 3. `agv_controller`

* **路径**: `src/agv_controller/`
* **功能**: 负责控制机器人底盘的移动。

| 节点                        | 功能描述                                              | 订阅的Topic                               | 发布的Topic                       |
| :-------------------------- | :---------------------------------------------------- | :---------------------------------------- | :-------------------------------- |
| **`agv_target_controller`** | 接收导航目标点，控制AGV移动到电梯前方的最佳操作位置。 | `/goal_pose` (geometry\_msgs/PoseStamped) | `/cmd_vel` (geometry\_msgs/Twist) |

### 4. `decision_maker`

* **路径**: `src/decision_maker/`
* **功能**: 作为系统的“大脑”，协调其他节点完成整个电梯操作任务流程。

| 节点                  | 功能描述                                                                                                                                          | 订阅的Topic                         | 发布的Topic                                                                              |
| :-------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------ | :---------------------------------- | :--------------------------------------------------------------------------------------- |
| **`task_assignment`** | 状态机，负责任务调度。例如，先调用`agv_controller`移动到底盘，然后触发`elevator_perception`进行按钮检测，最后命令`elevator_arm_control`执行按压。 | `/task_feedback` (std\_msgs/String) | `/goal_pose` (geometry\_msgs/PoseStamped)<br>`/target_pose` (geometry\_msgs/PoseStamped) |

### 5. `orbbec_camera`

* **路径**: `src/orbbec_sensor_node/orbbec_camera/`
* **功能**: 奥比中光Orbber 3D相机的驱动节点。

| 节点              | 功能描述                                                                          | 发布的Topic                                                                                     | 提供的Service                                          |
| :---------------- | :-------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------- | :----------------------------------------------------- |
| **`camera_node`** | 启动并管理Orbbec相机硬件，发布包括彩色图像、深度图像、点云和IMU在内的多种数据流。 | `/camera/color/image_raw`<br>`/camera/depth/image_raw`<br>`/camera/pointcloud`<br>`/camera/imu` | `get_camera_info` (orbbec\_camera\_msgs/GetCameraInfo) |

## 依赖

该项目依赖于ROS 2 Humble和以下主要软件包：

* `rclpy`, `rclcpp`
* `moveit_ros`
* `gazebo_ros`
* `cv_bridge`, `OpenCV`
* `tf2_ros`
* `sensor_msgs`, `geometry_msgs`, `vision_msgs`, `std_msgs`

## 安装

1.  **克隆仓库**:
    ```bash
    git clone git@github.com:yyyyymzzzzz/elevator_manipulation.git
    ```
2.  **修改部分依赖路径**
3.  **构建软件包**:
    ```bash
    colcon build --symlink-install
    ```

## 使用

1.  **Source工作空间**:
    ```bash
    source install/setup.sh
    ```
2.  **使用合并启动脚本启动**:
    ```bash 
    ros2 launch startup_contrainer.py
    ```
3. **启动任务发布节点**
    ```
    ros2 launch startup_control.py
    ```
    具体目标点和可在`config/task.yaml`自定义。

## 贡献

我们欢迎对该项目的贡献。如果您想做出贡献，请随时Fork该仓库并提交Pull Request。