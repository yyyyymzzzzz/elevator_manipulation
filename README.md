# 自主乘梯机器人

这是一个用于电梯操作任务的机器人ROS 2仓库。

## 简介

该仓库包含了一系列用于控制机器人在电梯环境中执行任务的软件包。这些任务包括使用机械臂进行精确操作（例如按下电梯按钮），以及通过移动底盘在环境中导航。该系统主要使用深度相机进行环境感知，并使用MoveIt 2进行运动规划。

项目使用ROS2 Humble，建议在Ubuntu 22.04系统中运行。

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
* **功能**: 负责环境感知，特别是电梯按钮和面板的检测和三维空间定位。

| 节点                        | 功能描述                                                                               | 订阅的Topic                                                                                                                                                     | 发布的Topic                                                                                         |
| :-------------------------- | :------------------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------- |
| **`button_detector_node`**  | 订阅相机图像流，使用YOLO模型检测图像中的电梯按钮，并发布2D检测框结果。                 | `/camera/color/image_raw` (sensor\_msgs/Image)                                                                                                                  | `/detection_results` (vision\_msgs/Detection2DArray)                                                |
| **`button_target_planner`** | 结合2D检测结果和深度相机信息，计算出目标按钮在三维空间中的精确坐标，并发布为目标位姿。 | `/detection_results` (vision\_msgs/Detection2DArray)<br>`/camera/depth/image_raw` (sensor\_msgs/Image)<br>`/camera/depth/camera_info` (sensor\_msgs/CameraInfo) | `/target_pose` (geometry\_msgs/PoseStamped)<br>`/visualization_marker` (visualization\_msgs/Marker) |
| **`button_3d_visualizer`**  | 订阅3D检测结果，并在RViz中将其可视化，方便调试。                                       | `/target_pose` (geometry\_msgs/PoseStamped)                                                                                                                     | `/visualization_marker` (visualization\_msgs/Marker)                                                |
| **`panel_perception`**      | 订阅相机画面识别结果，返回是否到达目标楼层                                             | `/detector/result` (String)                                                                                                                                     | `/look_floor/completed` (String)                                                                    |

### 2. `elevator_arm_control`

* **路径**: `src/elevator_arm_control/`
* **功能**: 负责控制机械臂完成指定的动作，如移动到目标点、执行按压。

| 节点                  | 功能描述                                                                                         | 订阅的Topic                                                                              | 提供的Service                      |
| :-------------------- | :----------------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------- | :--------------------------------- |
| **`arm_controller`**  | 接收目标位姿，控制机械臂移动到指定位置。它通常是MoveIt的上层封装，用于执行具体的轨迹规划和运动。 | `/target_pose` (geometry\_msgs/PoseStamped)                                              | `press_button` (std\_srvs/Trigger) |
| **`button_follower`** | 实现基于视觉伺服的精细控制，用于在最后阶段精确对准并按下按钮。                                   | `/target_pose` (geometry\_msgs/PoseStamped)<br>`/joint_states` (sensor\_msgs/JointState) | -                                  |

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
* `cv_bridge`, `OpenCV`
* `tf2_ros`
* `sensor_msgs`, `geometry_msgs`, `vision_msgs`, `std_msgs`

## 训练
- 采集数据
  - 在`/config/button_detector_params.yaml`中开启内录并正常启动程序，采集电梯按钮图像。
  - 采集的图片与实际运行时进行推理的图片完全一致。
- 标注数据
  - 使用[labelImg](https://github.com/HumanSignal/labelImg)标注按钮图像。
  - 注意数据集导出格式使用yolo格式。
- 划分数据集
  - 使用`/train/dataloader.py`将采集到的图像随机划分成训练集和验证集。能够自动将标签与同名图片进行匹配。
  - 划分比例为8:2，即80%为训练集，20%为验证集。
  - 注意修改标签目录、原始图片目录、导出目录。
- 开始训练
  - 使用`/train/train.py`开始训练模型。
  - 注意修改配置文件路径。
- 模型导出
  - 如果需要在rk3588上部署NPU推理，在训练完成后，需要使用`/train/pt_to_rknn.py`导出模型。
  - 安装 RKNN-Toolkit2 ，需要根据系统选择对应的 whl 包。如果没有 whl，需要去[rknn-tookit2仓库](https://github.com/rockchip-linux/rknn-toolkit2/tree/master/rknn-toolkit2/packages)下载并安装。
    ```bash 
    pip install rknn_toolkit2-*-cp38-*-linux_x86_64.w
    ```
  - 脚本会先将`*.pt`模型转换为`*.onnx`模型，然后将`*.onnx`模型转换为所需的`*.rknn`模型。
- 部署NPU推理
  - 将导出的`*.rknn`模型部署到rk3588平台需要进行以下操作。
  - 需要在rk3588平台上安装RKNN-Toolkit2运行时库，并将识别节点由`button_detector_node`替换为`button_detector_npu_node`。
  - 安装 RKNN-Toolkit-lite2 ，需要根据系统选择对应的 whl 包。如果没有 whl，需要去[rknn-tookit-lite2仓库](https://github.com/rockchip-linux/rknn-toolkit2/tree/master/rknn_toolkit_lite2/packages)下载并安装。
    ```bash 
    pip install rknn_toolkit_lite2-*.whl
    ```
  - 运行时`.so`库可以在[rknn-tookit2仓库](https://github.com/airockchip/rknn-toolkit2/blob/master/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so)下载，需要将`librknnrt.so`复制到rk3588平台的`/usr/lib`目录下。
  

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
4. **如果使用Orbbec相机，需要参考官方SDK仓库接口权限**
   
    [OrbbecSDK_ROS2 仓库链接](https://github.com/orbbec/OrbbecSDK_ROS2)

## 使用
- 在Lumi平台中启动完整功能包
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
- 在rk3588平台启动视觉模块
  1.  **修改相对坐标系名称**:
      由于没有运行机器人控制节点，缺少机器人坐标系，需要调整坐标系配置
      需要在`button_3d_visualizer_params.yaml`, `button_target_planner_params.yaml`, `realtime_button_target_planner_params.yaml`参数配置中修改`target_link`为`camera_link`。
  2.  **修正所有绝对路径**:
      将`/config/*.yaml`中规定的路径修正，以正确读取文件。
  3.  **修正相机启动脚本为对应的相机型号**:
      需要在`startup_vision.py`中将相机切换为匹配的型号。
  4.  **修改法向量方向和目标展示方式**:
      具体修改方法参考`realtime_button_target_planner_params.yaml`中的注释。
  5.  **修改模型推理方法**:
      由于rk3588平台计算能力有限，需要将模型推理方法切换为NPU推理。
      需要在`startup_vision.py`中修改启动的识别节点为`button_detector_npu_node`。
  6.  **Source工作空间**:
      ```bash
      source install/setup.sh
      ```
  7. **启动视觉部分发布节点**
      ```
      ros2 launch startup_vision.py
      ```

## 贡献

我们欢迎对该项目的贡献。如果您想做出贡献，请随时Fork该仓库并提交Pull Request。

## 作者

**Mingzhe Ye**  
上海交通大学

## 许可证

本项目采用 MIT License 开源许可证 - 详情请参阅 [LICENSE](LICENSE) 文件。