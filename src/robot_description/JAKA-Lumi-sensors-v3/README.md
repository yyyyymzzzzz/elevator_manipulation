# JAKA Lumi Sensors v3 - ROS2 Port

这是JAKA Lumi Sensors v3机器人的ROS2版本描述包。

## 从ROS1迁移的变化

### 主要变化
1. **包名变更**: `JAKA-Lumi-sensors-v3` → `jaka_lumi_sensors_v3` (遵循ROS2命名规范)
2. **构建系统**: Catkin → Ament CMake
3. **Launch文件**: XML格式 → Python格式
4. **包依赖**: 更新为ROS2对应包

### 文件变化

#### 新增文件
- `launch/display.launch.py` - ROS2 Python launch文件用于显示
- `launch/gazebo.launch.py` - ROS2 Python launch文件用于Gazebo仿真
- `urdf/jaka_lumi_sensors_v3.urdf` - 更新了包名引用的URDF文件

#### 更新文件
- `package.xml` - 更新为format="3"，使用ament_cmake构建系统
- `CMakeLists.txt` - 从catkin迁移到ament_cmake

#### 保留文件
- `urdf/JAKA-Lumi-sensors-v3.urdf` - 原始URDF文件（保留作为备份）
- `meshes/` - 所有STL文件保持不变
- `config/` - 配置文件保持不变

## 使用方法

### 构建包
```bash
cd ~/ros2_ws
colcon build --packages-select robot_description
source install/setup.bash
```

### 启动显示
```bash
ros2 launch jaka_lumi_sensors_v3 display.launch.py
```

### 启动Gazebo仿真
```bash
ros2 launch jaka_lumi_sensors_v3 gazebo.launch.py
```

## 依赖要求

确保安装了以下ROS2包：
- `robot_state_publisher`
- `rviz2`
- `joint_state_publisher_gui`
- `gazebo_ros`
- `gazebo_ros_pkgs`
- `tf2_ros`

## 注意事项

1. ROS2中不再使用`roslaunch`，改用`ros2 launch`
2. RViz现在叫`rviz2`
3. Gazebo spawn使用`spawn_entity.py`而不是`spawn_model`
4. 静态变换发布器从`tf`包移到了`tf2_ros`包
