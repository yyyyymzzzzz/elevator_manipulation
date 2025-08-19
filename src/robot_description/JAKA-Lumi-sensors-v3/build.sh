#!/bin/bash

# JAKA Lumi Sensors v3 - ROS2 构建脚本

echo "开始构建 jaka_lumi_sensors_v3 ROS2 包..."

# 检查是否在ROS2工作空间中
if [ ! -f "../../../install/setup.bash" ] && [ ! -f "../../install/setup.bash" ]; then
    echo "警告: 请确保在ROS2工作空间的src目录中运行此脚本"
    echo "推荐目录结构: ~/ros2_ws/src/jaka_lumi_sensors_v3/"
fi

# 返回到工作空间根目录
cd ../..

# 构建包
echo "正在构建包..."
colcon build --packages-select jaka_lumi_sensors_v3

if [ $? -eq 0 ]; then
    echo "构建成功!"
    echo ""
    echo "要使用此包，请运行:"
    echo "source install/setup.bash"
    echo ""
    echo "然后可以使用以下命令:"
    echo "ros2 launch jaka_lumi_sensors_v3 display.launch.py"
    echo "ros2 launch jaka_lumi_sensors_v3 gazebo.launch.py"
else
    echo "构建失败，请检查依赖和错误信息"
fi
