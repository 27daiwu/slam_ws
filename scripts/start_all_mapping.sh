#!/bin/bash

# 1. 环境初始化
# 加载 ROS Noetic 基础环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source "/opt/ros/noetic/setup.bash"
else
    echo "错误: 未找到 /opt/ros/noetic/setup.bash"
    exit 1
fi

# 加载 SLAM 工作空间环境
if [ -f "$HOME/slam_ws/devel/setup.bash" ]; then
    source "$HOME/slam_ws/devel/setup.bash"
else
    echo "错误: 未找到 $HOME/slam_ws/devel/setup.bash"
    exit 1
fi

# 2. 启动各个组件
echo "1/4 正在启动 Unitree Mid360 ROS1 Bridge..."
roslaunch unitree_mid360_ros1_bridge mid360_bridge.launch &
sleep 2 # 等待基础驱动初始化

echo "2/4 正在启动 Unitree Mid360 ROS1 tf..."
roslaunch unitree_mid360_ros1_bridge mid360_tf.launch &
sleep 1

echo "3/4 正在启动 unitree_livox_converter..."
roslaunch unitree_livox_converter pointcloud2_to_custom.launch &
sleep 1

echo "4/4 正在启动 FAST_LIVO Mapping (Unitree G1 Mid360)..."
# 最后一个进程不加 &，以保持终端不关闭，或根据需要决定是否也放后台
roslaunch fast_livo mapping_g1_mid360_lio.launch

# 3. 退出处理
# 当关闭脚本时，确保杀掉所有后台进程
wait