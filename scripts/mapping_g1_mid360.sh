#!/bin/bash

# 1. 加载 ROS Noetic 基础环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "错误: 未找到 /opt/ros/noetic/setup.bash"
    exit 1
fi

# 2. 加载 SLAM 工作空间环境
# 使用 $HOME 确保路径在不同 shell 下的兼容性
if [ -f "$HOME/slam_ws/devel/setup.bash" ]; then
    source "$HOME/slam_ws/devel/setup.bash"
else
    echo "错误: 未找到 $HOME/slam_ws/devel/setup.bash"
    exit 1
fi

# 3. 启动 FAST_LIVO 映射节点
echo "正在启动 FAST_LIVO Mapping (Unitree G1 Mid360)..."
roslaunch fast_livo mapping_g1_mid360_lio.launch