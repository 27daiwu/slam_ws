#!/bin/bash

# 设置 ROS Noetic 环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
else
    echo "错误: 找不到 /opt/ros/noetic/setup.bash"
    exit 1
fi

# 设置工作空间环境
if [ -f "$HOME/slam_ws/devel/setup.bash" ]; then
    source "$HOME/slam_ws/devel/setup.bash"
else
    echo "错误: 找不到 $HOME/slam_ws/devel/setup.bash"
    exit 1
fi

# 启动 ROS Launch 文件
echo "正在启动 Unitree Mid360 ROS1 tf..."
roslaunch unitree_mid360_ros1_bridge mid360_tf.launch