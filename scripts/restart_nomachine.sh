#!/bin/bash

# 停止显示管理器
sudo systemctl stop display-manager

# 重启 NoMachine 服务
sudo /etc/NX/nxserver --restart

# 切换至运行级别 3 (多用户文本模式)
sudo init 3