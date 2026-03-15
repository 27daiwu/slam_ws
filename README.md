# SLAM_WS

基于宇树 G1 + Mid-360 的 ROS1 SLAM 与导航工作空间。

本工作空间当前以 **FAST-LIVO2 建图** 为核心，后续规划采用：

- **FAST-LIVO2**：3D 激光-惯性建图 / 位姿估计
- **AMCL**：2D 地图定位
- **2D 栅格地图导航**：室内平地导航

本仓库的目标不是一次性完成所有功能，而是按照“先建图、再定位、再导航”的思路，逐步打通整套技术链路。

---

## 1. 项目简介

本项目面向宇树 G1 机器人，使用机载 **Livox Mid-360** 激光雷达与 IMU 数据，完成以下任务：

- Mid-360 DDS 数据桥接到 ROS1
- 点云 / IMU 坐标关系与 TF 配置
- FAST-LIVO2 实时建图与位姿估计
- 3D 点云地图保存
- 3D 点云转 2D 栅格地图
- 后续接入 AMCL 做 2D 定位
- 后续接入 2D 导航栈实现自主导航

当前阶段重点是：

1. 稳定获取 Mid-360 点云和 IMU
2. 正确桥接到 ROS1
3. 使用 FAST-LIVO2 进行建图
4. 生成后续导航可用的 2D 地图

---

## 2. 工作空间结构

```bash
SLAM_WS/
├── build/
├── devel/
├── doc/
├── maps/
├── scripts/
├── src/
│   ├── FAST-LIVO2/
│   ├── livox_ros_driver/
│   ├── Livox-SDK-master/
│   ├── pcd_to_2d_grid_map/
│   ├── rpg_vikit/
│   ├── unitree_livox_converter/
│   ├── unitree_mid360_ros1_bridge/
│   ├── CMakeLists.txt
│   └── .catkin_workspace
└── .gitignore
```
## 3. 目录说明

### `build/`
Catkin 编译生成目录。

### `devel/`
Catkin 开发环境目录，包含编译后生成的环境脚本与运行时文件。

### `doc/`
项目文档目录，建议存放：

- 传感器说明
- 坐标系定义
- 建图流程文档
- 导航技术路线文档
- 调试记录
- 参数说明

### `maps/`
地图输出目录，建议存放：

- FAST-LIVO2 导出的 `.pcd` 地图
- 2D 栅格地图 `.pgm`
- 2D 栅格地图 `.yaml`
- 地图处理过程中的中间文件

### `scripts/`
辅助脚本目录，建议存放：

- 启动脚本
- 建图脚本
- 录包脚本
- 点云转 2D 地图脚本
- 环境变量初始化脚本

### `src/`
ROS1 工作空间源码目录。

---

## 4. 各功能包说明

### `FAST-LIVO2/`
FAST-LIVO2 主体源码。

**主要功能：**

- 融合 Mid-360 点云与 IMU
- 输出实时位姿估计
- 输出局部 / 全局点云地图
- 用于 3D 建图与里程计估计

**在本项目中的定位：**

- 作为建图主算法
- 不直接负责完整导航
- 为后续 2D 地图生成提供 3D 点云地图数据

---

### `livox_ros_driver/`
Livox 官方 ROS 驱动依赖包。

**主要作用：**

- 提供 `livox_ros_driver/CustomMsg`
- 为 FAST-LIVO2 适配 Livox 数据格式提供支持

**在本项目中的定位：**

- 为点云格式转换与 FAST-LIVO2 输入提供基础消息类型

---

### `Livox-SDK-master/`
Livox SDK 源码。

**主要作用：**

- 提供底层 Livox 接口支持
- 为相关驱动和工具提供基础依赖

---

### `pcd_to_2d_grid_map/`
PCD 点云地图转 2D 栅格地图工具包。

**主要功能：**

- 读取 `.pcd` 地图
- 按高度范围过滤点云
- 将 3D 点云投影到地面平面
- 生成导航使用的 2D occupancy grid 地图

**在本项目中的定位：**

- 连接“3D 建图”和“2D 导航”的核心桥梁之一

---

### `rpg_vikit/`
FAST-LIVO2 相关依赖包。

**主要作用：**

- 提供视觉 / 几何相关工具支持
- 作为 FAST-LIVO2 的依赖项存在

---

### `unitree_livox_converter/`
宇树 Mid-360 点云格式转换包。

**主要功能：**

- 将桥接后的 `sensor_msgs/PointCloud2`
- 转换为 FAST-LIVO2 更适合处理的 `Livox CustomMsg` 格式

**在本项目中的定位：**

- 解决宇树 Mid-360 ROS1 数据与 FAST-LIVO2 输入格式之间的适配问题

**说明：**

当前项目中，点云时间字段 `time` 的语义与 FAST-LIVO2 所需 `offset_time` 并不完全一致，因此本包中相关时间转换逻辑需要特别注意。

---

### `unitree_mid360_ros1_bridge/`
宇树 Mid-360 DDS 到 ROS1 的桥接包。

**主要功能：**

- 订阅宇树 DDS 点云与 IMU 数据
- 发布 ROS1 话题
- 为 FAST-LIVO2 与后续导航提供原始传感器输入

**在本项目中的定位：**

- 整个系统的传感器接入入口

---

## 5. 当前技术路线

当前项目确定的技术路线为：

### 建图阶段

- Mid-360 点云 + IMU
- 通过 `unitree_mid360_ros1_bridge` 接入 ROS1
- 通过 `unitree_livox_converter` 适配 FAST-LIVO2 输入格式
- 使用 FAST-LIVO2 进行 3D 建图

### 地图处理阶段

- 保存 3D `.pcd` 地图
- 使用 `pcd_to_2d_grid_map` 将 3D 点云地图转换为 2D 栅格地图

### 定位阶段（后续）

- 使用实时点云生成 2D 激光观测
- 使用 AMCL 在已知 2D 地图中定位

### 导航阶段（后续）

- 基于 2D 地图做全局规划
- 基于局部障碍做局部避障
- 输出速度命令到 G1 控制接口

---

## 6. 数据流说明

### 建图数据流

```text
Mid-360 DDS 点云 / IMU
        ↓
unitree_mid360_ros1_bridge
        ↓
ROS1 点云 / IMU 话题
        ↓
unitree_livox_converter
        ↓
Livox CustomMsg
        ↓
FAST-LIVO2
        ↓
实时位姿 + 3D 点云地图
```
# 导航数据流（规划中）

```text
3D PCD 地图
    ↓
pcd_to_2d_grid_map
    ↓
2D 栅格地图（pgm + yaml）
    ↓
AMCL
    ↓
map -> odom
    ↓
2D 导航栈
    ↓
cmd_vel / 运动控制命令
    ↓
G1
```
