# GLINS (GNSS-LiDAR-Inertial Navigation System)

## 项目简介 (Project Overview)
GLINS 是一个基于因子图优化 (Factor Graph Optimization) 的多传感器紧耦合里程计与建图系统。它融合了 LiDAR (激光雷达)、IMU (惯性测量单元) 和 GNSS (全球导航卫星系统) 数据，利用 GTSAM 的 ISAM2 (Incremental Smoothing and Mapping) 实现高精度、鲁棒的全局位姿估计和地图构建。即使在单一传感器失效或退化的复杂环境中，该系统也能通过多源信息互补提供可靠的状态估计。

项目包含两个主要的 ROS package 目录：
1. **`glins`**: 核心的多传感器融合与建图包。
2. **`rtklib`**: GNSS 数据处理基础库及 ROS 接口封装包。

---

## 核心架构与模块 (System Architecture & Modules)

### 1. `glins` (融合与建图核心包)
这是整个系统的核心包，主要负责前端里程计提取、IMU 预积分以及后端因子图优化。
**核心组成与目录结构**：
- **`include/factor/`**: 包含构建因子图所需的各种残差约束因子 (Factors)。
  - `CarrierPhaseFactor.h` / `DopplerFactor.h` / `GnssFactor.h`: 处理 GNSS 伪距、载波相位及多普勒观测的因子。
  - `LidarFactor.h`: 处理 LiDAR 点云特征匹配的因子。
  - `MotionFactor.h`: 运动学约束因子。
  - `NhcFactor.h`: 非完整性约束 (Non-Holonomic Constraint) 因子，通常用于车载等具有平面运动特性的平台。
- **`include/lidar/`, `include/imu/`, `include/gnss/`**: 分别对应不同传感器的数据处理模块。
- **`src/`**: 包含核心的 ROS Node 入口点。
  - `test_gins.cpp`: GNSS-IMU 组合导航测试节点。
  - `test_lio.cpp`: 核心的 LiDAR-IMU-GNSS 全系统融合主节点。
  - `test_rtk.cpp`: RTK (Real-Time Kinematic) 定位测试。
- **`config/`**: 存放了丰富的 YAML 参数配置文件 (例如针对不同雷达 Ouster, Velodyne VLP, Pandar 以及不同场景的参数)。
- **`python/` / `scripts/`**: 包含评测轨迹的脚本 (如 `gt2tum.py`, `calerr.py`, 数据转换及画图工具 `plotpos.py`)。

### 2. `rtklib` (GNSS 处理基础包)
该目录包装了开源 RTKLIB，并加入了 ROS 消息通信接口。
**核心组成**：
- **`src/`**: 包含了原生的 RTKLIB 核心文件 (例如 `rtkpos.cpp`, `pntpos.cpp`, `lambda.cpp`, `ppp_ar.cpp` 等)，负责底层的卫星解算与星历解析。
- **`msg/`**: 定义了自定义的 ROS Message。
  - `GNSS_Info.msg`, `obsdt.msg`, `satdt.msg`, `sat_state.msg` 等用于在 `rtklib` 解析节点与主建图节点之间传递原始观测值、星历及卫星状态。

---

## 因子图优化设计 (Factor Graph Design)
系统的后端基于 **GTSAM (ISAM2)**。整个优化框架是一个紧耦合的图结构，包含：
1. **先验约束**: 系统初始状态及外参等。
2. **IMU 预积分因子**: 利用高频 IMU 提供的相邻帧之间相对位姿约束。
3. **LiDAR 里程计因子**: 通过点云与局部地图或特征的配准提供增量运动约束。
4. **GNSS 原始观测因子**: 系统并没有直接采用 GNSS 输出的经纬度，而是深入到了原始数据层面（伪距、多普勒、载波相位），提供了 `GnssFactor` 等。这种紧耦合的方法能有效处理可见卫星数不足的场景。

---

## 运行与使用说明 (How to Run)

### 编译项目
项目基于标准的 ROS 构建系统 (catkin_make)：
```bash
mkdir -p ~/catkin_ws/src
# 将项目链接或复制到 src 目录下
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 运行节点
启动系统通常由 `glins/launch` 目录下的 Launch 文件完成。
- **启动完整 LiDAR-Inertial 系统**:
  ```bash
  roslaunch glins run.launch
  ```
- **离线数据包测试**:
  可使用 `run_test_lio.launch` 来进行基于 ROSBag 的数据集复现和测试。
  ```bash
  roslaunch glins run_test_lio.launch
  ```
- **适配不同的 LiDAR**:
  Launch 目录下提供了专用的配置文件启动项，如 `ouster128.launch` (针对 Ouster 雷达) 和 `pandar.launch` (针对禾赛 Pandar 雷达)。

### 参数配置
在运行之前，请务必在 `glins/config` 下选择正确的 `.yaml` 配置文件，以匹配当前使用的传感器标定外参（LiDAR、IMU 和 GNSS 天线之间的外参）、GNSS 天线位置和噪声参数。
