# GLINS 项目架构与启动手册

> 文档版本：2026-07-27
> 适用分支：`fix_test_glins`
> 功能代码基线：`9367ffe`
> 远程服务器：`zbwang@172.30.5.103:17253`

本文档面向当前服务器上的 GLINS 工程，说明：

1. 项目的目录结构和主要模块；
2. LiDAR、IMU、GNSS 数据如何进入因子图；
3. 三个主要可执行程序的区别；
4. 如何在服务器上编译、运行120秒测试、运行完整路线；
5. 如何启动 RViz、查看结果和排查故障；
6. 当前代码和实验尚存的限制。

终端阅读：

```bash
less -R PROJECT_ARCHITECTURE_AND_RUNBOOK_CN.md
```

在 `less` 中：

- 按 `/关键词` 搜索；
- 按 `n` 跳到下一个匹配；
- 按 `b` 向上翻页；
- 按空格向下翻页；
- 按 `q` 退出。

---

## 1. 项目定位

GLINS 是一个基于因子图优化的 GNSS、IMU、LiDAR 多传感器融合系统。

系统的目标状态主要包括：

- 位置；
- 姿态；
- 速度；
- IMU 加速度计零偏；
- IMU 陀螺仪零偏；
- GNSS 接收机钟差或相关 GNSS 状态；
- 载波相位模糊度等扩展状态。

核心后端使用：

- GTSAM；
- ISAM2 增量优化；
- IMU 预积分；
- LiDAR 点到平面/特征约束；
- GNSS 伪距、载波相位和多普勒约束。

项目不是一个单独的“纯 LIO”程序，而是：

```text
LiDAR 前端
    +
IMU 预积分与状态传播
    +
RTKLIB GNSS 前端
    +
GTSAM 因子图后端
    +
轨迹、地图、日志和评估工具
```

---

## 2. 本地、服务器和数据路径

### 2.1 本地仓库

```text
/Users/wangzhibo/Desktop/博士研究/GLINS
```

本地 Git 信息：

```text
origin: git@github.com:wangzika/GLINS.git
branch: fix_test_glins
```

### 2.2 服务器 Catkin 工作空间

```text
/home/zbwang/GLINS
```

主要目录：

```text
/home/zbwang/GLINS/
├── build/                 Catkin/CMake 编译中间文件
├── devel/                 编译产生的可执行文件和 ROS 环境
├── src/
│   ├── CMakeLists.txt     Catkin 顶层链接
│   ├── glins/             GLINS 核心 ROS 包
│   └── rtklib/            RTKLIB ROS 包
├── .deps/                 服务器本地依赖，例如 GTSAM
└── output/                部分历史输出
```

当前服务器源码是通过文件同步放入 `src/` 的。若尚未执行 Git 接入步骤，
`/home/zbwang/GLINS/src` 还不是 Git 工作树。

### 2.3 公共数据

数据根目录：

```text
/data/zbwang/public/UrbanNav_HK_Medium_20210517
```

120秒测试 bag：

```text
/data/zbwang/public/UrbanNav_HK_Medium_20210517/medium_public_0_120_uncompressed.bag
```

完整路线 bag：

```text
/data/zbwang/public/UrbanNav_HK_Medium_20210517/full_original/ros/UrbanNav-HK_TST-20210517_sensors.bag
```

完整 bag 信息：

```text
时长：785.45秒，约13分05秒
大小：32.8 GB
总消息：386309
IMU：314194
Velodyne：7848
GPS周：2158
周内秒：约95593.55至96379.00
```

### 2.4 实验结果目录

推荐统一放在：

```text
/data/zbwang/results/glins_public_ablation
```

不要反复使用同一个结果目录，否则日志和轨迹可能被覆盖。

---

## 3. 源码目录架构

仓库根目录的两个主要 ROS 包：

```text
GLINS/
├── glins/
└── rtklib/
```

### 3.1 `glins/`：融合与建图核心

```text
glins/
├── CMakeLists.txt
├── package.xml
├── config/
├── launch/
├── include/
│   ├── factor/
│   ├── gnss/
│   ├── imu/
│   ├── lidar/
│   ├── mapping/
│   └── utility.h
├── msg/
├── scripts/
├── src/
└── srv/
```

各目录作用：

| 目录 | 作用 |
| --- | --- |
| `config/` | 传感器参数、外参、噪声、实验开关和 RTKLIB 配置 |
| `launch/` | 加载参数并启动测试节点、RViz |
| `include/lidar/` | 点云投影、去畸变、特征提取 |
| `include/imu/` | IMU 预积分、状态传播和融合 |
| `include/gnss/` | GNSS 解码、观测组织和GNSS因子处理 |
| `include/mapping/` | LiDAR匹配、地图维护、GTSAM图优化和结果发布 |
| `include/factor/` | 伪距、载波、多普勒、LiDAR、运动学和NHC因子 |
| `src/` | 三个主要实验入口 |
| `scripts/` | 批量运行、评估和绘图工具 |
| `msg/` | 自定义 ROS 消息 |
| `srv/` | 保存地图等 ROS 服务 |

### 3.2 `rtklib/`：GNSS基础处理

主要负责：

- 读取 RINEX 观测文件；
- 读取广播星历；
- 卫星位置和钟差计算；
- 单点定位和差分定位；
- 伪距、载波相位、多普勒观测预处理；
- 模糊度和周跳相关基础处理；
- 向 GLINS 提供 GNSS 数据结构。

构建后生成静态库：

```text
rtklib
```

GLINS 编译时通过 Catkin 依赖该包。

---

## 4. 主要类和模块

### 4.1 参数中心：`ParamServer`

文件：

```text
glins/include/utility.h
```

负责从 ROS 参数服务器读取：

- IMU、LiDAR、GNSS话题；
- 坐标系名称；
- 传感器类型；
- LiDAR扫描线数和水平分辨率；
- IMU噪声和随机游走；
- LiDAR到IMU外参；
- GNSS天线到IMU外参；
- GNSS时间同步容差；
- 因子图和消融实验开关；
- 地图保存和可视化设置。

大部分核心类继承 `ParamServer`，所以 launch/YAML 参数会进入各模块。

### 4.2 LiDAR前端

#### `ImageProjection`

文件：

```text
glins/include/lidar/imageProjection.cpp
glins/include/lidar/imageProjection.h
```

主要职责：

- 接收点云；
- 缓存并插值IMU；
- 点云运动补偿；
- 距离图投影；
- 按扫描线整理点；
- 输出去畸变点云和点云信息。

#### `FeatureExtraction`

文件：

```text
glins/include/lidar/featureExtraction.cpp
glins/include/lidar/featureExtraction.h
```

主要职责：

- 计算点云曲率；
- 剔除遮挡和不可靠点；
- 提取角点；
- 提取平面点；
- 发布：

```text
/glins/feature/cloud_corner
/glins/feature/cloud_surface
```

### 4.3 IMU模块

文件：

```text
glins/include/imu/imuPreintegration.cpp
glins/include/imu/imuPreintegration.h
```

主要职责：

- 高频IMU积分；
- 相邻状态间的IMU预积分因子；
- 位姿、速度和零偏传播；
- LiDAR结果到IMU传播链的反馈；
- 发布IMU里程计和路径；
- 在紧耦合模式中管理GNSS、LiDAR、IMU因子。

关键话题：

```text
/odometry/imu
/odometry/imu_incremental
/glins/imu/path
/glins/lidar/odom
/glins/gps/path
```

### 4.4 GNSS模块

主要文件：

```text
glins/include/gnss/gnssProcessor.h
glins/include/gnss/gnssContainer.h
glins/include/gnss/gnssEstimator.h
```

职责：

- `gnssProcessor`：调用RTKLIB，按时间范围解码观测和星历；
- `gnssContainer`：组织基站、流动站、卫星和差分观测；
- `gnssEstimator`：构建GNSS相关状态和因子并输出结果。

使用的GNSS观测主要包括：

- 伪距；
- 载波相位；
- 多普勒；
- 差分观测；
- 卫星位置和钟差。

### 4.5 地图与后端优化

文件：

```text
glins/include/mapping/mapOptmizationGps.cpp
glins/include/mapping/mapOptmizationGps.h
```

主要职责：

- 管理关键帧；
- 构建局部地图；
- LiDAR帧间或帧到地图匹配；
- 构造LiDAR因子；
- 添加GNSS和IMU约束；
- 调用GTSAM ISAM2优化；
- 发布轨迹、点云和地图；
- 保存最终轨迹和PCD。

主要话题：

```text
/glins/mapping/trajectory
/glins/mapping/map_global
/glins/mapping/map_local
/glins/mapping/cloud_registered
/glins/mapping/cloud_registered_raw
/glins/mapping/odometry
/glins/mapping/odometry_incremental
/path
```

---

## 5. 因子图数据流

整体流程：

```text
rosbag
  │
  ├── /imu/data
  │      │
  │      ├── 点云去畸变
  │      └── IMU预积分
  │
  └── /velodyne_points
         │
         ├── ImageProjection
         ├── FeatureExtraction
         └── LiDAR特征/相对位姿

RINEX观测 + 广播星历 + 基站观测
  │
  └── RTKLIB / gnssProcessor
         │
         └── 伪距、载波、多普勒和差分观测

IMU因子 + LiDAR因子 + GNSS因子
  │
  └── GTSAM ISAM2
         │
         ├── 优化位姿
         ├── 优化速度和IMU零偏
         ├── 更新GNSS相关状态
         └── 输出轨迹、地图和日志
```

主要因子：

| 因子 | 作用 |
| --- | --- |
| Prior | 固定初始位置、姿态、速度和零偏 |
| IMU预积分因子 | 连接相邻状态，提供高频运动约束 |
| Bias Between | 描述IMU零偏随时间缓慢变化 |
| LiDAR特征因子 | 点到平面或特征级约束 |
| LiDAR位姿因子 | 位姿级相对运动约束 |
| Pseudorange | GNSS伪距约束 |
| Carrier Phase | GNSS载波相位约束 |
| Doppler | GNSS速度相关约束 |
| NHC | 车载非完整性约束，当前公共实验关闭 |

如果GNSS和LiDAR约束同时未进入图，新状态可能只剩不充分的惯性传播，
从而出现欠约束或 `IndeterminantLinearSystemException`。

---

## 6. 三个可执行入口

编译后主要生成：

```text
/home/zbwang/GLINS/devel/lib/glins/glins_test_rtk
/home/zbwang/GLINS/devel/lib/glins/glins_test_gins
/home/zbwang/GLINS/devel/lib/glins/glins_test_lio
```

### 6.1 `glins_test_rtk`

源码：

```text
glins/src/test_rtk.cpp
```

用途：

- 只测试GNSS/RTK前端；
- 排查RINEX、星历、基站和模糊度；
- 不运行完整LiDAR链。

### 6.2 `glins_test_gins`

源码：

```text
glins/src/test_gins.cpp
```

用途：

- GNSS/IMU组合导航；
- 运行RTK/INS消融组；
- 输出 `gins.pos`；
- 不使用LiDAR时验证惯导与GNSS主链。

对应公共数据启动文件：

```text
glins/launch/run_public_gins.launch
```

### 6.3 `glins_test_lio`

源码：

```text
glins/src/test_lio.cpp
```

用途：

- 当前公共数据实验的主入口；
- 自己打开并遍历rosbag；
- 把IMU和LiDAR消息主动送入各模块；
- 可选解码GNSS；
- 可运行LIO、FM、FF、GG和GG pose-level；
- 最终保存轨迹。

对应公共数据启动文件：

```text
glins/launch/run_public_ablation.launch
```

---

## 7. 公共数据配置

### 7.1 GLINS YAML

```text
glins/config/params_urbannav_medium_ablation.yaml
```

包括：

- UrbanNav话题；
- Velodyne HDL-32E参数；
- Xsens IMU噪声；
- LiDAR、IMU、GNSS外参；
- 初始ECEF位置；
- 初始姿态；
- GPS周；
- 时间关联容差；
- LiDAR因子门限；
- 地图参数；
- 消融实验开关。

### 7.2 RTKLIB配置

```text
glins/config/conf/Urban_medium_public.conf
```

包括：

- 流动站RINEX；
- 基站RINEX；
- 广播星历；
- 基站ECEF坐标；
- 使用的卫星系统；
- 频点；
- 高度角；
- 模糊度和差分处理设置。

基站坐标必须显式提供。若基站坐标为零，双差伪距残差会达到极大数值，
GNSS因子可能全部被拒绝。

---

## 8. 消融实验模式

批处理脚本：

```text
glins/scripts/run_public_ablation.sh
```

支持方法：

```text
rtk_gins,lio,fm,ff,gg,gg_pose
```

对应关系：

| 方法 | `lidarAssociateMode` | `coupleMode` | GNSS | 说明 |
| --- | ---: | ---: | --- | --- |
| `rtk_gins` | 不适用 | 不适用 | 开 | GNSS/IMU |
| `lio` | 1 | 1 | 关 | LiDAR/IMU |
| `fm` | 0 | 1 | 开 | 帧到地图，特征级 |
| `ff` | 1 | 1 | 开 | 帧到前一帧，特征级 |
| `gg` | 2 | 1 | 开 | 帧到最近GNSS有效帧，特征级 |
| `gg_pose` | 2 | 0 | 开 | GG关联，位姿级补充对照 |

其中：

```text
lidarAssociateMode=0  Frame-to-Map
lidarAssociateMode=1  Frame-to-Frame
lidarAssociateMode=2  GNSS-guided association

coupleMode=0          位姿级LiDAR约束
coupleMode=1          特征级LiDAR紧耦合约束
```

`gg_pose` 不是论文中严格定义的 Semi-TC，因为当前实现仍保留IMU图因子。

---

## 9. 登录服务器和加载环境

### 9.1 登录

```bash
ssh -p 17253 zbwang@172.30.5.103
```

### 9.2 Zsh终端

服务器桌面终端当前使用Zsh，执行：

```bash
cd /home/zbwang/GLINS

source /opt/ros/noetic/setup.zsh
source /home/zbwang/GLINS/devel/setup.zsh

export LD_LIBRARY_PATH="/home/zbwang/FAST_GLIO/.remote_deps/root/opt/ros/noetic/lib:/home/zbwang/GLINS/.deps/gtsam/lib:${LD_LIBRARY_PATH:-}"
```

### 9.3 Bash终端

若使用Bash：

```bash
cd /home/zbwang/GLINS

source /opt/ros/noetic/setup.bash
source /home/zbwang/GLINS/devel/setup.bash

export LD_LIBRARY_PATH="/home/zbwang/FAST_GLIO/.remote_deps/root/opt/ros/noetic/lib:/home/zbwang/GLINS/.deps/gtsam/lib:${LD_LIBRARY_PATH:-}"
```

不要在Zsh里直接 `source setup.bash`，也不要在Bash里使用 `setup.zsh`。

### 9.4 检查环境

```bash
rospack find glins
rospack find rtklib

ls -lh devel/lib/glins/glins_test_lio
ls -lh devel/lib/glins/glins_test_gins
```

---

## 10. 编译方法

只有修改C++、消息定义、CMake或依赖后才需要重新编译。

Zsh示例：

```bash
cd /home/zbwang/GLINS

source /opt/ros/noetic/setup.zsh

catkin_make \
  -j4 -l4 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/home/zbwang/FAST_GLIO/.remote_deps/root/opt/ros/noetic;/opt/ros/noetic" \
  -DGTSAM_DIR=/home/zbwang/GLINS/.deps/gtsam/lib/cmake/GTSAM

source devel/setup.zsh
```

验证可执行文件：

```bash
test -x devel/lib/glins/glins_test_lio && echo "LIO ready"
test -x devel/lib/glins/glins_test_gins && echo "GINS ready"
```

只修改以下文件时通常不需要重新编译：

```text
*.launch
*.yaml
*.conf
*.sh
*.py
*.rviz
```

---

## 11. 运行120秒公共数据

### 11.1 当前脚本的真实范围

当前 `run_public_ablation.sh` 固定使用：

```text
bag: medium_public_0_120_uncompressed.bag
start_sec: 95593
end_sec: 95713
duration: 120秒
```

脚本支持以下环境变量：

```text
URBANNAV_DATASET
URBANNAV_BAG
URBANNAV_START_SEC
URBANNAV_END_SEC
```

不设置这些变量时，默认仍运行120秒测试段。

### 11.2 只运行GG，不开RViz

```bash
cd /home/zbwang/GLINS
source devel/setup.zsh

/home/zbwang/GLINS/src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/gg_120s_test \
  gg
```

预计运行约5至6分钟。

### 11.3 只运行GG，打开RViz

必须从服务器图形桌面终端执行，并确认：

```bash
echo $DISPLAY
```

输出应类似：

```text
:0
:1
```

运行：

```bash
GLINS_VISUALIZE=true \
/home/zbwang/GLINS/src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/gg_120s_rviz \
  gg
```

### 11.4 运行完整消融

```bash
/home/zbwang/GLINS/src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/full_ablation_120s
```

默认依次运行：

```text
rtk_gins
lio
fm
ff
gg
gg_pose
```

预计约25至35分钟。

### 11.5 只运行指定方法

```bash
/home/zbwang/GLINS/src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/fm_ff_gg_120s \
  fm,ff,gg
```

---

## 12. 运行完整785秒路线

### 12.1 全程参数

完整路线使用：

```text
bag: full_original/ros/UrbanNav-HK_TST-20210517_sensors.bag
start_sec: 95593
end_sec: 96379
```

脚本通过 `URBANNAV_BAG`、`URBANNAV_START_SEC` 和
`URBANNAV_END_SEC` 接收这些参数。

### 12.2 完整GG，不开RViz

```bash
cd /home/zbwang/GLINS
source devel/setup.zsh

URBANNAV_BAG=/data/zbwang/public/UrbanNav_HK_Medium_20210517/full_original/ros/UrbanNav-HK_TST-20210517_sensors.bag \
URBANNAV_START_SEC=95593 \
URBANNAV_END_SEC=96379 \
GLINS_VISUALIZE=false \
./src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/gg_full_785s \
  gg
```

预计约35至50分钟，具体取决于服务器负载和地图规模。

### 12.3 完整GG，打开RViz

从服务器图形桌面执行：

```bash
cd /home/zbwang/GLINS
source devel/setup.zsh

URBANNAV_BAG=/data/zbwang/public/UrbanNav_HK_Medium_20210517/full_original/ros/UrbanNav-HK_TST-20210517_sensors.bag \
URBANNAV_START_SEC=95593 \
URBANNAV_END_SEC=96379 \
GLINS_VISUALIZE=true \
./src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/gg_full_785s_rviz \
  gg
```

### 12.4 完整路线评估限制

当前评估脚本默认真值是：

```text
medium_public_0_120.reference_ecef.csv
```

它只覆盖前120秒。

因此完整785秒运行结束后：

- 可以得到完整 `total.pos` 和 `fgo.pos`；
- 不能直接使用默认真值声称完整路线RMS；
- 需要先把 `UrbanNav_TST_GT_raw.txt` 转换成完整ECEF真值CSV；
- 转换后的CSV需包含：

```text
timestamp,x,y,z,latitude,longitude
```

然后使用：

```bash
python3 glins/scripts/evaluate_public_ablation.py \
  RESULT_ROOT \
  --truth FULL_REFERENCE_ECEF.csv
```

---

## 13. RViz说明

### 13.1 可视化开关做了什么

```bash
GLINS_VISUALIZE=true
```

会通过Shell脚本传给：

```text
run_public_ablation.launch
```

launch会：

1. 设置：

```text
/glins/enableOfflineMapVisualizationThread=true
```

2. 启动：

```text
/glins_public_rviz
```

3. 加载：

```text
glins/launch/include/config/vlp.rviz
```

### 13.2 推荐显示话题

RViz中设置：

```text
Global Options -> Fixed Frame -> odom
```

推荐添加：

| 类型 | 话题 |
| --- | --- |
| PointCloud2 | `/glins/mapping/cloud_registered` |
| PointCloud2 | `/glins/mapping/map_local` |
| PointCloud2 | `/glins/mapping/map_global` |
| Path | `/path` |
| Path | `/glins/gps/path` |
| Path | `/glins/imu/path` |
| Odometry | `/glins/mapping/odometry` |

全局地图线程为 `0.2 Hz`，约每5秒发布一次。

### 13.3 RViz空白

检查：

```bash
echo $DISPLAY
rosnode list
rostopic list | grep glins
rostopic hz /glins/mapping/cloud_registered
```

如果出现：

```text
No transform from [odom] to [map]
```

将Fixed Frame改为：

```text
odom
```

---

## 14. 结果目录和文件含义

典型结果：

```text
RESULT_ROOT/
├── RESULT_ROOT
├── params_used.yaml
├── rtklib_used.conf
├── rtk.pos
├── rtk_gins/
│   ├── gins.pos
│   ├── run.log
│   └── status.txt
├── lio/
│   ├── total.pos
│   ├── fgo.pos
│   ├── run.log
│   └── status.txt
├── fm/
├── ff/
├── gg/
└── gg_pose/
```

文件作用：

| 文件 | 作用 |
| --- | --- |
| `total.pos` | LiDAR/融合主轨迹 |
| `fgo.pos` | 因子图GNSS/融合结果 |
| `gins.pos` | GNSS/IMU组合导航结果 |
| `rtk.pos` | RTKLIB基准解 |
| `run.log` | ROS和算法详细日志 |
| `status.txt` | 退出状态、耗时、回滚和拒绝统计 |
| `params_used.yaml` | 本次实验使用的GLINS参数快照 |
| `rtklib_used.conf` | 本次实验使用的RTKLIB配置快照 |

`status.txt` 示例：

```text
status=0
elapsed_s=327
gps_keys=119
rollbacks=0
lidar_rejects=0
```

解释：

- `status=0`：程序正常结束；
- `gps_keys`：加入图中的GNSS历元数量；
- `rollbacks`：优化失败后回滚次数；
- `lidar_rejects`：LiDAR约束被一致性门限拒绝的次数。

---

## 15. 评估和画图

120秒实验完成后：

```bash
python3 /home/zbwang/GLINS/src/glins/scripts/evaluate_public_ablation.py \
  /data/zbwang/results/glins_public_ablation/RESULT_NAME
```

产生：

```text
metrics.csv
metrics.json
figures/trajectory_enu.png
figures/error_3d_timeseries.png
figures/error_3d_stable_zoom.png
figures/rms_comparison.png
figures/rms_stable_zoom.png
figures/availability.png
```

终端查看指标：

```bash
column -s, -t \
  /data/zbwang/results/glins_public_ablation/RESULT_NAME/metrics.csv
```

重点指标：

- `rms_e_m`；
- `rms_n_m`；
- `rms_u_m`；
- `rms_3d_m`；
- `median_3d_m`；
- `p95_3d_m`；
- `availability_lt_1m_pct`；
- `run_rollbacks`；
- `run_lidar_rejects`。

---

## 16. 日志监控

脚本把roslaunch输出写入 `run.log`，所以主终端通常只显示：

```text
START gg
END gg status=0
```

新开终端实时查看：

```bash
tail -f \
  /data/zbwang/results/glins_public_ablation/RESULT_NAME/gg/run.log
```

只看关键事件：

```bash
grep -E \
  "GPS KEY|Reject lidar factor|rolled back optimization|Exception|ERROR|FATAL" \
  /data/zbwang/results/glins_public_ablation/RESULT_NAME/gg/run.log
```

统计：

```bash
grep -c "GPS KEY" RESULT_ROOT/gg/run.log
grep -c "Reject lidar factor" RESULT_ROOT/gg/run.log
grep -c "rolled back optimization" RESULT_ROOT/gg/run.log
```

---

## 17. 常见故障

### 17.1 一秒结束，`status=1`

先看：

```bash
cat RESULT_ROOT/gg/status.txt
sed -n '1,200p' RESULT_ROOT/gg/run.log
```

如果出现：

```text
Invalid roslaunch XML syntax
```

说明launch标签结构有误。

当前正确结构必须只有一个根标签：

```xml
<launch>
    ...
    <group if="$(arg visualize)">
        ...
    </group>
</launch>
```

不能把 `<group>` 放在 `<launch>` 外面。

### 17.2 `Missing required input`

脚本找不到：

- bag；
- YAML；
- RTKLIB配置。

检查：

```bash
ls -lh /data/zbwang/public/UrbanNav_HK_Medium_20210517/medium_public_0_120_uncompressed.bag
ls -lh /home/zbwang/GLINS/src/glins/config/params_urbannav_medium_ablation.yaml
ls -lh /home/zbwang/GLINS/src/glins/config/conf/Urban_medium_public.conf
```

### 17.3 共享库找不到

如果出现：

```text
error while loading shared libraries
```

重新加载：

```bash
source /opt/ros/noetic/setup.zsh
source /home/zbwang/GLINS/devel/setup.zsh

export LD_LIBRARY_PATH="/home/zbwang/FAST_GLIO/.remote_deps/root/opt/ros/noetic/lib:/home/zbwang/GLINS/.deps/gtsam/lib:${LD_LIBRARY_PATH:-}"
```

检查：

```bash
ldd /home/zbwang/GLINS/devel/lib/glins/glins_test_lio | grep "not found"
```

### 17.4 RViz没有出现

检查：

```bash
echo $DISPLAY
```

SSH纯终端通常没有DISPLAY。应在服务器图形桌面终端运行，或者单独配置X11转发。

### 17.5 RViz有窗口但没有点云

检查：

```bash
rostopic list | grep glins
rostopic hz /glins/mapping/cloud_registered
```

将Fixed Frame改为 `odom`。

### 17.6 欠约束或GTSAM回滚

重点检查：

```bash
grep -E \
  "GPS KEY|Reject lidar factor|rolled back optimization|IndeterminantLinearSystem" \
  RESULT_ROOT/gg/run.log
```

可能原因：

- GNSS时间没有匹配到LiDAR帧；
- 基站ECEF坐标错误；
- GNSS残差过大，因子被拒绝；
- 姿态轴映射错误；
- LiDAR/IMU一致性门限过严；
- 新状态缺少足够约束。

### 17.7 ROS Master冲突

检查：

```bash
echo $ROS_MASTER_URI
rosnode list
pgrep -af "roscore|rosmaster|roslaunch"
```

不要随意杀死其他用户或其他实验正在使用的ROS Master。

---

## 18. Git工作流

正确仓库：

```text
git@github.com:wangzika/GLINS.git
```

推荐服务器Git根：

```text
/home/zbwang/GLINS/src
```

因为仓库内同时包含：

```text
glins/
rtklib/
```

关联后日常更新：

```bash
cd /home/zbwang/GLINS/src

git status
git branch -vv
git pull --ff-only
```

然后编译：

```bash
cd /home/zbwang/GLINS

catkin_make \
  -j4 -l4 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/home/zbwang/FAST_GLIO/.remote_deps/root/opt/ros/noetic;/opt/ros/noetic" \
  -DGTSAM_DIR=/home/zbwang/GLINS/.deps/gtsam/lib/cmake/GTSAM
```

在确认远端源码已备份、GitHub分支包含最新提交前，不要执行：

```text
git reset --hard
git clean -fd
```

---

## 19. 推荐实验流程

### 19.1 修改代码后

```text
本地修改
  -> 本地测试和提交
  -> 推送fix_test_glins
  -> 服务器git pull --ff-only
  -> catkin_make
  -> 120秒GG测试
  -> 检查status和日志
  -> 完整785秒GG
  -> 其他消融组
  -> 完整真值评估
```

### 19.2 最小验证

```bash
cd /home/zbwang/GLINS
source devel/setup.zsh

/home/zbwang/GLINS/src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/smoke_gg \
  gg
```

检查：

```bash
cat /data/zbwang/results/glins_public_ablation/smoke_gg/gg/status.txt
```

期望：

```text
status=0
rollbacks=0
lidar_rejects=0
```

### 19.3 最终实验建议

1. 先运行120秒GG；
2. 再运行完整785秒GG，不开RViz；
3. 完整GG稳定后再运行FM和FF；
4. 最后运行全部消融；
5. 使用完整真值CSV评估；
6. 每组保留参数快照、RTKLIB配置、日志和Git提交号。

---

## 20. 当前已知限制

1. 当前默认评估真值只覆盖120秒；
2. 完整路线虽可由Shell环境变量启动，但需要完整ECEF真值才能正确评估；
3. 公共120秒区间没有RTK fixed解，不能直接复现论文0.20米；
4. `gg_pose` 不等于论文严格Semi-TC；
5. IMU初始零偏仍有进一步参数化和在线估计空间；
6. GNSS权重尚可根据fixed/float/DGPS、卫星高度角和残差继续自适应；
7. RViz会增加计算和图形开销，不建议在最终批量精度实验中开启；
8. 完整路线地图规模更大，需要关注内存、回滚和LiDAR拒绝数量。

---

## 21. 快速命令索引

登录：

```bash
ssh -p 17253 zbwang@172.30.5.103
```

加载Zsh环境：

```bash
cd /home/zbwang/GLINS
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
```

编译：

```bash
catkin_make \
  -j4 -l4 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/home/zbwang/FAST_GLIO/.remote_deps/root/opt/ros/noetic;/opt/ros/noetic" \
  -DGTSAM_DIR=/home/zbwang/GLINS/.deps/gtsam/lib/cmake/GTSAM
```

120秒GG：

```bash
./src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/gg_120s \
  gg
```

120秒GG加RViz：

```bash
GLINS_VISUALIZE=true \
./src/glins/scripts/run_public_ablation.sh \
  /data/zbwang/results/glins_public_ablation/gg_120s_rviz \
  gg
```

评估：

```bash
python3 ./src/glins/scripts/evaluate_public_ablation.py \
  /data/zbwang/results/glins_public_ablation/gg_120s
```

查看状态：

```bash
cat /data/zbwang/results/glins_public_ablation/gg_120s/gg/status.txt
```

查看日志：

```bash
tail -f /data/zbwang/results/glins_public_ablation/gg_120s/gg/run.log
```

检查Git：

```bash
git -C /home/zbwang/GLINS/src status
git -C /home/zbwang/GLINS/src branch -vv
git -C /home/zbwang/GLINS/src remote -v
```

---

## 22. 一句话理解整个项目

```text
test_lio读取bag
 -> LiDAR前端提取特征
 -> IMU预积分传播状态
 -> RTKLIB生成GNSS观测
 -> GTSAM加入IMU、LiDAR和GNSS因子
 -> ISAM2增量优化
 -> 发布轨迹与地图
 -> 保存pos、日志并进行误差评估
```
