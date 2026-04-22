# GLINS 文件夹整体说明

本文档用于从工程结构和运行链路两个角度，系统说明 `glins/` 目录：

1. `glins` 整体是怎么组织的
2. 代码从哪里启动，主流程如何串起来
3. `src/` 里几个主 `cpp` 入口分别做什么
4. `python/` 和 `scripts/` 里的脚本分别起什么作用

---

## 1. `glins` 整体定位

从当前代码看，`glins` 不是单一“纯 LIO”项目，而是一个把以下几条链路揉在一起的研究型 ROS 包：

- LiDAR 前端：点云去畸变、投影、特征提取
- IMU 预积分：用于运动传播与短时约束
- GNSS/RTKLIB 前端：RINEX 解码、观测组织、差分信息生成
- GTSAM 优化后端：把 LiDAR、IMU、GNSS 因子放进统一图优化
- 结果输出：轨迹、地图、日志、可视化

它的代码来源和演化痕迹很明显：

- `package.xml` 仍然把项目描述为 `Lidar Odometry`
- `launch/` 里保留了典型 LIO-SAM 风格的模块化启动方式
- 但 `CMakeLists.txt` 当前实际编译出来的主入口，已经偏向研究/实验用的 `test_*.cpp`

所以理解这个目录时，最好把它看成：

**一个以 LIO-SAM 风格骨架为基础，叠加 GNSS/RTKLIB/GTSAM 紧耦合实验代码的综合导航工程。**

---

## 2. 目录结构总览

### 2.1 根目录下各子目录职责

| 目录 | 作用 |
| --- | --- |
| `config/` | 参数文件、传感器配置、RTK/GNSS 相关配置 |
| `launch/` | ROS 启动文件，负责加载参数、启动模块、打开 RViz |
| `include/` | 核心 C++ 实现，实际包含大量 `.h` 与部分 `.cpp` |
| `src/` | 主程序入口，当前最关键的是三个 `test_*.cpp` |
| `msg/` | 自定义 ROS 消息，尤其是 GNSS 和点云处理中间结果 |
| `srv/` | ROS 服务定义 |
| `python/` | 数据预处理、bag 生成、轨迹转换、误差分析、绘图脚本 |
| `scripts/` | 较早期/备用脚本，功能和 `python/` 有一定重复 |
| `data/` | 运行输出或默认数据目录 |
| `data_test/` | 测试数据目录 |

### 2.2 `include/` 里的模块分工

| 子目录/文件 | 作用 |
| --- | --- |
| `include/utility.h` | 参数中心。`ParamServer` 在这里统一从 ROS 参数服务器读取话题名、传感器类型、外参、GNSS/IMU/LiDAR 参数 |
| `include/gnss/` | GNSS 前端和 GNSS 图优化相关代码 |
| `include/imu/` | IMU 预积分、IMU/LiDAR 融合、里程计桥接 |
| `include/lidar/` | 点云投影、去畸变、特征提取 |
| `include/mapping/` | 地图优化、回环、GPS 约束、路径与地图输出 |
| `include/factor/` | GTSAM 自定义因子，包含伪距、载波、Doppler、LiDAR、NHC 等 |
| `include/dataSaver.h` | 保存轨迹、点云地图、图优化结果、TUM/KITTI 格式结果 |

### 2.3 核心模块之间的关系

从逻辑上看，`glins` 可以拆成四层：

1. 数据输入层  
   ROS topic、rosbag、RINEX/GNSS 文件

2. 传感器前端层  
   `ImageProjection`、`FeatureExtraction`、`gnssProcessor`

3. 状态传播与约束层  
   `IMUPreintegration`、`gnssContainer`、各类 GTSAM factor

4. 后端优化与输出层  
   `mapOptimization`、`gnssEstimator`、`DataSaver`

---

## 3. 当前工程“真正可执行”的入口

根据当前 `glins/CMakeLists.txt`，实际明确编译的主入口只有三个：

- `glins_test_rtk`
- `glins_test_gins`
- `glins_test_lio`

此外还编译了一个库：

- `lio`

这点很关键，因为 `launch/include/module_loam.launch` 中引用的是：

- `glins_imuPreintegration`
- `glins_imageProjection`
- `glins_featureExtraction`
- `glins_mapOptmization`

也就是说：

**从目录设计上看，项目保留了“模块化 ROS 节点”的启动方式；但从当前 `CMakeLists.txt` 看，真正稳定暴露出来的主程序已经是三个 `test_*.cpp`。**

这说明仓库存在两套理解方式：

- 一套是“模块化节点架构”的历史设计
- 一套是“研究数据集实验入口”的当前主用方式

---

## 4. `glins` 的整体启动流程

这里分成两个视角说明：**工程视角**和**代码执行视角**。

### 4.1 工程视角：从 launch 到结果输出

一个典型的 `glins` 启动过程可以概括为：

1. 通过 `launch/*.launch` 加载 YAML 参数  
   例如传感器类型、话题名、外参、GNSS 开关、保存目录等。

2. `ParamServer` 从 ROS 参数服务器读取参数  
   几乎所有核心类都继承自 `ParamServer`，因此模块启动时会先完成统一配置读取。

3. 选择数据源  
   有两种常见方式：
   - 直接订阅 ROS topic
   - 离线读取 rosbag / RINEX 文件

4. LiDAR 链开始工作  
   典型顺序是：
   `ImageProjection -> FeatureExtraction -> IMUPreintegration -> mapOptimization`

5. GNSS 链开始工作  
   `gnssProcessor` 负责调用 RTKLIB 相关逻辑，对观测文件/导航文件进行解码、预处理、生成 GNSS 观测信息。

6. 因子图融合  
   GNSS、IMU、LiDAR 的信息通过 GTSAM 因子进入后端优化。

7. 输出结果  
   发布 odometry/path/map，或把结果写成 `.pos`、TUM、KITTI、PCD、bag 等格式。

### 4.2 代码视角：各模块实际串接关系

如果按代码依赖关系压缩成一条主链，可以写成：

`launch/参数 -> ParamServer -> 数据输入 -> LiDAR/GNSS 前端 -> IMU/GNSS/LiDAR 约束构造 -> GTSAM 优化 -> 轨迹/地图/日志输出`

进一步展开为：

1. `utility.h` 中的 `ParamServer` 读取配置
2. `test_*.cpp` 决定本次实验模式
3. `gnssProcessor.decode(...)` 先把 GNSS 数据准备好
4. 若有 LiDAR：
   `ImageProjection` 完成点云缓存、去畸变、投影、地面去除、点云拆分
5. `FeatureExtraction` 从点云中提取角点和平面点
6. `IMUPreintegration` 处理 IMU 累积、传播、与 LiDAR/GNSS 对齐
7. `mapOptimization` 进行 scan-to-map、回环、GPS 约束与全局优化
8. `gnssEstimator` 或 `test_GINS` 执行更完整的 GNSS/IMU/LiDAR 图优化
9. `DataSaver` / `savePath` / `saveMapService` 落盘结果

---

## 5. 三个主 `cpp` 文件的区别

当前最重要的三个入口都在 `glins/src/`。

### 5.1 `test_rtk.cpp`

**定位：GNSS-only / RTK-only 的最小实验入口。**

它做的事情非常集中：

1. 初始化 ROS 节点 `gnss_estimator`
2. 创建 `gnssProcessor`
3. 创建 `gnssEstimator`
4. 指定一个 GNSS 时间窗
5. 调用 `processor.decode(ts, te)`
6. 调用 `estimator.solveOptimization()`
7. `ros::spin()`

这个入口几乎不碰 LiDAR，也不负责完整的多传感器回放。它更像：

- 验证 GNSS 前端是否能正确解码
- 验证 `gnssEstimator` 的图优化流程
- 单独调试 RTK/GNSS 因子图

适用场景：

- 想只看 GNSS 解算链是否通
- 想排查伪距、载波、Doppler、整周模糊度一类问题
- 想把 GNSS 优化和 LIO 完全分开验证

### 5.2 `test_lio.cpp`

**定位：离线 rosbag 回放入口，用于驱动 LiDAR-IMU-GNSS 组合流程。**

它的特点是最“工程化”的数据回放逻辑：

1. 初始化 ROS 节点 `lio`
2. 从命令行参数或 ROS 参数读取 `bagpath`、`imu_topic`、`lidar_topic`
3. 打开 rosbag
4. 创建以下对象：
   - `gnssProcessor`
   - `ImageProjection`
   - `FeatureExtraction`
   - `IMUPreintegration`
   - `mapOptimization`
   - `TransformFusion`
5. 指定 GNSS 时间窗并执行 `GP.decode(ts, te)`
6. 手工遍历 bag 中消息：
   - LiDAR 消息送入 `IP.cloudHandler(...)`
   - IMU 消息同时送入 `IP.imuHandler(...)` 和 `PT.imuHandler(...)`
7. 回放过程中发布 `/clock`
8. 启动全局地图可视化线程
9. 最后保存轨迹和地图

这个入口的本质是：

**它不是“等 ROS topic 自然跑起来”，而是自己作为一个总控程序，把 bag 中的消息按时间顺序主动喂给各模块。**

适用场景：

- 跑固定数据集做离线实验
- 生成轨迹和地图结果
- 分析 LiDAR-IMU 主链，再挂 GNSS 约束

需要注意的实际问题：

- 里面存在硬编码的 GNSS 时间窗
- 结果保存路径也有硬编码痕迹，例如固定写到某个 `.pos`
- 所以它更像实验驱动程序，而不是通用产品化入口

### 5.3 `test_gins.cpp`

**定位：最完整、最偏算法研究的综合入口。**

它是三者中体量最大、耦合最深的入口。核心特征是：

- 定义了大型的 `test_GINS` 类
- 管理 IMU 队列、GNSS 队列、图优化状态、预积分状态、LIO 相关状态
- 同时涉及 GNSS、IMU、GTSAM、多种自定义因子
- 主函数流程是：  
  `gnssProcessor processor -> test_GINS gins -> 设置时间窗 -> processor.decode(...) -> gins.run() -> ros::spin()`

可以把它理解成：

**一个面向完整 GINS/紧耦合融合实验的主控程序。**

相比 `test_rtk.cpp`：

- 它不是只做 GNSS 解算
- 而是把 GNSS 作为整体导航系统中的一个约束源

相比 `test_lio.cpp`：

- 它不只是回放 LiDAR/IMU 数据
- 而是更强调统一状态估计、图优化和多约束融合

适用场景：

- 跑完整 GINS 算法实验
- 研究 GNSS/IMU/LiDAR 紧耦合
- 调整因子图结构、状态量定义、先验和约束策略

### 5.4 三个入口的对照总结

| 文件 | 核心目标 | 数据组织方式 | 重点模块 | 更像什么 |
| --- | --- | --- | --- | --- |
| `test_rtk.cpp` | GNSS/RTK 解算与优化验证 | 直接按时间窗解 GNSS | `gnssProcessor` + `gnssEstimator` | GNSS-only 实验入口 |
| `test_lio.cpp` | 离线回放 LiDAR-IMU-GNSS | 手工遍历 rosbag | `ImageProjection` / `FeatureExtraction` / `IMUPreintegration` / `mapOptimization` | 数据集回放总控程序 |
| `test_gins.cpp` | 完整 GINS 紧耦合融合 | GNSS 预解码 + 综合运行 | `test_GINS` + 因子图 + IMU/GNSS/LiDAR 状态管理 | 算法研究主入口 |

如果只想快速建立认识，可以这样记：

- `test_rtk.cpp`：先看 GNSS
- `test_lio.cpp`：再看 LiDAR-IMU 链怎么跑
- `test_gins.cpp`：最后看全系统怎么融合

---

## 6. `launch/` 文件应该怎么理解

`launch/` 里的文件更多反映了项目的“模块化架构意图”，不完全等于当前最常用的编译入口。

### 6.1 关键 launch 文件

| 文件 | 作用 |
| --- | --- |
| `launch/run.launch` | 典型 LIO 启动入口，加载参数、模块化启动 LOAM 相关节点、打开 RViz |
| `launch/run_imu_gps.launch` | 偏 IMU + GPS/GNSS 的启动方式 |
| `launch/run_test_lio.launch` | 为 `test_lio` 预留的回放入口模板，但实际节点部分目前被注释掉了 |
| `launch/test.launch` | 辅助测试和 RViz 可视化 |
| `launch/include/module_loam.launch` | 启动 `imuPreintegration`、`imageProjection`、`featureExtraction`、`mapOptmization` 四个核心模块 |

### 6.2 为什么 launch 和 CMake 看起来有点“不一致”

因为当前仓库同时保留了两种使用习惯：

- 一种是按 ROS 节点拆开的模块化运行方式
- 一种是按数据集/实验封装成单入口 `test_*.cpp` 的方式

这通常说明项目在研究迭代中经历过如下演化：

1. 先有 LIO-SAM 风格的模块化节点
2. 后来为了做固定数据集实验、紧耦合验证、批量调试，又写了更大的单入口程序

所以实际阅读时，建议把 `launch/` 看成“架构示意图”，把 `src/test_*.cpp` 看成“当前最直接的可执行入口”。

---

## 7. Python 脚本的作用

`glins/python/` 里的脚本总体上**不是核心在线运行链的一部分**，而是围绕实验流程服务的辅助工具。它们主要做三类事：

1. 把原始数据转成 rosbag
2. 把结果转成评估格式
3. 画图、算误差、做轨迹对比

### 7.1 数据准备与 rosbag 生成

#### `csvToRosbag.py`

作用：

- 读取 LiDAR CSV 文件
- 转成 `sensor_msgs/PointCloud2`
- 写入 rosbag
- 同时生成 `summary.txt`

适用：

- 原始激光雷达数据是逐帧 CSV
- 需要先转 bag 再喂给 ROS/LIO 流水线

#### `csvToRosbag_multiprocess.py`

作用：

- 与 `csvToRosbag.py` 类似
- 但加入并行处理，提高大量 CSV 转 bag 的速度

适用：

- 数据量大
- 单进程转换太慢

#### `memsToRosbag.py`

作用：

- 读取 MEMS/IMU 文本数据
- 生成 ROS `Imu` 消息
- 写入 rosbag
- 内部自己做了 GPS 周/周内秒到时间戳的转换

适用：

- IMU 数据原始格式不是 ROS bag
- 需要补成标准 IMU topic

#### `merge_bag.py`

作用：

- 合并多个 bag 文件
- 支持按 topic 过滤

适用：

- LiDAR、IMU、GNSS 分别在不同 bag 中
- 需要合成一个统一 bag 用于离线实验

#### `pcapToRosbag.py`

作用：

- 解析 `pcap` / `pcapng`
- 读取 UDP 负载
- 更偏底层地处理 LiDAR 网络数据包

适用：

- 数据源是传感器抓包，而不是 CSV 或 bag
- 需要从原始网络包往上恢复数据

#### `rosbagRead.py`

作用：

- 简单读 bag、快速查看内容

适用：

- 调试 bag 中到底有没有某个 topic
- 临时检查消息格式

### 7.2 轨迹格式转换与结果导出

#### `gt2tum.py`

作用：

- 把参考轨迹或地面真值转换成 TUM 轨迹格式
- 内部使用 `rtkcmn.py` 做坐标系转换

适用：

- 要和 EVO、SLAM 评估工具对接
- 需要统一成 TUM 格式

#### `pcd2tum.py`

作用：

- 把某些点云/位姿结果导出成 TUM 风格文本

适用：

- 需要把 GLINS/LIO 的结果拿去做统一轨迹对比

### 7.3 误差分析与绘图

#### `calerr.py`

作用：

- 读取不同结果文件
- 与参考解做对比
- 把 ECEF 误差转换到 ENU
- 统计误差序列，适合进一步画 CDF/时间序列图

适用：

- 比较不同算法结果
- 对比 GNSS / GINS / LIO / 参考轨迹

#### `plotpos.py`

作用：

- 读取位置解文件
- 转 ENU
- 绘制东、北、天方向误差或轨迹曲线

适用：

- 快速看轨迹趋势
- 快速做实验结果可视化

### 7.4 公共数学与 GNSS 工具

#### `rtkcmn.py`

作用：

- 这是 Python 脚本里的基础工具库
- 功能包括：
  - GNSS 枚举和卫星编号转换
  - GPST/UTC/历元时间转换
  - ECEF、LLA、ENU 坐标变换
  - DOP 计算
  - 电离层/对流层相关模型
  - 若干小型滤波与矩阵工具

可以把它看成：

**Python 版的 RTK/GNSS 公共函数仓库。**

很多分析脚本都依赖它。

### 7.5 其他文件

#### `main.py`

作用不大，更像一个非常简单的测试脚本或占位文件，不属于核心流程。

#### `glio.txt` / `gt.txt` / `lio.txt` / `pcd2tum.txt`

这些不是脚本，而是中间结果或输出文本，用于轨迹对比、评估或脚本调试。

---

## 8. `scripts/` 与 `python/` 的关系

`glins/scripts/` 中还能看到：

- `calerr.py`
- `plotpos.py`
- `rtkcmn.py`
- `navstate.cc`

可以把它理解为：

- `scripts/` 更像较早期或临时使用的工具区
- `python/` 是后来更集中、更完整的数据处理与评估脚本目录

其中：

- `scripts/calerr.py`、`scripts/plotpos.py`、`scripts/rtkcmn.py`
  与 `python/` 下同名文件存在功能重叠
- `scripts/navstate.cc`
  只是一个很小的 C++ 文件，当前看不构成主要工作流

因此，如果是现在要整理流程，建议：

- 优先看 `python/`
- 把 `scripts/` 视为历史脚本或备份工具区

---

## 9. 推荐的阅读顺序

如果你后面还要继续写总文档，建议按下面顺序看代码，效率最高：

1. 先看 `glins/CMakeLists.txt`  
   先确定当前到底有哪些真实入口被编译。

2. 再看 `glins/include/utility.h`  
   先掌握参数系统、话题名、外参、传感器配置。

3. 再看 `glins/src/test_rtk.cpp`  
   先搞清 GNSS-only 最小流程。

4. 再看 `glins/src/test_lio.cpp`  
   理解离线 rosbag 如何驱动 LiDAR-IMU 主链。

5. 最后看 `glins/src/test_gins.cpp`  
   这是全系统最复杂、最适合最后啃的入口。

6. 然后回头看 `include/gnss/`、`include/imu/`、`include/lidar/`、`include/mapping/`

7. 最后用 `python/` 脚本理解数据准备、结果导出和评估闭环

---

## 10. 一句话总结

如果要用最短的话概括 `glins/`：

**它是一个以 LIO-SAM 式 LiDAR-IMU 处理链为骨架，叠加 RTKLIB GNSS 前端和 GTSAM 多因子优化后端的研究型紧耦合导航工程；`test_rtk.cpp` 偏 GNSS，`test_lio.cpp` 偏离线回放，`test_gins.cpp` 偏完整融合，而 `python/` 主要负责数据准备、格式转换、误差分析和结果绘图。**
