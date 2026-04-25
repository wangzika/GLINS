# macOS 适配与功能实现总结

本文档总结当前仓库为了在 macOS 上运行 GLINS/RTKLIB 所做的改造、目前已经验证可用的功能，以及仍然存在的限制。

## 1. 当前目标

当前适配目标不是在 macOS 原生安装整套 ROS1/catkin，而是通过：

- macOS 宿主机
- Docker Desktop 或 OrbStack
- 容器内 ROS Noetic + Ubuntu 20.04

来稳定运行整套工程。

## 2. 已验证可用的功能

以下能力已经在当前仓库配置下完成打通，至少跑通过一次完整链路。

### 2.1 镜像构建与开发容器

已经可以在 macOS 上通过下面的入口完成构建和进入容器：

- `./docker/macos.sh build`
- `./docker/macos.sh shell`

当前特性：

- 默认使用 `linux/amd64`
- 持久化 `docker/build`、`docker/devel`、`docker/logs`、`docker/output`
- 容器启动时支持自动增量编译
- 构建阶段不再执行整套 `catkin_make`，降低 Docker Desktop 内存压力

相关文件：

- `Dockerfile`
- `docker-compose.yml`
- `docker/macos.sh`
- `docker/entrypoint.sh`

### 2.2 离线 rosbag 主链运行

当前已经能够通过：

- `./docker/macos.sh bag ...`
- `roslaunch glins run_bag.launch ...`

运行离线数据链路。

针对当前 `20240129` 数据，已经修正了：

- bag 时间窗
- GNSS 配置路径
- 输出路径
- 传感器参数文件选择

因此不再出现之前那类：

- 所有 bag 消息被时间窗过滤
- PatchWork++ 参数缺失
- RTKLIB 配置路径指向作者本机目录
- 9 轴 IMU 参数误用于 6 轴数据

### 2.3 GNSS/RTKLIB 后处理链

当前 GNSS 后处理链已经能够在容器内完成运行，日志中已经出现过：

- `----> gnss Processor Finished.`

并且已经生成过以下结果文件：

- `docker/output/rtklib.pos`
- `docker/output/rtklib.pos.stat`
- `docker/output/rtklib_events.pos`
- `docker/output/rtklib_fgo.pos`

相关文件：

- `glins/include/gnss/gnssProcessor.h`
- `glins/config/conf/20240129.conf`

### 2.4 LIO / 建图主链

当前 LIO 主链已经能够跑完整段 bag，并生成关键帧与地图。之前已经在日志中看到过：

- `pose saved`
- `Processing feature cloud ...`
- `Saving map to pcd files completed`

这说明当前至少在“稳定参数版本”下，已经能够完成：

- 点云预处理
- PatchWork++ 地面分割
- IMU/LiDAR 前端
- 地图保存

### 2.5 输出结果保存

当前输出目录已经统一到宿主机可直接访问的位置：

- `docker/output/total.pos`
- `docker/output/rtklib.pos`
- `docker/output/rtklib_fgo.pos`
- `docker/output/MAP/globalmap_lidar_feature.pcd`

说明：

- `total.pos` 对应 GLINS 最终轨迹
- `rtklib.pos` 对应 RTKLIB GNSS 解
- `rtklib_fgo.pos` 对应融合过程中的 GNSS/FGO 输出
- `globalmap_lidar_feature.pcd` 对应点云地图

### 2.6 宿主机本地可视化

考虑到 macOS 上 `RViz + XQuartz` 不稳定，仓库中已经新增本地可视化脚本：

- `python glins/scripts/visualize_local_results.py`

它默认读取：

- `docker/output/total.pos`
- `docker/output/MAP/globalmap_lidar_feature.pcd`

并在宿主机上直接用 `numpy + matplotlib` 画出：

- 地图 XY 散点
- 轨迹叠加
- 轨迹随时间变化曲线

相关文件：

- `glins/scripts/visualize_local_results.py`

## 3. 这次新增的主要改造

这一部分总结“为了让项目在 macOS 上可用”额外新增或修改的能力。

### 3.1 CMake 跨平台改造

已处理的 Linux 写死依赖包括：

- 去掉 `rtklib` 里写死的 `libm.so`、`libpthread.so`
- 改为使用 `Threads`、`BLAS`、`LAPACK`
- `glins` 中 `dw` 仅在 Linux 上链接
- `OpenMP` 改为“找到就用”

同时补了：

- `jsk_recognition_msgs` 依赖

相关文件：

- `rtklib/CMakeLists.txt`
- `glins/CMakeLists.txt`
- `glins/package.xml`

### 3.2 Docker 构建链增强

当前 Docker 方案已经额外支持：

- GTSAM 从源码编译安装
- `LD_LIBRARY_PATH` 自动加入 `/usr/local/lib`
- 可调编译并行度
- 容器启动时自动增量编译

为了解决构建中碰到的问题，已经处理过：

- `libgtsam-dev` 不存在
- 构建阶段 `catkin_make` 爆内存
- 运行时 `libmetis-gtsam.so` 找不到

### 3.3 macOS 入口脚本

新增了面向 macOS 的统一入口脚本：

- `docker/macos.sh`

当前支持的命令：

- `build`
- `shell`
- `bag`
- `gui`
- `desktop`
- `gui-web`

其中：

- `bag` 用于纯计算离线运行
- `gui` 用于尝试通过 XQuartz 跑 RViz
- `desktop` 用于打开容器内浏览器桌面
- `gui-web` 用于在容器桌面里尝试启动 RViz

### 3.4 浏览器桌面方案

为了绕开 `XQuartz` 的 GLX 问题，仓库里新增了容器内桌面启动脚本：

- `docker/start-desktop.sh`

它会启动：

- `Xvfb`
- `fluxbox`
- `x11vnc`
- `noVNC`

目标是让用户通过浏览器访问：

- `http://localhost:6080/vnc.html`

从而查看容器桌面，并在容器内部尝试打开 RViz。

注意：这一套脚本和端口映射已经接入，但是否在当前 macOS 环境中稳定可用，仍需继续验证。

### 3.5 launch 与参数修正

为了让当前数据集真正能跑起来，已经做了多项修正：

- `run_bag.launch` 默认改为 `params_20240129.yaml`
- 默认时间窗改为 `gps_week=2299`、`start_sec=111965`、`end_sec=112280`
- `rtklibConfigPath`、`fgoPath`、`savePCDDirectory` 等改为容器可用路径
- `20240129.conf` 中 GNSS 输入输出路径统一改为 `/data` 与 `/output`

相关文件：

- `glins/launch/run_bag.launch`
- `glins/src/test_lio.cpp`
- `glins/include/utility.h`
- `glins/config/conf/20240129.conf`
- `glins/config/params_20240129.yaml`

### 3.6 PatchWork++ 参数补齐

之前 `params_HDL_32.yaml` 缺少完整的 PatchWork++ 参数，导致：

- `Num. zones: 0`
- `Some parameters are wrong! Check the num_zones and num_rings/sectors_each_zone`

目前已经补齐相关参数，并完成过这一轮问题的定位与修复。

不过当前真正用于 `20240129` 数据的默认参数文件已经切到：

- `glins/config/params_20240129.yaml`

## 4. 当前默认稳定配置

为了先保证链路能跑通，当前 `20240129` 的默认稳定配置是“偏保守”的：

- `useGPS: true`
- `useObs: false`
- `useCarrier: false`
- `useNHC: false`

这样做的原因是此前在更激进配置下出现过：

- `ambiguity validation failed`
- `gtsam::IndeterminantLinearSystemException`

也就是说，当前版本优先保证：

- 能完整跑完
- 能输出轨迹和地图

而不是优先打开全部 GNSS 高精度约束。

## 5. 当前已知限制

### 5.1 RViz + XQuartz 仍不稳定

目前在 macOS 上通过：

- Docker
- XQuartz
- OGRE/GLX

直接运行 RViz，仍然可能出现：

- `Could not connect to display`
- `Unable to create a suitable GLXContext`

因此当前不建议把 `RViz + XQuartz` 作为主显示方案。

### 5.2 noVNC 方案仍待进一步验证

浏览器桌面方案已经接入：

- `docker/start-desktop.sh`
- `docker/macos.sh desktop`
- `docker/macos.sh gui-web`

但在实际环境里，`localhost:6080/vnc.html` 仍需继续验证是否稳定输出内容。

### 5.3 输出会被后续运行覆盖

当前运行流程会清理旧的地图目录，因此如果后面又启动了一次失败或中断的运行，之前的：

- `total.pos`
- `MAP/globalmap_lidar_feature.pcd`

可能被重新创建为空文件或空目录。

因此建议在一次成功运行结束后立即备份结果。

## 6. 推荐使用方式

当前最推荐的工作流是：

1. 用 `./docker/macos.sh build` 构建镜像
2. 用 `./docker/macos.sh bag ...` 跑纯计算链路
3. 在宿主机检查 `docker/output/` 结果
4. 用 `python glins/scripts/visualize_local_results.py` 做本地可视化

如果后面继续推进交互式可视化，再优先验证：

- `desktop`
- `gui-web`

而不是优先继续折腾 `RViz + XQuartz`

## 7. 相关文档

如果需要继续查看细节，可以结合以下文档：

- `DOCKER.md`
- `PROJECT_OVERVIEW.md`
- `rtklib/RTKLIB_FILE_GUIDE.md`
- `rtklib/RTKLIB_WORKFLOW.md`

## 8. 一句话总结

当前仓库已经完成了“在 macOS 上通过 Docker 跑通 GLINS/RTKLIB 主链、生成轨迹与地图、并在宿主机本地查看结果”的核心目标；交互式 RViz 显示仍然存在图形栈兼容性问题，因此目前最稳的方案是“容器内计算 + 宿主机本地可视化”。
