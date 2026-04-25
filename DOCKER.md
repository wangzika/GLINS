# Docker 运行说明

本项目是 ROS1 catkin 工程。要在 macOS 上把整套跑起来，推荐方案是：

- 宿主机使用 macOS
- 运行环境使用 Docker Desktop 或 OrbStack
- 容器内运行 ROS Noetic + Ubuntu 20.04

这比尝试在 macOS 原生安装整套 ROS1/catkin 生态稳定得多。

## 1. macOS 推荐启动方式

仓库里已经提供了一个 macOS 包装脚本：

```bash
chmod +x docker/macos.sh
./docker/macos.sh build
./docker/macos.sh shell
```

默认会使用：

- `linux/amd64` 平台
- 持久化的 `docker/build`、`docker/devel`、`docker/logs`
- 容器启动时自动 `catkin_make`
- 较保守的默认并行度：`GLINS_BUILD_JOBS=2`、`GLINS_BUILD_LOAD=2`

如果你不想用包装脚本，也可以直接用 `docker compose`。

## 2. 构建镜像

### 用脚本

```bash
./docker/macos.sh build
```

### 直接用 compose

```bash
DOCKER_PLATFORM=linux/amd64 docker compose build
```

说明：

- Apple Silicon 上默认建议用 `linux/amd64`
- Intel Mac 上也可以继续用这套配置
- 这样可以避免部分 `apt` 包在 `arm64` 上缺失

## 3. 进入开发容器

### 用脚本

```bash
./docker/macos.sh shell
```

### 直接用 compose

```bash
DOCKER_PLATFORM=linux/amd64 docker compose run --rm glins bash
```

进入容器后，ROS 与工作空间环境会自动加载。

## 4. 运行离线 rosbag

把数据放到宿主机 `数据/` 目录，容器内会挂载到 `/data`。

### 用脚本

```bash
./docker/macos.sh bag \
  bagpath:=/data/your.bag \
  imu_topic:=/imu/data \
  lidar_topic:=/velodyne_points \
  gps_week:=2350 \
  start_sec:=117500 \
  end_sec:=118300
```

### 直接用 compose

```bash
DOCKER_PLATFORM=linux/amd64 docker compose run --rm glins roslaunch glins run_bag.launch \
  bagpath:=/data/your.bag \
  imu_topic:=/imu/data \
  lidar_topic:=/velodyne_points \
  gps_week:=2350 \
  start_sec:=117500 \
  end_sec:=118300
```

输出默认写到容器内 `/output/total.pos`，对应宿主机：

```bash
docker/output/total.pos
```

## 5. 运行完整 launch

如果你想直接跑整套 launch：

```bash
./docker/macos.sh gui
```

它会执行：

```bash
roslaunch glins run_bag.launch rviz:=true robot_state_publisher:=true
```

这样做的原因是当前仓库里真正编译出来并可直接运行的离线主程序是 `glins_test_lio`，它由 `run_bag.launch` 启动。

相比之下，`run.launch` 里引用的是一套更早期的节点命名方式，在当前仓库状态下不适合作为 macOS/Docker 的默认入口。

如果你只想跑不带 GUI 的离线链路，优先使用 `run_bag.launch`。

## 6. GUI / RViz on macOS

macOS 上 RViz 需要额外的 X11 支持，推荐两种方式：

### 方案 A：XQuartz

1. 安装 XQuartz
2. 打开 XQuartz
3. 在 XQuartz 偏好设置中允许网络客户端连接
4. 在 macOS 终端执行：

```bash
xhost + 127.0.0.1
```

5. 再运行：

```bash
./docker/macos.sh gui
```

当前 compose 默认会把 `DISPLAY` 设为：

```bash
host.docker.internal:0
```

如果你的 macOS 终端里 `echo $DISPLAY` 是这种形式：

```bash
/private/tmp/com.apple.launchd.../org.xquartz:0
```

不要把它原样带进容器。仓库里的 `./docker/macos.sh` 现在会自动把这类宿主机专用显示地址改成容器可用的：

```bash
host.docker.internal:0
```

如果你是直接在容器里手动执行 `roslaunch`，先显式设置：

```bash
export DISPLAY=host.docker.internal:0
```

### 方案 B：只跑计算，不开 RViz

这是更稳的默认选择，特别是第一次验证环境时。直接运行：

```bash
./docker/macos.sh bag ...
```

### 方案 C：浏览器看容器桌面（推荐给 macOS）

如果 `RViz + XQuartz` 依然不稳定，可以直接打开容器内桌面：

```bash
./docker/macos.sh desktop
```

然后在 macOS 浏览器里打开：

```text
http://localhost:6080/vnc.html
```

你会看到容器里的轻量桌面和一个终端窗口，可以在里面手动运行 ROS 命令。

如果你想直接在这个浏览器桌面里启动 GLINS + RViz：

```bash
./docker/macos.sh gui-web \
  bagpath:=/data/lidar_imu.bag \
  imu_topic:=/imu/data \
  lidar_topic:=/velodyne_points
```

这样 RViz 会尝试在容器自己的虚拟显示里运行，不再依赖 macOS 的 `XQuartz`。

## 7. 持久化目录

为了让你在 macOS 上反复启动容器时不需要每次全量重编，现在这些目录会持久化到宿主机：

- `docker/build`
- `docker/devel`
- `docker/logs`
- `docker/output`

其中：

- `docker/output` 是结果输出目录
- `docker/build` 和 `docker/devel` 是 catkin 编译缓存

## 8. 常见问题

### `libgtsam-dev` 或其他 apt 包在 arm64 上不可用

优先使用：

```bash
DOCKER_PLATFORM=linux/amd64
```

脚本已经默认这样做。

### 每次启动都会重新编译

当前默认会自动执行 `catkin_make`，但因为 `build/devel` 已持久化，通常只会增量编译改动部分。

另外，镜像在第一次构建时会从源码编译 GTSAM，所以首轮 `./docker/macos.sh build` 会明显更久，这是正常现象。现在镜像构建阶段不再执行整套 `catkin_make`，这样可以显著降低 Docker Desktop 在 macOS 上爆内存的概率。

如果你想关闭自动编译：

```bash
GLINS_AUTOBUILD=0 DOCKER_PLATFORM=linux/amd64 docker compose run --rm glins bash
```

如果你的 Docker Desktop 内存依然紧张，可以进一步降低并行度：

```bash
GLINS_BUILD_JOBS=1 GLINS_BUILD_LOAD=1 GTSAM_BUILD_JOBS=1 ./docker/macos.sh build
```

### GUI 启不来

优先确认：

- XQuartz 已启动
- `xhost + 127.0.0.1` 已执行
- 先用 `./docker/macos.sh bag ...` 验证纯计算链路没问题

## 9. 不用 RViz 的本地可视化

如果 macOS 上的 `RViz + XQuartz` 仍然因为 `GLXContext` 失败，可以直接在宿主机本地画结果：

```bash
python3 glins/scripts/visualize_local_results.py
```

默认会读取：

- `docker/output/total.pos`
- `docker/output/MAP/globalmap_lidar_feature.pcd`

如果你想把图片直接保存成文件：

```bash
python3 glins/scripts/visualize_local_results.py \
  --save docker/output/local_visualization.png
```

这个脚本只依赖 `numpy` 和 `matplotlib`，不依赖 `RViz`。
