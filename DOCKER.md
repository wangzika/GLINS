# Docker 运行说明

本项目是 ROS1 catkin 工程，推荐用 ROS Noetic + Ubuntu 20.04 容器运行。

## 构建镜像

```bash
docker compose build
```

如果 Apple Silicon/arm64 上遇到 `libgtsam-dev` 等 apt 包不可用，可以改用 amd64 仿真构建：

```bash
DOCKER_DEFAULT_PLATFORM=linux/amd64 docker compose build
```

## 进入开发容器

```bash
docker compose run --rm glins
```

进入后如果修改了源码，重新编译：

```bash
catkin_make
source devel/setup.bash
```

## 运行离线 rosbag

把数据放到宿主机 `数据/` 目录，容器内会挂载到 `/data`。

```bash
docker compose run --rm glins roslaunch glins run_bag.launch \
  bagpath:=/data/your.bag \
  imu_topic:=/imu/data \
  lidar_topic:=/velodyne_points \
  gps_week:=2350 \
  start_sec:=117500 \
  end_sec:=118300
```

输出默认写到容器内 `/output/total.pos`，对应宿主机 `docker/output/total.pos`。

## GUI/RViz

默认更适合跑无 GUI 的离线处理。macOS 上 RViz 需要额外配置 XQuartz 或使用 OrbStack/本机 X11 转发；否则建议先只跑计算节点和 rosbag。
