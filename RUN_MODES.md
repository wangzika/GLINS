# GLINS 三种模式运行说明

本文档说明如何运行当前工程里的三种模式：

- 纯 GNSS + RTK
- GNSS + 惯导
- GNSS + LIO 紧耦合

以下命令默认工作空间为 `/home/ys/glins_ws`，shell 为 zsh。

## 1. 运行前准备

先加载环境：

```zsh
cd /home/ys/glins_ws
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
```

确认三个可执行文件已经生成：

```zsh
ls devel/lib/glins/
```

应至少包含：

```text
glins_test_rtk
glins_test_gins
glins_test_lio
```

## 2. 数据和配置文件

建议把数据放在一个独立目录，例如：

```text
/home/ys/glins_ws/src/GLINS/glins/data_test/
  rover.obs
  base.obs
  brdm.nav 或 brdm*.rnx
  imu.bag
  lidar_imu.bag
```

GNSS/RTK 输入文件在 RTKLIB `.conf` 文件里配置，例如：

```text
inpstr1-path       =/home/ys/glins_ws/src/GLINS/glins/data_test/rover.obs
inpstr2-path       =/home/ys/glins_ws/src/GLINS/glins/data_test/base.obs
inpstr3-path       =/home/ys/glins_ws/src/GLINS/glins/data_test/brdm3640.22p
outstr1-path       =/home/ys/glins_ws/src/GLINS/glins/data_test/rtklib.pos
```

主参数文件 `.yaml` 里至少要确认：

```yaml
glins:
  rtklibConfigPath: /home/ys/glins_ws/src/GLINS/glins/config/conf/your_data.conf
  fgoPath: /home/ys/glins_ws/src/GLINS/glins/data_test/fgo.pos
  imuTopic: "imu/data"
  pointCloudTopic: "velodyne_points"
```

其中：

- `rtklibConfigPath`：指向 RTKLIB 配置文件。
- `fgoPath`：GNSS+惯导或融合结果输出路径。
- `imuTopic`：bag 里的 IMU topic。
- `pointCloudTopic`：bag 里的点云 topic。

注意：不要直接使用 `glins/config/params.yaml` 跑当前三个测试入口。这个文件的顶层命名空间是 `lio_sam:`，而当前代码读取的是 `glins/...`，会导致 `rtklibConfigPath` 没有被加载。当前本机数据可直接使用：

```text
/home/ys/glins_ws/src/GLINS/glins/config/params_user.yaml
```

时间段使用 GPS 周和周内秒传入。如果不传，默认 `0`，表示 RTKLIB 解算全部 GNSS 数据；但 GINS/LIO 模式建议显式传入时间段，避免 bag 过大。

## 3. 模式一：纯 GNSS + RTK

对应可执行文件：

```text
glins_test_rtk
```

运行命令：

```zsh
roslaunch glins run_rtk.launch \
  param_file:=/home/ys/glins_ws/src/GLINS/glins/config/params_user.yaml \
  start_week:=0 \
  start_sec:=0 \
  end_week:=0 \
  end_sec:=0
```

如果只想解算某个时间段：

```zsh
roslaunch glins run_rtk.launch \
  param_file:=/home/ys/glins_ws/src/GLINS/glins/config/params_user.yaml \
  start_week:=2299 \
  start_sec:=111965 \
  end_week:=2299 \
  end_sec:=113000
```

输出文件由 `.conf` 中的 `outstr1-path` 决定。

默认 `run_rtk.launch` 会跳过 FGO 优化，只做 RTKLIB 解算，避免在“纯 RTK”模式下进入额外优化器。如果需要调试 GNSS FGO，可显式加入：

```zsh
run_fgo:=true
```

默认还会启动 RViz 和 `/rtk_path` 轨迹发布节点。RViz 中手动添加 `Path`，topic 选择：

```text
/rtk_path
```

## 4. 模式二：GNSS + 惯导

对应可执行文件：

```text
glins_test_gins
```

这个模式会先运行 RTKLIB 解算 GNSS，再读取 IMU bag 做 GNSS/INS 优化。运行前确认：

- `.conf` 里的 `inpstr*-path` 和 `outstr1-path` 正确。
- `.yaml` 里的 `fgoPath` 输出目录存在。
- `imu_bag` 是包含 IMU topic 的 rosbag。
- `imu_topic` 与 bag 内实际 topic 一致。

运行命令：

```zsh
roslaunch glins run_gins.launch \
  param_file:=/home/ys/glins_ws/src/GLINS/glins/config/params_user.yaml \
  imu_bag:=/home/ys/glins_ws/src/GLINS/glins/data_test/imu.bag \
  imu_topic:=/imu/data \
  start_week:=2299 \
  start_sec:=111965 \
  end_week:=2299 \
  end_sec:=113000
```

输出文件：

- RTK 结果：`.conf` 中的 `outstr1-path`
- GNSS+惯导结果：`.yaml` 中的 `glins/fgoPath`

## 5. 模式三：GNSS + LIO 紧耦合

对应可执行文件：

```text
glins_test_lio
```

这个模式会读取包含 IMU 和 LiDAR 的 rosbag，并在内部运行：

- GNSS/RTK 解算
- IMU 预积分
- LiDAR 前端与建图
- GNSS + IMU + LIO 融合优化

运行前确认：

- `bagpath` 是包含 IMU 和点云 topic 的 rosbag。
- `imu_topic` 与 bag 内 IMU topic 一致。
- `lidar_topic` 与 bag 内点云 topic 一致。
- `.yaml` 中的雷达类型、线数、外参、IMU 噪声参数已经和数据匹配。

运行命令：

```zsh
roslaunch glins run_glins.launch \
  param_file:=/home/ys/glins_ws/src/GLINS/glins/config/params_user.yaml \
  bagpath:=/home/ys/glins_ws/src/GLINS/glins/data_test/lidar_imu.bag \
  imu_topic:=/imu/data \
  lidar_topic:=/velodyne_points \
  lio_output:=/home/ys/glins_ws/src/GLINS/glins/data_test/lio.pos \
  start_week:=2299 \
  start_sec:=111965 \
  end_week:=2299 \
  end_sec:=113000
```

输出文件：

- RTK 结果：`.conf` 中的 `outstr1-path`
- 融合结果：`.yaml` 中的 `glins/fgoPath`
- LIO 轨迹：`lio_output` 参数指定的路径

## 6. 查看 bag 内 topic

如果不确定 topic 名称：

```zsh
rosbag info /path/to/your.bag
```

常见 topic 示例：

```text
/imu/data
/velodyne_points
/ouster/points
/hesai/pandar
```

launch 中传入的 `imu_topic` 和 `lidar_topic` 必须和 `rosbag info` 显示一致。

## 7. 常见问题

### `glins/imuBagPath is empty`

GNSS+惯导模式没有传入 IMU bag。重新运行时加上：

```zsh
imu_bag:=/path/to/imu.bag
```

### `Unable to open output file`

输出目录不存在。先创建目录：

```zsh
mkdir -p /home/ys/glins_ws/src/GLINS/glins/data_test
```

然后确认 `.conf` 的 `outstr1-path` 和 `.yaml` 的 `fgoPath` 都指向存在的目录。

### 没有 GNSS 数据进入优化

检查：

- `.conf` 中 rover/base/nav 文件路径是否正确。
- `start_week/start_sec/end_week/end_sec` 是否覆盖了数据时间段。
- `rtklib.pos` 是否已经正常输出。

### LIO 模式不出点云结果

检查：

- `lidar_topic` 是否和 bag 一致。
- `.yaml` 中 `sensor`、`N_SCAN`、`Horizon_SCAN` 是否匹配雷达。
- `start_week/start_sec/end_week/end_sec` 是否覆盖了 bag 时间段。

## 8. 推荐运行顺序

建议按下面顺序排查：

1. 先跑纯 GNSS + RTK，确认 `.conf` 和 GNSS 数据没问题。
2. 再跑 GNSS + 惯导，确认 IMU bag、IMU topic、外参和时间段没问题。
3. 最后跑 GNSS + LIO 紧耦合，确认 LiDAR topic 和雷达参数没问题。
