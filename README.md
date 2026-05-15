# GLINS 编译说明

本文档说明如何在 Ubuntu + ROS1 环境下编译本仓库。当前代码包含两个 catkin 包：

- `rtklib`：GNSS/RTK 相关静态库与消息定义。
- `glins`：主程序包，依赖 `rtklib`，会编译 `glins_test_rtk`、`glins_test_gins`、`glins_test_lio` 等可执行文件。

## 1. 推荐环境

已在本机检测到 ROS Noetic，因此推荐使用：

- Ubuntu 20.04
- ROS Noetic
- C++14
- catkin

其他 ROS1 版本也可能可用，但包名和依赖版本需要自行对应调整。

## 2. 安装依赖

先加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
```

安装 ROS 和系统依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  ros-noetic-desktop-full \
  ros-noetic-tf \
  ros-noetic-cv-bridge \
  ros-noetic-pcl-conversions \
  ros-noetic-pcl-ros \
  ros-noetic-rosbag \
  ros-noetic-rosbag-storage \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  ros-noetic-gtsam \
  ros-noetic-jsk-recognition-msgs \
  libpcl-dev \
  libopencv-dev \
  libceres-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libbackward-cpp-dev \
  libopenblas-dev \
  liblapack-dev \
  libsuitesparse-dev \
  libdw-dev
```

如果系统源里没有 `ros-noetic-gtsam`，需要从源码安装 GTSAM。安装完成后确认 CMake 可以找到 GTSAM：

```bash
ldconfig -p | grep gtsam
```

## 3. 准备 catkin 工作空间

本仓库目录 `GLINS` 里面直接放着 catkin 包 `glins` 和 `rtklib`。当前机器上的工作空间路径为：

```text
/home/ys/glins_ws
```

目录结构应类似：

```text
/home/ys/glins_ws/
  src/
    GLINS/
      glins/
        package.xml
        CMakeLists.txt
      rtklib/
        package.xml
        CMakeLists.txt
```

如果是新机器，从零准备工作空间时可以按下面方式放置源码：

```bash
mkdir -p /home/ys/glins_ws/src
cd /home/ys/glins_ws/src
# 将 GLINS 仓库放到这里，最终路径为 /home/ys/glins_ws/src/GLINS
```

## 4. 编译

进入工作空间根目录并编译：

```bash
cd /home/ys/glins_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

如果当前终端是 zsh，加载 ROS 环境时使用：

```zsh
source /opt/ros/noetic/setup.zsh
```

如果并行编译输出太多，真实错误不容易定位，可以用单线程编译：

```bash
cd /home/ys/glins_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -j1 -l1
```

编译成功后加载当前工作空间环境：

```bash
source /home/ys/glins_ws/devel/setup.bash
```

如果当前终端是 zsh，应该加载：

```zsh
source /home/ys/glins_ws/devel/setup.zsh
```

不要在 zsh 里执行 `source ./devel/setup.bash`，否则可能出现下面的错误：

```text
./devel/setup.bash:.:8: no such file or directory: /home/ys/glins_ws/setup.sh
```

可以把下面两行加入 `~/.bashrc`，以后打开 bash 新终端会自动加载：

```bash
source /opt/ros/noetic/setup.bash
source /home/ys/glins_ws/devel/setup.bash
```

如果默认 shell 是 zsh，则加入 `~/.zshrc`：

```zsh
source /opt/ros/noetic/setup.zsh
source /home/ys/glins_ws/devel/setup.zsh
```

## 5. 验证编译结果

检查包是否能被 ROS 找到：

```bash
rospack find rtklib
rospack find glins
```

检查可执行文件是否生成：

```bash
ls /home/ys/glins_ws/devel/lib/glins/
```

正常情况下应看到类似文件：

```text
glins_test_gins
glins_test_lio
glins_test_rtk
```

## 6. 运行示例

三种模式的详细运行方式见 [RUN_MODES.md](RUN_MODES.md)。

启动前请先加载工作空间环境：

```bash
source /home/ys/glins_ws/devel/setup.bash
```

zsh 终端使用：

```zsh
source /home/ys/glins_ws/devel/setup.zsh
```

运行 launch 文件示例：

```bash
roslaunch glins run.launch
```

其他可用 launch 文件在 `glins/launch/` 下，例如：

```bash
roslaunch glins run_test_lio.launch
roslaunch glins test.launch
roslaunch glins pandar.launch
roslaunch glins ouster128.launch
```

实际运行前请根据自己的雷达、IMU、GNSS、bag 数据路径修改 `glins/config/*.yaml` 和对应 launch 文件中的参数。

## 7. 常见问题

### 找不到 `rtklib`

错误示例：

```text
Could not find a package configuration file provided by "rtklib"
```

处理方式：

1. 确认 `GLINS/rtklib/package.xml` 存在。
2. 确认 `GLINS` 位于 `/home/ys/glins_ws/src/` 下。
3. 回到工作空间根目录重新编译：

```bash
cd /home/ys/glins_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 找不到 GTSAM

错误示例：

```text
Could not find GTSAM
```

处理方式：

```bash
sudo apt install -y ros-noetic-gtsam
```

如果 apt 源没有该包，需要从源码安装 GTSAM，并确认安装路径能被 CMake 找到。

### 找不到 `jsk_recognition_msgs/PolygonArray.h`

错误示例：

```text
fatal error: jsk_recognition_msgs/PolygonArray.h: No such file or directory
```

处理方式：

```bash
sudo apt install -y ros-noetic-jsk-recognition-msgs
```

本项目的 `patchworkpp.hpp` 使用了 `jsk_recognition_msgs::PolygonArray`，因此需要安装并在 `glins` 包中声明该依赖。

### 找不到 `backward.hpp`

错误示例：

```text
fatal error: backward.hpp: No such file or directory
```

`backward.hpp` 只用于程序崩溃时打印调用栈，不影响核心算法。当前代码已经将它改成可选包含：未安装时会跳过，安装后自动启用。

如需启用该功能：

```bash
sudo apt install -y libbackward-cpp-dev
```

### 找不到 Ceres、glog、gflags

处理方式：

```bash
sudo apt install -y libceres-dev libgoogle-glog-dev libgflags-dev
```

### 找不到 `dw`

本项目的 `glins/CMakeLists.txt` 链接了 `dw`，对应系统库通常来自 `libdw-dev`：

```bash
sudo apt install -y libdw-dev
```

### 编译时提示消息头文件不存在

如果出现类似 `xxx.h: No such file or directory`，且文件来自 ROS msg/srv 生成目录，通常是上一次编译中断导致生成不完整。可以清理后重编：

```bash
cd /home/ys/glins_ws
catkin_make clean
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 终端找不到包或可执行文件

每个新终端都需要加载环境：

```bash
source /opt/ros/noetic/setup.bash
source /home/ys/glins_ws/devel/setup.bash
```

zsh 终端使用：

```zsh
source /opt/ros/noetic/setup.zsh
source /home/ys/glins_ws/devel/setup.zsh
```

## 8. 快速命令汇总

如果使用 bash：

```bash
source /opt/ros/noetic/setup.bash
cd /home/ys/glins_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

rospack find glins
ls devel/lib/glins/
```

如果使用 zsh：

```zsh
source /opt/ros/noetic/setup.zsh
cd /home/ys/glins_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.zsh

rospack find glins
ls devel/lib/glins/
```

如果需要清理后重新完整编译：

```bash
cd /home/ys/glins_ws
catkin_make clean
catkin_make -DCMAKE_BUILD_TYPE=Release -j1 -l1
```
