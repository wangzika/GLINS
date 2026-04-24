# RTKLIB File Guide

这份文档面向当前仓库中的 `rtklib/` 子模块，目标是回答两个问题：

1. `rtklib` 里每个文件大致负责什么。
2. 如果我想顺着一条功能链读代码，下一步应该跳到哪个相关文件。

本文档覆盖的范围包括：

- `rtklib/CMakeLists.txt`
- `rtklib/package.xml`
- `rtklib/include/*`
- `rtklib/msg/*`
- `rtklib/src/*`
- `rtklib/src/rcv/*`

不覆盖 `glins/` 等上层业务目录，但会在需要时指出 `rtklib` 与 ROS 消息层的连接点。

## 1. 推荐先读哪些文档

如果你想先建立整体认知，再回来看源码，建议先配合这些已有文档：

- [RTKLIB_WORKFLOW.md](RTKLIB_WORKFLOW.md): RTKLIB 的总体流程图与主调用链。
- [RTKLIB_STRUCTS_GUIDE.md](RTKLIB_STRUCTS_GUIDE.md): `rtk_t`、`raw_t`、`nav_t`、`obs_t` 等核心结构体说明。
- [RTKLIB_ALGORITHM_WORKFLOW.md](RTKLIB_ALGORITHM_WORKFLOW.md): 定位算法主链。
- [RTKLIB_FORMULA_WORKFLOW.md](RTKLIB_FORMULA_WORKFLOW.md): 观测模型和公式层说明。
- [RTKLIB_COMPLETE_GUIDE.md](RTKLIB_COMPLETE_GUIDE.md): 综合说明文档。

## 2. 先建立总图

### 2.1 原始接收机数据到解算结果的主链

`stream/文件 -> rcvraw/rtcm -> 观测与星历 -> pntpos/rtkpos/ppp -> solution`

对应源码通常是：

- 输入层: [src/stream.cpp](src/stream.cpp), [src/streamsvr.cpp](src/streamsvr.cpp), [src/rtksvr.cpp](src/rtksvr.cpp)
- 原始接收机协议层: [src/rcvraw.cpp](src/rcvraw.cpp), [src/rcv](src/rcv)
- RTCM 层: [src/rtcm.cpp](src/rtcm.cpp), [src/rtcm2.cpp](src/rtcm2.cpp), [src/rtcm3.cpp](src/rtcm3.cpp), [src/rtcm3e.cpp](src/rtcm3e.cpp)
- 星历/改正层: [src/ephemeris.cpp](src/ephemeris.cpp), [src/preceph.cpp](src/preceph.cpp), [src/sbas.cpp](src/sbas.cpp), [src/ionex.cpp](src/ionex.cpp), [src/tides.cpp](src/tides.cpp)
- 解算层: [src/pntpos.cpp](src/pntpos.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp), [src/ppp_ar.cpp](src/ppp_ar.cpp), [src/postpos.cpp](src/postpos.cpp)
- 输出层: [src/solution.cpp](src/solution.cpp), [src/rinex.cpp](src/rinex.cpp), [src/convrnx.cpp](src/convrnx.cpp), [src/convkml.cpp](src/convkml.cpp), [src/convgpx.cpp](src/convgpx.cpp)

### 2.2 这份仓库的一个额外特点

这个 `rtklib` 不是纯原版 RTKLIB，它额外接入了 ROS 消息和一些工程定制代码：

- 头文件接口: [include/gnss_tools.h](include/gnss_tools.h)
- ROS 消息: [msg](msg)
- 消息桥接: [src/gnssutils.cpp](src/gnssutils.cpp)
- 抗差估计扩展: [src/robust.cpp](src/robust.cpp)

## 3. 阅读路线建议

### 3.1 如果你关注“原始接收机数据怎么进来”

建议顺序：

1. [include/rtklib.h](include/rtklib.h)
2. [src/rcvraw.cpp](src/rcvraw.cpp)
3. [src/rcv/ublox.cpp](src/rcv/ublox.cpp) 或 [src/rcv/novatel.cpp](src/rcv/novatel.cpp)
4. [src/rtksvr.cpp](src/rtksvr.cpp)
5. [src/rtkpos.cpp](src/rtkpos.cpp)

### 3.2 如果你关注“RINEX/日志转换”

建议顺序：

1. [src/convrnx.cpp](src/convrnx.cpp)
2. [src/rinex.cpp](src/rinex.cpp)
3. [src/rcvraw.cpp](src/rcvraw.cpp)
4. [src/rtcm.cpp](src/rtcm.cpp)

### 3.3 如果你关注“RTK/PPP 解算”

建议顺序：

1. [src/rtkcmn.cpp](src/rtkcmn.cpp)
2. [src/ephemeris.cpp](src/ephemeris.cpp)
3. [src/pntpos.cpp](src/pntpos.cpp)
4. [src/rtkpos.cpp](src/rtkpos.cpp)
5. [src/ppp.cpp](src/ppp.cpp)
6. [src/ppp_ar.cpp](src/ppp_ar.cpp)

## 4. 根目录与构建文件

### [CMakeLists.txt](CMakeLists.txt)

作用：
负责把 `src/` 和 `src/rcv/` 下的源码编译成静态库 `rtklib`，同时生成 `msg/` 中定义的 ROS 消息。

你从这里能看出的关键信息：

- `aux_source_directory(./src RTKLIB_SRCS)` 和 `aux_source_directory(./src/rcv RTKLIB_RCV)` 说明 `src` 与 `src/rcv` 全量参与构建。
- `generate_messages()` 说明本仓库的 RTKLIB 已经和 ROS 消息系统打通。
- 依赖了 `Ceres`、`gflags`、`glog`、`catkin`，说明它并不只是原始 RTKLIB 工具库。

相关文件：
[package.xml](package.xml), [include/rtklib.h](include/rtklib.h), [include/gnss_tools.h](include/gnss_tools.h), [msg](msg), [src](src)

### [package.xml](package.xml)

作用：
声明这是一个 catkin 包，并列出 ROS 构建与运行依赖。

它主要服务于 ROS 工程集成，而不是 RTK 算法本身。

相关文件：
[CMakeLists.txt](CMakeLists.txt), [msg](msg), [include/gnss_tools.h](include/gnss_tools.h)

## 5. 头文件

### [include/rtklib.h](include/rtklib.h)

作用：
整个 RTKLIB 的总头文件。常量、宏、核心结构体、函数声明都在这里。

这是一切源码的共同入口。理解 `obsd_t`、`obs_t`、`nav_t`、`rtk_t`、`raw_t`、`rtcm_t`、`sol_t` 等结构，是理解后续源码的前提。

相关文件：
[src/rtkcmn.cpp](src/rtkcmn.cpp), [src/rcvraw.cpp](src/rcvraw.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [RTKLIB_STRUCTS_GUIDE.md](RTKLIB_STRUCTS_GUIDE.md)

### [include/gnss_tools.h](include/gnss_tools.h)

作用：
工程扩展头文件，把 RTKLIB、ROS 消息、Eigen、gflags、glog 接到一起，并提供 `GNSS_Tools` 类和一些与图优化/FGO 相关的宏定义。

它更像“项目封装层”，不是原版 RTKLIB 的核心头文件。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/gnssutils.cpp](src/gnssutils.cpp), [msg/GNSS_Info.msg](msg/GNSS_Info.msg), [msg/obsdt.msg](msg/obsdt.msg), [msg/sat_state.msg](msg/sat_state.msg)

## 6. ROS 消息定义

### [msg/obsdt.msg](msg/obsdt.msg)

作用：
把 RTKLIB 的单颗卫星单历元观测 `obsd_t` 结构映射到 ROS 消息，包括伪距、载波、多普勒、SNR、锁定状态和 GLONASS 频点。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/gnssutils.cpp](src/gnssutils.cpp), [msg/GNSS_Info_ZD.msg](msg/GNSS_Info_ZD.msg), [msg/GNSS_Info_SD.msg](msg/GNSS_Info_SD.msg)

### [msg/satdt.msg](msg/satdt.msg)

作用：
表示卫星位置、速度、钟差和钟漂，适合把 RTKLIB 中的卫星状态向 ROS 层发布。

相关文件：
[src/ephemeris.cpp](src/ephemeris.cpp), [msg/GNSS_Info_ZD.msg](msg/GNSS_Info_ZD.msg), [msg/GNSS_Info_SD.msg](msg/GNSS_Info_SD.msg)

### [msg/sat_state.msg](msg/sat_state.msg)

作用：
表示单颗卫星在解算中的状态量，包括残差、锁定、周跳、模糊度状态、相位缠绕和改正项等，基本对应 `ssat_t`。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp), [src/gnssutils.cpp](src/gnssutils.cpp)

### [msg/GNSS_Info_ZD.msg](msg/GNSS_Info_ZD.msg)

作用：
零差观测封装，聚合一组单接收机观测、卫星状态和卫星轨道/钟差信息。

相关文件：
[msg/obsdt.msg](msg/obsdt.msg), [msg/satdt.msg](msg/satdt.msg), [msg/sat_state.msg](msg/sat_state.msg), [msg/GNSS_Info.msg](msg/GNSS_Info.msg)

### [msg/GNSS_Info_SD.msg](msg/GNSS_Info_SD.msg)

作用：
单差观测封装，同时包含 rover/base 的观测和卫星状态，适合差分或 FGO 侧消费。

相关文件：
[msg/obsdt.msg](msg/obsdt.msg), [msg/satdt.msg](msg/satdt.msg), [msg/sat_state.msg](msg/sat_state.msg), [msg/GNSS_Info.msg](msg/GNSS_Info.msg)

### [msg/GNSS_Info.msg](msg/GNSS_Info.msg)

作用：
GNSS 总消息，包含解算状态、时间、位置、速度、方差、模糊度以及零差/单差子消息数组。

它相当于 RTKLIB 解算输出在 ROS 侧的综合载体。

相关文件：
[msg/GNSS_Info_ZD.msg](msg/GNSS_Info_ZD.msg), [msg/GNSS_Info_SD.msg](msg/GNSS_Info_SD.msg), [src/gnssutils.cpp](src/gnssutils.cpp), [include/gnss_tools.h](include/gnss_tools.h)

### [msg/cloud_info.msg](msg/cloud_info.msg)

作用：
点云与 IMU/里程计初始化信息，服务于激光与 GNSS 的上层融合。

它不属于 RTKLIB 内核，但说明该库运行在一个更大的多传感器系统中。

相关文件：
[package.xml](package.xml), [CMakeLists.txt](CMakeLists.txt)

### [msg/feature_info.msg](msg/feature_info.msg)

作用：
激光特征匹配与里程计信息消息，同样偏向上层感知/融合模块。

相关文件：
[msg/cloud_info.msg](msg/cloud_info.msg), [package.xml](package.xml)

## 7. 基础公共层

### [src/rtkcmn.cpp](src/rtkcmn.cpp)

作用：
RTKLIB 的公共底座。这里集中放了时间系统、坐标变换、矩阵运算、统计工具、文件路径、卫星系统判别、观测码映射、天线/PCV/ERP/跳秒读取等通用能力。

如果一个功能不明显属于“输入协议”“定位求解”“输出格式”这三类，大概率就在这里或依赖这里。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/ephemeris.cpp](src/ephemeris.cpp), [src/pntpos.cpp](src/pntpos.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp), [src/stream.cpp](src/stream.cpp)

### [src/options.cpp](src/options.cpp)

作用：
系统配置项的解析、保存、加载和默认值管理。

你在 GUI 或配置文件里看到的许多 `pos1-*`、`pos2-*`、`misc-*` 参数，最终都会经过这里映射到 `prcopt_t`、`solopt_t` 等结构体。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/postpos.cpp](src/postpos.cpp), [src/rtksvr.cpp](src/rtksvr.cpp), [src/solution.cpp](src/solution.cpp)

### [src/solution.cpp](src/solution.cpp)

作用：
解算结果的格式化与输出，包括 RTKLIB 自定义解格式、NMEA 输出等。

它是“解算结果如何对外表达”的核心文件，不负责算解本身。

相关文件：
[src/pntpos.cpp](src/pntpos.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp), [src/convkml.cpp](src/convkml.cpp), [src/convgpx.cpp](src/convgpx.cpp)

### [src/datum.cpp](src/datum.cpp)

作用：
大地基准转换参数与相关变换函数。

一般不会成为主流程入口，但在坐标系统转换和输出时会被调用。

相关文件：
[src/rtkcmn.cpp](src/rtkcmn.cpp), [src/solution.cpp](src/solution.cpp)

### [src/geoid.cpp](src/geoid.cpp)

作用：
大地水准面模型处理，例如 EGM96、EGM2008，对椭球高与正常高之间的转换提供支持。

相关文件：
[src/solution.cpp](src/solution.cpp), [src/convkml.cpp](src/convkml.cpp), [src/convgpx.cpp](src/convgpx.cpp)

### [src/gis.cpp](src/gis.cpp)

作用：
GIS 数据处理，主要是 shapefile 相关读写与几何支持。

它是偏配套功能的模块，通常不在标准 RTK 解算主链上。

相关文件：
[src/convkml.cpp](src/convkml.cpp), [src/solution.cpp](src/solution.cpp)

### [src/tle.cpp](src/tle.cpp)

作用：
两行轨道根数 TLE 的解析与传播，主要用于与非 GNSS 轨道数据相关的辅助功能。

相关文件：
[src/preceph.cpp](src/preceph.cpp), [src/ephemeris.cpp](src/ephemeris.cpp)

## 8. 环境与改正模型

### [src/ephemeris.cpp](src/ephemeris.cpp)

作用：
广播星历、卫星钟差、卫星位置计算的核心文件。

定位模块几乎都会依赖这里来计算某颗卫星在某时刻的坐标、钟差、健康状态和精度指标。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/rcvraw.cpp](src/rcvraw.cpp), [src/rinex.cpp](src/rinex.cpp), [src/pntpos.cpp](src/pntpos.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp)

### [src/preceph.cpp](src/preceph.cpp)

作用：
精密星历和精密钟差处理，例如 SP3、CLK 等精密产品的读取与插值。

PPP 模式尤其依赖这个文件。

相关文件：
[src/rinex.cpp](src/rinex.cpp), [src/ppp.cpp](src/ppp.cpp), [src/postpos.cpp](src/postpos.cpp), [src/download.cpp](src/download.cpp)

### [src/ionex.cpp](src/ionex.cpp)

作用：
IONEX 电离层格网产品的读取与插值。

它服务于改正模型，不负责观测预处理或解算框架本身。

相关文件：
[src/pntpos.cpp](src/pntpos.cpp), [src/ppp.cpp](src/ppp.cpp), [src/postpos.cpp](src/postpos.cpp)

### [src/tides.cpp](src/tides.cpp)

作用：
潮汐位移改正，包括固体潮、极潮、海潮等地球物理改正。

高精度 PPP/RTK 解算会调用这里改正站坐标或观测模型。

相关文件：
[src/pntpos.cpp](src/pntpos.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp), [src/preceph.cpp](src/preceph.cpp)

### [src/sbas.cpp](src/sbas.cpp)

作用：
SBAS 消息解码与 SBAS 轨道/钟差/电离层改正应用。

它既和原始接收机数据层有关，也和解算层有关，是一个“中间层”模块。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/rcv/novatel.cpp](src/rcv/novatel.cpp), [src/rcv/septentrio.cpp](src/rcv/septentrio.cpp), [src/pntpos.cpp](src/pntpos.cpp)

### [src/download.cpp](src/download.cpp)

作用：
下载 GNSS 数据产品，例如精密星历、钟差、RINEX 等远程资源。

它属于工具支持层，常用于后处理的数据准备阶段。

相关文件：
[src/preceph.cpp](src/preceph.cpp), [src/rinex.cpp](src/rinex.cpp), [src/postpos.cpp](src/postpos.cpp)

## 9. 标准格式与协议层

### [src/rtcm.cpp](src/rtcm.cpp)

作用：
RTCM 框架层，管理 RTCM 状态结构、公共入口和类型分发。

它自己不承担所有具体消息细节，而是把 V2/V3 消息分给子模块。

相关文件：
[src/rtcm2.cpp](src/rtcm2.cpp), [src/rtcm3.cpp](src/rtcm3.cpp), [src/rtcm3e.cpp](src/rtcm3e.cpp), [src/streamsvr.cpp](src/streamsvr.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rtcm2.cpp](src/rtcm2.cpp)

作用：
RTCM Version 2 消息解码。

主要是传统差分改正与较早期消息类型的处理。

相关文件：
[src/rtcm.cpp](src/rtcm.cpp), [src/streamsvr.cpp](src/streamsvr.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rtcm3.cpp](src/rtcm3.cpp)

作用：
RTCM Version 3 消息解码，包括 MSM、SSR、星历、观测消息等。

现代网络 RTK、NTRIP 差分链路基本都会用到这里。

相关文件：
[src/rtcm.cpp](src/rtcm.cpp), [src/rtcm3e.cpp](src/rtcm3e.cpp), [src/stream.cpp](src/stream.cpp), [src/streamsvr.cpp](src/streamsvr.cpp), [src/rtksvr.cpp](src/rtksvr.cpp)

### [src/rtcm3e.cpp](src/rtcm3e.cpp)

作用：
RTCM 3 编码器，把 RTKLIB 内部观测/改正信息重新编码成 RTCM 3 消息输出。

如果你在做“输入接收机原始数据，输出 RTCM”的流转换，这个文件很关键。

相关文件：
[src/rtcm3.cpp](src/rtcm3.cpp), [src/streamsvr.cpp](src/streamsvr.cpp), [src/rtksvr.cpp](src/rtksvr.cpp)

### [src/stream.cpp](src/stream.cpp)

作用：
统一的流输入输出抽象层，封装串口、TCP、NTRIP、文件等通道。

它是“数据怎么进进出出”的基础设施，不关心 GNSS 内容本身。

相关文件：
[src/streamsvr.cpp](src/streamsvr.cpp), [src/rtksvr.cpp](src/rtksvr.cpp), [src/rtcm.cpp](src/rtcm.cpp), [src/rcvraw.cpp](src/rcvraw.cpp)

### [src/streamsvr.cpp](src/streamsvr.cpp)

作用：
流服务器与流格式转换器，支持把输入流解码后转换成 RTCM 等输出流。

它经常扮演“中间转发站”的角色。

相关文件：
[src/stream.cpp](src/stream.cpp), [src/rtcm.cpp](src/rtcm.cpp), [src/rtcm3e.cpp](src/rtcm3e.cpp), [src/rcvraw.cpp](src/rcvraw.cpp)

### [src/rtksvr.cpp](src/rtksvr.cpp)

作用：
实时 RTK 服务器主控模块，负责多个输入流、解码、同步、解算和输出组织。

如果你想找实时模式下“真正把所有模块串起来”的核心文件，优先看它。

相关文件：
[src/stream.cpp](src/stream.cpp), [src/streamsvr.cpp](src/streamsvr.cpp), [src/rcvraw.cpp](src/rcvraw.cpp), [src/rtcm.cpp](src/rtcm.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [src/solution.cpp](src/solution.cpp)

### [src/rinex.cpp](src/rinex.cpp)

作用：
RINEX 文件读写与解析，是离线数据处理中最重要的标准格式文件之一。

它负责把观测文件、导航文件、钟差文件等标准文本格式映射到 RTKLIB 内部结构。

相关文件：
[src/convrnx.cpp](src/convrnx.cpp), [src/preceph.cpp](src/preceph.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/postpos.cpp](src/postpos.cpp)

### [src/convrnx.cpp](src/convrnx.cpp)

作用：
把 RTCM 或接收机原始日志转换为 RINEX 的翻译器。

它会同时依赖 RTCM 解码、原始接收机解码、RINEX 输出三条链。

相关文件：
[src/rinex.cpp](src/rinex.cpp), [src/rcvraw.cpp](src/rcvraw.cpp), [src/rtcm.cpp](src/rtcm.cpp), [src/stream.cpp](src/stream.cpp)

### [src/convkml.cpp](src/convkml.cpp)

作用：
把解算结果转成 Google Earth KML，方便轨迹可视化。

相关文件：
[src/solution.cpp](src/solution.cpp), [src/geoid.cpp](src/geoid.cpp), [src/gis.cpp](src/gis.cpp)

### [src/convgpx.cpp](src/convgpx.cpp)

作用：
把解算结果转成 GPX 轨迹格式。

相关文件：
[src/solution.cpp](src/solution.cpp), [src/geoid.cpp](src/geoid.cpp)

## 10. 原始接收机与导航电文层

### [src/rcvraw.cpp](src/rcvraw.cpp)

作用：
原始接收机输入的统一入口，以及 GPS/GLO/GAL/BDS/QZSS/NavIC 等导航电文的公共解码库。

这个文件一半是“输入分发器”，把字节流按格式分发到 `src/rcv/*`；另一半是“通用导航解码器”，供不同厂商协议复用。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/rtksvr.cpp](src/rtksvr.cpp), [src/convrnx.cpp](src/convrnx.cpp), [src/rcv](src/rcv), [src/ephemeris.cpp](src/ephemeris.cpp), [RTKLIB_WORKFLOW.md](RTKLIB_WORKFLOW.md)

## 11. 定位与估计层

### [src/pntpos.cpp](src/pntpos.cpp)

作用：
单点定位，负责最基础的伪距定位与测速等功能。

很多高级模式在初始化、粗定位或某些回退路径上都会经过这里。

相关文件：
[src/ephemeris.cpp](src/ephemeris.cpp), [src/tides.cpp](src/tides.cpp), [src/ionex.cpp](src/ionex.cpp), [src/sbas.cpp](src/sbas.cpp), [src/solution.cpp](src/solution.cpp)

### [src/rtkpos.cpp](src/rtkpos.cpp)

作用：
RTK 差分精密定位主实现，包括状态更新、残差构建、周跳探测、模糊度固定等。

这是实时/相对定位模式中最核心的解算文件之一。

相关文件：
[src/pntpos.cpp](src/pntpos.cpp), [src/lambda.cpp](src/lambda.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/rtkcmn.cpp](src/rtkcmn.cpp), [src/solution.cpp](src/solution.cpp), [src/robust.cpp](src/robust.cpp)

### [src/ppp.cpp](src/ppp.cpp)

作用：
PPP 精密单点定位主实现。

它依赖精密产品、潮汐改正、电离层/对流层建模和更复杂的状态参数设计。

相关文件：
[src/preceph.cpp](src/preceph.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/tides.cpp](src/tides.cpp), [src/ppp_ar.cpp](src/ppp_ar.cpp), [src/solution.cpp](src/solution.cpp), [src/robust.cpp](src/robust.cpp)

### [src/ppp_ar.cpp](src/ppp_ar.cpp)

作用：
PPP 模式下的模糊度固定逻辑，是 `ppp.cpp` 的子模块。

相关文件：
[src/ppp.cpp](src/ppp.cpp), [src/lambda.cpp](src/lambda.cpp)

### [src/lambda.cpp](src/lambda.cpp)

作用：
整数模糊度求解的 LAMBDA/MLAMBDA 算法实现。

它是 RTK/PPP 模糊度固定的通用数学内核。

相关文件：
[src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp_ar.cpp](src/ppp_ar.cpp), [src/rtkcmn.cpp](src/rtkcmn.cpp)

### [src/postpos.cpp](src/postpos.cpp)

作用：
后处理定位总控模块。它组织文件读取、产品加载、模式选择和解算流程，面向离线任务。

与 `rtksvr.cpp` 相比，它更偏离线批处理，而不是实时服务。

相关文件：
[src/rinex.cpp](src/rinex.cpp), [src/preceph.cpp](src/preceph.cpp), [src/pntpos.cpp](src/pntpos.cpp), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp), [src/options.cpp](src/options.cpp)

## 12. 工程扩展与项目定制

### [src/gnssutils.cpp](src/gnssutils.cpp)

作用：
把 RTKLIB 内部结构转换成 ROS 消息，例如 `obsd_t -> obsdt.msg`、`ssat_t -> sat_state.msg`。

这是当前仓库把 RTKLIB 解算状态接到 ROS 发布链上的桥接层。

相关文件：
[include/gnss_tools.h](include/gnss_tools.h), [include/rtklib.h](include/rtklib.h), [msg/obsdt.msg](msg/obsdt.msg), [msg/sat_state.msg](msg/sat_state.msg), [msg/GNSS_Info.msg](msg/GNSS_Info.msg)

### [src/robust.cpp](src/robust.cpp)

作用：
项目自定义的抗差估计模块，提供 Huber、IGG、Cauchy 等权函数与稳健最小二乘权重更新。

这不是原版 RTKLIB 的标准主干文件，而是当前工程为鲁棒解算引入的扩展。

相关文件：
[include/rtklib.h](include/rtklib.h), [src/rtkpos.cpp](src/rtkpos.cpp), [src/ppp.cpp](src/ppp.cpp), [include/gnss_tools.h](include/gnss_tools.h)

## 13. `src/rcv/` 厂商接收机协议目录

这一组文件的共同职责是：

- 从厂商私有二进制协议中拆包
- 生成 RTKLIB 内部统一观测/导航结构
- 必要时调用 [src/rcvraw.cpp](src/rcvraw.cpp) 中的公共导航电文解码函数

### [src/rcv/novatel.cpp](src/rcv/novatel.cpp)

作用：
NovAtel OEM7/OEM6/OEM5/OEM4/OEM3 原始协议解析器。

支持原始观测、GPS/GLO/GAL/QZSS/BDS/NavIC 星历、SBAS、电离层与时间参数等，是最完整的厂商解析器之一。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/sbas.cpp](src/sbas.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rcv/ublox.cpp](src/rcv/ublox.cpp)

作用：
u-blox UBX 协议解析器，支持 `RXM-RAW/RAWX`、`SFRBX`、`TRK-*` 等消息。

在实际工程中很常见，适合作为学习 `src/rcv` 目录的入门文件。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/rtksvr.cpp](src/rtksvr.cpp), [src/convrnx.cpp](src/convrnx.cpp), [src/ephemeris.cpp](src/ephemeris.cpp)

### [src/rcv/septentrio.cpp](src/rcv/septentrio.cpp)

作用：
Septentrio SBF 协议解析器。

对多系统、多信号支持比较完整，观测和导航页解析都较系统化。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/sbas.cpp](src/sbas.cpp), [src/ephemeris.cpp](src/ephemeris.cpp)

### [src/rcv/swiftnav.cpp](src/rcv/swiftnav.cpp)

作用：
Swift Navigation SBP 协议解析器。

除二进制输入外，还支持从 SBP JSON 日志读入，这在离线分析时很有用。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rcv/javad.cpp](src/rcv/javad.cpp)

作用：
Javad GREIS/GRIL 协议解析器。

它把大量两字符消息拆分后再重组为统一观测，是 `src/rcv` 中实现风格比较“离散”的一个文件。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/rtksvr.cpp](src/rtksvr.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rcv/skytraq.cpp](src/rcv/skytraq.cpp)

作用：
SkyTraq 二进制协议解析器，支持原始观测、扩展观测、GPS/QZSS、GLONASS、BDS 等导航消息。

还提供命令生成函数用于配置接收机。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rcv/crescent.cpp](src/rcv/crescent.cpp)

作用：
Hemisphere Crescent/Eclipse `$BIN` 协议解析器。

支持原始观测、星历、WAAS、GLONASS 相关消息。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/sbas.cpp](src/sbas.cpp), [src/ephemeris.cpp](src/ephemeris.cpp)

### [src/rcv/binex.cpp](src/rcv/binex.cpp)

作用：
BINEX 通用二进制交换格式解析器。

它不是单一厂商协议，更接近标准交换格式，因此经常用于跨设备数据流。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rcv/nvs.cpp](src/rcv/nvs.cpp)

作用：
NVS BINR 协议解析器，支持原始观测、星历、导航比特、iono/utc 等消息。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/ephemeris.cpp](src/ephemeris.cpp), [src/convrnx.cpp](src/convrnx.cpp)

### [src/rcv/rt17.cpp](src/rcv/rt17.cpp)

作用：
Trimble RT-17 协议解析器。

它比较特殊，因为需要自己维护额外状态和多包重组缓冲，所以同时依赖 `init_rt17()` / `free_rt17()` 这一类私有初始化逻辑。

相关文件：
[src/rcvraw.cpp](src/rcvraw.cpp), [src/rtksvr.cpp](src/rtksvr.cpp), [src/convrnx.cpp](src/convrnx.cpp)

## 14. 读源码时可以按这些“链路”串起来

### 14.1 实时 RTK 链路

[src/stream.cpp](src/stream.cpp) -> [src/rtksvr.cpp](src/rtksvr.cpp) -> [src/rcvraw.cpp](src/rcvraw.cpp) / [src/rtcm.cpp](src/rtcm.cpp) -> [src/ephemeris.cpp](src/ephemeris.cpp) -> [src/rtkpos.cpp](src/rtkpos.cpp) -> [src/solution.cpp](src/solution.cpp)

### 14.2 原始接收机日志转 RINEX

[src/convrnx.cpp](src/convrnx.cpp) -> [src/rcvraw.cpp](src/rcvraw.cpp) -> [src/rcv](src/rcv) -> [src/rinex.cpp](src/rinex.cpp)

### 14.3 RTCM 输入输出链

[src/stream.cpp](src/stream.cpp) -> [src/rtcm.cpp](src/rtcm.cpp) -> [src/rtcm2.cpp](src/rtcm2.cpp) / [src/rtcm3.cpp](src/rtcm3.cpp) -> [src/rtcm3e.cpp](src/rtcm3e.cpp) -> [src/streamsvr.cpp](src/streamsvr.cpp)

### 14.4 PPP 链路

[src/rinex.cpp](src/rinex.cpp) -> [src/preceph.cpp](src/preceph.cpp) / [src/ionex.cpp](src/ionex.cpp) / [src/tides.cpp](src/tides.cpp) -> [src/ppp.cpp](src/ppp.cpp) -> [src/ppp_ar.cpp](src/ppp_ar.cpp) -> [src/solution.cpp](src/solution.cpp)

### 14.5 ROS 输出链

[include/rtklib.h](include/rtklib.h) -> [src/gnssutils.cpp](src/gnssutils.cpp) -> [msg/obsdt.msg](msg/obsdt.msg) / [msg/sat_state.msg](msg/sat_state.msg) / [msg/GNSS_Info.msg](msg/GNSS_Info.msg)

## 15. 如果只想抓住最重要的 10 个文件

优先建议：

1. [include/rtklib.h](include/rtklib.h)
2. [src/rtkcmn.cpp](src/rtkcmn.cpp)
3. [src/rcvraw.cpp](src/rcvraw.cpp)
4. [src/rtcm.cpp](src/rtcm.cpp)
5. [src/ephemeris.cpp](src/ephemeris.cpp)
6. [src/pntpos.cpp](src/pntpos.cpp)
7. [src/rtkpos.cpp](src/rtkpos.cpp)
8. [src/ppp.cpp](src/ppp.cpp)
9. [src/rtksvr.cpp](src/rtksvr.cpp)
10. [src/rinex.cpp](src/rinex.cpp)

如果你已经确认后续重点是某个协议，再继续深挖对应的 [src/rcv](src/rcv) 文件即可。
