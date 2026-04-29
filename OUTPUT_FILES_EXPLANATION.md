# GLINS 输出文件说明

本文档解释当前运行后 `docker/output/` 目录中各个结果文件的作用、生成位置、字段含义以及适合用来分析什么问题。

当前常见输出目录结构如下：

```text
docker/output/
├── MAP/
│   └── globalmap_lidar_feature.pcd
├── rtklib.pos
├── rtklib.pos.stat
├── rtklib_events.pos
├── rtklib_fgo.pos
├── total.pos
└── rtk_fgo_compare.png
```

## 1. `rtklib.pos`

`rtklib.pos` 是 RTKLIB 纯 GNSS RTK 解算结果文件。

它不包含 LiDAR、IMU、因子图优化结果，只表示 RTKLIB 根据 rover/base RINEX 观测文件和导航星历独立解算出来的 GNSS 轨迹。

对应关系：

```text
输入：rover.obs + base.obs + nav/rnx 星历文件
算法：RTKLIB RTK
输出：docker/output/rtklib.pos
```

典型文件头：

```text
% program   : RTKLIB ver.demo5 b34c
% inp file  : /data/gnss/rover.obs
% inp file  : /data/gnss/base.obs
% inp file  : /data/gnss/BRDM00DLR_S_20240290000_01D_MN.rnx
% obs start : 2024/01/29 07:06:05.0 GPST (week2299 111965.0s)
% obs end   : 2024/01/29 07:11:20.0 GPST (week2299 112280.0s)
% pos mode  : Kinematic
% freqs     : L1+L2/E5b+L5
% solution  : Forward
% elev mask : 10.0 deg
% dynamics  : on
% navi sys  : GPS Galileo BDS
% amb res   : Fix and Hold
```

文件头字段说明：

| 字段 | 含义 |
| --- | --- |
| `program` | 使用的 RTKLIB 版本 |
| `inp file` | 参与解算的输入文件，包括流动站观测、基站观测、导航星历 |
| `obs start/end` | GNSS 原始观测的起止时间，时间系统是 GPST |
| `pos mode` | 解算模式，`Kinematic` 表示动态 RTK |
| `freqs` | 使用的频点，例如 GPS L1/L2、Galileo E5b/L5 等 |
| `solution` | 解算方向，`Forward` 表示正向滤波 |
| `elev mask` | 高度角截止角，低于该角度的卫星一般不参与 |
| `dynamics` | 是否启用动态模型 |
| `tidecorr` | 是否做地球潮汐改正 |
| `ionos opt` | 电离层改正方式 |
| `tropo opt` | 对流层改正方式 |
| `ephemeris` | 星历类型，当前为广播星历 |
| `navi sys` | 使用的卫星系统 |
| `amb res` | 模糊度固定策略，`Fix and Hold` 表示固定后保持 |
| `val thres` | 模糊度 ratio 检验阈值 |
| `ref pos` | 基站 ECEF 坐标 |

数据行格式：

```text
GPST_week GPST_sow x_ecef y_ecef z_ecef Q ns sdx sdy sdz sdxy sdyz sdzx age ratio
```

示例：

```text
2299 111975.000  -2267128.6552   5009529.1568   3221136.8047   1  46   0.0052   0.0082   0.0069  -0.0038   0.0058  -0.0037   0.00   39.2
```

每列说明：

| 列 | 含义 |
| --- | --- |
| `GPST_week` | GPS 周数 |
| `GPST_sow` | GPS 周内秒，单位秒 |
| `x_ecef/y_ecef/z_ecef` | WGS84 ECEF 坐标，单位米 |
| `Q` | 解状态，`1=fix`，`2=float`，`4=dgps`，`5=single`，`6=ppp` |
| `ns` | 参与解算的卫星数量 |
| `sdx/sdy/sdz` | ECEF 三方向位置标准差，单位米 |
| `sdxy/sdyz/sdzx` | 位置协方差相关项 |
| `age` | 差分龄期，单位秒 |
| `ratio` | LAMBDA 整周模糊度固定 ratio 值 |

重点看法：

- `Q=1` 表示 RTK fixed，通常最可靠。
- `Q=2` 表示 RTK float，载波模糊度未成功固定。
- `ratio` 越大，固定越可信；当前配置阈值为 `3.0`。
- `sdx/sdy/sdz` 在 fixed 后通常会明显变小。

## 2. `rtklib.pos.stat`

`rtklib.pos.stat` 是 RTKLIB 解算状态诊断文件。

它不是最终轨迹，而是 RTKLIB 在每个历元输出的内部状态日志。它适合用来分析：

- 为什么某段从 `fix` 掉到 `float`
- 哪颗卫星残差异常
- 哪个频点发生周跳
- 哪些观测被剔除
- 哪些卫星的模糊度进入 fixed/hold
- 当前 RTK 结果是否被多路径或低信噪比影响

生成代码主要在：

```text
rtklib/src/rtkpos.cpp
outsolstat()
```

文件中常见记录类型：

```text
$POS
$VELACC
$CLK
$SAT
```

### 2.1 `$POS`

格式：

```text
$POS,week,tow,stat,posx,posy,posz,posxf,posyf,poszf
```

示例：

```text
$POS,2299,111975.000,1,-2267128.9791,5009530.0916,3221137.3214,-2267128.6552,5009529.1568,3221136.8047
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `week` | GPS 周数 |
| `tow` | GPS 周内秒 |
| `stat` | 解状态，含义同 `rtklib.pos` 的 `Q` |
| `posx/posy/posz` | 浮点解 ECEF 坐标 |
| `posxf/posyf/poszf` | 固定解 ECEF 坐标 |

如果 `stat=2`，后面的固定解坐标常常是 `0,0,0`，说明当前还没有 fixed 解。

如果 `stat=1`，通常前 3 个坐标是 float 状态，后 3 个坐标是 fixed 状态。

### 2.2 `$VELACC`

格式：

```text
$VELACC,week,tow,stat,vele,veln,velu,acce,accn,accu,velef,velnf,veluf,accef,accnf,accuf
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `vele/veln/velu` | 浮点解 ENU 速度，单位 m/s |
| `acce/accn/accu` | 浮点解 ENU 加速度，单位 m/s² |
| `velef/velnf/veluf` | 固定解 ENU 速度，单位 m/s |
| `accef/accnf/accuf` | 固定解 ENU 加速度，单位 m/s² |

这个记录和 RTKLIB 配置中的 `dynamics : on` 有关。打开动态模型后，RTKLIB 会估计运动状态，因此会输出速度和加速度诊断信息。

### 2.3 `$CLK`

格式：

```text
$CLK,week,tow,stat,receiver_id,clk1,clk2,clk3,clk4,cmn_bias
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `receiver_id` | 接收机编号，通常 `1` 表示 rover |
| `clk1` | GPS 接收机钟差，单位 ns |
| `clk2` | GLONASS-GPS 系统间钟差，单位 ns |
| `clk3` | Galileo-GPS 系统间钟差，单位 ns |
| `clk4` | BDS-GPS 系统间钟差，单位 ns |
| `cmn_bias` | RTKLIB 内部公共相位偏差 |

当前配置使用 `GPS Galileo BDS`，所以重点看 GPS、Galileo、BDS 相关钟差。

### 2.4 `$SAT`

`$SAT` 是 `.stat` 文件里最重要的记录。它按“每颗卫星、每个频点”输出一行。

格式：

```text
$SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc,bias,bias_var,icbias
```

示例：

```text
$SAT,2299,111975.000,G02,1,160.3,48.0,0.5878,0.0040,1,45,2,0,1,0,0,0,-183.61,151.073324,0.00000
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `sat` | 卫星编号，例如 `G02`、`E13`、`C24` |
| `frq` | 频点编号，`1/2/3` 表示第几个频点 |
| `az` | 方位角，单位度 |
| `el` | 高度角，单位度 |
| `resp` | 伪距残差，单位米 |
| `resc` | 载波相位残差，单位米 |
| `vsat` | 当前频点观测是否有效，`1=有效`，`0=无效` |
| `snr` | rover 信噪比，单位 dB-Hz |
| `fix` | 模糊度状态 |
| `slip` | 周跳标志 |
| `lock` | 载波锁定计数 |
| `outc` | 数据中断计数 |
| `slipc` | 周跳累计次数 |
| `rejc` | 观测被拒绝/剔除次数 |
| `bias` | RTKLIB 内部载波相位 bias / ambiguity 状态 |
| `bias_var` | bias 状态方差 |
| `icbias` | GLONASS 频间/通道偏差，非 GLONASS 通常为 0 |

`fix` 字段常见含义：

| 值 | 含义 |
| --- | --- |
| `0` | 无数据或未使用 |
| `1` | float |
| `2` | fixed |
| `3` | hold |
| `4` | PPP 相关状态 |

诊断建议：

- 如果 `resc` 明显变大，说明载波相位残差异常。
- 如果 `resp` 很大，说明伪距残差异常，可能受多路径影响。
- 如果 `snr` 很低，说明信号质量差。
- 如果 `slip` 非零，说明发生周跳，载波连续性被破坏。
- 如果 `rejc` 增加，说明该观测经常被判为异常。
- 如果 `fix` 从 `3` 退回 `1`，说明固定保持状态被破坏。

这个文件对 GLINS 载波相位问题也很有用。因为 GLINS 中 carrier factor 的稳定性高度依赖卫星有效性、载波残差、周跳检测和模糊度连续性。

## 3. `rtklib_events.pos`

`rtklib_events.pos` 是 RTKLIB 的事件时间标记输出文件。

它不是普通轨迹文件，而是用于记录 GNSS 接收机外部事件触发时刻的定位结果，例如：

- 相机曝光触发
- LiDAR 扫描触发
- PPS/time mark
- 其他外部硬件事件

当前文件只有文件头，没有数据行，说明本次输入的 RINEX 观测中没有有效的 external event / time mark 事件。

所以当前实验中：

```text
rtklib_events.pos 只表示 RTKLIB 准备好了事件输出文件
但没有实际事件数据可写
```

普通 RTK 轨迹不要看这个文件，应看：

```text
rtklib.pos
```

## 4. `rtklib_fgo.pos`

`rtklib_fgo.pos` 是 GLINS 融合优化后的 GNSS/INS/LiDAR 输出结果。

它不是 RTKLIB 纯 GNSS 解，而是因子图优化后的状态写到 ECEF 坐标系下，便于和 RTKLIB 结果对比。

生成代码主要在：

```text
glins/include/imu/imuPreintegration.cpp
IMUPreintegration::writeGPSfile2()
```

文件中的数据来自：

```text
因子图优化后的 IMU/LiDAR/GNSS 融合位姿
通过 gps2imu 外参转换到 GNSS 天线位置
再由 ENU/local 坐标转换到 ECEF
```

当前数据行格式实际为：

```text
week sow x_ecef y_ecef z_ecef state roll pitch yaw
std_rot_x std_rot_y std_rot_z std_pos_x std_pos_y std_pos_z
bias_acc_x bias_acc_y bias_acc_z bias_gyr_x bias_gyr_y bias_gyr_z
corner_num surf_num
```

示例：

```text
2299 111975.000 -2267128.6563 5009529.1573 3221136.8068 6 0.522 2.564 48.346 ...
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `week/sow` | GPST 周数和周内秒 |
| `x_ecef/y_ecef/z_ecef` | 融合优化后的 GNSS 天线 ECEF 坐标 |
| `state` | 输出状态标志，当前代码里经常写 `6`，若启用 ambiguity resolve 可能表示固定状态 |
| `roll/pitch/yaw` | 优化后的姿态角，单位度 |
| `std_rot_x/std_rot_y/std_rot_z` | 姿态协方差对角线开方，近似标准差 |
| `std_pos_x/std_pos_y/std_pos_z` | 位置协方差对角线开方，近似标准差 |
| `bias_acc_x/y/z` | IMU 加速度计 bias |
| `bias_gyr_x/y/z` | IMU 陀螺仪 bias |
| `corner_num` | 当前帧角点特征数量 |
| `surf_num` | 当前帧平面特征数量 |

注意：

- `rtklib.pos` 的 `Q` 是 RTKLIB 解状态。
- `rtklib_fgo.pos` 的 `state` 是 GLINS 自己写出的状态标志，不能直接和 RTKLIB 的 `Q` 完全等同。
- `rtklib_fgo.pos` 的时间范围可能比 `rtklib.pos` 短，因为 GLINS 只有在 LiDAR/IMU/GNSS 融合同步并完成优化后才写结果。

## 5. `total.pos`

`total.pos` 是 GLINS 保存的本地轨迹文件。

生成代码主要在：

```text
glins/include/mapping/mapOptmizationGps.cpp
mapOptimization::savePath()
```

数据格式：

```text
ros_time x y z qx qy qz qw
```

示例：

```text
1706511947.10006809 -0.10549425 0.34102023 -0.09827174 -0.00447515 0.02143096 0.41016792 0.91174720
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `ros_time` | ROS 时间戳，单位秒 |
| `x/y/z` | GLINS 局部/map 坐标系下的位置，单位米 |
| `qx/qy/qz/qw` | 四元数姿态 |

注意：

- `total.pos` 不是 ECEF 坐标。
- 它适合看 LiDAR/SLAM 局部轨迹。
- 如果要和 `rtklib.pos` 比较，需要统一坐标系，不能直接把 `total.pos` 和 `rtklib.pos` 的 xyz 相减。

## 6. `MAP/globalmap_lidar_feature.pcd`

`MAP/globalmap_lidar_feature.pcd` 是保存下来的全局点云地图。

生成代码主要在：

```text
glins/include/mapping/mapOptmizationGps.cpp
mapOptimization::saveMapService()
```

当前代码中会遍历关键帧点云，将每一帧点云根据优化后的关键帧位姿变换到全局坐标系，然后合成全局地图并保存为 PCD。

用途：

- 用 PCL/CloudCompare/RViz 查看建图效果。
- 检查轨迹是否漂移导致地图重影。
- 检查 LiDAR 特征匹配与闭环/约束效果。

注意：

- 这是点云地图，不是轨迹文本文件。
- 坐标系通常是 GLINS 的局部/map 坐标系，不是 ECEF。

## 7. `rtk_fgo_compare.png`

`rtk_fgo_compare.png` 是对比图，由脚本生成：

```text
glins/scripts/compare_rtk_fgo.py
```

它通常对比：

```text
rtklib.pos      纯 GNSS RTK
rtklib_fgo.pos  GLINS FGO 融合结果
```

图中一般包含：

- ENU 平面轨迹
- East 分量随时间变化
- North 分量随时间变化
- Up 分量随时间变化
- 匹配历元 RMS 误差统计

注意：

- 这个图是分析对比图，不是程序原始输出。
- 如果 `rtklib.pos` 比 `rtklib_fgo.pos` 多一截，通常是因为 RTKLIB 解算了完整 GNSS 时间段，而 GLINS 只在 LiDAR/IMU/GNSS 都同步并完成优化的时间段内输出。

## 8. 文件之间的关系

可以按下面这张关系理解：

```text
RINEX GNSS 原始观测
    ├── RTKLIB 解算
    │     ├── rtklib.pos          纯 GNSS RTK 轨迹
    │     ├── rtklib.pos.stat     RTKLIB 内部状态诊断
    │     └── rtklib_events.pos   外部事件 time mark 结果
    │
    └── /gnss_raw ROS 消息
          ↓
LiDAR + IMU + GNSS 原始观测
          ↓
GLINS 因子图优化
    ├── rtklib_fgo.pos            融合优化后的 ECEF 轨迹
    ├── total.pos                 GLINS 局部轨迹
    └── MAP/*.pcd                 全局点云地图
```

## 9. 分析时应该看哪个文件

| 目标 | 应看文件 |
| --- | --- |
| 看纯 GNSS RTK 结果 | `rtklib.pos` |
| 看 GLINS 融合结果 | `rtklib_fgo.pos` |
| 看 RTK 为什么 fix/float 切换 | `rtklib.pos.stat` |
| 看每颗卫星残差、SNR、周跳 | `rtklib.pos.stat` 的 `$SAT` |
| 看局部轨迹和姿态 | `total.pos` |
| 看点云地图质量 | `MAP/globalmap_lidar_feature.pcd` |
| 看 RTK 和 FGO 的 ENU 对比 | `rtk_fgo_compare.png` |
| 看外部事件触发定位 | `rtklib_events.pos` |

## 10. 和载波相位问题相关的重点

如果要分析 GLINS 的 carrier phase factor 是否稳定，建议重点检查：

```text
rtklib.pos.stat 中的 $SAT 行
```

重点字段：

| 字段 | 说明 |
| --- | --- |
| `resc` | 载波相位残差，异常变大时要注意 |
| `resp` | 伪距残差，能反映多路径/粗差 |
| `snr` | 信号强度，低 SNR 更容易出问题 |
| `fix` | 模糊度状态，`1=float`，`2=fixed`，`3=hold` |
| `slip` | 周跳标志，非零表示载波连续性被破坏 |
| `lock` | 锁定计数，越连续通常越可靠 |
| `rejc` | 被拒绝次数，增多说明该卫星观测不稳定 |
| `bias_var` | 模糊度/bias 方差，过大说明不确定性强 |

如果某个时间段 GLINS 载波约束异常，可以先在 `.stat` 中定位同一 GPST 秒，看是否同时出现：

```text
resc 变大
slip 非零
snr 降低
fix 从 hold/fixed 退回 float
rejc 增加
```

这些现象都可能导致 GLINS 里 `N(key)` 模糊度状态约束变弱，甚至引发因子图欠约束或数值不稳定。
