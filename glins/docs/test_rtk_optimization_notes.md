# test_rtk 优化与问题排查记录

本文记录 `glins_test_rtk` 的运行方式、调试过程中遇到的问题、定位依据、已经做过的修复，以及后续建议。当前工作空间路径为：

```bash
/home/ys/glins_ws
```

## 1. 目标程序

源码：

```text
src/GLINS/glins/src/test_rtk.cpp
```

编译目标：

```text
glins_test_rtk
```

可执行文件：

```text
devel/lib/glins/glins_test_rtk
```

`test_rtk.cpp` 的主流程是：

```cpp
ros::init(argc, argv, "gnss_estimator");
gnssProcessor processor;
gnssEstimator estimator;

gtime_t ts = gpst2time(2299, 111965);
gtime_t te = gpst2time(2299, 113000);

processor.decode(ts, te);
estimator.solveOptimization();
ros::spin();
```

也就是说它先用 RTKLIB 后处理生成 GNSS 结果，再进入 FGO/GTSAM 优化。

## 2. 运行命令

```bash
cd /home/ys/glins_ws
source /opt/ros/noetic/setup.zsh
source devel/setup.zsh
rosrun glins glins_test_rtk
```

重新编译：

```bash
cd /home/ys/glins_ws
source /opt/ros/noetic/setup.zsh
catkin_make --pkg glins
source devel/setup.zsh
```

## 3. 配置路径修复

最初 `20240129.conf` 使用了容器式路径：

```text
inpstr1-path       =/data/gnss/rover.obs
inpstr2-path       =/data/gnss/base.obs
inpstr3-path       =/data/gnss/BRDM00DLR_S_20240290000_01D_MN.rnx
outstr1-path       =/output/rtklib.pos
```

本机实际数据在：

```text
/home/ys/glins_ws/src/full_data/gnss/
```

已改为：

```text
inpstr1-path       =/home/ys/glins_ws/src/full_data/gnss/rover.obs
inpstr2-path       =/home/ys/glins_ws/src/full_data/gnss/base.obs
inpstr3-path       =/home/ys/glins_ws/src/full_data/gnss/BRDM00DLR_S_20240290000_01D_MN.rnx
outstr1-path       =/home/ys/glins_ws/output/rtklib.pos
```

同时 `utility.h` 中 `ParamServer` 的默认路径也从：

```text
/catkin_ws/src/glins/config/conf/20240129.conf
/output/rtklib_fgo.pos
```

改为：

```text
/home/ys/glins_ws/src/GLINS/glins/config/conf/20240129.conf
/home/ys/glins_ws/output/rtklib_fgo.pos
```

否则直接 `rosrun glins glins_test_rtk` 时，如果没有提前 `rosparam set /glins/rtklibConfigPath ...`，程序会在打印 `gnss Estimator Started` 后因 `loadopts()` 失败直接退出。

## 4. 结果文件

RTKLIB 原始后处理结果：

```text
output/rtklib.pos
```

FGO 优化结果：

```text
output/rtklib_fgo.pos
```

注意：`rtklib_fgo.pos` 为了复用 RTKLIB `.pos` 风格，表头仍写作 `ns`，但当前代码实际写入的是：

```cpp
nv = npr + nb + ndop;
```

也就是 FGO 图优化中的约束/残差数量，不是真正的卫星数。对应代码：

```cpp
logResult(fgoPath, sol_pos, gnss_info.gpsWeek, gnss_info.weekSec, state, nv);
```

因此：

```text
rtklib.pos     第 7 列 ns = RTKLIB 使用卫星数
rtklib_fgo.pos 第 7 列 ns = nv = 伪距因子数 + 载波相位因子数 + 多普勒因子数
```

两者不能直接比较。

## 5. 遇到的问题一：启动后直接退出

现象：

```text
[INFO] ----> gnss Estimator Started.
```

随后程序直接回到 shell。

原因：

`ParamServer` 默认读取：

```text
/catkin_ws/src/glins/config/conf/20240129.conf
```

该路径在本机不存在，`gnssProcessor` 构造函数中：

```cpp
if (!loadopts(rtklibConfigPath.c_str(), sysopts) || !loadopts(rtklibConfigPath.c_str(), rcvopts))
{
    exit(1);
}
```

因此程序直接退出。

解决：

修改 `utility.h` 中默认路径，或运行前手动设置参数：

```bash
rosparam set /glins/rtklibConfigPath /home/ys/glins_ws/src/GLINS/glins/config/conf/20240129.conf
rosparam set /glins/fgoPath /home/ys/glins_ws/output/rtklib_fgo.pos
```

当前代码已使用本机默认路径，不需要每次手动设置。

## 6. 遇到的问题二：epoch 791 附近 GTSAM 欠约束崩溃

现象：

```text
epoch 791 add factor nv:11 npr:4 nb:7 ndop:0 ncp:7
position variance too large: 34.4253 ...
terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'
what():
Indeterminant linear system detected while working near variable
7926335344172073751 (Symbol: n791).
```

含义：

`n791` 是 GTSAM shorthand 中的 `N(791)`，对应第 791 个 FGO 历元的 ambiguity 状态，不是位置状态 `X(791)`。

GTSAM 报 `IndeterminantLinearSystemException`，说明该变量附近的线性系统欠约束，矩阵不可逆。

### 6.1 定位证据

从 `output/rtklib.pos` 可见，GPST 112756 秒附近 RTKLIB 自身已经退化：

```text
112755 Q=1 ns=13 ratio=61.8
112756 Q=2 ns=7  ratio=0.0
112757 Q=2 ns=10 ratio=0.0
112758 Q=2 ns=10 ratio=0.0
```

即 RTKLIB 从 fixed 解变为 float 解，卫星数下降，ratio 变为 0。

FGO 输出在同一位置也出现异常跳变：

```text
112755 ... Q=1
112756 ... Q=6
```

且位置有明显跳变。

因此崩溃不是单纯代码路径问题，而是该时间段 GNSS 观测质量退化后，FGO 仍继续把欠约束的 ambiguity 变量插入图中。

### 6.2 代码原因

`solveOptimization()` 中每个历元都会执行：

```cpp
initialEstimate.insert(X(epoch), pos);
nb = addDDCpFactor();
npr = addDDPsrFactor();
...
updateAndMarginalize(graph, initialEstimate, {}, optimizer);
```

`addDDCpFactor()` 会构建压缩 ambiguity 状态：

```cpp
initialEstimate.insert(N(epoch), compress_ar);
```

后续历元中 `N(epoch)` 主要依赖两类约束：

```cpp
GNSSDDCpFactorCompress(X(epoch), N(epoch), ...)
GNSSAmbConstraintCompress(N(epoch), N(epoch - 1), ...)
```

当卫星数下降、周跳、失锁、参考星变化或 `last_ar_index` 对不上时，一部分 `N(epoch)` 分量没有足够相位约束或连续约束，就会使图不满秩。

该处日志显示：

```text
npr=4
nb=7
ndop=0
ncp=7
```

说明伪距约束少、载波约束少、没有多普勒约束辅助，位置协方差也已经过大：

```text
position variance too large
```

### 6.3 修复思路

采用坏历元跳过策略，避免把弱约束历元的 `N(epoch)` 放进 GTSAM。

当前门控条件：

```cpp
if (!initializing_epoch && (npr < 5 || fgo.num_factor[1] < 8 || gnss_info.stat != SOLQ_FIX))
{
    ROS_WARN("skip weak GNSS epoch ...");
    graph.resize(0);
    initialEstimate.clear();
    last_ar_index.clear();
    fgo.rtk.nfix = 0;
    gnss_queue.pop_front();
    gnss_mutex.unlock();
    continue;
}
```

含义：

```text
npr < 5                伪距约束不足
fgo.num_factor[1] < 8  压缩 ambiguity 维度/载波相关约束不足
gnss_info.stat != 1    RTKLIB 当前历元不是 fixed 解
```

跳过时清空 `last_ar_index`，是为了断开坏段前后的 ambiguity 连续性，后续恢复 fixed 后重新初始化 ambiguity，避免跨坏段错误连续。

## 7. 遇到的问题三：epoch 10 段错误

现象：

```text
[WARN] skip weak GNSS epoch ...
...
epoch 10 ...
segmentation fault (core dumped)
```

原因：

原代码在很短窗口内做 iSAM2 边缘化：

```cpp
if (epoch >= 10)
{
    marginalKeys.insert(X(epoch - 10));
    if (optimizer.valueExists(N(epoch - 10)))
    {
        marginalKeys.insert(N(epoch - 10));
    }
    updateAndMarginalize({}, {}, marginalKeys, optimizer);
}
```

跳过若干 float 历元后，早期图结构不稳定，`marginalizeLeaves()` 要求被边缘化变量必须是 Bayes tree 叶节点；不满足时容易崩溃。

修复：

改为使用类中已有窗口参数：

```cpp
int windows_size = 1000;
```

边缘化逻辑改为：

```cpp
if (epoch >= windows_size)
{
    marginalKeys.insert(X(epoch - windows_size));
    if (optimizer.valueExists(N(epoch - windows_size)))
    {
        marginalKeys.insert(N(epoch - windows_size));
    }
    updateAndMarginalize({}, {}, marginalKeys, optimizer);
}
```

这样不会在 `epoch 10` 这么早触发危险边缘化。

## 8. ratio 的含义

`ratio` 是 ambiguity fixing 的整数解检验值，来自 LAMBDA 搜索中次优整数解与最优整数解残差的比值：

```text
ratio = s1 / s0
```

其中：

```text
s0 = 最优整数候选残差
s1 = 次优整数候选残差
```

直观理解：

```text
ratio 越大，最优整数解越明显优于次优解，fix 越可信
ratio 接近 1，说明多个整数候选差不多，不应 fix
```

配置中：

```text
pos2-arthres = 3
```

通常表示：

```text
ratio >= 3  可以 fix
ratio < 3   不 fix
```

但 `rtklib_fgo.pos` 中的 ratio 来自 FGO 内部 `fgo.rtk.sol.ratio`，不一定与 `rtklib.pos` 中 RTKLIB 每秒输出的 ratio 完全同源。

## 9. 为什么 FGO 后 fixed 数量变多

`rtklib.pos` 的 `Q` 是 RTKLIB 原始后处理状态。

`rtklib_fgo.pos` 的 `Q` 是 FGO 代码中自己赋值：

```cpp
state = 6;
...
if (ambiguity fix success) state = 1;
else if (nb == -1) state = 4;
else if (nb == 0) state = 3;
```

因此 FGO 的 `Q=1` 表示 FGO 内部 ambiguity fixing 成功，不是 RTKLIB 原始 `Q=1` 的直接复制。

FGO 可能出现更多 fixed，原因是：

```text
1. FGO 使用多历元图优化，不是单历元独立解。
2. ambiguity 有跨历元连续约束。
3. 位置和 ambiguity 的协方差来自整张因子图。
4. 历史 fixed 信息会影响后续历元。
```

这可以提升连续性，但也可能在坏数据段造成过度乐观。因此需要坏历元门控，避免将退化观测强行纳入图。

## 10. 可视化脚本

新增脚本：

```text
src/GLINS/glins/scripts/plot_rtk_pose.py
```

默认读取：

```text
output/rtklib.pos
output/rtklib_fgo.pos
```

输出：

```text
output/rtk_pose_plot.png
```

运行：

```bash
cd /home/ys/glins_ws
src/GLINS/glins/scripts/plot_rtk_pose.py
```

图中包含：

```text
1. ENU 平面轨迹
2. East/North/Up 随时间变化
3. 独立 Quality 子图：Q 状态与 ns/nv
```

注意：

```text
rtklib 的 ns 是卫星数
rtklib_fgo 的 ns 当前实际是 nv，即约束数
```

后续建议把 FGO 输出文件表头或脚本标签改为 `nv`，避免误解。

## 11. 当前修改摘要

已修改：

```text
src/GLINS/glins/config/conf/20240129.conf
src/GLINS/glins/include/utility.h
src/GLINS/glins/include/gnss/gnssEstimator.h
src/GLINS/glins/scripts/plot_rtk_pose.py
```

主要行为变化：

```text
1. 默认路径适配 /home/ys/glins_ws。
2. 退化 GNSS 历元不再插入 FGO 图。
3. 坏段会断开 ambiguity 连续性。
4. 早期 iSAM2 边缘化从 epoch >= 10 改为 epoch >= windows_size。
5. GTSAM update 异常会被捕获，避免直接 core dump。
6. 增加 RTK/FGO 轨迹可视化脚本。
```

## 12. 后续建议

### 12.1 区分 ns 和 nv

建议修改 `logResult()`，不要把 `nv` 写到 `ns` 列，或者修改表头为：

```text
nv = number of graph residuals/factors
```

如果需要真实卫星数，可统计 `gnss_info.SD_Infos` 中 unique satellite id。

### 12.2 门控参数配置化

当前门控写死：

```cpp
npr < 5
ncp < 8
stat != SOLQ_FIX
```

建议后续改成 ROS 参数：

```text
glins/minPseudorangeFactors
glins/minCarrierPhaseFactors
glins/requireFixedForFGO
```

这样不同数据集不用重新编译。

### 12.3 更稳健的边缘化

当前修复是推迟边缘化，避免早期崩溃。更完整的做法是：

```text
1. 检查待边缘化变量是否为 Bayes tree leaf。
2. 如果不是 leaf，跳过或先重排序。
3. 对被跳过的坏历元做明确状态管理。
```

### 12.4 多普勒约束

日志中 `ndop=0`，说明当前 FGO 没有启用多普勒约束。若数据质量较差，可考虑恢复 `addSDDopFactor()` 或加入速度/运动约束，提高弱观测时段的可观性。

