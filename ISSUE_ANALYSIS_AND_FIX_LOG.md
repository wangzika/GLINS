# GLINS 运行问题分析与修复记录

本文档总结本次将 GLINS 在 `macOS + Docker + 20240129 数据` 上从“基础模式可运行”推进到“复杂模式可运行”的问题分析、修复过程，以及每一轮具体修改了哪些文件。

## 1. 最终结果

当前这套工程已经形成了几种明确的运行入口：

- `./docker/macos.sh bag`
  - 基础稳定模式
- `./docker/macos.sh bag-advanced`
  - 开启 GNSS 原始观测，不加载波
- `./docker/macos.sh bag-carrier-float`
  - 开启载波，关闭整数固定
- `./docker/macos.sh bag-carrier-no-nhc`
  - 开启载波和整数固定，关闭 NHC
- `./docker/macos.sh bag-full`
  - 全量模式

按当前最新验证结果，复杂模式已经可以进入可运行状态，载波相位链路不再像最初那样频繁在 `n...` 状态上直接崩溃。

## 2. 问题背景

本次问题不是单点错误，而是一个逐层暴露的链式问题，主要分成两大阶段：

### 2.1 先把“能跑起来”打通

早期遇到的问题包括：

- Docker / macOS 环境适配
- bag 时间窗不匹配，导致所有消息被过滤
- `PatchWork++` 参数未正确加载
- `params_HDL_32.yaml` 与 `20240129` 数据不匹配
- 输出目录和 RTKLIB 配置路径仍然指向作者原始环境

这一阶段的目标是：

- 让基础链路可以稳定运行
- 让 GNSS / LIO / 地图保存都先通一遍

### 2.2 再把“复杂模式”打通

基础模式跑通之后，问题集中转移到了：

- `useObs=true`
- `useCarrier=true`
- `useNHC=true`
- `useAmbFix=true`

也就是原始观测 + 载波 + NHC + 整数固定这一条完整 GNSS 因子图链。

最典型的错误是：

- `gtsam::IndeterminantLinearSystemException`
- 变量名落在：
  - `x179`
  - `n109`
  - `n939`
  - `n1099`

其中：

- `x...` 表示位姿状态
- `n...` 表示 ambiguity 状态 `N(key)`

这说明复杂模式的核心不稳定点，在载波模糊度状态链上。

## 3. 最终问题分析结论

当前这轮排查最终收敛到下面几个结论：

### 3.1 不是单独由 NHC 引起

通过 `bag-carrier-no-nhc` 的对照实验可以排除：

- 不是“只要开 NHC 就一定崩”

因为在更早阶段，关闭 NHC 后仍然出现过 `n...` 相关病态。

### 3.2 不是只有整数固定才触发

通过 `bag-carrier-float` 的对照实验可以排除：

- 不是“只有 `GNSSFixConstraint` 加入后才崩”

因为关闭 `useAmbFix` 后，早期依然可能在 `n...` 状态上报错。

### 3.3 根因在载波 ambiguity 子图的约束充分性

最关键的问题在于：

- 当前历元的部分相位观测可能被剔除
- 当前 `N(key)` 仍然被构造并接入因子图
- `N(key)` 与 `N(lastkey)` 的时序连接可能依赖已经边缘化或不可用的旧状态
- 即使没有明显 outlier，某些“被接纳”的 ambiguity 块仍然可能数值上过弱

这些情况叠加后，会导致：

- 当前 ambiguity 子图秩不足
- 最终触发 `IndeterminantLinearSystemException`

## 4. 修复过程

下面按照时间顺序记录每一轮修复。

### 第 1 轮：打通基础运行链路

这一轮目标是先把最基本的离线运行链路跑通。

主要处理内容：

- 修正 `run_bag.launch` 默认参数与时间窗
- 切换到 `params_20240129.yaml`
- 统一 RTKLIB 配置路径和输出路径
- 确保容器内 `/data`、`/output` 等路径一致
- 加入参数覆盖机制，给后续多模式实验打基础

修改文件：

- [glins/launch/run_bag.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag.launch:1)
  - 增加 `params` / `use_params_override` / `params_override`
  - 修正默认 bag 参数、时间窗、输出路径
- [glins/include/utility.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/utility.h:240)
  - 补充运行时读取的参数项

这一轮完成后，基础模式可以稳定运行。

### 第 2 轮：增加复杂模式入口，做对照实验

这一轮目标是把“复杂模式”拆开，不再只盯着一个 `full` 模式硬跑。

新增了 4 套模式：

- `advanced`
- `full`
- `carrier_float`
- `carrier_no_nhc`

对应新增文件：

- [glins/config/params_20240129_advanced.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_advanced.yaml:1)
- [glins/config/params_20240129_full.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_full.yaml:1)
- [glins/config/params_20240129_carrier_float.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_carrier_float.yaml:1)
- [glins/config/params_20240129_carrier_no_nhc.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_carrier_no_nhc.yaml:1)
- [glins/launch/run_bag_advanced.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_advanced.launch:1)
- [glins/launch/run_bag_full.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_full.launch:1)
- [glins/launch/run_bag_carrier_float.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_carrier_float.launch:1)
- [glins/launch/run_bag_carrier_no_nhc.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_carrier_no_nhc.launch:1)

配套修改：

- [docker/macos.sh](/Users/wangzhibo/Desktop/博士研究/GLINS/docker/macos.sh:1)
  - 新增：
    - `bag-advanced`
    - `bag-full`
    - `bag-carrier-float`
    - `bag-carrier-no-nhc`

这一步的价值是：

- 把问题拆成多个实验入口
- 明确比较：
  - 是否是 NHC 导致
  - 是否是整数固定导致

### 第 3 轮：新增 `useAmbFix`，把“是否整数固定”从代码里分离出来

为了能单独验证“浮点 ambiguity”和“整数固定”的差异，新增了参数：

- `glins/useAmbFix`

修改文件：

- [glins/include/utility.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/utility.h:240)
  - 新增 `useAmbFix` 参数读取
- [glins/include/imu/imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:860)
  - 将 `ambiguityResolve(...)` 包到 `useAmbFix` 判断里

效果：

- `carrier_float` 能做到“用载波，但不做整数固定”
- 这一步直接帮助排除了“是不是只有整数固定才崩”

### 第 4 轮：修复旧 ambiguity 状态不可用时的时序连接问题

早期一类核心问题是：

- 当前 `N(key)` 还在尝试与 `N(lastkey)` 做连续性连接
- 但 `N(lastkey)` 可能已经不在当前优化器里

修改文件：

- [glins/include/imu/imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:273)
  - 增加 `allowCarrierTemporalLink`
  - 若 `optimizer` 中不存在 `N(lastGNSSepoch)`，则关闭当前时序连接
- [glins/include/gnss/gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:338)
  - `addDDCpFactorENU(...)` 增加 `allow_temporal_link`
  - 当不可连接时：
    - 清空 `last_ar_index`
    - 不再强行加入 `GNSSAmbConstraintCompress(...)`
    - 回退到当前 epoch 的先验初始化方式

这一步避免了：

- ambiguity 因子图继续引用无效旧状态

### 第 5 轮：增加相位观测 outlier 检查与整帧跳过策略

进一步实验表明，即使不做整数固定，当前 epoch 的相位观测仍可能因为残差过大把 `N(key)` 拖成病态。

修改文件：

- [glins/include/gnss/gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:489)

本轮增加了：

- `carrier_outlier_count`
- `carrier_factor_count`
- 局部 `carrierGraphFactors`

新策略：

- 若当前 epoch 出现 `phase residual out of range`
- 则直接：
  - `skip carrier factors at key ...`
  - 整帧跳过该 carrier epoch
  - 回退成“仅伪距 / 多普勒”的更新

这一步避免了：

- 明显坏的载波观测直接进入主图

### 第 6 轮：修复“carrier 被跳过后仍引用不存在的 `N(key)`”的问题

在第 5 轮之后出现过一类新的 bug：

- 当前 epoch 已经决定跳过 carrier
- 但仍有因子引用了 `N(key)`
- 最终报：
  - `ValuesKeyDoesNotExist`

根因是：

- `GNSSPhasePriorConstraintCompress(N(key), ...)`
  仍然直接写进了总图

修改文件：

- [glins/include/gnss/gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:521)
- [glins/include/gnss/gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:544)

修复方式：

- 将这些 prior 也统一先写入 `carrierGraphFactors`
- 只有当前 epoch 最终确认接纳 carrier 时，才整体加入主图

### 第 7 轮：对 ambiguity 块增加稳定化先验

在跳过 outlier epoch 之后，仍观察到一种更隐蔽的问题：

- 当前 epoch 没有被判定为坏观测
- 但 `N(key)` 整块仍然数值上偏弱

修改文件：

- [glins/include/gnss/gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:604)

本轮增加了：

- `PriorFactor<Vector>(N(key), compress_ar, ...)`

目的：

- 给整块 ambiguity 一个数值稳定化先验
- 防止 `n...` 变量在“看起来可用、实际上秩仍不足”的 epoch 上继续炸掉

### 第 8 轮：提高对“可接受 carrier epoch”的筛选强度

最新一轮处理不再只判断“有没有 outlier”，还判断：

- 当前 ambiguity 块是否太小
- 当前真正加入的有效 carrier 因子是否太少

修改文件：

- [glins/include/gnss/gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:584)

新增条件：

- `ncp < 2`
- 或 `carrier_factor_count < 2`

满足时直接跳过当前 carrier epoch，并记录 warning。

同时对整块 ambiguity 的稳定化先验量级做了调整，使其与现有 phase prior 更一致，而不是使用过弱正则。

这一步的目标是：

- 不再只防“坏观测”
- 也防“当前约束结构太弱”的伪健康 epoch

## 5. 当前主要修改文件总表

这次从基础模式到复杂模式打通，主要涉及这些文件：

### 5.1 运行入口与参数

- [docker/macos.sh](/Users/wangzhibo/Desktop/博士研究/GLINS/docker/macos.sh:1)
- [glins/launch/run_bag.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag.launch:1)
- [glins/launch/run_bag_advanced.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_advanced.launch:1)
- [glins/launch/run_bag_full.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_full.launch:1)
- [glins/launch/run_bag_carrier_float.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_carrier_float.launch:1)
- [glins/launch/run_bag_carrier_no_nhc.launch](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/launch/run_bag_carrier_no_nhc.launch:1)
- [glins/config/params_20240129_advanced.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_advanced.yaml:1)
- [glins/config/params_20240129_full.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_full.yaml:1)
- [glins/config/params_20240129_carrier_float.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_carrier_float.yaml:1)
- [glins/config/params_20240129_carrier_no_nhc.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129_carrier_no_nhc.yaml:1)

### 5.2 参数读取与主流程调度

- [glins/include/utility.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/utility.h:240)
- [glins/include/imu/imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:260)

### 5.3 载波模糊度建图与修复核心

- [glins/include/gnss/gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:338)

### 5.4 过程文档

- [CARRIER_PHASE_ISSUE_ANALYSIS.md](/Users/wangzhibo/Desktop/博士研究/GLINS/CARRIER_PHASE_ISSUE_ANALYSIS.md:1)
- [MACOS_IMPLEMENTATION_SUMMARY.md](/Users/wangzhibo/Desktop/博士研究/GLINS/MACOS_IMPLEMENTATION_SUMMARY.md:1)

## 6. 当前建议的使用方式

如果只是稳定跑通：

- 用 `bag`

如果需要更完整的 GNSS 原始观测：

- 先用 `bag-advanced`

如果需要继续看载波链：

- 先试 `bag-carrier-no-nhc`
- 再试 `bag-carrier-float`
- 最后再试 `bag-full`

这样最容易判断：

- 是原始观测层不稳
- 是 carrier ambiguity 链不稳
- 还是 `carrier + NHC` 的组合耦合不稳

## 7. 总结

这次问题的核心不是一个简单的配置错误，而是一个典型的“多源融合系统在复杂模式下逐层暴露数值问题”的过程。

最终收获主要有三点：

- 基础链路已经被整理成稳定可运行的 macOS + Docker 方案
- 复杂模式被拆分成多个可控实验入口
- 载波相位 ambiguity 子图的主要不稳定点已经被定位，并通过多轮保守修复显著改善

如果后面继续做更深入优化，下一步最值得继续打磨的地方仍然是：

- `N(key)` 的接入判据
- `GNSSAmbConstraintCompress` 的时序连接策略
- `carrier + NHC` 组合下的数值稳定性
