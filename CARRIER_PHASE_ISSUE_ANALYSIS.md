# 载波相位模式问题分析

本文档总结 `bag-advanced`、`bag-full` 以及两组调试实验的现象、定位过程和当前修复思路。

## 1. 现象概述

当前 `20240129` 数据在不同模式下的运行结果如下：

- `run_bag.launch`
  - 可以稳定运行
  - 当前默认配置为：
    - `useObs: false`
    - `useCarrier: false`
    - `useNHC: false`

- `run_bag_advanced.launch`
  - 可以运行完成
  - 当前配置为：
    - `useObs: true`
    - `useCarrier: false`
    - `useNHC: true`

- `run_bag_full.launch`
  - 运行失败
  - 当前配置为：
    - `useObs: true`
    - `useCarrier: true`
    - `useNHC: true`

为了进一步定位问题，又增加了两组调试模式：

- `run_bag_carrier_float.launch`
  - `useCarrier: true`
  - `useNHC: true`
  - `useAmbFix: false`
  - 仍然失败
  - 报错变量为 `n939`

- `run_bag_carrier_no_nhc.launch`
  - `useCarrier: true`
  - `useNHC: false`
  - `useAmbFix: true`
  - 仍然失败
  - 报错变量为 `n109`

## 2. 关键结论

通过上面的对照实验，可以排除两个误判：

### 2.1 不是单独由 NHC 造成的

原因：

- `carrier_no_nhc` 已经关闭 `useNHC`
- 但依然报 `IndeterminantLinearSystemException`

因此，`NHC` 不是根因。

### 2.2 不是只有整数固定才会触发

原因：

- `carrier_float` 已经关闭 `useAmbFix`
- 也就是不再做整数固定约束
- 但依然报 `IndeterminantLinearSystemException`

因此，问题不是“只有 `GNSSFixConstraint` 加入后才崩”，而是更早就已经出现在载波模糊度状态建图阶段。

## 3. 为什么判断问题在 ambiguity 状态 `N(key)` 链上

最关键的线索是异常变量名字：

- `x179`
- `n109`
- `n939`

其中：

- `x...` 表示位姿状态
- `n...` 表示载波模糊度状态 `N(key)`

在两次针对性实验中，异常都直接落在 `n...` 上：

- `carrier_float` 崩在 `n939`
- `carrier_no_nhc` 崩在 `n109`

这说明：

**当前主要问题不在位姿端，而在载波模糊度状态的约束充分性。**

## 4. 对应代码链

载波相位相关逻辑主要在以下位置：

### 4.1 入口

[`glins/include/imu/imuPreintegration.cpp`](glins/include/imu/imuPreintegration.cpp)

在 `IMUPreintegration::addGPSFactor()` 中：

- `useObs=true` 时加入伪距因子
- `useCarrier=true` 时进一步调用 `addDDCpFactorENU()`

### 4.2 载波相位因子构造

[`glins/include/gnss/gnssContainer.h`](glins/include/gnss/gnssContainer.h)

函数：

- `addDDCpFactorENU(...)`

该函数会做几件事：

- 构造当前历元的 ambiguity 状态 `N(key)`
- 加入 `GNSSDDCpFactorCompress_ENU`
- 在条件满足时加入 `GNSSAmbConstraintCompress(N(key), N(lastkey), ...)`
- 在需要时给新 ambiguity 加 `GNSSPhasePriorConstraintCompress`

### 4.3 整数固定

同样在 [`glins/include/gnss/gnssContainer.h`](glins/include/gnss/gnssContainer.h)

函数：

- `ambiguityResolve(...)`

成功时会再加入：

- `GNSSFixConstraint(N(key), ...)`

## 5. 当前推断的根因

结合日志和代码，当前更接近的根因是：

### 5.1 当前 ambiguity 状态在某些 epoch 上没有被充分约束

尤其是这些情形叠加时：

- 某些载波观测因为残差过大被剔除
- 仍然尝试构造当前 `N(key)` 状态
- 当前 `N(key)` 与上一历元 `N(lastkey)` 的连续性约束依赖旧状态
- 但旧的 ambiguity 状态可能已经被边缘化，或不再可用

这会导致：

- 当前 ambiguity 子图不满秩
- 最终触发 `IndeterminantLinearSystemException`

### 5.2 早期报 `x179` 只是次级表现

最开始 `full` 模式崩在 `x179`，更像是 ambiguity 子图问题进一步传播到位姿状态上的表现。

而后续专门实验中直接崩在 `n...`，说明问题源头更靠近 ambiguity 链本身。

## 6. 当前修复思路

先采用“保守修复”：

- 如果上一历元的 ambiguity 状态已经不在优化器中
- 就不再强行把当前 `N(key)` 和旧的 `N(lastkey)` 用连续性约束绑在一起
- 而是回退为对当前 ambiguity 加弱先验

这样做的目的不是立即获得最优精度，而是：

- 先避免 ambiguity 子图因为引用了不可用旧状态而退化
- 让 `useCarrier=true` 模式至少具备继续调试和运行的基础

## 7. 本轮修复目标

本轮代码修改优先实现：

1. 在载波链中检查上一历元 ambiguity 状态是否还存在
2. 若不存在，则清空连续性映射并退回弱先验模式
3. 保留后续继续验证的空间：
   - 观察 `carrier_float`
   - 观察 `carrier_no_nhc`
   - 再判断是否还需要进一步跳过整帧载波因子

## 8. 第二轮补充修复

在继续实验后，发现即使：

- 上一历元 ambiguity 状态仍然存在
- 但当前历元的部分相位观测被残差筛除

仍然可能导致当前 `N(key)` 这一块欠约束，并直接报：

- `Symbol: n939`

因此又补充了一层保护：

- 对每个当前 ambiguity 块 `N(key)` 增加一个非常弱的高斯先验
- 它不用于强行固定 ambiguity
- 只用于避免当前 ambiguity 子图在相位观测变稀疏时完全失去秩

同时增加了日志：

- 当前 epoch 剔除了多少相位观测
- 当前 ambiguity 是否在“只有正则、没有有效载波因子”的情况下被插入

这轮修复的目标是：

- 先把 `n...` 变量上的病态系统压住
- 再继续观察后续是否还会传播到 `x...` 状态

在进一步实验后，发现“弱正则”仍不足以保证当前 ambiguity 块稳定。

因此又增加了更保守的一条策略：

- 只要当前 epoch 出现 `phase residual out of range`
- 就整帧跳过本 epoch 的载波因子
- 让该 epoch 自动退回为“仅伪距/多普勒”更新

这相当于把载波观测从“硬接入”改成“按 epoch 做健康检查后再接入”。

## 9. 第三轮补充修复

继续实验后又出现了两类新现象：

### 9.1 跳过 carrier epoch 后仍引用了不存在的 `N(key)`

现象：

- 日志出现：
  - `skip carrier factors at key ... because ... phase observations were rejected as outliers`
- 随后又报：
  - `ValuesKeyDoesNotExist`
  - `Attempting to at the key "n339", which does not exist in the Values.`

原因：

- 虽然当前 epoch 已经决定跳过载波因子
- 但仍有一条 `GNSSPhasePriorConstraintCompress(N(key), ...)` 直接写进了总图
- 于是因子引用了 `n339`
- 但 `Values` 里没有插入 `N(339)`

修复：

- 将这条 prior 也改为先进入局部 `carrierGraphFactors`
- 只有当前 epoch 最终确认接纳 carrier 时，才统一并入总图

### 9.2 没有 outlier 的“被接纳 epoch”仍可能在 `n...` 上病态

现象：

- 某些 epoch 没有触发 `skip carrier factors...`
- 但仍然在后续优化时报：
  - `IndeterminantLinearSystemException`
  - `Symbol: n1099`

这说明：

- 当前问题已经不只是“坏相位观测没有被正确跳过”
- 而是即便当前 epoch 被判定为“可以接入”
- `N(key)` 这一整块仍可能数值上过弱

当前新增处理：

- 如果当前 ambiguity 块太小，或有效 carrier 因子数太少，则直接跳过：
  - `ncp < 2`
  - 或 `carrier_factor_count < 2`
- 对最终被接纳的 `N(key)` 整块增加全向量稳定化先验
- 该先验的量级与现有 phase prior 保持一致，而不再使用几乎不起作用的超弱正则

这一步的目标是：

- 不再只防“坏观测”
- 还要防“看起来可用、但实际秩仍然不足”的 carrier epoch
## 10. 当前判断总结

一句话总结：

**当前 `full` 模式的不稳定，核心在于载波模糊度状态 `N(key)` 的时序约束链不稳定，而不是单独由 NHC 或整数固定本身引起。**
