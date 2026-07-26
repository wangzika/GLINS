# GLINS 模糊度欠约束与后端锁死优化说明

## 1. 问题结论

当前故障由两个层次的问题叠加产生：

1. 双差载波只观测模糊度之差。若状态中保存每颗卫星的单差模糊度，则每个“星座 × 频率”都存在一个公共自由方向，需要显式基准。
2. 代码每个 GNSS 历元创建一个新的 `N(key)`，再以极强的连续因子连接前后历元；滑窗反复边缘化后，信息尺度差异会放大数值病态。
3. `addGPSFactor()` 在 `ISAM2::update()` 成功前就修改 `lastGNSSepoch` 和 `last_ar_index`。一旦更新抛异常，旧代码只清空外部缓存并返回，不能撤销 iSAM2 和元数据的部分更新，最终出现重复 `b220`、缺失 `n220` 和 key 永久不推进。

因此，论文的联合因子图方向可以成立；当前直接崩溃主要是状态可观性、权重尺度和异常事务一致性没有同时闭合。

## 2. 本次代码优化

### 2.1 为双差公共模态增加显式 datum

每个星座、每个频率的双差观测块选出的参考星，对应一个 `GNSSPhasePriorConstraintCompress`：

```text
N(reference, frequency) ≈ RTK float ambiguity
```

默认标准差为 `ambiguityDatumSigma = 1.0 cycle`。它不替代双差观测，只固定双差无法看到的公共平移方向。

### 2.2 调整模糊度信息尺度

旧参数：

```text
新模糊度先验 σ = 30
历元连续约束 σ = 0.001
信息强度比约为 9×10^8
载波因子内部预白化后，外层又使用 σ = 0.1
```

新默认参数：

```yaml
ambiguityPriorSigma: 3.0
ambiguityDatumSigma: 1.0
ambiguityContinuitySigma: 0.05
carrierOuterSigma: 1.0
```

载波因子已经根据观测协方差在内部预白化，所以外层保持单位尺度。连续约束仍然表达“无周跳期间模糊度近似不变”，但不再作为数值上的刚性等式。

### 2.3 延迟提交 GNSS 元数据

`addGPSFactor()` 不再提前执行：

```cpp
lastGNSSepoch = key;
```

只有当 iSAM2 更新成功后才提交新的 `lastGNSSepoch`。`last_ar_index`、GNSS key 队列、最近关键帧和前一帧位姿在更新前均保留已提交快照。

### 2.4 iSAM2 更新事务回滚

更新前复制当前 iSAM2 状态。若 `update()` 抛异常：

1. 恢复更新前的 iSAM2；
2. 恢复 `lastGNSSepoch` 和 `last_ar_index`；
3. 恢复 GNSS key 队列和最近关键帧信息；
4. 清空本次未提交的 factors/values；
5. 保留已经积分的 IMU 量，使下一帧可以继续用同一个 key 构建更长时间段的 IMU 因子。

这样异常不会再把一次数值问题升级成重复插入或缺失变量。

### 2.5 边缘化失败回滚

滑窗边缘化也使用 iSAM2 和 GNSS key 队列快照。若 `marginalizeLeaves()` 失败，恢复边缘化前的完整状态，不清空仍与当前图一致的 `last_ar_index`。

### 2.6 拒绝失真的 LiDAR 增量

完整回放后段还暴露出一个独立问题：局部地图匹配偶尔输出数百米到数万米的跳变，旧代码即使发现 LiDAR 与 IMU 预测不一致，仍会把该相对位姿和旋转先验加入图中，并把未验证的 LiDAR 位姿登记成下一帧参考。极端残差会污染状态，随后表现为位姿变量 `X(key)` 的数值欠约束。

现在先计算 LiDAR 位姿与 IMU 预积分预测之间的 6-DoF 差异。默认平移超过 5 m 或旋转超过 20° 时，不加入该帧 LiDAR 因子；下一帧参考位姿使用优化成功后的融合状态，而不是原始 scan-matching 输出。IMU、GNSS 因子仍可继续推进图。

## 3. 参数解释

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `ambiguityPriorSigma` | 3.0 cycle | 新出现或周跳后模糊度的初始化不确定度 |
| `ambiguityDatumSigma` | 1.0 cycle | 每个双差块公共模态的基准强度 |
| `ambiguityContinuitySigma` | 0.05 cycle | 无周跳前后历元模糊度连续性 |
| `carrierOuterSigma` | 1.0 | 已预白化载波残差的外层单位噪声 |
| `lidarImuGateTranslation` | 5.0 m | LiDAR 与 IMU 预测的最大允许平移差 |
| `lidarImuGateRotationDeg` | 20.0° | LiDAR 与 IMU 预测的最大允许旋转差 |

调参顺序：

1. 若仍有 `IndeterminantLinearSystemException`，先把 `ambiguityDatumSigma` 从 1.0 减小到 0.5。
2. 若轨迹被载波约束拉动过强，增大 `carrierOuterSigma`，例如 1.5 或 2.0。
3. 若连续弧段的模糊度抖动明显，把 `ambiguityContinuitySigma` 从 0.05 减小到 0.02；不建议恢复到 0.001。
4. 若周跳恢复慢，可把 `ambiguityPriorSigma` 从 3.0 增大到 5.0，但应保留 datum。

## 4. 远端验证标准

使用 `/data/zbwang/self_collected` 完整时段验证：

1. 编译无错误；
2. 日志中 `IndeterminantLinearSystemException` 为 0；
3. 日志中 `already exists`、`does not exist` 为 0；
4. key 持续增长，不在 220 附近重复；
5. `rtklib_fgo.pos` 覆盖完整数据时段；
6. 轨迹坐标和协方差均为有限值；
7. 对比独立 RTK 的 fixed/float 状态，FGO 退化发生在可解释的低卫星或周跳区间，而不是后端结构损坏。

## 5. 后续结构性改造

本次优化保留了现有 `N(key)` 接口，以最小改动验证根因。长期建议把模糊度生命周期从帧 key 解耦：

```text
N(satellite, frequency, continuous_arc)
```

无周跳时，多帧载波因子复用同一 ambiguity arc；检测周跳后才结束旧 arc 并创建新状态。参考星变化只改变双差映射，不重新复制物理模糊度。该结构更接近论文图 4，也能减少变量维数和边缘化压力。
