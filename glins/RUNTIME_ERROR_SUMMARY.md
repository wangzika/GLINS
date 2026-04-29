# GLINS 运行报错与定位总结

## 1. 文档目的

这份文档专门总结本次 `GLINS` 在运行、调试和修复过程中出现过的主要报错、警告和异常现象，重点回答下面几个问题：

- 运行时都报过哪些错
- 这些错分别是什么意思
- 它们大多出现在什么阶段
- 与当前工程里的哪条链路相关
- 这次排查最终是怎么定位和修复的

如果你后面再次遇到相似问题，可以先对照本文件，再去看更详细的过程文档：

- [ISSUE_ANALYSIS_AND_FIX_LOG.md](/Users/wangzhibo/Desktop/博士研究/GLINS/ISSUE_ANALYSIS_AND_FIX_LOG.md:1)
- [CARRIER_PHASE_ISSUE_ANALYSIS.md](/Users/wangzhibo/Desktop/博士研究/GLINS/CARRIER_PHASE_ISSUE_ANALYSIS.md:1)
- [GTSAM_COMMON_GUIDE.md](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/GTSAM_COMMON_GUIDE.md:1)

---

## 2. 错误总体分类

这次运行里出现过的问题，大致可以分成 4 类：

1. 启动链路和输入数据问题
2. GNSS / 载波观测质量问题
3. 因子图状态引用和滑窗边缘化问题
4. GTSAM 优化阶段的数学病态问题

其中真正最关键、最影响 `full / carrier` 模式稳定性的，是后两类。

---

## 3. 启动链路与输入侧问题

### 3.1 `Unable to open rosbag: ...`

典型位置：

- [test_lio.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_lio.cpp:95)

典型日志：

```text
Unable to open rosbag: /path/to/bag
```

含义：

- rosbag 路径不对
- bag 文件不存在
- 当前容器或运行环境无法访问这个路径

常见原因：

- `bagpath` 参数传错
- 宿主机路径和 Docker 容器路径没对应起来
- 以为在读主机路径，实际程序运行在容器里

处理方式：

- 检查 launch 里的 `bagpath`
- 检查 Docker 挂载是否生效
- 确认当前程序真正运行在哪个环境里

---

### 3.2 时间窗不匹配导致“程序像没数据一样”

现象：

- 程序能启动
- 但 bag 主循环很快结束
- 几乎看不到点云、IMU、GNSS 真正进入后端

原因：

- `gps_week / start_sec / end_sec` 和数据集不匹配
- 导致消息在主循环里被提前 `continue` 或 `break`

对应主循环位置：

- [test_lio.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_lio.cpp:123)

```cpp
if (m.getTime().toSec() < ((double)ts.time + ts.sec - 18 - 0.005))
    continue;
if (m.getTime().toSec() > ((double)te.time + te.sec - 18))
    break;
```

处理方式：

- 修正 launch 默认时间窗
- 保证 `gpst2time(gps_week, start_sec/end_sec)` 和 bag 里的时间能对齐

---

### 3.3 参数和路径仍指向作者原始环境

现象：

- 程序虽然编译通过
- 但运行时找不到配置文件、输出目录、bag 路径

典型原因：

- YAML / launch / 代码中仍留有：
  - `/home/...`
  - `/mnt/...`
  - 作者机器上的原始路径

处理方式：

- 统一替换为当前 Docker / macOS 可访问路径
- 例如：
  - `/catkin_ws/src/glins/config/...`
  - `/data/...`
  - `/output/...`

---

## 4. GNSS / 载波观测质量相关警告

这类问题有时不会立刻让程序退出，但它们常常是后续 carrier 模式崩溃的前兆。

### 4.1 `phase residual out of range ...`

典型位置：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:527)

典型日志：

```text
phase residual out of range 2.424
phase residual out of range 1.069
```

含义：

- 当前某条载波相位观测残差过大
- 被判定为异常值

本质上说明：

- 这条观测与当前几何、模糊度、状态估计不一致
- 如果硬塞进图，容易破坏 `N(key)` 子图稳定性

常见原因：

- 周跳
- 失锁
- 观测噪声变大
- 当前历元卫星组合不好
- 当前位姿初值偏差较大

处理方式：

- 当前修复中采取了更保守策略：
  - 一旦某个 epoch 出现相位 outlier
  - 直接整帧跳过本 epoch carrier 因子

---

### 4.2 `skip carrier factors at key ... because ... phase observations were rejected as outliers`

典型位置：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:615)

典型日志：

```text
skip carrier factors at key 339 because 4 phase observations were rejected as outliers
```

含义：

- 当前历元的 carrier 相位观测中出现了异常值
- 整个 carrier epoch 被拒绝，不接入因子图

这不是新的错误，而是当前修复逻辑故意加的一层保护。

目的：

- 不让“坏 carrier epoch”把当前 `N(key)` 拉成欠约束或病态

---

### 4.3 `skip carrier factors at key ... because ambiguity block is too small ...`

典型位置：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:621)

典型日志：

```text
skip carrier factors at key ... because ambiguity block is too small (ncp=... carrier_factors=...)
```

含义：

- 当前历元虽然没有明显 outlier
- 但真正留下来的 ambiguity 状态维数或 carrier 因子数太少
- 不足以支撑稳定的 ambiguity 子图

这同样是一个“主动保护”日志，不是额外 bug。

---

## 5. 状态引用和滑窗边缘化相关问题

### 5.1 `%d does not exist`

典型位置：

- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:998)
- [test_gins.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_gins.cpp:914)

典型日志：

```text
0 does not exist
109 does not exist
```

含义：

- 某个旧的状态 key（通常是 `N(lastGNSSepoch)`）已经不在当前优化器里
- 但逻辑上仍可能有人想用它做时序连接

最典型场景：

- 因子图做滑窗边缘化
- 旧 GNSS 区间出窗
- 对应的 `N(i)` 也被一起边缘化
- 这时再尝试构造 `N(key) -> N(lastkey)` 连续性，就可能引用失效状态

这类问题后来通过：

- `optimizer.valueExists(N(lastGNSSepoch))`

检查来规避。

---

### 5.2 `ValuesKeyDoesNotExist`

典型现象：

```text
Attempting to at the key "n339", which does not exist in the Values.
```

含义：

- 某个因子引用了 `n339`
- 但 `Values` 里并没有插入 `N(339)`

出现背景：

- 当前 epoch 决定跳过 carrier 因子
- 但有一条 `GNSSPhasePriorConstraintCompress(N(key), ...)` 仍被直接加进总图
- 导致因子里有 `n339`
- 而 `graphValues` 里没有 `N(339)`

修复方式：

- 将这类 prior 也先放进局部 `carrierGraphFactors`
- 只有 carrier epoch 最终被接受时，才整体并入主图

---

## 6. GTSAM 优化阶段的核心异常

这是本轮最重要的一类报错。

### 6.1 `gtsam::IndeterminantLinearSystemException`

典型日志：

```text
terminate called after throwing an instance of 'gtsam::IndeterminantLinearSystemException'
```

含义：

- GTSAM 在线性化求解时发现当前系统不可唯一确定
- 典型原因：
  - 欠约束
  - 不满秩
  - 数值病态

它不是文件错误，也不是数组越界，而是：

**优化问题本身的数学结构出了问题。**

---

### 6.2 `Symbol: x179`

含义：

- 问题在 `x179` 这个位姿状态附近暴露出来

它通常说明：

- 位姿子图已经被 ambiguity 或其他约束问题“传染”
- 但不一定是最原始根因

在这次排查里，更早期的 `x179` 更像是次级表现。

---

### 6.3 `Symbol: n109 / n939 / n1099`

含义：

- 问题直接落在 ambiguity 状态 `N(key)` 上

这是本轮最关键的定位线索，因为它说明：

- 问题不只是普通 pose 端
- 更核心的是载波模糊度子图本身的约束充分性

这些日志最终帮助把根因收敛到：

- 当前 `N(key)` 在某些 epoch 上没有被充分约束
- 或只剩相对约束，没有足够绝对锚点
- 或时序连接引用了不可用旧状态

---

## 7. 其他运行中出现过的错误/提示

### 7.1 `Invalid quaternion, please use a 9-axis IMU!`

典型位置：

- [utility.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/utility.h:404)

含义：

- IMU 姿态四元数无效
- 程序认为当前 IMU 不能提供可靠姿态

一般与：

- 6 轴 / 9 轴 IMU 配置不匹配
- `imuType` 设置不对
- 输入姿态字段本身有问题

有关。

---

### 7.2 `NO ENCOUGH POSE!`

典型位置：

- [mapOptmizationGps.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/mapping/mapOptmizationGps.cpp:497)

含义：

- 当前用于保存地图或轨迹的位姿数量不足

通常说明：

- 还没跑够
- 或前面主链没有真正生成足够关键帧

---

### 7.3 `ambiguity validation failed ...`
### 7.4 `lambda error (info=...)`

典型位置：

- [gnssEstimator.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssEstimator.h:253)

含义：

- 整数模糊度固定验证失败
- 或 LAMBDA 求解过程失败

这类错误不一定意味着程序主链必崩，但说明：

- 当前 ambiguity fixing 条件不满足
- 不能把浮点 ambiguity 安全固定到整数

---

## 8. 这次错误链是怎么一步步暴露出来的

按照排查顺序，问题大概经历了下面几个阶段：

### 阶段 1：基础运行问题

- bag 时间窗不对
- 配置路径不对
- Docker 路径和宿主机路径混淆

这阶段的错误更多是“程序没真正跑起来”。

---

### 阶段 2：复杂模式开始接触 carrier 子图

出现：

- `phase residual out of range`
- `IndeterminantLinearSystemException`
- `Symbol: n...`

这阶段说明：

- 不是基础链路坏了
- 而是 ambiguity 子图在某些 epoch 上不稳

---

### 阶段 3：加入整帧跳过后暴露新的状态引用问题

出现：

- `skip carrier factors at key ...`
- `ValuesKeyDoesNotExist`

这阶段说明：

- carrier 跳过策略方向是对的
- 但代码里仍有因子在引用不存在的 `N(key)`

---

### 阶段 4：接受的 epoch 仍然可能病态

即使没有 `skip carrier factors...`
也仍然出现：

- `IndeterminantLinearSystemException`
- `Symbol: n1099`

这进一步说明：

- 不是只有 outlier 才会导致问题
- 某些“看起来可接受”的 ambiguity 块仍然过弱

于是后续又补了：

- 更强的 ambiguity block 稳定化先验
- 更严格的 accepted-epoch 检查

---

## 9. 本轮最终结论

综合这次出现过的错误，可以把最核心的问题总结成：

1. 基础运行错误主要来自路径、时间窗和参数不匹配
2. 复杂模式的核心不稳定点在载波 ambiguity 子图 `N(key)`
3. 载波 epoch 如果观测异常、观测太少、旧状态失效或只剩相对约束，就容易在 `n...` 上崩
4. 滑窗边缘化会进一步放大 ambiguity 子图的脆弱性
5. 程序后续稳定下来，靠的是：
   - 禁止引用无效 `N(lastkey)`
   - 整帧跳过坏 carrier epoch
   - 给弱 ambiguity block 增加稳定化先验
   - 提高 accepted carrier epoch 的筛选强度

---

## 10. 一句话总结

这次运行里报过的错误，表面上看有很多种，但真正主线非常清楚：

**基础阶段是“路径和时间窗问题”，复杂阶段是“载波 ambiguity 子图在滑窗因子图里不够稳定”，最终主要表现为 `phase residual out of range`、`ValuesKeyDoesNotExist` 和 `IndeterminantLinearSystemException`。**
