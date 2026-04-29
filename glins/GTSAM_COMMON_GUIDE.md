# GLINS 中常用 GTSAM 变量、类与函数说明

## 1. 文档目的

这份文档不是泛泛介绍 GTSAM 全部功能，而是专门结合当前 `GLINS` 工程，整理：

- 常见状态变量是什么意思
- 常见 GTSAM 类型各自负责什么
- 常见因子、噪声模型、优化器怎么用
- 常见函数调用在代码里代表什么含义

适合配合以下文件一起看：

- [glins/GTSAM_GRAPH_OPTIMIZATION_EXAMPLE.md](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/GTSAM_GRAPH_OPTIMIZATION_EXAMPLE.md:1)
- [glins/src/test_gins.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_gins.cpp:1)
- [glins/include/imu/imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:1)
- [glins/include/imu/imuPreintegration.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.h:1)
- [glins/include/factor/GnssFactor.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/factor/GnssFactor.h:1)
- [glins/include/factor/LidarFactor.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/factor/LidarFactor.h:1)

---

## 2. GTSAM 在这个工程里主要做什么

GTSAM 在这里承担的是：

**把 IMU、Lidar、GNSS、载波相位、多普勒、NHC 等观测统一写成因子图，然后做非线性优化。**

你可以把它理解成下面这个流程：

1. 定义状态变量
2. 给状态之间加因子
3. 给新状态提供初值
4. 调增量优化器更新
5. 读取优化结果
6. 边缘化旧状态，维持滑窗

---

## 3. 这个工程里最常见的状态变量

在 [imuPreintegration.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.h:24) 和 [test_gins.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_gins.cpp:30) 里，可以看到：

```cpp
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::T;
```

它们分别表示：

- `X(key)`：位姿状态 `Pose3`
- `V(key)`：速度状态 `Vector3`
- `B(key)`：IMU bias 状态
- `N(key)`：载波模糊度状态
- `T(key)`：外参或平移外参状态

这里的 `key` 不是时间戳本身，而是：

**因子图内部给每个时刻状态编号的索引。**

例如：

- `X(0)`：第 0 个时刻位姿
- `X(100)`：第 100 个时刻位姿
- `N(939)`：第 939 个 GNSS 关键历元的 ambiguity 状态

---

## 4. 常用几何类型

### 4.1 `gtsam::Rot3`

表示三维旋转。

常见用法：

```cpp
gtsam::Rot3(1, 0, 0, 0)
Rot3::Quaternion(w, x, y, z)
Rot3::RzRyRx(r, p, y)
```

常见成员函数：

- `yaw()`：取偏航角
- `pitch()`：取俯仰角
- `roll()`：取横滚角
- `toQuaternion()`：转四元数
- `Logmap(...)`：把旋转映射到李代数小量

工程里典型位置：

- [test_gins.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_gins.cpp:97)
- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:748)

---

### 4.2 `gtsam::Point3`

表示三维点。

常用来构造位姿里的平移部分，例如：

```cpp
gtsam::Point3(x, y, z)
```

---

### 4.3 `gtsam::Pose3`

表示三维刚体位姿，由：

- 一个旋转 `Rot3`
- 一个平移 `Point3`

组成。

构造方式：

```cpp
gtsam::Pose3(R, t)
```

常见成员函数：

- `translation()`：取平移
- `rotation()`：取旋转
- `compose(other)`：位姿复合
- `between(other)`：求相对位姿
- `x() / y() / z()`：取平移分量

在工程里最常见，因为：

- `X(key)` 的类型就是 `Pose3`
- 外参 `imu2gps`、`gps2imu`、`imu2Lidar` 等也都是 `Pose3`

典型位置：

- [test_gins.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_gins.cpp:236)
- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:467)

---

### 4.4 `gtsam::NavState`

表示一个导航状态，通常包含：

- 当前位姿
- 当前速度

常见用法：

```cpp
gtsam::NavState(prevPose_, prevVel_)
```

常见成员函数：

- `pose()`
- `v()`

它在 IMU 预积分里很常见，因为预积分传播的目标就是：

**从上一时刻 `NavState` 预测下一时刻 `NavState`。**

---

## 5. 常用向量、矩阵、bias 类型

### 5.1 `gtsam::Vector` / `Vector3`

- `Vector3`：固定 3 维向量
- `Vector`：动态长度向量

工程里常见用途：

- `Vector3`：速度、GNSS 外参、三轴噪声参数
- `Vector`：模糊度向量 `N(key)`、6 维 bias 参数、噪声 sigma 列表

---

### 5.2 `gtsam::Matrix`

表示矩阵。

常用来：

- 存储雅可比
- 存储协方差
- 手工构造噪声模型

例如：

```cpp
gtsam::Matrix33::Identity(3, 3)
gtsam::Matrix66::Identity(6, 6)
```

---

### 5.3 `gtsam::imuBias::ConstantBias`

表示 IMU bias。

通常按 6 维组织：

- 前 3 维：加速度 bias
- 后 3 维：陀螺仪 bias

构造方式：

```cpp
gtsam::imuBias::ConstantBias((gtsam::Vector(6) << ...).finished())
```

常见成员函数：

- `accelerometer()`
- `gyroscope()`

工程里典型位置：

- [test_gins.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_gins.cpp:315)
- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:1014)

---

## 6. 常见噪声模型

GTSAM 中很多因子都需要一个噪声模型，告诉优化器：

**这个因子有多可信。**

### 6.1 `noiseModel::Diagonal::Sigmas(...)`

最常见。

意思是：

- 每一维标准差自己指定
- 维度之间互相独立

例如：

```cpp
gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, ...).finished())
```

常用于：

- 位姿先验
- Lidar 校正
- bias between 因子

---

### 6.2 `noiseModel::Isotropic::Sigma(dim, sigma)`

表示：

- `dim` 维
- 每一维标准差相同，都是 `sigma`

例如：

```cpp
gtsam::noiseModel::Isotropic::Sigma(3, 1e4)
```

工程里常用于：

- 初始速度先验
- 初始 bias 先验

---

### 6.3 `noiseModel::Robust::Create(...)`

表示：

**在普通高斯噪声外面再包一层鲁棒核。**

常见鲁棒核：

- `Huber`
- `Cauchy`

作用：

- 降低异常值对优化的破坏

工程里例如：

- `NHC` 因子
- 某些 Lidar 相对位姿约束

---

## 7. 常见因子类

### 7.1 `PriorFactor<T>`

对某个变量加先验。

例如：

```cpp
PriorFactor<Pose3>(X(0), prevPose_, priorPoseNoise)
PriorFactor<Vector3>(V(0), prevVel_, priorVelNoise)
PriorFactor<imuBias::ConstantBias>(B(0), prevBias_, priorBiasNoise)
```

含义是：

- 第 0 帧位姿接近某个值
- 第 0 帧速度接近某个值
- 第 0 帧 bias 接近某个值

典型作用：

- 初始化因子图
- 给新引入但容易漂的状态一个锚点

---

### 7.2 `BetweenFactor<T>`

约束两个同类型变量之间的相对关系。

例如：

```cpp
BetweenFactor<Pose3>(X(lastKeyIndex), X(key), delta_Pose1, odometryNoise)
BetweenFactor<imuBias::ConstantBias>(B(key - 1), B(key), ..., ...)
```

含义是：

- 两个位姿之间应满足某个相对位姿
- 相邻 bias 不应突变

`BetweenFactor` 是因子图里非常核心的结构，因为它天然表达“相邻状态关系”。

---

### 7.3 `ImuFactor`

表示一段 IMU 预积分构成的约束。

工程里典型形式：

```cpp
gtsam::ImuFactor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu)
```

它会把：

- 上一帧位姿/速度
- 当前帧位姿/速度
- 上一帧 bias
- 一段预积分测量

绑在一起。

作用是：

**把高频 IMU 连续运动信息转成一个离散因子。**

---

### 7.4 `CombinedImuFactor`

是 `ImuFactor` 的更完整版本。

它会更完整地把：

- 测量噪声
- bias 漂移
- 状态之间耦合

一起建模。

如果工程里启用了：

```cpp
#ifdef USE_COMBINED_IMU
```

就会走这条链。

如果没启用，一般就是：

- `ImuFactor`
- 再单独加 `BetweenFactor<imuBias::ConstantBias>`

---

### 7.5 自定义因子

这个工程大量使用了自定义因子，例如：

- `GnssFactor`
- `LidarEdgeFactor`
- `LidarPlaneFactor`
- `NhcFactor`
- `GNSSDDCpFactorCompress_ENU`
- `GNSSAmbConstraintCompress`

它们通常继承自：

- `NoiseModelFactor1<T>`
- `NoiseModelFactor2<T1, T2>`
- `NoiseModelFactor3<T1, T2, T3>`

区别只在于：

- 这个因子连接几个状态变量

例如：

- `NoiseModelFactor1<Pose3>`：只约束一个位姿
- `NoiseModelFactor2<Pose3, Vector>`：同时约束位姿和模糊度

自定义因子的核心函数是：

```cpp
Vector evaluateError(...)
```

它返回：

**当前变量取值下，这个因子的残差是多少。**

如果还提供 `H1`, `H2`, `H3`，那就是同时返回雅可比。

---

## 8. 因子图容器与变量容器

### 8.1 `gtsam::NonlinearFactorGraph`

表示一组非线性因子的集合。

在工程里常见变量名：

- `graphFactors`
- `odomGraphFactors`
- `graph`

常见操作：

```cpp
graphFactors.add(factor)
graphFactors.resize(0)
```

含义：

- `add(...)`：往图里加因子
- `resize(0)`：清空这批待提交因子

---

### 8.2 `gtsam::Values`

表示变量初值或当前估计值的集合。

在工程里常见变量名：

- `graphValues`
- `result`
- `initialEstimate`

常见操作：

```cpp
graphValues.insert(X(key), pose)
graphValues.insert(V(key), vel)
graphValues.insert(B(key), bias)
```

含义：

- 当你给优化器引入一个新状态时，除了加因子，还必须提供该状态的初值

读取方式：

```cpp
result.at<gtsam::Pose3>(X(key))
result.at<gtsam::Vector3>(V(key))
result.at<gtsam::imuBias::ConstantBias>(B(key))
```

---

## 9. 常见优化器与相关函数

### 9.1 `gtsam::ISAM2`

这是当前工程主优化器。

作用是：

**做增量式因子图优化。**

常见用法：

```cpp
gtsam::ISAM2 optimizer;
optimizer.update(graphFactors, graphValues);
gtsam::Values result = optimizer.calculateEstimate();
```

优点：

- 每来一批新因子就增量更新
- 不必每次从零开始全量求解

工程里当前主要使用：

- `ISAM2`
- `factorization = CHOLESKY`

---

### 9.2 `ISAM2Params`

用来配置 `ISAM2`。

常见参数：

- `relinearizeThreshold`
- `relinearizeSkip`
- `factorization`

含义：

- `relinearizeThreshold`：变量变化多大时重新线性化
- `relinearizeSkip`：隔多少次更新才检查一次
- `factorization`：线性求解方式，例如 `CHOLESKY`

---

### 9.3 `optimizer.update(...)`

表示把：

- 新因子
- 新变量初值

提交给 `ISAM2`。

例如：

```cpp
optimizer.update(graphFactors, graphValues);
```

有时还会继续多调几次：

```cpp
optimizer.update();
optimizer.update();
```

这是工程上常见的“再压几轮增量更新”写法。

---

### 9.4 `optimizer.calculateEstimate()`

从当前优化器里取出最新状态估计。

例如：

```cpp
gtsam::Values result = optimizer.calculateEstimate();
```

然后再通过：

```cpp
result.at<...>(X(key))
```

读出某个具体变量。

---

### 9.5 `optimizer.valueExists(key)`

判断某个状态变量当前是否还在优化器活动图里。

这在本工程里很重要，因为：

- 状态会被边缘化
- `N(lastkey)`、`T(i)` 不一定总还在图里

例如：

```cpp
if (optimizer.valueExists(N(i))) { ... }
```

---

### 9.6 `optimizer.marginalCovariance(key)`

取某个变量的边缘协方差。

工程里常见：

```cpp
posCovariance = optimizer.marginalCovariance(X(key));
```

含义：

- 当前位姿估计的不确定性大小

GNSS、可视化、结果保存时经常会用到。

---

## 10. IMU 预积分相关类型和函数

### 10.1 `PreintegrationParams` / `PreintegrationCombinedParams`

它们是 IMU 预积分参数对象。

里面存：

- 重力大小
- 加速度计噪声
- 陀螺仪噪声
- 积分误差
- 在 combined 版本里还有 bias 漂移相关协方差

构造方式：

```cpp
gtsam::PreintegrationParams::MakeSharedU(imuGravity)
gtsam::PreintegrationCombinedParams::MakeSharedU(imuGravity)
```

---

### 10.2 `PreintegratedImuMeasurements`

普通 IMU 预积分器。

它会不断接收：

- 加速度
- 角速度
- `dt`

然后累计成一段预积分测量。

常见函数：

```cpp
integrateMeasurement(acc, gyro, dt)
resetIntegrationAndSetBias(bias)
predict(prevState, prevBias)
deltaTij()
deltaRij()
```

含义：

- `integrateMeasurement(...)`：继续积分一条 IMU 数据
- `resetIntegrationAndSetBias(...)`：优化后重置积分器并用新 bias 继续
- `predict(...)`：根据上一状态和 bias 预测当前状态
- `deltaTij()`：当前累计积分时长
- `deltaRij()`：当前累计旋转增量

---

### 10.3 `PreintegratedCombinedMeasurements`

是 combined IMU 版本的预积分器。

和上面类似，但建模更完整，会更深地考虑 bias 漂移耦合。

---

## 11. 常见几何操作函数

### 11.1 `compose(...)`

位姿复合。

例如：

```cpp
prevPose_ = lidarPose.compose(lidar2Imu);
gpsPose = prevPose_.compose(gps2imu);
```

表示：

- 先有一个位姿
- 再右乘一个外参或相对位姿

---

### 11.2 `between(...)`

求两个位姿之间的相对位姿。

例如：

```cpp
delta_Pose1 = lastKeyPose.between(curPose);
```

含义：

- 从 `lastKeyPose` 到 `curPose` 的相对运动是多少

这是 odometry / Lidar relative factor 很常用的操作。

---

### 11.3 `translation()` / `rotation()`

分别取平移和旋转。

例如：

```cpp
pose.translation().x()
pose.rotation().yaw()
```

---

## 12. 自定义因子里最常见的接口

### 12.1 `evaluateError(...)`

这是自定义因子的核心。

例如：

```cpp
Vector evaluateError(const Pose3& p,
    boost::optional<gtsam::Matrix&> H = boost::none) const override
```

返回值：

- 当前因子的残差向量

如果填写 `H`：

- 还同时给优化器提供解析雅可比

你可以把它理解成：

**告诉 GTSAM：当前变量如果取这个值，观测误差是多少。**

---

### 12.2 `clone()`

很多自定义因子会实现：

```cpp
virtual gtsam::NonlinearFactor::shared_ptr clone() const
```

作用是：

- 让 GTSAM 可以复制这个因子对象

通常是标准写法，工程里一般不用主动调用。

---

## 13. Marginalization 相关

这个工程里有自定义边缘化辅助：

- [marginalize_isam2.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/marginalize_isam2.h:1)

### 13.1 `updateAndMarginalize(...)`

这是工程里非常常见的一个封装函数。

作用是同时完成：

- 往 `ISAM2` 里加新因子和新状态
- 指定一批旧状态做边缘化

典型调用：

```cpp
updateAndMarginalize(graphFactors, graphValues, {}, optimizer);
updateAndMarginalize({}, {}, marginalKeys, optimizer);
```

含义分别是：

- 第一种：更新优化器，不边缘化
- 第二种：不加新因子，只边缘化一批旧 key

---

## 14. GTSAM 里最容易混淆的几组概念

### 14.1 `graphFactors` vs `graphValues`

`graphFactors`
- 是约束

`graphValues`
- 是新变量初值

很多人刚接触时容易只加 factor，不给新变量插入初值，这是不行的。

---

### 14.2 `result` vs `optimizer`

`optimizer`
- 是整个增量优化器对象，保存图结构和内部状态

`result`
- 是某一时刻从优化器里取出来的当前估计快照

---

### 14.3 测量噪声 vs bias 漂移噪声

例如：

- `accelerometerCovariance`
- `gyroscopeCovariance`

描述的是：

**当前 IMU 测量本身有多 noisy**

而：

- `biasAccCovariance`
- `biasOmegaCovariance`
- `noiseModelBetweenBias`

描述的是：

**bias 会不会随时间慢慢漂**

---

## 15. 结合本工程的一帧典型 GTSAM 使用流程

以 `featureHandler()` 为例，一帧大致会做：

1. 用 IMU 积分出一段预积分量
2. 构造 `ImuFactor`
3. 构造 `BetweenFactor` 约束 bias 漂移
4. 插入当前 `X(key)`, `V(key)`, `B(key)` 初值
5. 如果有 GNSS，就加 GNSS 因子
6. 如果有载波，就加 `N(key)` 相关因子
7. 如果有 Lidar，就加 Lidar 位姿或几何因子
8. 调 `updateAndMarginalize(...)`
9. 用 `calculateEstimate()` 取结果
10. 读取 `X(key)`, `V(key)`, `B(key)` 更新系统状态
11. 适时边缘化旧变量

所以你在代码里看到的 GTSAM 语句，大多都能归到这条链上。

---

## 16. 推荐阅读顺序

如果你准备系统理解这个工程里的 GTSAM，用下面顺序最顺：

1. 看 [test_gins.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_gins.cpp:121)
   先理解成员变量、噪声模型、预积分器初始化

2. 看 [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:424)
   理解一帧数据到来后如何建图、优化、边缘化

3. 看 [GnssFactor.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/factor/GnssFactor.h:1)
   理解最基础的自定义因子长什么样

4. 再看 [LidarFactor.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/factor/LidarFactor.h:1)
   理解几何因子如何写残差和雅可比

5. 最后看载波相关自定义因子
   理解 `N(key)` 为什么更复杂

---

## 17. 一句话总结

在这个工程里，GTSAM 的核心角色就是：

**用 `Pose3 / NavState / Bias / Ambiguity` 这些状态，配合 `PriorFactor / BetweenFactor / ImuFactor / 自定义 GNSS/Lidar 因子`，通过 `ISAM2` 做增量式融合优化。**
