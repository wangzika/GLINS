# GTSAM 图优化完整例子

## 1. 一个图优化问题需要什么

一个完整的 GTSAM 图优化通常包含 5 个部分：

- `变量 key`：要估计什么，例如 `X(0), X(1), X(2)`。
- `初值 Values`：每个变量优化开始前的初始猜测。
- `因子 graph`：每条观测/约束，例如先验、里程计、GNSS。
- `噪声模型 noise model`：告诉优化器每条观测有多可信。
- `优化器 optimizer`：根据因子和初值求最优状态。

可以把它想成：

```text
变量 X
  |
因子 residual(X)
  |
优化器最小化 Σ || residual ||²
  |
得到优化后的 X
```

---

## 2. 最小完整例子：二维小车位姿图

这个例子估计 3 个二维位姿：

```text
X0 ---- odom01 ---- X1 ---- odom12 ---- X2
|
prior
```

### 2.1 完整代码

```cpp
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

using gtsam::symbol_shorthand::X;

int main()
{
    // 1. 创建非线性因子图：所有约束都放在这里。
    gtsam::NonlinearFactorGraph graph;

    // 2. 创建噪声模型。
    // Pose2 的误差维度是 3: x, y, theta。
    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << 0.1, 0.1, 0.05).finished());

    auto odomNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << 0.2, 0.2, 0.1).finished());

    // 3. 添加先验因子。
    // 这相当于告诉优化器：X0 大概在原点附近。
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(
        X(0),
        gtsam::Pose2(0.0, 0.0, 0.0),
        priorNoise));

    // 4. 添加里程计 BetweenFactor。
    // X0 到 X1 的相对运动观测是向前 1 米。
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        X(0),
        X(1),
        gtsam::Pose2(1.0, 0.0, 0.0),
        odomNoise));

    // X1 到 X2 也是向前 1 米。
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
        X(1),
        X(2),
        gtsam::Pose2(1.0, 0.0, 0.0),
        odomNoise));

    // 5. 给每个变量插入初值。
    // 初值可以不完全准确，但必须存在。
    gtsam::Values initial;
    initial.insert(X(0), gtsam::Pose2(0.2, -0.1, 0.05));
    initial.insert(X(1), gtsam::Pose2(1.1, 0.1, -0.02));
    initial.insert(X(2), gtsam::Pose2(2.2, -0.1, 0.03));

    // 6. 优化。
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    gtsam::Values result = optimizer.optimize();

    // 7. 读取结果。
    std::cout << "X0 = " << result.at<gtsam::Pose2>(X(0)) << std::endl;
    std::cout << "X1 = " << result.at<gtsam::Pose2>(X(1)) << std::endl;
    std::cout << "X2 = " << result.at<gtsam::Pose2>(X(2)) << std::endl;

    return 0;
}
```

### 2.2 这个例子发生了什么

`PriorFactor` 固定了图的坐标系。如果没有先验，整个轨迹可以整体平移/旋转，问题会欠约束。

`BetweenFactor` 表示两个位姿之间的相对运动观测。它不会直接说 `X1` 在哪里，而是说：

```text
X0.between(X1) 应该接近 Pose2(1, 0, 0)
```

优化目标可以写成：

```text
min || prior_error(X0) ||²
  + || between_error(X0, X1) ||²
  + || between_error(X1, X2) ||²
```

如果考虑噪声模型，就是：

```text
min e0^T R0^-1 e0 + e1^T R1^-1 e1 + e2^T R2^-1 e2
```

---

## 3. 贴近 GLINS 的 Pose3 + GNSS 因子例子

在 GLINS 里，核心变量通常是：

```cpp
X(key) // Pose3，当前 IMU/LiDAR/body 位姿
V(key) // Vector3，速度
B(key) // imuBias::ConstantBias，IMU bias
N(key) // Vector，载波模糊度
T(key) // Vector3，GNSS 杆臂或外参平移
```

下面是一个简化版流程，展示一帧里怎样把 IMU/LiDAR/GNSS 约束加入图。

```cpp
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::T;

gtsam::NonlinearFactorGraph graphFactors;
gtsam::Values graphValues;

int key = 10;
int lastKey = 9;

// 1. 给当前位姿、速度、bias 插入初值。
// 初值通常来自 IMU 预积分预测或 LiDAR 里程计。
gtsam::Pose3 predictedPose = propState.pose();
gtsam::Vector3 predictedVel = propState.v();
gtsam::imuBias::ConstantBias predictedBias = prevBias;

graphValues.insert(X(key), predictedPose);
graphValues.insert(V(key), predictedVel);
graphValues.insert(B(key), predictedBias);

// 2. 添加 IMU 预积分因子。
// 约束上一帧状态和当前帧状态之间的运动关系。
auto imuFactor = gtsam::CombinedImuFactor(
    X(lastKey), V(lastKey),
    X(key),     V(key),
    B(lastKey), B(key),
    *imuIntegratorOpt_);

graphFactors.add(imuFactor);

// 3. 添加 bias 随机游走因子。
// 假设相邻时刻 bias 不会突变。
auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.001, 0.001, 0.001).finished());

graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
    B(lastKey),
    B(key),
    gtsam::imuBias::ConstantBias(),
    biasNoise));

// 4. 添加 LiDAR 里程计因子。
// deltaPose 来自 LiDAR 前端/scan-to-map。
auto lidarNoise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.5, 0.5, 0.5).finished());

graphFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(
    X(lastKey),
    X(key),
    deltaPose,
    lidarNoise));

// 5. 添加 GNSS 双差伪距因子。
// 这一步在当前工程中由 gnssContainer 完成。
// 它内部会构造 GNSSDDPsrFactor_ENU 并 add 到 graphFactors。
int npr = container.addDDPsrFactorENU(&graphFactors, &graphValues, key);

// 6. 如果使用载波相位，还要插入 ambiguity 初值并添加载波因子。
if (useCarrier)
{
    gtsam::Vector initialAmb = container.getCompressedAmbiguity();
    graphValues.insert(N(key), initialAmb);

    int ncp = container.addDDCpFactorENU(
        &graphFactors,
        &graphValues,
        key,
        lastGNSSepoch,
        allowCarrierTemporalLink);
}

// 7. 如果在线估计 GNSS 杆臂，需要插入 T(key) 并加先验/连续约束。
if (estimateExtGPS)
{
    graphValues.insert(T(key), extGPS);

    auto extNoise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << 0.1, 0.1, 0.1).finished());

    graphFactors.add(gtsam::PriorFactor<gtsam::Vector3>(
        T(key),
        extGPS,
        extNoise));
}

// 8. 调用 ISAM2 增量优化。
optimizer.update(graphFactors, graphValues);
gtsam::Values result = optimizer.calculateEstimate();

// 9. 取优化结果。
gtsam::Pose3 optPose = result.at<gtsam::Pose3>(X(key));
gtsam::Vector3 optVel = result.at<gtsam::Vector3>(V(key));
gtsam::imuBias::ConstantBias optBias =
    result.at<gtsam::imuBias::ConstantBias>(B(key));
```

---

## 4. GLINS 中一条 GNSS 伪距因子的完整链路

以 `GNSSDDPsrFactor_ENU` 为例：

```cpp
GNSSDDPsrFactor_ENU::shared_ptr dd_psr_factor(
    new GNSSDDPsrFactor_ENU(
        X(key),
        Info_Master,
        Infos_Else,
        prcopt,
        f,
        false,
        lla_origin,
        extGPS,
        extRot.transpose(),
        huber));

graphFactors->add(dd_psr_factor);
```

它加入图以后，优化器会在每次线性化时调用：

```cpp
Vector evaluateError(const Pose3& RxPos,
    boost::optional<gtsam::Matrix&> H1) const
```

其中：

- `RxPos` 来自当前优化器中的 `X(key)`。
- `evaluateError()` 会根据 `RxPos` 计算 GNSS 天线 ECEF 坐标。
- 然后计算双差伪距残差。
- `H1` 返回残差对 `X(key)` 的雅可比。

残差公式是：

```text
residual_j =
[(P_r_i - rho_r_i) - (P_b_i - rho_b_i)]
-
[(P_r_j - rho_r_j) - (P_b_j - rho_b_j)]
```

其中：

- `i` 是参考星 `Info_Master`。
- `j` 是从卫星 `Infos_Else[j]`。
- `P` 是接收机伪距观测。
- `rho` 是根据当前状态预测出来的几何距离。

优化器会调整 `X(key)`，让这些 residual 尽量接近 0。

---

## 5. 因子、初值、结果之间的关系

很容易混淆的一点是：

```text
graphFactors 里放的是约束
graphValues 里放的是初值
result 里放的是优化后的值
```

例如：

```cpp
graphFactors.add(PriorFactor<Pose3>(X(0), pose0, noise));
graphValues.insert(X(0), initialPose0);
```

含义不是“把 `X(0)` 固定成 `initialPose0`”，而是：

```text
initialPose0 是优化起点；
PriorFactor 是希望 X(0) 接近 pose0 的约束。
```

优化后：

```cpp
Pose3 optimizedPose0 = result.at<Pose3>(X(0));
```

这个 `optimizedPose0` 才是最终估计值。

---

## 6. 一个因子必须具备什么

如果你自己写一个自定义因子，最少需要：

- 继承合适的 GTSAM 因子基类，例如 `NoiseModelFactor1<Pose3>`。
- 保存观测数据，例如 GNSS 的 `Info_Master / Infos_Else`。
- 保存模型参数，例如 `prcopt / lla_origin / extGPS`。
- 实现 `evaluateError()`。
- 在 `evaluateError()` 中返回残差。
- 如果 `H1/H2/...` 存在，写入对应雅可比。

典型结构：

```cpp
class MyFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
    MyFactor(gtsam::Key key, Measurement meas,
        const gtsam::SharedNoiseModel& noise)
        : NoiseModelFactor1<gtsam::Pose3>(noise, key),
          meas_(meas)
    {
    }

    gtsam::Vector evaluateError(const gtsam::Pose3& pose,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const override
    {
        gtsam::Vector residual(1);

        // 1. 根据当前 pose 预测观测。
        double predicted = predictMeasurement(pose);

        // 2. 残差 = 预测 - 实际，或者实际 - 预测。
        residual(0) = predicted - meas_.value;

        // 3. 如果优化器需要雅可比，就填 H1。
        if (H1)
        {
            H1->resize(1, 6);
            *H1 = computeJacobian(pose);
        }

        return residual;
    }

private:
    Measurement meas_;
};
```

---

## 7. 最核心的一句话

图优化就是不断重复下面这件事：

```text
用当前状态预测观测
-> 和真实观测做差得到 residual
-> 用雅可比告诉优化器怎么改状态
-> 优化器调整所有状态
-> 让所有 residual 的加权平方和最小
```

在 GLINS 里：

```text
IMU 因子让运动连续
LiDAR 因子让局部几何匹配
GNSS 伪距因子拉住绝对位置
GNSS 载波因子提供高精度相对约束
Bias 因子限制 IMU bias 漂移
外参因子让杆臂/外参不要乱跑
```

