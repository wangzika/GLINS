# IMU Bias 噪声参数说明

## 1. 这几行代码在描述什么

`test_gins.cpp` 里这三行：

```cpp
p->biasAccCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccBiasN, 2);
p->biasOmegaCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrBiasN, 2);
p->biasAccOmegaInt = gtsam::Matrix66::Identity(6, 6) * 1e-5;
```

描述的不是“IMU 当前这一拍测量值有多抖”，而是：

- 加速度计 bias 会不会随时间慢慢漂
- 陀螺仪 bias 会不会随时间慢慢漂
- 在 combined IMU 预积分模型里，这种 bias 漂移如何进入联合不确定性传播

也就是说，这几项针对的是 **bias 的时间演化模型**。

---

## 2. 它和上面那几项的区别

在这几行前面通常还会看到：

```cpp
p->accelerometerCovariance = ...
p->gyroscopeCovariance = ...
p->integrationCovariance = ...
```

这两组参数的核心区别是：

### 第一组：测量噪声

- `accelerometerCovariance`
- `gyroscopeCovariance`
- `integrationCovariance`

它们描述的是：

- 当前时刻 IMU 测量值本身的白噪声
- 以及预积分数值过程中的附加误差

更像是在说：

```text
a_measured = a_true + noise
w_measured = w_true + noise
```

这里建模的是 `noise`。

### 第二组：bias 漂移噪声

- `biasAccCovariance`
- `biasOmegaCovariance`
- `biasAccOmegaInt`

它们描述的是：

- bias 本身不是固定常数
- bias 会随着时间慢慢漂移
- 这种漂移通常按随机游走（random walk）来建模

更像是在说：

```text
b_{k+1} = b_k + random_walk
```

这里建模的是 `random_walk`。

---

## 3. 三个参数分别是什么意思

### 3.1 `biasAccCovariance`

```cpp
p->biasAccCovariance =
    gtsam::Matrix33::Identity(3, 3) * pow(imuAccBiasN, 2);
```

表示：

**加速度计 bias 的随机游走协方差**

含义是：

- 加速度计零偏不是永远不变
- 它会随时间缓慢变化
- `imuAccBiasN` 越大，说明系统认为这种漂移越明显

---

### 3.2 `biasOmegaCovariance`

```cpp
p->biasOmegaCovariance =
    gtsam::Matrix33::Identity(3, 3) * pow(imuGyrBiasN, 2);
```

表示：

**陀螺仪 bias 的随机游走协方差**

含义与上一项一致，只是对象变成了陀螺仪零偏。

---

### 3.3 `biasAccOmegaInt`

```cpp
p->biasAccOmegaInt = gtsam::Matrix66::Identity(6, 6) * 1e-5;
```

表示：

**combined IMU 模型里 acc bias 与 gyro bias 的联合积分不确定性项**

为什么是 `6x6`：

- 前 3 维对应加速度 bias
- 后 3 维对应角速度 bias

它主要用于 combined 预积分时的联合协方差传播，所以只会在：

- `PreintegrationCombinedParams`
- `PreintegratedCombinedMeasurements`

这条链里出现。

---

## 4. 一个直观模型

可以把 IMU 测量写成：

```text
a_measured = a_true + b_a + n_a
w_measured = w_true + b_g + n_g
```

其中：

- `n_a`：加速度计瞬时白噪声
- `n_g`：陀螺仪瞬时白噪声
- `b_a`：加速度计 bias
- `b_g`：陀螺仪 bias

那么对应关系就是：

- `accelerometerCovariance` 对应 `n_a`
- `gyroscopeCovariance` 对应 `n_g`
- `biasAccCovariance` 对应 `b_a` 的漂移
- `biasOmegaCovariance` 对应 `b_g` 的漂移

所以：

- 前一组在描述“每一拍测量值有多 noisy”
- 后一组在描述“零偏会不会随着时间慢慢跑掉”

---

## 5. 为什么只在 combined IMU 分支里看到这些

这些参数位于：

```cpp
#ifdef USE_COMBINED_IMU
```

下面，原因是 combined IMU 模型会更完整地把：

- 测量噪声
- bias 漂移
- bias 与状态之间的联合不确定性

一起传播。

而普通 `ImuFactor` 路线更常见的做法是：

- 在 `PreintegrationParams` 里放测量噪声
- 再用 `BetweenFactor<imuBias::ConstantBias>` 单独约束 bias 漂移

所以 combined 路线里会看到更多与 bias 协方差直接相关的参数。

---

## 6. 一句话总结

这三行不是在描述“IMU 当前读数有多抖”，而是在描述：

**IMU 的加速度 bias 和陀螺仪 bias 会不会随时间漂移，以及这种漂移在 combined 预积分里如何传播。**
