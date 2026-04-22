# RTKLIB 算法工作流程说明

本文档从 **算法视角** 解释当前仓库中的 RTKLIB 工作流程。  
它和 [`RTKLIB_WORKFLOW.md`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/RTKLIB_WORKFLOW.md:1>) 的区别是：

- `RTKLIB_WORKFLOW.md` 偏工程链路，回答“程序是怎么跑起来的”
- 本文档偏算法链路，回答“每个历元里到底算了什么”

重点覆盖：

- 单点定位 `pntpos()`
- 相对定位 `relpos()`
- 零差 / 双差残差构建
- 卡尔曼滤波状态传播与更新
- 模糊度固定 `LAMBDA`
- 当前仓库在速度估计、鲁棒处理和 GNSS 因子输出上的改造

---

## 1. 一句话概括

当前仓库里的 RTKLIB 算法主链可以概括为：

**先用广播星历和伪距做单点定位，得到 rover 的位置与速度初值；再用 rover/base 的零差和双差观测建立 RTK 状态空间模型，通过卡尔曼滤波更新位置、速度、模糊度等状态；最后用 LAMBDA 尝试整周固定，并输出 float/fix 解。**

---

## 2. 算法层的总体分层

如果只看单个历元，RTKLIB 的算法可以分成 5 层：

1. 卫星与观测预处理层
   - 卫星位置、钟差、频率、方位角/高度角
2. 单点定位层
   - 用伪距先解一个 rover 粗位置
3. 相对定位建模层
   - 构建 rover/base 的零差和双差残差
4. 状态估计层
   - 卡尔曼预测 + 量测更新
5. 整周模糊度固定层
   - LAMBDA + ratio 检验 + fix-and-hold

按代码入口看，对应关系是：

- 单点定位：[`pntpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2255>)
- RTK 主控：[`rtkpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3336>)
- 相对定位：[`relpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2591>)

---

## 3. 第一步：先算卫星位置和钟差

不管是单点定位还是 RTK，相同的第一步都是：

- 根据导航文件算出卫星在当前历元的位置、速度和钟差

这一步在两个主链里都能看到：

- `pntpos()` 中：
  - [`satposs(sol->time, obs, n, nav, opt_.sateph, rs, dts, var, svh)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2307>)
- `relpos()` 中：
  - [`satposs(time, obs, n, nav, opt->sateph, rs, dts, var, svh)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2662>)

这一步输出的核心量包括：

- `rs`：卫星位置和速度
- `dts`：卫星钟差和钟漂
- `var`：星历误差方差
- `svh`：卫星健康状态

算法上，它相当于把：

**导航电文 -> 卫星时空状态**

如果这一步算不准，后面所有观测方程都会错。

---

## 4. 第二步：单点定位 `pntpos()`

### 4.1 为什么 RTK 之前一定先做单点定位

在 `rtkpos()` 中，第一件核心算法操作就是：

- [`pntpos(obs, nu, nav, &rtk->opt, &rtk->sol, NULL, rtk->ssat, msg)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3367>)

原因是：

- 需要一个 rover 位置初值
- 需要得到每颗卫星的几何关系
- 需要形成卫星可用性和残差信息
- 需要给相对定位状态初始化提供先验

所以算法上，RTK 并不是“直接从双差开始”，而是：

**SPP 初始化 -> RTK 精化**

### 4.2 `pntpos()` 的主要计算步骤

`pntpos()` 的主链很清晰：

1. `satposs()` 计算卫星位置和钟差
2. `estpos()` 用伪距做位置估计
3. 可选 `raim_fde()` 做完整性剔除
4. `estvel()` 用多普勒估速度

对应位置：

- [`estpos(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:701>)
- [`raim_fde(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:789>)
- [`estvel(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:1806>)

### 4.3 单点定位的观测模型

单点定位的核心观测是伪距。  
伪距模型可以抽象成：

`P = rho + c(dt_r - dt_s) + iono + trop + code_bias + noise`

其中：

- `rho`：几何距离
- `dt_r`：接收机钟差
- `dt_s`：卫星钟差
- `iono`：电离层延迟
- `trop`：对流层延迟
- `code_bias`：码偏差/TGD/BGD 等

在代码里这些改正分别由下面函数处理：

- 电离层：
  - [`ionocorr(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:280>)
- 对流层：
  - [`tropcorr(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:330>)
- 组延迟/TGD：
  - [`gettgd(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:139>)

### 4.4 `rescode()` 的作用

单点定位真正组装伪距残差的位置是：

- [`rescode(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:356>)

它负责：

1. 对每颗卫星计算几何距离
2. 叠加电离层、对流层、钟差、码偏差等改正
3. 构造残差向量 `v`
4. 构造设计矩阵 `H`
5. 构造观测方差 `var`

从算法角度，`rescode()` 就是在做：

**把物理测量写成线性化观测方程**

### 4.5 `estpos()` 的作用

位置估计主函数是：

- [`estpos(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:701>)

它本质上是一个迭代最小二乘流程：

1. 假设当前位置
2. 调 `rescode()` 得到残差和雅可比
3. 解线性增量
4. 更新位置和钟差
5. 直到收敛

所以 `estpos()` 解决的是：

**给定一组伪距，求接收机位置和时钟参数**

### 4.6 `estvel()` 的作用

速度不是从伪距中直接来的，而主要来自多普勒：

- [`estvel(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:1806>)

当前仓库里这个函数做了几件值得注意的事：

- 用多普勒构建速度残差
- 做鲁棒最小二乘
- 输出速度协方差

文中可以把它理解成：

**位置靠伪距，速度靠多普勒**

---

## 5. 第三步：进入 RTK 相对定位 `relpos()`

当 `pntpos()` 给出 rover 初值后，`rtkpos()` 继续调用：

- [`relpos(rtk, obs, nu, nr, nav)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3461>)

`relpos()` 是 RTK 算法核心。它做的不是简单的最小二乘，而是：

- 维护一个跨历元的状态向量
- 对每个历元进行预测与更新
- 同时估计位置、速度、模糊度、部分大气参数等

---

## 6. RTK 状态向量是什么

在 `rtkpos.cpp` 里，状态量规模由一组宏定义控制：

- [`NP(opt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:84>)
- [`NI(opt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:85>)
- [`NT(opt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:86>)
- [`NL(opt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:87>)
- [`NB(opt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:88>)
- [`NX(opt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:89>)

它们的含义是：

- `NP`：位置/速度/加速度状态数
- `NI`：电离层状态数
- `NT`：对流层状态数
- `NL`：GLONASS 频间偏差状态数
- `NB`：载波相位模糊度状态数
- `NX`：总状态维度

### 6.1 当前位置模式下的核心状态

对于当前这个仓库常见的 RTK 模式，最重要的状态一般包括：

1. 位置 `x y z`
2. 速度 `vx vy vz`
3. 加速度 `ax ay az`，如果启用 dynamics
4. 各卫星各频点的相位模糊度
5. 可选的大气和硬件偏差状态

所以当前仓库的 RTK 算法本质是一个：

**大状态卡尔曼滤波器**

---

## 7. 第四步：`relpos()` 的主流程

从代码上看，`relpos()` 的一轮历元计算可以拆成下面几步。

### 7.1 计算基站零差残差 `zdres(base=1)`

位置：

- [`zdres(1, obs + nu, nr, ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2704>)

这一步的作用是：

- 用基站已知坐标
- 计算基站对每颗卫星的零差观测模型
- 得到基站侧的几何和改正信息

### 7.2 选择 rover/base 共同可见卫星 `selsat()`

位置：

- [`selsat(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2724>)

这一步非常关键，因为双差必须建立在：

- rover 和 base 同时都观测到
- 满足高度角、健康状态、系统配置约束

的共同卫星上。

算法上，这一步相当于：

**构建本历元可用卫星集合**

### 7.3 进行状态传播 `udstate()`

位置：

- [`udstate(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1250>)
- 调用位置：
  - [`udstate(rtk, obs, sat, iu, ir, ns, nav)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2738>)

`udstate()` 内部又包括：

- `udpos()`：传播位置/速度/加速度
- `udion()`：传播电离层状态
- `udtrop()`：传播对流层状态
- `udbias()`：传播模糊度状态

对应位置：

- [`udpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:579>)
- [`udion()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:827>)
- [`udtrop()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:859>)
- [`udbias()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1108>)

从滤波角度讲，这一步就是：

**先验预测**

也就是把上一历元的状态和协方差，传播成当前历元的先验。

---

## 8. 周跳检测为什么放在状态传播里

`udbias()` 内部会做周跳检测。  
当前仓库用了多种周跳线索：

- LLI 检测：
  - [`detslp_ll()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:921>)
- 几何无关组合检测：
  - [`detslp_gf()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:980>)
- 多普勒辅助检测：
  - [`detslp_dop()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1044>)

这一步的重要性在于：

- 模糊度状态是时间连续的
- 一旦周跳没有被及时检测，模糊度状态就会被错误继承

所以算法上，周跳检测不是附属功能，而是 **状态正确传播的前提**。

---

## 9. 第五步：构建 rover 零差残差 `zdres(base=0)`

在滤波迭代内，会重新对 rover 计算零差残差：

- [`zdres(0, obs, nu, ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2790>)

`zdres()` 位于：

- [`rtklib/src/rtkpos.cpp:1359`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1359>)

它的作用是计算：

- 伪距零差残差
- 载波相位零差残差
- 当前几何方向向量
- 各频点频率、方位角、高度角

如果把单点定位阶段的 `rescode()` 看作“单站伪距模型”，  
那么 `zdres()` 就是“RTK 使用的单站多观测模型准备器”。

---

## 10. 第六步：构建双差残差 `ddres()`

位置：

- [`ddres(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1630>)
- 调用位置：
  - [`ddres(... xp, Pp, sat, y, e, azel, freq, iu, ir, ns, v, H, R, vflg)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2808>)

这是 RTK 数学模型最关键的一步。

### 10.1 为什么要做双差

双差的目的，是尽量消掉公共误差：

- 接收机钟差
- 卫星钟差
- 部分电离层/对流层公共分量

从物理上看，双差就是：

1. 先对 rover/base 做单差
2. 再在两个卫星之间做差

最后留下更“干净”的几何信息和模糊度信息。

### 10.2 `ddres()` 实际产物

`ddres()` 的输出包括：

- `v`：双差残差向量
- `H`：对状态量的偏导矩阵
- `R`：观测噪声协方差
- `vflg`：每条残差对应的卫星/频点标记

这一步之后，滤波器就拿到了标准的线性化量测模型：

`v = H * dx + noise`

---

## 11. 第七步：卡尔曼滤波更新

在 `relpos()` 里，完成 `ddres()` 之后，马上进入滤波更新：

- [`filter(xp, Pp, H, v, R, rtk->nx, nv)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2825>)

算法上就是标准线性高斯更新：

- 输入：
  - 先验状态 `xp`
  - 先验协方差 `Pp`
  - 量测残差 `v`
  - 观测矩阵 `H`
  - 量测噪声 `R`
- 输出：
  - 更新后状态
  - 更新后协方差

文档层面可以把这一段理解成：

**预测靠状态模型，校正靠双差观测**

---

## 12. 第八步：后验一致性检查 `valpos()`

滤波更新后，还要做一次后验检验：

- [`valpos(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2560>)
- 调用位置：
  - [`valpos(rtk, v, R, vflg, nv, 4.0)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2879>)

它的作用是检查：

- 更新后残差是否还在合理范围内
- 是否存在明显异常测量

如果这里不过，当前历元解可能被降级为 `SOLQ_NONE` 或保留为较差状态。

所以它相当于：

**滤波结果的质量门**

---

## 13. 第九步：整周模糊度固定

如果当前历元还是 float 解，就会进入模糊度固定阶段：

- [`manage_amb_LAMBDA(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2443>)
- 调用位置：
  - [`if (manage_amb_LAMBDA(rtk, bias, xa, sat, nf, ns) > 1)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2942>)

内部核心函数是：

- [`resamb_LAMBDA(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2260>)

### 13.1 固定流程的本质

固定流程通常可以概括为：

1. 从 float 协方差中抽取模糊度子空间
2. 用 LAMBDA 搜索最近整数候选
3. 做 ratio 检验
4. 如果通过，就把模糊度从 float 转成 integer
5. 得到 fixed 状态 `xa`

这一步是 RTK 为什么能从分米级跳到厘米级的关键。

### 13.2 固定后还要再做一次残差检验

固定成功后，代码没有立刻宣布 success，而是又做了：

1. `zdres(..., xa, ...)`
2. `ddres(..., xa, ...)`
3. `valpos(...)`

对应位置：

- [`zdres(... xa ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2946>)
- [`ddres(... xa ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2950>)
- [`valpos(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2954>)

也就是说，**固定解必须在后验残差上也说得过去，才会真正被接受。**

---

## 14. 第十步：Fix-and-Hold 是怎么起作用的

当前配置使用的是 `fix-and-hold`，对应代码会在满足连续固定次数后调用：

- [`holdamb(rtk, xa)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2961>)

函数定义：

- [`holdamb(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2135>)

它的含义是：

- 把已经固定成功的整数模糊度，以软约束方式回灌到滤波状态中
- 让后续历元更容易继续保持 fix

从算法角度，这一步相当于：

**把“已经证明可靠的整数结果”变成未来历元的先验约束**

所以 fix-and-hold 通常更稳，但也要求周跳检测和异常值控制足够可靠。

---

## 15. 当前仓库里的解状态怎么产生

`relpos()` 最后会根据 float / fix 结果，把最终状态写回：

- float 解写回 `rtk->x / rtk->P`
- fixed 解写回 `rtk->xa / rtk->Pa`

关键位置：

- float 解：
  - [`matcpy(rtk->x, xp, ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2883>)
- fix 解：
  - [`rtk->sol.rr[i] = rtk->xa[i]`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3060>)

最终解状态有几种典型值：

- `SOLQ_NONE`
- `SOLQ_DGPS`
- `SOLQ_FLOAT`
- `SOLQ_FIX`

实际 `.pos` 文件里常见的就是：

- `Q=2`：float
- `Q=1`：fix

---

## 16. 单点定位和 RTK 的关系

很多人第一次看 RTKLIB 源码时，会误以为：

- `pntpos()` 是单独模式
- `relpos()` 是另一个独立系统

实际上不是。  
在当前实现里，它们是串联关系：

1. `pntpos()` 先给 rover 初值和速度
2. `relpos()` 再在 rover/base 公共卫星上构建差分模型
3. `relpos()` 输出的 float/fix 解覆盖 `pntpos()` 的粗解

所以可以把它理解为：

- `pntpos()`：粗定位与初始化层
- `relpos()`：精定位与整数固定层

---

## 17. 当前仓库在算法上的几个定制点

这个仓库并不是原样使用 RTKLIB，在算法层做了一些值得单独指出的改动。

### 17.1 Doppler 约束被显式纳入配置

在封装层里：

- [`prcopt.DopplerConstraint`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:86>)
- [`prcopt.Dop2PrRatio`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:88>)

说明这个仓库把 Doppler 观测显式作为约束的一部分来调权。

### 17.2 速度估计做了增强

在 `pntpos.cpp` 中，除了标准多普勒速度，还扩展了：

- 单差 Doppler 速度
- TDCP 速度
- SD-TDCP 速度

对应函数：

- [`estvel_sd()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:1874>)
- [`estvel_td()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2027>)
- [`estvel_td_sd()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2141>)

虽然当前主链默认还是 `estvel()`，但这些扩展说明项目作者在速度估计上做过深入尝试。

### 17.3 鲁棒估计

在 `relpos()` 的滤波更新前，可以看到：

- [`if (opt->robustopt > RMODE_NONE) { adap = robust(...); }`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2820>)

这说明当前仓库支持一定程度的鲁棒处理，而不仅仅是标准高斯假设。

### 17.4 GNSS 因子中间量输出

在 `relpos()` 里，会把零差和双差相关信息打包成：

- `GNSS_Info_ZD`
- `GNSS_Info_SD`
- `GNSS_Info`

并发布到：

- [`pub_raw.publish(epoch)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3168>)

这一步不影响 RTK 解本身，但对后续图优化非常关键。

---

## 18. 从数学结构看一遍完整算法链

可以把当前仓库里的 RTKLIB 算法链压缩成下面这个顺序：

### 阶段 A：单点初始化

1. 读取当前历元观测
2. 根据导航电文算卫星位置与钟差
3. 建立伪距观测模型
4. 迭代最小二乘求 rover 粗位置
5. 用多普勒求 rover 速度

### 阶段 B：差分建模

6. 用已知 base 坐标构建 base 零差残差
7. 选共同可见卫星
8. 构建 rover 零差残差
9. 构建双差残差和观测矩阵

### 阶段 C：状态估计

10. 传播上一个历元状态
11. 检测周跳并重置相关模糊度
12. 用双差观测做卡尔曼更新
13. 检查后验残差一致性

### 阶段 D：整数固定

14. 提取模糊度子空间
15. LAMBDA 搜索整数候选
16. ratio 检验
17. 固定解复检
18. 通过则输出 `fix`
19. 必要时执行 `holdamb`

---

## 19. 为什么 RTK 算法难点主要集中在 4 个地方

如果你要写算法说明，最值得重点展开的不是“所有函数都解释一遍”，而是下面 4 个点：

### 19.1 观测建模

难点在于：

- 多系统
- 多频点
- 多种偏差
- 广播/精密模型切换

### 19.2 周跳与模糊度管理

难点在于：

- 模糊度要跨历元连续
- 但周跳会打断连续性
- 一旦漏检，固定结果会严重错误

### 19.3 滤波稳定性

难点在于：

- 状态维度大
- 观测噪声不完全高斯
- 几何构型变化快
- 城市场景容易异常值很多

### 19.4 整数固定可靠性

难点在于：

- 固定错了比不固定更糟
- 所以需要 ratio、后验检验、hold 策略一起工作

---

## 20. 当前项目里最推荐的算法阅读顺序

如果你要继续深入源码，建议按下面顺序看：

1. `pntpos()`
   - 先理解单点定位如何构造伪距模型
2. `estpos()` + `rescode()`
   - 看单点残差和雅可比是怎么来的
3. `relpos()`
   - 看 RTK 主循环结构
4. `udstate()`
   - 看状态传播和周跳管理
5. `zdres()` + `ddres()`
   - 看双差观测模型
6. `filter()`
   - 看卡尔曼更新
7. `manage_amb_LAMBDA()` + `resamb_LAMBDA()`
   - 看整数固定
8. `holdamb()`
   - 看 fix-and-hold 机制

按这个顺序，比直接从 `rtkpos.cpp` 头读到尾更容易建立整体认知。

---

## 21. 与当前工程文档的关系

这份文档建议和下面那份一起看：

- 工程流程版：
  - [`RTKLIB_WORKFLOW.md`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/RTKLIB_WORKFLOW.md:1>)

推荐阅读顺序是：

1. 先读工程版，知道模块怎么串起来
2. 再读算法版，理解每个历元内部到底怎么算

这样最不容易把“程序结构”和“数学结构”混在一起。

---

## 22. 一句话总结

从算法角度看，当前仓库中的 RTKLIB 本质上是一个 **以单点定位初始化、以双差观测驱动、以卡尔曼滤波为主干、以 LAMBDA 整周固定为精度跃迁核心的多状态 GNSS 相对定位系统**；在此基础上，仓库又额外把零差/双差中间量结构化输出给后端图优化使用。
