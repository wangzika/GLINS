# RTKLIB 公式工作流程说明

本文档从 **数学公式** 的角度说明当前仓库中 RTKLIB 的 GNSS 定位流程。  
它是对下面两份文档的继续补充：

- 工程流程版：
  - [`RTKLIB_WORKFLOW.md`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/RTKLIB_WORKFLOW.md:1>)
- 算法流程版：
  - [`RTKLIB_ALGORITHM_WORKFLOW.md`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/RTKLIB_ALGORITHM_WORKFLOW.md:1>)

本文重点回答：

- 单点定位的观测方程长什么样
- RTK 的零差、单差、双差分别是什么
- 状态向量和滤波方程怎么写
- 整周模糊度固定的输入输出是什么
- 这些公式在当前仓库里由哪些函数实现

注意：本文是为了帮助你写文档和讲清思路，公式写法会采用 **标准 GNSS/RTK 表达**，而不是逐行复述源码中的变量命名。

---

## 1. 一句话概括

当前仓库中的 RTKLIB 数学主链可以概括为：

1. 用广播星历计算卫星位置与钟差
2. 用伪距方程做单点定位，得到 rover 初值
3. 在 rover/base 之间构建零差和双差观测方程
4. 用卡尔曼滤波估计位置、速度、模糊度等状态
5. 用 LAMBDA 把 float 模糊度固定成整数
6. 输出 float/fix 解并向后端提供 GNSS 因子信息

---

## 2. 记号说明

为了统一公式，本文使用下面的记号。

### 2.1 接收机与卫星

- `r`：rover 接收机
- `b`：base 接收机
- `s`：某颗卫星
- `q`：参考卫星
- `f`：频点索引

### 2.2 主要变量

- `P`：伪距观测
- `L`：载波相位观测，以米表示时通常写成 `\lambda \Phi`
- `D`：多普勒观测
- `\rho`：几何距离
- `c`：光速
- `\delta t_r`：接收机钟差
- `\delta t_s`：卫星钟差
- `I`：电离层延迟
- `T`：对流层延迟
- `N`：整周模糊度
- `\lambda`：载波波长
- `\varepsilon`：测量噪声

### 2.3 差分算子

- 零差：原始单站观测
- 单差：
  - 接收机单差：`\nabla_{rb}`
  - 卫星单差：`\nabla_{sq}`
- 双差：
  - `\nabla\Delta = \nabla_{sq}\nabla_{rb}`

---

## 3. 第一步：卫星位置与钟差计算

在任一历元 `t`，RTKLIB 先根据导航电文计算：

- 卫星位置 `\mathbf{x}_s(t)`
- 卫星速度 `\mathbf{v}_s(t)`
- 卫星钟差 `\delta t_s(t)`

几何距离定义为：

\[
\rho_r^s = \left\| \mathbf{x}_s - \mathbf{x}_r \right\|
\]

其中：

- `\mathbf{x}_s` 是卫星位置
- `\mathbf{x}_r` 是接收机位置

在当前仓库里的实现入口是：

- [`satposs()` in `pntpos.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2307>)
- [`satposs()` in `rtkpos.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2662>)

---

## 4. 单点定位的伪距观测方程

单点定位主要使用伪距观测。标准伪距模型可写成：

\[
P_r^s
=
\rho_r^s
+
c(\delta t_r-\delta t_s)
+
T_r^s
+
I_r^s
+
b_{P,r}^s
+
\varepsilon_{P,r}^s
\]

其中：

- `P_r^s`：接收机 `r` 对卫星 `s` 的伪距观测
- `\rho_r^s`：几何距离
- `c(\delta t_r-\delta t_s)`：接收机与卫星钟差项
- `T_r^s`：对流层延迟
- `I_r^s`：电离层延迟
- `b_{P,r}^s`：码偏差、TGD/BGD 等系统误差
- `\varepsilon_{P,r}^s`：噪声与未建模误差

### 4.1 线性化形式

对接收机状态在当前近似点 `\mathbf{x}_0` 处线性化，可以写成：

\[
v = y - h(\mathbf{x}_0) \approx H \, \Delta \mathbf{x} + \varepsilon
\]

更具体地，对于某颗卫星：

\[
v_P^s
=
P_r^s
- \hat{\rho}_r^s
- c(\delta \hat{t}_r-\delta \hat{t}_s)
- \hat{T}_r^s
- \hat{I}_r^s
- \hat{b}_{P,r}^s
\]

其中几何部分对位置的偏导是视线方向：

\[
\frac{\partial \rho}{\partial \mathbf{x}_r}
=
-\mathbf{e}_r^s
=
-\frac{\mathbf{x}_s-\mathbf{x}_r}{\|\mathbf{x}_s-\mathbf{x}_r\|}
\]

所以设计矩阵中位置部分的每一行，本质上就是视线向量的负号。

### 4.2 对应到当前仓库的实现

- 伪距改正项计算：
  - [`ionocorr()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:280>)
  - [`tropcorr()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:330>)
  - [`gettgd()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:139>)
- 残差和设计矩阵构造：
  - [`rescode()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:356>)
- 单点位置求解：
  - [`estpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:701>)

---

## 5. 单点定位的最小二乘问题

把所有卫星的伪距残差叠起来，可写成：

\[
\mathbf{v}
=
\mathbf{H}\Delta\mathbf{x}
+
\boldsymbol{\varepsilon}
\]

加权最小二乘问题是：

\[
\min_{\Delta\mathbf{x}}
\quad
\mathbf{v}^T \mathbf{W} \mathbf{v}
\]

其中 `\mathbf{W} = \mathbf{R}^{-1}`，`R` 是观测协方差矩阵。

正规方程写成：

\[
(\mathbf{H}^T\mathbf{W}\mathbf{H}) \Delta\mathbf{x}
=
\mathbf{H}^T\mathbf{W}\mathbf{v}
\]

解得：

\[
\Delta\mathbf{x}
=
(\mathbf{H}^T\mathbf{W}\mathbf{H})^{-1}
\mathbf{H}^T\mathbf{W}\mathbf{v}
\]

更新：

\[
\mathbf{x}_{k+1}
=
\mathbf{x}_k + \Delta\mathbf{x}
\]

直到 `\|\Delta\mathbf{x}\|` 足够小为止。

这就是 `estpos()` 的数学本质。

---

## 6. 多普勒速度观测方程

当前仓库里的单点速度主要来自多普勒。  
多普勒本质上提供的是 **视线方向上的距离变化率**。

可以写成：

\[
\dot{\rho}_r^s
=
(\mathbf{v}_s-\mathbf{v}_r)^T \mathbf{e}_r^s
\]

多普勒观测可近似写为：

\[
D_r^s
\approx
-\frac{1}{\lambda}
\left(
(\mathbf{v}_s-\mathbf{v}_r)^T \mathbf{e}_r^s
+
c(\dot{\delta t}_r-\dot{\delta t}_s)
\right)
+
\varepsilon_D
\]

把它整理成速度未知量的线性模型，就能对：

- `v_x, v_y, v_z`
- 接收机钟漂

做最小二乘估计。

对应实现：

- [`estvel()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:1806>)

---

## 7. RTK 为什么要做差分

单点定位中，很多误差难以直接精确建模，例如：

- 接收机钟差
- 卫星钟差
- 部分电离层/对流层残差

RTK 的核心思想就是通过 **差分** 消掉公共项。

---

## 8. 零差观测方程

### 8.1 伪距零差

rover 端：

\[
P_r^s
=
\rho_r^s
+
c(\delta t_r-\delta t_s)
+
T_r^s
+
I_r^s
+
b_{P,r}^s
+
\varepsilon_{P,r}^s
\]

base 端：

\[
P_b^s
=
\rho_b^s
+
c(\delta t_b-\delta t_s)
+
T_b^s
+
I_b^s
+
b_{P,b}^s
+
\varepsilon_{P,b}^s
\]

### 8.2 载波相位零差

载波相位方程与伪距类似，但电离层符号相反，并多出模糊度项：

\[
\lambda \Phi_r^s
=
\rho_r^s
+
c(\delta t_r-\delta t_s)
+
T_r^s
-
I_r^s
+
\lambda N_r^s
+
b_{\Phi,r}^s
+
\varepsilon_{\Phi,r}^s
\]

base 端同理。

在当前仓库中，零差残差构造对应：

- [`zdres()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1359>)

---

## 9. 单差观测方程

对 rover/base 做接收机单差：

\[
\nabla_{rb} P^s
=
P_r^s - P_b^s
\]

代入后可得：

\[
\nabla_{rb} P^s
=
(\rho_r^s-\rho_b^s)
+
c(\delta t_r-\delta t_b)
+
(T_r^s-T_b^s)
+
(I_r^s-I_b^s)
+
\nabla_{rb} b_P^s
+
\nabla_{rb}\varepsilon_P^s
\]

可以看到：

- 卫星钟差 `\delta t_s` 被消掉了

对载波相位做同样的接收机单差：

\[
\nabla_{rb} (\lambda \Phi^s)
=
(\rho_r^s-\rho_b^s)
+
c(\delta t_r-\delta t_b)
+
(T_r^s-T_b^s)
-
(I_r^s-I_b^s)
+
\lambda \nabla_{rb}N^s
+
\nabla_{rb} b_\Phi^s
+
\nabla_{rb}\varepsilon_\Phi^s
\]

---

## 10. 双差观测方程

再对两个卫星 `s` 与 `q` 做卫星单差，得到双差。

### 10.1 伪距双差

\[
\nabla\Delta P^{sq}_{rb}
=
\nabla_{sq}\nabla_{rb} P
\]

展开后：

\[
\nabla\Delta P^{sq}_{rb}
=
(\rho_r^s-\rho_b^s-\rho_r^q+\rho_b^q)
+
\nabla\Delta T^{sq}_{rb}
+
\nabla\Delta I^{sq}_{rb}
+
\nabla\Delta b_P^{sq}
+
\nabla\Delta \varepsilon_P^{sq}
\]

可以看到双差之后：

- 接收机钟差被消掉
- 卫星钟差也被消掉

### 10.2 载波相位双差

\[
\nabla\Delta (\lambda \Phi)^{sq}_{rb}
=
(\rho_r^s-\rho_b^s-\rho_r^q+\rho_b^q)
+
\nabla\Delta T^{sq}_{rb}
-
\nabla\Delta I^{sq}_{rb}
+
\lambda \nabla\Delta N^{sq}_{rb}
+
\nabla\Delta b_\Phi^{sq}
+
\nabla\Delta \varepsilon_\Phi^{sq}
\]

这就是 RTK 中最重要的观测方程。  
它把未知整数模糊度直接引入到了模型中。

### 10.3 在当前仓库中的实现位置

- 共同卫星选择：
  - [`selsat()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:555>)
- 双差残差构造：
  - [`ddres()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1630>)

---

## 11. 为什么双差后还需要状态滤波

因为双差方程里的未知量不止一个：

- 当前位置
- 速度/加速度
- 各卫星各频点模糊度
- 可能还有电离层、对流层、GLONASS 偏差

而且这些量在时间上是相关的。  
所以不能只在每个历元做一次独立最小二乘，更适合用 **递推状态估计**。

---

## 12. RTK 状态向量的公式表达

在当前仓库常见的动态 RTK 模式下，可以把状态向量抽象成：

\[
\mathbf{x}
=
\begin{bmatrix}
\mathbf{p} \\
\mathbf{v} \\
\mathbf{a} \\
\mathbf{i} \\
\mathbf{t} \\
\mathbf{b}_{\text{glo}} \\
\mathbf{N}
\end{bmatrix}
\]

其中：

- `\mathbf{p}`：位置 `x,y,z`
- `\mathbf{v}`：速度 `v_x,v_y,v_z`
- `\mathbf{a}`：加速度 `a_x,a_y,a_z`
- `\mathbf{i}`：电离层状态
- `\mathbf{t}`：对流层状态
- `\mathbf{b}_{\text{glo}}`：GLONASS 频间偏差
- `\mathbf{N}`：模糊度状态

在当前代码里，状态维度由宏控制：

- [`NP/NI/NT/NL/NB/NX`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:83>)

---

## 13. 状态预测方程

### 13.1 位置、速度、加速度传播

如果启用了动态模型，则通常采用匀加速度近似：

\[
\mathbf{p}_{k|k-1}
=
\mathbf{p}_{k-1|k-1}
+
\mathbf{v}_{k-1|k-1}\Delta t
+
\frac{1}{2}\mathbf{a}_{k-1|k-1}\Delta t^2
\]

\[
\mathbf{v}_{k|k-1}
=
\mathbf{v}_{k-1|k-1}
+
\mathbf{a}_{k-1|k-1}\Delta t
\]

\[
\mathbf{a}_{k|k-1}
\approx
\mathbf{a}_{k-1|k-1}
+
\mathbf{w}_a
\]

其中 `\mathbf{w}_a` 是过程噪声。

这部分对应：

- [`udpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:579>)

### 13.2 电离层和对流层传播

如果启用了估计型电离层/对流层模型，则通常采用随机游走：

\[
i_k = i_{k-1} + w_i
\]

\[
t_k = t_{k-1} + w_t
\]

对应：

- [`udion()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:827>)
- [`udtrop()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:859>)

### 13.3 模糊度传播

模糊度在无周跳条件下通常看作常值：

\[
N_k = N_{k-1} + w_N
\]

一旦发生周跳，则对应模糊度状态需要重置。

对应：

- [`udbias()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1108>)

---

## 14. 周跳检测的数学意义

周跳的本质是：

\[
N_k \neq N_{k-1}
\]

也就是载波相位观测中的整数模糊度发生突变。  
如果不检测出来，滤波器会把错误的 `N` 当成连续状态继承下去。

当前仓库里主要用了 3 类信息：

1. LLI 标志
2. 几何无关组合
3. 多普勒与相位差一致性

对应函数：

- [`detslp_ll()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:921>)
- [`detslp_gf()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:980>)
- [`detslp_dop()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1044>)

---

## 15. 滤波量测更新方程

双差残差构造完成后，可写成标准线性化量测模型：

\[
\mathbf{v}_k
=
\mathbf{H}_k \Delta \mathbf{x}_k
+
\boldsymbol{\varepsilon}_k
\]

观测噪声协方差：

\[
\boldsymbol{\varepsilon}_k \sim \mathcal{N}(0,\mathbf{R}_k)
\]

### 15.1 预测

\[
\mathbf{x}_{k|k-1} = \mathbf{F}_k \mathbf{x}_{k-1|k-1}
\]

\[
\mathbf{P}_{k|k-1}
=
\mathbf{F}_k \mathbf{P}_{k-1|k-1}\mathbf{F}_k^T
+
\mathbf{Q}_k
\]

### 15.2 更新

卡尔曼增益：

\[
\mathbf{K}_k
=
\mathbf{P}_{k|k-1}\mathbf{H}_k^T
\left(
\mathbf{H}_k \mathbf{P}_{k|k-1}\mathbf{H}_k^T + \mathbf{R}_k
\right)^{-1}
\]

状态更新：

\[
\mathbf{x}_{k|k}
=
\mathbf{x}_{k|k-1}
+
\mathbf{K}_k \mathbf{v}_k
\]

协方差更新：

\[
\mathbf{P}_{k|k}
=
(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}_{k|k-1}
\]

在当前仓库里，这一步的核心实现是：

- [`filter(xp, Pp, H, v, R, rtk->nx, nv)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2825>)

---

## 16. 后验残差检验

滤波更新后，需要检查：

- 当前残差是否仍在合理统计范围内
- 是否存在明显失配的观测

这在代码中由：

- [`valpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2560>)

完成。

可以把它理解成检验：

\[
\mathbf{v}^T \mathbf{R}^{-1} \mathbf{v}
\]

是否落在允许范围内，或者各残差是否明显超过阈值。

---

## 17. float 模糊度固定问题

滤波更新后的模糊度一般是实数解：

\[
\hat{\mathbf{N}} \in \mathbb{R}^m
\]

对应协方差为：

\[
\mathbf{Q}_{\hat{N}\hat{N}}
\]

RTK 的关键就是：  
尝试把它变成整数向量

\[
\mathbf{N} \in \mathbb{Z}^m
\]

这就是 **整数最小二乘问题**。

---

## 18. LAMBDA 的数学目标

LAMBDA 的目标可以写成：

\[
\min_{\mathbf{N}\in\mathbb{Z}^m}
\quad
(\hat{\mathbf{N}}-\mathbf{N})^T
\mathbf{Q}_{\hat{N}\hat{N}}^{-1}
(\hat{\mathbf{N}}-\mathbf{N})
\]

它会在整数格点中寻找最可能的模糊度候选。

当前仓库的关键实现位置：

- [`resamb_LAMBDA()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2260>)
- [`manage_amb_LAMBDA()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2443>)

---

## 19. ratio 检验的意义

LAMBDA 一般会给出：

- 最优整数候选
- 次优整数候选

设它们对应目标函数值为 `s_1, s_2`，则 ratio 常写成：

\[
\text{ratio} = \frac{s_2}{s_1}
\]

直觉上：

- `s_1` 越小越好
- `s_2/s_1` 越大，说明最优整数解和次优解区分越明显

只有当 ratio 超过阈值时，才认为固定足够可靠。

这也是 `.pos` 文件里 `ratio` 字段的来源之一。

---

## 20. 固定解的状态回代

当整数模糊度固定成功后，需要从 float 状态得到 fixed 状态：

\[
\mathbf{x}^{a}
=
\mathbf{x}^{f}
-
\mathbf{Q}_{xN}\mathbf{Q}_{NN}^{-1}
(\hat{\mathbf{N}}-\mathbf{N})
\]

其中：

- `\mathbf{x}^{f}`：float 状态
- `\mathbf{x}^{a}`：fixed 状态
- `\mathbf{Q}_{xN}`：非模糊度状态与模糊度状态的互协方差
- `\mathbf{Q}_{NN}`：模糊度协方差

固定后的位置、速度、协方差都会变得更紧。

在当前实现中，fixed 解会写入：

- `rtk->xa`
- `rtk->Pa`

对应结果赋值位置：

- [`rtk->sol.rr[i] = rtk->xa[i]`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3060>)

---

## 21. Fix-and-Hold 的公式理解

Fix-and-Hold 可以看成在后续历元中增加一个伪量测：

\[
\mathbf{H}_{hold}\mathbf{x}
=
\mathbf{z}_{hold}
+
\boldsymbol{\eta}
\]

其中 `\mathbf{z}_{hold}` 表示已经固定成功的整数关系，  
`R_{hold}` 由 hold 过程中的人工方差设定。

这样做的效果是：

- 把过去成功固定得到的整数关系继续施加到当前状态上
- 提高后续历元维持 fix 的稳定性

对应实现：

- [`holdamb()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2135>)

---

## 22. 当前仓库的速度扩展公式

当前仓库除了基础多普勒速度，还尝试了：

- TDCP
- 单差多普勒
- 单差 TDCP

例如 TDCP 的核心思想是：

\[
\Delta (\lambda \Phi)
\approx
\Delta \rho
+
\Delta T
-
\Delta I
+
\lambda \Delta N
\]

如果两个相邻历元之间没有周跳，则 `\Delta N = 0`，  
那么相邻历元的载波相位差可以非常精确地反映位移或速度。

对应函数：

- [`estvel_td()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2027>)
- [`estvel_td_sd()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2141>)

虽然当前主链默认没有完全启用这些分支，但它们反映了仓库作者在速度估计上的增强方向。

---

## 23. 当前仓库的 GNSS 因子输出在数学上对应什么

当前仓库会把：

- 零差信息 `GNSS_Info_ZD`
- 双差信息 `GNSS_Info_SD`

打包输出给后端。

这在数学上对应于：

- `GNSS_Info_ZD`：保留每颗卫星的单站观测与几何关系
- `GNSS_Info_SD`：保留 rover/base 之间可形成 RTK 因子的差分观测关系

也就是说，RTKLIB 不只是算出一个点位结果，而是把：

**构造因子图所需的观测约束原料**

也整理了出来。

对应输出位置：

- [`GNSS_Info_ZD` 构造](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2673>)
- [`GNSS_Info_SD` 构造](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2977>)
- [`pub_raw.publish(epoch)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3168>)

---

## 24. 写论文/汇报时最推荐的公式组织方式

如果你要把 RTKLIB 工作流写进论文、开题、汇报或项目说明，最推荐按下面顺序写：

### 第一部分：单点定位

1. 卫星位置与钟差计算
2. 伪距观测方程
3. 电离层/对流层/钟差改正
4. 加权最小二乘求位置
5. 多普勒求速度

### 第二部分：RTK 相对定位

6. 零差观测方程
7. 单差观测方程
8. 双差观测方程
9. 状态向量定义
10. 卡尔曼预测与更新

### 第三部分：整数固定

11. float 模糊度和协方差提取
12. LAMBDA 整数搜索
13. ratio 检验
14. fixed 解回代
15. fix-and-hold

这样组织最符合“从观测到状态，再到整数约束”的逻辑。

---

## 25. 这份公式版和前两份文档的关系

三份文档分别负责不同层面：

- 工程版：
  - 程序和模块怎么串起来
- 算法版：
  - 每个函数在干什么
- 公式版：
  - 每一步背后的数学模型是什么

如果后面你要整理成正式文档，最好的组合方式通常是：

1. 用工程版做章节骨架
2. 用算法版做实现说明
3. 用公式版做理论依据

---

## 26. 一句话总结

从公式角度看，当前仓库中的 RTKLIB 工作流，本质上是：  
**以伪距单点定位给出初值，以零差/双差观测建立 RTK 状态方程，以卡尔曼滤波递推位置和模糊度，以 LAMBDA 将 float 模糊度整数化，并在此基础上向后端图优化提供结构化 GNSS 观测因子。**
