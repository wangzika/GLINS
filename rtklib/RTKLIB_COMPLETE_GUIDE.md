# RTKLIB 总体工作流程与算法说明

本文档是当前仓库中 RTKLIB 相关说明的总文档，目标是把前面已经拆开的三层内容重新组织成一份可直接用于：

- 项目文档
- 组会汇报
- 开题/中期/结题材料
- 论文“系统实现”与“算法原理”章节草稿

它整合了三类视角：

1. 工程视角：程序是怎么启动、配置、读取数据、输出结果的
2. 算法视角：每个历元里先算什么、后算什么
3. 公式视角：伪距、双差、卡尔曼滤波、模糊度固定的数学形式

如果你只想先建立整体认知，读这一份就够。  
如果后续还要继续深挖源码细节，可以再配合下面三份分层文档一起看：

- 工程版：
  - [`RTKLIB_WORKFLOW.md`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/RTKLIB_WORKFLOW.md:1>)
- 算法版：
  - [`RTKLIB_ALGORITHM_WORKFLOW.md`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/RTKLIB_ALGORITHM_WORKFLOW.md:1>)
- 公式版：
  - [`RTKLIB_FORMULA_WORKFLOW.md`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/RTKLIB_FORMULA_WORKFLOW.md:1>)

---

## 1. 先给出总图

当前仓库里 RTKLIB 的完整链路可以压缩成下面这条主线：

**配置文件 -> `gnssProcessor` -> `postpos()` -> 读 RINEX 观测/导航 -> 按历元执行 `pntpos()` + `relpos()` -> 输出 `rtklib.pos` -> 发布 `/gnss_raw` 与 `/rtklib_odom` -> `gnssEstimator` 继续做后端优化**

如果按数据文件来理解，就是：

`rover.obs + base.obs + BRDM...rnx -> RTKLIB 后处理 -> rtklib.pos + GNSS_Info`

这条链路的关键点在于：

- RTKLIB 在当前仓库里不只是“算一个坐标”
- 它同时也是 GNSS 前端求解器和 GNSS 因子生成器

所以你可以把当前仓库里的 RTKLIB 理解成：

**GNSS 前端引擎**

---

## 2. RTKLIB 在当前项目中的角色

在标准理解里，RTKLIB 常常只是一个独立工具，比如：

- `rtkpost`
- `rnx2rtkp`
- `rtknavi`

但在这个项目里，它被直接嵌入到了 GLINS 的系统流程中。  
因此它的角色发生了变化：

### 2.1 它负责读 GNSS 原始文件

包括：

- `rover.obs`
- `base.obs`
- `BRDM00DLR_S_20240290000_01D_MN.rnx`

### 2.2 它负责做 GNSS 前端定位

包括：

- 单点定位
- 差分定位
- RTK 相对定位
- 整周模糊度固定

### 2.3 它负责把 GNSS 中间量输送给后端

除了最终 `.pos` 文件，它还会发布：

- `/gnss_raw`
- `/rtklib_odom`

其中 `/gnss_raw` 里面不只是位置结果，还包含：

- 零差信息
- 双差信息
- 卫星状态
- 模糊度状态

这正是后端 `gnssEstimator` / 因子图优化所需要的输入。

---

## 3. 从程序入口开始看

当前仓库里最典型的 RTKLIB 启动入口在：

- [`glins/src/test_rtk.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:7>)

这个入口非常适合拿来讲“系统是怎么跑起来的”，因为主流程几乎是一目了然的。

它做了下面几件事：

1. 初始化 ROS 节点
2. 创建 `gnssProcessor`
3. 创建 `gnssEstimator`
4. 指定处理时间段
5. 调用 `processor.decode(ts, te)`
6. 调用 `estimator.solveOptimization()`

对应关键代码：

- [`processor.decode(ts, te)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:39>)
- [`estimator.solveOptimization()`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:40>)

所以从系统结构上，当前项目的 GNSS 处理明确分为两段：

- 前端：`gnssProcessor`
- 后端：`gnssEstimator`

---

## 4. 前端入口：`gnssProcessor`

`gnssProcessor` 定义在：

- [`glins/include/gnss/gnssProcessor.h`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:23>)

它本质上是对 RTKLIB 后处理接口的一层封装。

### 4.1 它做的第一件事是加载配置

相关代码：

- [`loadopts(rtklibConfigPath.c_str(), sysopts)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:59>)
- [`loadopts(rtklibConfigPath.c_str(), rcvopts)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:59>)
- [`getsysopts(&prcopt, &solopt, &filopt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:63>)

这一步决定：

- 定位模式
- 频点设置
- 使用哪些星座
- 电离层/对流层模型
- 模糊度固定策略
- 输入输出文件路径

### 4.2 它做的第二件事是收集输入文件路径

输入输出路径映射来自：

- [`strpath` 和 `rcvopts` 定义](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:7>)

最终对应为：

1. `inpstr1-path` -> `rover.obs`
2. `inpstr2-path` -> `base.obs`
3. `inpstr3-path` -> `BRDM...rnx`
4. `outstr1-path` -> `rtklib.pos`

### 4.3 它做的第三件事是调用 RTKLIB 核心入口

真正触发解算的是：

- [`postpos(ts, te, ti, tu, &prcopt, &solopt, &filopt, infile, n, outfile, rov, base)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:106>)

这里 `postpos()` 就是 RTKLIB 后处理总控入口。

---

## 5. 配置文件如何影响整个流程

当前数据集最直接使用的配置文件是：

- [`glins/config/conf/20240129.conf`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:1>)

你在解释 RTKLIB 时，最好把配置参数分成下面几组来讲。

### 5.1 定位模式

- [`pos1-posmode = kinematic`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:3>)
- [`pos1-soltype = forward`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:5>)

它决定：

- 这是动态 RTK，而不是静态或 PPP
- 解算方向是前向，而不是前后向组合

### 5.2 星座与频率

- [`pos1-frequency = l1+l2+l5`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:4>)
- [`pos1-navsys = 41`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:24>)

`41 = 1 + 8 + 32`，表示使用：

- GPS
- Galileo
- BDS

### 5.3 误差模型

- [`pos1-ionoopt = brdc`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:14>)
- [`pos1-tropopt = saas`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:15>)
- [`pos1-sateph = brdc`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:16>)

表示：

- 电离层采用广播模型
- 对流层采用 Saastamoinen
- 卫星轨道采用广播星历

### 5.4 模糊度固定

- [`pos2-armode = 3`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:28>)

也就是：

- `Fix-and-Hold`

### 5.5 输入输出文件

- [`inpstr1-path`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:135>)
- [`inpstr2-path`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:136>)
- [`inpstr3-path`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:137>)
- [`outstr1-path`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:139>)

从系统角度说，这个配置文件就是 RTKLIB 工作流的“任务定义文件”。

---

## 6. RTKLIB 后处理总控：`postpos()`

`postpos()` 的声明在：

- [`rtklib/include/rtklib.h:1894`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/include/rtklib.h:1894>)

实现位于：

- [`rtklib/src/postpos.cpp:1677`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1677>)

它不直接负责所有细节计算，而是作为一个后处理调度器，组织以下步骤：

1. 打开处理会话
2. 读取观测和导航文件
3. 设置天线参数和站点位置
4. 创建输出文件并写表头
5. 按历元调用求解器
6. 输出 `.pos` / `.stat` / 事件文件
7. 关闭会话并释放资源

所以可以把 `postpos()` 理解成：

**RTKLIB 后处理模式的主控制器**

---

## 7. 数据读取流程

### 7.1 `readobsnav()` 是观测与导航读入核心

位置：

- [`readobsnav()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:957>)

它的核心工作是：

1. 读取所有输入文件
2. 把观测数据装入 `obs_t`
3. 把导航数据装入 `nav_t`
4. 把站点头信息装入 `sta_t`
5. 对观测按时间排序
6. 删除重复星历

关键调用：

- [`readrnxt(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:982>)
- [`sortobs(obs)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1003>)
- [`uniqnav(nav)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1006>)

### 7.2 `readrnxt()` 再往下怎么走

RINEX 读取逻辑主要在：

- [`rtklib/src/rinex.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rinex.cpp:1892>)

典型调用链是：

1. `readrnxt()`
2. `readrnxfile()`
3. `readrnxfp()`
4. `readrnxh()` 读取头
5. 分发到：
   - `readrnxobs()`
   - `readrnxnav()`

所以工程上，RTKLIB 先完成的是：

**文本文件 -> 内存结构体**

---

## 8. 历元循环是怎么开始的

后处理进入主循环的位置在：

- [`procpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:603>)

它的主循环写法非常清楚：

- [`while ((nobs = inputobs(...)) >= 0)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:623>)

也就是说，RTKLIB 是按“历元”驱动整个求解的：

1. 取一个 rover 历元
2. 找最匹配的 base 历元
3. 拼成当前历元观测集
4. 把这一组观测送进 `rtkpos()`

### 8.1 `inputobs()` 的作用

位置：

- [`inputobs()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:411>)

它负责：

- 找 rover 当前历元
- 找 base 最近历元
- 拼接成当前历元 `obsd_t[]`
- 更新 SBAS / SSR 改正

这一步非常重要，因为 RTK 算法依赖 rover/base 的同步或近同步观测。

---

## 9. 每个历元先进入 `rtkpos()`

真正的历元求解入口是：

- [`rtkpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3336>)

它是单个历元的定位总控函数。

`rtkpos()` 内部主线可以概括为：

1. 设置基站坐标
2. 区分 rover/base 观测
3. 调用 `pntpos()` 做 rover 单点定位
4. 如果是单点模式，直接结束
5. 如果是 PPP 模式，走 `pppos()`
6. 如果是 RTK 模式，继续调用 `relpos()`

所以算法结构上，`rtkpos()` 是：

**单历元调度器**

---

## 10. 单点定位：`pntpos()`

位置：

- [`pntpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2255>)

单点定位主要完成两件事：

1. 用伪距求粗位置
2. 用多普勒求速度

### 10.1 单点伪距模型

标准伪距方程写成：

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

- `\rho_r^s`：几何距离
- `\delta t_r`：接收机钟差
- `\delta t_s`：卫星钟差
- `T_r^s`：对流层延迟
- `I_r^s`：电离层延迟
- `b_{P,r}^s`：码偏差/TGD/BGD 等

相关改正函数：

- [`gettgd()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:139>)
- [`ionocorr()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:280>)
- [`tropcorr()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:330>)

### 10.2 单点残差构造：`rescode()`

位置：

- [`rescode()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:356>)

它做的是：

- 构造伪距残差向量 `v`
- 构造设计矩阵 `H`
- 构造观测方差

也就是把物理观测模型写成线性化形式：

\[
\mathbf{v}
=
\mathbf{H}\Delta\mathbf{x}
+
\boldsymbol{\varepsilon}
\]

### 10.3 单点位置估计：`estpos()`

位置：

- [`estpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:701>)

它本质上做的是加权最小二乘迭代：

\[
\Delta\mathbf{x}
=
(\mathbf{H}^T\mathbf{W}\mathbf{H})^{-1}\mathbf{H}^T\mathbf{W}\mathbf{v}
\]

然后更新状态：

\[
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta\mathbf{x}
\]

直到收敛。

### 10.4 单点速度估计：`estvel()`

位置：

- [`estvel()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:1806>)

它主要使用多普勒构建速度方程。  
多普勒观测近似对应：

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

所以可以对：

- 接收机速度
- 接收机钟漂

做最小二乘估计。

### 10.5 为什么 RTK 前要先做单点定位

因为后续 RTK 相对定位需要：

- rover 的粗位置初值
- 卫星可用性判断
- 方位角/高度角
- SNR、残差信息

所以流程不是“直接双差”，而是：

**SPP 初始化 -> RTK 精化**

---

## 11. RTK 相对定位：`relpos()`

位置：

- [`relpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2591>)

这是真正的 RTK 主战场。

它完成的事情包括：

1. 基站零差建模
2. 共同卫星选择
3. 状态传播
4. rover 零差建模
5. 双差残差构造
6. 卡尔曼更新
7. 后验检验
8. 整周模糊度固定
9. 输出 float/fix 解

---

## 12. RTK 的状态向量

在当前仓库里，状态量规模通过宏定义控制：

- [`NP/NI/NT/NL/NB/NX`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:83>)

抽象写法可以记成：

\[
\mathbf{x}
=
\begin{bmatrix}
\mathbf{p} \\
\mathbf{v} \\
\mathbf{a} \\
\mathbf{i} \\
\mathbf{t} \\
\mathbf{b}_{glo} \\
\mathbf{N}
\end{bmatrix}
\]

其中：

- `\mathbf{p}`：位置
- `\mathbf{v}`：速度
- `\mathbf{a}`：加速度
- `\mathbf{i}`：电离层状态
- `\mathbf{t}`：对流层状态
- `\mathbf{b}_{glo}`：GLONASS 偏差
- `\mathbf{N}`：模糊度

这就是为什么当前 RTK 解算不是一个小型最小二乘，而是一个多状态滤波问题。

---

## 13. 差分观测模型

### 13.1 零差

rover 载波相位模型：

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

在代码里，零差残差构造对应：

- [`zdres()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1359>)

### 13.2 接收机单差

对 rover/base 做接收机单差：

\[
\nabla_{rb}(\lambda \Phi^s)
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
\nabla_{rb}b_\Phi^s
+
\nabla_{rb}\varepsilon_\Phi^s
\]

可以看到：

- 卫星钟差被消掉了

### 13.3 双差

再对参考卫星 `q` 做卫星单差，得到双差：

\[
\nabla\Delta(\lambda \Phi)^{sq}_{rb}
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

伪距双差类似，只是没有模糊度项，且电离层符号不同。

### 13.4 双差残差构造：`ddres()`

位置：

- [`ddres()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1630>)

它的输出包括：

- `v`：双差残差
- `H`：设计矩阵
- `R`：量测协方差

也就是把 RTK 问题转成标准的滤波更新形式。

---

## 14. 共同卫星选择和基站建模

在 `relpos()` 中，先做基站零差：

- [`zdres(1, obs + nu, nr, ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2704>)

再做共同卫星选择：

- [`selsat()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:555>)
- 调用位置：
  - [`if ((ns = selsat(...)) <= 0)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2724>)

这一步决定：

- 哪些卫星能参与双差
- 哪些卫星能成为参考卫星候选

如果共同卫星不足，当前历元 RTK 就不成立。

---

## 15. 状态传播：`udstate()`

位置：

- [`udstate()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1250>)

它负责在量测更新前，把上一个历元的状态传播到当前历元。  
内部包括：

- [`udpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:579>)
- [`udion()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:827>)
- [`udtrop()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:859>)
- [`udbias()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1108>)

### 15.1 位置和速度传播

如果启用了 dynamics，则可以近似理解为匀加速度模型：

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

### 15.2 模糊度传播

若无周跳，则模糊度可近似看成常值：

\[
N_k = N_{k-1} + w_N
\]

一旦发生周跳，则模糊度状态要被重置。

---

## 16. 周跳检测

周跳检测在当前仓库里很重要，因为它直接决定模糊度状态是否连续。

主要函数有：

- [`detslp_ll()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:921>)
- [`detslp_gf()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:980>)
- [`detslp_dop()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:1044>)

从数学上，周跳意味着：

\[
N_k \neq N_{k-1}
\]

如果不处理，滤波器会把错误的整数状态继续往后传，导致 fix 失真。

---

## 17. 卡尔曼滤波更新

在 `relpos()` 中，构造完双差残差之后，会调用：

- [`filter(xp, Pp, H, v, R, rtk->nx, nv)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2825>)

### 17.1 预测

\[
\mathbf{x}_{k|k-1} = \mathbf{F}_k \mathbf{x}_{k-1|k-1}
\]

\[
\mathbf{P}_{k|k-1}
=
\mathbf{F}_k\mathbf{P}_{k-1|k-1}\mathbf{F}_k^T + \mathbf{Q}_k
\]

### 17.2 更新

\[
\mathbf{K}_k
=
\mathbf{P}_{k|k-1}\mathbf{H}_k^T
\left(
\mathbf{H}_k \mathbf{P}_{k|k-1}\mathbf{H}_k^T + \mathbf{R}_k
\right)^{-1}
\]

\[
\mathbf{x}_{k|k}
=
\mathbf{x}_{k|k-1} + \mathbf{K}_k \mathbf{v}_k
\]

\[
\mathbf{P}_{k|k}
=
(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}_{k|k-1}
\]

这一步本质上就是：

- 先验由状态模型给出
- 校正由双差观测给出

---

## 18. 后验残差检验：`valpos()`

更新之后，RTKLIB 不会立刻相信结果，而是再做一次质量检验：

- [`valpos()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2560>)

它的作用相当于检验：

\[
\mathbf{v}^T \mathbf{R}^{-1} \mathbf{v}
\]

是否仍然在可接受范围内，或者单个残差是否过大。

所以 `valpos()` 可以理解为：

**后验一致性门限**

---

## 19. 整周模糊度固定：LAMBDA

当当前解是 float 状态时，代码会尝试整数固定：

- [`manage_amb_LAMBDA()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2443>)
- 内部核心：
  - [`resamb_LAMBDA()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2260>)

### 19.1 float 模糊度问题

设滤波得到的模糊度估计为：

\[
\hat{\mathbf{N}} \in \mathbb{R}^m
\]

协方差为：

\[
\mathbf{Q}_{\hat{N}\hat{N}}
\]

### 19.2 LAMBDA 的目标

\[
\min_{\mathbf{N}\in\mathbb{Z}^m}
(\hat{\mathbf{N}}-\mathbf{N})^T
\mathbf{Q}_{\hat{N}\hat{N}}^{-1}
(\hat{\mathbf{N}}-\mathbf{N})
\]

也就是寻找最可能的整数模糊度向量。

### 19.3 ratio 检验

设最优和次优解目标函数分别为 `s_1, s_2`，则：

\[
\text{ratio}=\frac{s_2}{s_1}
\]

ratio 越大，表示最优整数候选与次优候选区分越明显，固定越可靠。

### 19.4 固定解回代

固定后，非模糊度状态会从 float 状态更新为 fixed 状态：

\[
\mathbf{x}^{a}
=
\mathbf{x}^{f}
-
\mathbf{Q}_{xN}\mathbf{Q}_{NN}^{-1}
(\hat{\mathbf{N}}-\mathbf{N})
\]

fixed 结果最终写入：

- `rtk->xa`
- `rtk->Pa`

并反映到最终 `SOLQ_FIX` 解中。

---

## 20. Fix-and-Hold

当前配置是：

- [`pos2-armode = 3`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:28>)

也就是 fix-and-hold。

当连续固定次数满足条件时，代码会调用：

- [`holdamb()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2135>)

它的作用可以理解为向后续历元引入一个“已知整数关系”的伪量测：

\[
\mathbf{H}_{hold}\mathbf{x}
=
\mathbf{z}_{hold}
+
\boldsymbol{\eta}
\]

这样可以提高后续历元维持 fix 的能力。

---

## 21. float 与 fix 解是如何输出的

在 `relpos()` 最后，RTKLIB 会根据当前状态决定输出 float 还是 fix。

### 21.1 float 解

对应代码：

- [`matcpy(rtk->x, xp, ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2883>)

当前位置来自：

- `rtk->x`
- `rtk->P`

### 21.2 fix 解

对应代码：

- [`rtk->sol.rr[i] = rtk->xa[i]`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3060>)

当前位置来自：

- `rtk->xa`
- `rtk->Pa`

### 21.3 `.pos` 文件中的状态码

常见对应关系是：

- `Q=1`：fix
- `Q=2`：float

所以 `rtklib.pos` 中从 `Q=2` 跳到 `Q=1`，本质上对应的是：

**整周模糊度成功固定**

---

## 22. 当前仓库对标准 RTKLIB 的算法增强

当前仓库并不是原样使用 RTKLIB，而是在几个地方做了扩展。

### 22.1 Doppler 约束

封装层显式设置了：

- [`prcopt.DopplerConstraint`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:86>)
- [`prcopt.Dop2PrRatio`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:88>)

说明多普勒在当前实现中被明确纳入权重设计。

### 22.2 速度增强分支

除了基础 `estvel()`，仓库还保留了：

- [`estvel_sd()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:1874>)
- [`estvel_td()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2027>)
- [`estvel_td_sd()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2141>)

说明作者尝试过：

- 单差 Doppler
- TDCP
- 单差 TDCP

### 22.3 鲁棒估计

在滤波更新前可以看到：

- [`if (opt->robustopt > RMODE_NONE) { adap = robust(...); }`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2820>)

说明当前仓库支持一定的鲁棒化处理，而不是严格依赖标准高斯噪声假设。

### 22.4 中间量发布给后端

在 `relpos()` 中会构造：

- [`GNSS_Info_ZD`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2673>)
- [`GNSS_Info_SD`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2977>)

最终发布：

- [`pub_raw.publish(epoch)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3168>)

从数学上看，这意味着 RTKLIB 不只输出最终估计值，还输出了：

**构造 GNSS 因子的原始约束结构**

---

## 23. 与后端优化的连接

后端 `gnssEstimator` 订阅位置在：

- [`glins/include/gnss/gnssEstimator.h:134`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssEstimator.h:134>)

它消费的是：

- `gnss_raw`

所以整个系统不是：

`RTKLIB -> 结束`

而是：

`RTKLIB 前端 -> 发布 GNSS_Info -> 因子图后端继续优化`

这也是当前仓库和“只用 RTKLIB 输出 `.pos` 文件”的最大区别。

---

## 24. 如果你要把它写进正式报告，推荐章节结构

最推荐的写法不是直接按照源码文件名展开，而是按下面顺序组织：

### 第一章：系统框架

1. 数据输入
2. 配置文件
3. `gnssProcessor`
4. `postpos()`
5. 输出与后端连接

### 第二章：单点定位原理

6. 卫星位置与钟差
7. 伪距方程
8. 最小二乘位置估计
9. 多普勒速度估计

### 第三章：RTK 相对定位原理

10. 零差、单差、双差
11. 状态向量设计
12. 卡尔曼预测与更新
13. 周跳检测与模糊度管理

### 第四章：整周模糊度固定

14. float 模糊度
15. LAMBDA
16. ratio 检验
17. fix-and-hold

### 第五章：当前仓库的工程化改造

18. Doppler 约束
19. 鲁棒处理
20. GNSS_Info 发布
21. 与 `gnssEstimator` 的连接

按这个顺序写，最适合正式成文。

---

## 25. 本文档对应的源码入口

如果后面要交叉引用，下面这些位置最重要：

- 系统入口：
  - [`glins/src/test_rtk.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:7>)
- RTKLIB 封装入口：
  - [`glins/include/gnss/gnssProcessor.h`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:23>)
- 配置解析：
  - [`rtklib/src/options.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/options.cpp:342>)
- 后处理总控：
  - [`rtklib/src/postpos.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1677>)
- RINEX 读取：
  - [`rtklib/src/rinex.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rinex.cpp:1892>)
- 单点定位：
  - [`rtklib/src/pntpos.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2255>)
- RTK 相对定位：
  - [`rtklib/src/rtkpos.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3336>)
- 后端消费：
  - [`glins/include/gnss/gnssEstimator.h`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssEstimator.h:134>)

---

## 26. 最后一段总结

当前仓库中的 RTKLIB，总体上可以概括为一个 **以配置驱动、以 RINEX 文件为输入、以单点定位初始化、以双差 RTK 滤波为主干、以 LAMBDA 模糊度固定为精度跃迁核心、并向后端图优化输出结构化 GNSS 中间量的前端 GNSS 解算系统**。

如果你只需要一句最适合放在报告摘要里的话，可以直接用下面这句：

> 本项目将 RTKLIB 作为 GNSS 前端处理引擎，利用广播星历和 rover/base 观测构建单点与双差 RTK 解算链路，在卡尔曼滤波和 LAMBDA 整周固定的基础上输出高精度定位结果，并进一步向后端图优化模块提供结构化 GNSS 约束信息。
