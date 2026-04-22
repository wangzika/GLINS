# RTKLIB 工作流程说明

本文档详细说明当前仓库中 **RTKLIB 的工作流程**。  
重点不是泛泛介绍 RTKLIB，而是结合这个项目里的实际调用链，解释：

- RTKLIB 是如何被启动的
- 配置文件如何进入处理流程
- RINEX 观测和导航文件如何被读取
- 每个历元的数据如何进入 `pntpos()` / `rtkpos()`
- RTK 相对定位是如何产出 `rtklib.pos` 的
- 这个仓库相对原版 RTKLIB 做了哪些工程化改造

---

## 1. 一句话概括

在这个项目里，RTKLIB 的核心工作流可以概括为：

**读取配置 -> 读取 `rover/base/nav` 文件 -> 整理观测历元 -> 每个历元先做单点定位 -> 再做 RTK 相对定位 -> 输出 `.pos` 文件 -> 同时发布 `GNSS_Info` / `Odometry` 给后端优化模块。**

对应到当前数据集，就是：

`rover.obs + base.obs + BRDM...rnx -> postpos() -> rtkpos() -> rtklib.pos + /gnss_raw + /rtklib_odom`

---

## 2. 这个仓库里 RTKLIB 的角色

这个项目并不是把 RTKLIB 当成一个独立命令行工具来用，而是把 RTKLIB 作为底层 GNSS 解算引擎，嵌进了 GLINS 的整体流程里。

从当前代码看，RTKLIB 主要承担 3 件事：

1. 从 RINEX 文件中读取 GNSS 原始观测和导航电文
2. 执行单点定位、差分定位、RTK/PPP 等求解
3. 把每个历元的 GNSS 解以及更底层的双差/零差信息发布给后续因子图优化模块

因此在这个项目里，RTKLIB 不是“最后一层结果展示工具”，而是 **GNSS 前端求解器**。

---

## 3. 项目中的启动入口

当前仓库里最直接的入口是：

- [`glins/src/test_rtk.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:7>)

它的主流程非常直接：

1. 初始化 ROS 节点
2. 构造 `gnssProcessor`
3. 构造 `gnssEstimator`
4. 指定处理时间段
5. 调用 `processor.decode(ts, te)`
6. 调用 `estimator.solveOptimization()`

这里最关键的是这两行：

- [`processor.decode(ts, te)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:39>)
- [`estimator.solveOptimization()`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:40>)

可以把它理解为：

- `gnssProcessor`：负责跑 RTKLIB 后处理，生成 GNSS 原始解
- `gnssEstimator`：负责消费 RTKLIB 发布出来的 GNSS 信息，继续做图优化

---

## 4. 第 0 层：配置文件先决定 RTKLIB 怎么工作

RTKLIB 的行为首先由配置文件决定。当前数据集对应的配置文件是：

- [`glins/config/conf/20240129.conf`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:1>)

其中最重要的几类配置是：

### 4.1 定位模式

- [`pos1-posmode = kinematic`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:3>)
- [`pos1-soltype = forward`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:5>)

这表示：

- 模式是运动状态 RTK
- 解算方向是前向

### 4.2 使用哪些观测频率和星座

- [`pos1-frequency = l1+l2+l5`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:4>)
- [`pos1-navsys = 41`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:24>)

`41 = 1 + 8 + 32`，对应：

- GPS
- Galileo
- BDS

### 4.3 误差模型和改正策略

- [`pos1-ionoopt = brdc`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:14>)
- [`pos1-tropopt = saas`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:15>)
- [`pos1-sateph = brdc`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:16>)

表示：

- 电离层：广播模型
- 对流层：Saastamoinen 模型
- 卫星轨道：广播星历

### 4.4 整周模糊度固定策略

- [`pos2-armode = 3`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:28>)

这里是 `fix-and-hold`。

### 4.5 输入输出文件

- [`inpstr1-path = rover.obs`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:135>)
- [`inpstr2-path = base.obs`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:136>)
- [`inpstr3-path = BRDM...rnx`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:137>)
- [`outstr1-path = rtklib.pos`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:139>)

所以，**配置文件其实就是 RTKLIB 工作流的“入口参数包”**。

---

## 5. 第 1 层：`gnssProcessor` 如何把配置交给 RTKLIB

`gnssProcessor` 定义在：

- [`glins/include/gnss/gnssProcessor.h`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:23>)

它的工作流程分成两部分：构造函数初始化 + `decode()` 触发解算。

### 5.1 构造函数阶段

在构造函数里，主要做了下面几步：

#### 第一步：注册 ROS 发布器

- [`rtkposRegisterPub(nh)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:51>)
- [`pntposRegisterPub(nh)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:52>)

这说明 RTKLIB 在这个仓库里不仅写文件，还会向 ROS 话题发消息。

#### 第二步：加载配置

- [`loadopts(rtklibConfigPath.c_str(), sysopts)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:59>)
- [`loadopts(rtklibConfigPath.c_str(), rcvopts)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:59>)
- [`getsysopts(&prcopt, &solopt, &filopt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:63>)

这里有两个层次：

- `sysopts`：标准 RTKLIB 处理参数
- `rcvopts`：这个仓库额外定义的文件路径参数表

`loadopts()` 的实现位于：

- [`rtklib/src/options.cpp:342`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/options.cpp:342>)

它的逻辑很简单：

1. 逐行读取配置文件
2. 按 `key=value` 解析
3. 在选项表里查找对应项
4. 转换成内部结构体字段

`getsysopts()` 位于：

- [`rtklib/src/options.cpp:568`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/options.cpp:568>)

它负责把缓冲区中的配置内容转换成真正的：

- `prcopt_t`
- `solopt_t`
- `filopt_t`

#### 第三步：把输入输出文件路径放入 `infile/outfile`

相关代码：

- [`strpath[0..4]` 映射定义](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:7>)
- [`inpstr1-path` 到 `outstr1-path` 的选项表](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:9>)
- [`for (i = 0; i < 3; i++) strcpy(infile[i], strpath[i]);`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:70>)
- [`strcpy(outfile, strpath[4]);`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:78>)

最终，RTKLIB 真正接收到的 3 路输入就是：

1. rover 观测文件
2. base 观测文件
3. nav 导航文件

#### 第四步：本项目对 RTKLIB 参数做了少量二次修改

例如：

- [`prcopt.DopplerConstraint = (prcopt.dynamics == 1 ? 1 : 0);`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:86>)
- [`prcopt.StateModel = 1;`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:87>)
- [`prcopt.Dop2PrRatio = 12.5;`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:88>)
- [`prcopt.thresdop = 10;`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:89>)

说明这个项目不是原封不动调用 RTKLIB，而是在误差建模和观测融合上做了定制。

### 5.2 `decode()` 阶段

`decode()` 真正触发 RTKLIB 求解：

- [`postpos(ts, te, ti, tu, &prcopt, &solopt, &filopt, infile, n, outfile, rov, base)`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:106>)

这就是整个 RTKLIB 后处理工作流的核心入口。

---

## 6. 第 2 层：`postpos()` 是 RTKLIB 后处理总控入口

`postpos()` 的声明在：

- [`rtklib/include/rtklib.h:1894`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/include/rtklib.h:1894>)

实现位于：

- [`rtklib/src/postpos.cpp:1677`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1677>)

从工程视角看，`postpos()` 是一个 **总控函数**。它本身不直接完成每一项计算，而是组织下面这些步骤：

1. 打开处理会话
2. 读取所有观测与导航数据
3. 设定天线、基站位置和各种改正模型
4. 按历元循环调用求解器
5. 输出 `.pos` / `.stat` / 事件文件
6. 释放内存和关闭会话

可以把它理解成 RTKLIB 后处理模式下的“调度器”。

---

## 7. 第 3 层：`postpos()` 里的完整执行顺序

### 7.1 打开处理会话 `openses()`

`postpos()` 先调用：

- [`openses(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1690>)

`openses()` 位于：

- [`rtklib/src/postpos.cpp:1163`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1163>)

它主要做：

- 读取卫星天线 PCV
- 读取接收机天线 PCV
- 打开 geoid 数据

如果你的配置没有给这些文件，流程仍然可以跑，只是对应改正不会启用。

### 7.2 执行处理会话 `execses()`

后续主要工作发生在：

- [`execses(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1327>)

它是 `postpos()` 内部真正负责“读数据、跑解算、写结果”的函数。

---

## 8. 第 4 层：`execses()` 内部做了什么

### 8.1 读辅助改正文件

如果配置里给出了这些文件，`execses()` 会先尝试读入：

- IONEX TEC
- ERP
- DCB
- BLQ

相关代码集中在：

- [`readtec`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1362>)
- [`readerp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1370>)
- [`readdcb`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1389>)
- [`readotl`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1408>)

在当前 `20240129.conf` 里这些外部精密改正基本没配置，所以当前主流程主要依赖 **广播模型**。

### 8.2 读取观测与导航数据 `readobsnav()`

这是后处理流程里非常关键的一步：

- [`readobsnav(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:957>)

这个函数完成 4 件核心工作：

1. 调用 `readrnxt()` 读入所有 RINEX 文件
2. 把 rover / base / nav 数据汇总到内存结构体
3. 对观测数据按时间排序
4. 对导航数据去重

相关关键代码：

- 读文件：
  - [`readrnxt(infile[i], ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:982>)
- 观测排序：
  - [`sortobs(obs)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1003>)
- 星历去重：
  - [`uniqnav(nav)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1006>)

#### `readrnxt()` 再往下怎么走

RINEX 读取主链路在：

- [`rtklib/src/rinex.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rinex.cpp:1892>)

主要顺序是：

1. `readrnxfile()`
2. `readrnxfp()`
3. `readrnxh()` 读取头部
4. 根据文件类型分发到：
   - `readrnxobs()` 读观测文件
   - `readrnxnav()` 读导航文件
   - `readrnxclk()` 读钟差文件

对应位置：

- [`readrnxh()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rinex.cpp:728>)
- [`readrnxobs()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rinex.cpp:1266>)
- [`readrnxnav()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rinex.cpp:1702>)

所以从文件层看，RTKLIB 的第一阶段是：

**文本 RINEX -> 内存中的 `obs_t/nav_t/sta_t` 结构体**

### 8.3 设置基站/天线参数

读取完数据后，`execses()` 会继续配置求解环境：

- [`setpcv(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1402>)
- [`antpos(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1428>)

`antpos()` 会根据配置决定基站位置来自哪里：

- 配置文件直接给定
- 位置文件
- RINEX 头
- 单点平均结果

在当前 `20240129.conf` 里，基站位置直接通过 `ant2-pos1/2/3` 给出，是 ECEF 坐标。

### 8.4 打开输出文件并写表头

执行位置：

- [`outhead(outfile, ...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1444>)

表头生成逻辑位于：

- [`outheader(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:131>)

它会把这些信息写进 `.pos` 文件头：

- program 版本
- 输入文件路径
- 观测起止时间
- 处理配置摘要
- 参考站坐标

这也是为什么 `rtklib.pos` 头部已经能完整看出这次解算怎么跑的。

---

## 9. 第 5 层：进入历元循环 `procpos()`

真正“按时间一步一步处理”的位置在：

- [`procpos(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:603>)

这是 RTKLIB 后处理最核心的历元处理循环。

主循环结构是：

- [`while ((nobs = inputobs(...)) >= 0)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:623>)

也就是说，RTKLIB 每一轮都会：

1. 取出下一个历元对应的观测
2. 做必要预处理
3. 调用 `rtkpos()` 求解
4. 输出当前历元结果

---

## 10. 第 6 层：`inputobs()` 如何把 rover/base 历元配对

函数位置：

- [`inputobs(...)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:411>)

这是 RTKLIB 后处理里非常关键但经常被忽略的一步。

它的职责是：

1. 从已经排序好的 `obss` 中找出 rover 当前历元
2. 找出与之最匹配的 base 历元
3. 把两边观测拼到同一个 `obsd_t obs[]` 数组里
4. 更新 SBAS / SSR 改正

### 10.1 rover/base 观测同步

前向处理时，它先找 rover 当前历元：

- [`nextobsf(&obss, &iobsu, 1)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:428>)

再找 base 的最近时间戳：

- [`nextobsf(&obss, &iobsr, 2)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:432>)

这个仓库里还特别改成了“找最近时间戳”的实现，而不是简单的前后追随：

- [`fabs(dt_next) > fabs(dt)` 比较最近历元](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:440>)

### 10.2 观测平滑

这里还支持本项目加入的平滑选项：

- [`lgccsmooth`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:460>)
- [`lgcdsmooth`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:461>)

这属于项目对 RTKLIB 的增强点。

### 10.3 更新改正

如果有 SBAS / SSR 数据，这里也会随历元推进更新：

- [`sbsupdatecorr`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:471>)
- [`update_rtcm_ssr`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:479>)

---

## 11. 第 7 层：每个历元都进入 `rtkpos()`

在 `procpos()` 中，当前历元观测准备好之后，会调用：

- [`rtkpos(rtk, obs_ptr, n, &navs)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:639>)

实现位于：

- [`rtklib/src/rtkpos.cpp:3336`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3336>)

`rtkpos()` 是 **每个历元的定位核心求解器**。

它的逻辑可以概括为：

1. 划分 rover/base 观测
2. 先做 rover 单点定位 `pntpos()`
3. 如果是单点模式就直接输出
4. 如果是 PPP 模式则走 `pppos()`
5. 如果是相对定位模式，则继续做 RTK `relpos()`

---

## 12. 为什么 `rtkpos()` 先做 `pntpos()`

在 `rtkpos()` 里第一步关键调用是：

- [`pntpos(obs, nu, nav, &rtk->opt, &rtk->sol, NULL, rtk->ssat, msg)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3367>)

也就是说，**即使最终目标是 RTK，相对定位之前也先做一遍单点定位**。

原因很实际：

- 需要得到 rover 的初值
- 需要得到卫星可见性、方位角、高度角、SNR 等辅助信息
- 需要为后续相对定位和滤波提供先验状态

所以整个 RTK 流程不是“跳过单点直接双差”，而是：

**SPP 初值 -> 相对定位精化**

---

## 13. `pntpos()` 在流程中的作用

`pntpos()` 位于：

- [`rtklib/src/pntpos.cpp:2255`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/pntpos.cpp:2255>)

它负责：

- 用伪距为主完成单点定位
- 计算基础卫星几何
- 估计接收机钟差
- 形成当前历元的单点解

在工程流程里，`pntpos()` 更像是：

- 一个“初始化器”
- 一个“观测筛选器”
- 一个“给 RTK/PPP 提供初值的前置解”

---

## 14. 相对定位核心：`relpos()`

在 `rtkpos()` 里，完成单点定位后，真正的 RTK 相对定位发生在：

- [`relpos(rtk, obs, nu, nr, nav)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3461>)

`relpos()` 位于：

- [`rtklib/src/rtkpos.cpp:2591`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2591>)

从算法上说，这一层主要负责：

1. 利用 rover/base 同步观测构建差分模型
2. 筛选参考卫星和可用卫星
3. 建立状态量和观测方程
4. 用卡尔曼滤波更新位置、速度、模糊度等状态
5. 尝试整周模糊度固定
6. 更新当前历元最终解状态

如果简化理解，`relpos()` 就是：

**RTK 的主战场**

---

## 15. 当前仓库对 `rtkpos()` 的一个关键改造

这个仓库里的 `rtkpos.cpp` 不只是“输出最终坐标”，还会构造并发布更底层的 GNSS 中间信息。

### 15.1 ROS 发布器注册

在：

- [`rtkposRegisterPub()`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:113>)

注册了两个话题：

- [`/gnss_raw`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:115>)
- [`/rtklib_odom`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:116>)

### 15.2 在 `relpos()` 中组织 `GNSS_Info`

你可以看到它在相对定位过程中构造：

- `GNSS_Info`
- `GNSS_Info_ZD`
- `GNSS_Info_SD`

相关位置：

- [`rtklib/src/rtkpos.cpp:2664`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2664>)
- [`rtklib/src/rtkpos.cpp:2678`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2678>)
- [`rtklib/src/rtkpos.cpp:2977`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:2977>)

### 15.3 发布原始 GNSS 因子化信息

最终发布位置：

- [`pub_raw.publish(epoch)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:3168>)

这一步非常关键。  
它意味着 RTKLIB 在这个仓库里不只是“最终定位器”，还是后端因子图的 **GNSS 测量前端**。

---

## 16. RTKLIB 的输出不只有 `rtklib.pos`

在标准后处理视角，最显眼的输出当然是：

- `rtklib.pos`

但在这个项目里，RTKLIB 实际上有三类输出：

### 16.1 文本定位结果

通过：

- [`outsol(fp, &rtk->sol, rtk->rb, sopt)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:659>)

写入 `.pos` 文件。

### 16.2 状态/残差文件

如果启用了 `out-outstat`，会写 `.stat` 文件：

- [`rtkopenstat(statfile, sopt->sstat)`](</Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/postpos.cpp:1441>)

### 16.3 ROS 话题

- `/gnss_raw`
- `/rtklib_odom`

所以从系统集成角度，RTKLIB 的输出包括：

- 文件输出
- 中间状态输出
- ROS 在线消息输出

---

## 17. `gnssEstimator` 如何消费 RTKLIB 输出

后续模块订阅位置在：

- [`glins/include/gnss/gnssEstimator.h:134`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssEstimator.h:134>)

订阅的是：

- `gnss_raw`

回调函数：

- [`gnssInfoHandler`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssEstimator.h:1571>)

优化主循环入口：

- [`solveOptimization()`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssEstimator.h:268>)

这说明当前仓库的全链路不是“RTKLIB 到此为止”，而是：

1. RTKLIB 做 GNSS 前端解算
2. 发布带有双差/模糊度/卫星状态的 `GNSS_Info`
3. `gnssEstimator` 继续用这些信息做图优化

因此，当前项目里的 RTKLIB 工作流其实是：

**前端 RTK 解算器 + 后端因子图输入生成器**

---

## 18. 从时间顺序看一遍完整工作流

下面用时间顺序把整条链再压缩一遍：

### 阶段 1：程序启动

1. `test_rtk.cpp` 创建 `gnssProcessor`
2. `gnssProcessor` 注册 RTKLIB ROS 发布器
3. 加载 `20240129.conf`
4. 解析出 `prcopt/solopt/filopt`
5. 取出 `rover/base/nav` 输入路径和 `.pos` 输出路径

### 阶段 2：进入 RTKLIB 后处理入口

6. `decode(ts, te)` 调用 `postpos()`
7. `postpos()` 打开会话 `openses()`
8. `execses()` 读取辅助改正、RINEX 观测和导航文件
9. `readobsnav()` 把文本文件转换成 `obs_t/nav_t/sta_t`
10. `sortobs()` 对观测按时间排序
11. `uniqnav()` 去除重复星历

### 阶段 3：准备解算环境

12. 设置天线参数和基站位置
13. 打开输出文件并写 header
14. 初始化历元索引、状态结构体

### 阶段 4：逐历元求解

15. `procpos()` 循环调用 `inputobs()`
16. `inputobs()` 找到当前 rover 历元与最近的 base 历元
17. 形成当前历元观测数组
18. 更新 SBAS / SSR 改正
19. 调用 `rtkpos()`

### 阶段 5：单点 + 相对定位

20. `rtkpos()` 先调用 `pntpos()` 给 rover 求单点初值
21. 如果是单点模式，则直接输出
22. 如果是 PPP 模式，则调用 `pppos()`
23. 如果是 RTK/差分模式，则继续调用 `relpos()`
24. `relpos()` 做差分建模、滤波和整周固定
25. 更新本历元最终解状态

### 阶段 6：输出与后端衔接

26. `postpos.cpp` 通过 `outsol()` 把解写入 `rtklib.pos`
27. `rtkpos.cpp` 同步发布 `/gnss_raw` 和 `/rtklib_odom`
28. `gnssEstimator` 订阅 `/gnss_raw`
29. 后端图优化继续处理

---

## 19. 这条流程里最重要的几个数据结构

理解 RTKLIB 工作流时，下面几个结构体最重要：

| 结构体 | 作用 |
| --- | --- |
| `prcopt_t` | 处理配置，决定模式、频率、误差模型、AR 等 |
| `solopt_t` | 输出配置，决定结果格式和输出内容 |
| `filopt_t` | 外部辅助文件配置 |
| `obs_t` | 全部观测数据缓冲 |
| `nav_t` | 全部导航/星历/改正数据缓冲 |
| `sta_t` | 站点头信息 |
| `rtk_t` | 当前 RTK 求解状态，包括滤波状态和结果 |
| `sol_t` | 当前历元解结果 |

如果你能把这 8 个结构体之间的关系看懂，RTKLIB 的主流程就基本清楚了。

---

## 20. 当前数据集对应的实际流程实例

以 `20240129` 数据为例，当前实际执行的是：

- 模式：`kinematic`
- 解算方向：`forward`
- 星座：GPS + Galileo + BDS
- 频率：L1 + L2 + L5
- 星历：广播星历
- 电离层：广播模型
- 对流层：Saastamoinen
- AR：Fix-and-Hold

也就是说，这次流程不是：

- PPP
- Precise ephemeris
- Combined forward/backward

而是一个很典型的：

**广播星历驱动的前向 RTK 后处理流程**

---

## 21. 为什么这个流程对 GLINS 很重要

对 GLINS 来说，RTKLIB 的意义不只是生成 `rtklib.pos`。

更重要的是：

- 它把 GNSS 原始观测转换成可以被后端使用的结构化 GNSS 约束
- 它输出了单差/双差、模糊度、卫星状态等中间量
- 它让图优化模块不必从 RINEX 原始文件重新做一遍 GNSS 建模

所以在这个仓库里，RTKLIB 本质上是：

**GNSS 解算前端 + GNSS 因子生成前端**

---

## 22. 如果你要讲清楚 RTKLIB 工作流程，最推荐的讲法

最清晰的讲法不是按源码文件顺序念，而是按下面 5 层来讲：

1. 配置层
   - `20240129.conf`
   - `loadopts()`
   - `getsysopts()`

2. 会话层
   - `postpos()`
   - `execses()`
   - `openses()`

3. 数据读取层
   - `readobsnav()`
   - `readrnxt()`
   - `readrnxobs()/readrnxnav()`

4. 历元解算层
   - `inputobs()`
   - `procpos()`
   - `rtkpos()`
   - `pntpos()`
   - `relpos()`

5. 输出与集成层
   - `outsol()`
   - `/gnss_raw`
   - `gnssEstimator`

按这个框架写，逻辑会比直接按源码注释展开更清楚。

---

## 23. 相关源码入口

如果后面你要继续深入，优先看下面这些文件：

- 总入口：
  - [`glins/src/test_rtk.cpp`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_rtk.cpp:7>)
- GLINS 对 RTKLIB 的封装：
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
- RTKLIB 与后端因子图连接：
  - [`glins/include/gnss/gnssEstimator.h`](</Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssEstimator.h:134>)

---

## 24. 一句话总结

当前仓库里的 RTKLIB 工作流程，本质上是：

**由配置文件驱动的 RINEX 后处理引擎。它先把 `rover/base/nav` 文件读成内存观测与星历，再按历元执行 `单点定位 -> RTK相对定位`，输出 `rtklib.pos`，同时把中间 GNSS 约束发布给 GLINS 后端优化模块。**
