# RTKLIB 核心结构体说明

本文档详细说明 `rtklib/include/rtklib.h` 中几个最核心的数据结构：

- `prcopt_t`
- `solopt_t`
- `filopt_t`
- `obs_t`
- `nav_t`
- `sta_t`
- `rtk_t`
- `sol_t`

这几个结构体几乎把 RTKLIB 的“输入、配置、状态、输出”全部串起来了。  
如果只用一句话概括它们之间的关系，可以写成：

`prcopt_t / solopt_t / filopt_t` 决定“怎么解、怎么输出、还要读哪些外部文件”；  
`obs_t / nav_t / sta_t` 提供“观测、星历、站点信息”；  
`rtk_t` 负责“当前求解状态机”；  
`sol_t` 则保存“当前历元解结果”。

---

## 1. 先给出整体关系

从 RTKLIB 工作流看，这几个结构体大致按下面的顺序参与计算：

1. 先读取配置  
   把 `.conf` 之类的选项读入 `prcopt_t`、`solopt_t`、`filopt_t`

2. 再读取数据  
   把观测文件/流数据读入 `obs_t`，把星历和改正数据读入 `nav_t`，把站点头信息读入 `sta_t`

3. 初始化解算器  
   用 `prcopt_t` 初始化 `rtk_t`

4. 逐历元处理  
   每个历元从 `obs_t` 取观测，从 `nav_t` 查星历和改正，用 `rtk_t` 更新滤波状态

5. 产出结果  
   当前历元结果写到 `rtk_t.sol`，其类型就是 `sol_t`

6. 输出格式化  
   最终结果按 `solopt_t` 规定的格式写文件或打印

7. 外部文件辅助  
   `filopt_t` 指向天线文件、DCB、IONEX、EOP、geoid 等辅助资源

所以可以把这 8 个结构体按角色分成四组：

| 分组 | 结构体 | 角色 |
| --- | --- | --- |
| 配置层 | `prcopt_t` | 控制解算模式、误差模型、AR、动态模型等 |
| 配置层 | `solopt_t` | 控制结果输出格式 |
| 配置层 | `filopt_t` | 指定外部辅助文件路径 |
| 数据层 | `obs_t` | 保存观测数据历元缓存 |
| 数据层 | `nav_t` | 保存星历、钟差、改正和模型参数 |
| 数据层 | `sta_t` | 保存站点和天线头信息 |
| 状态层 | `rtk_t` | 保存滤波器状态、模糊度状态、卫星状态和当前解 |
| 输出层 | `sol_t` | 保存当前历元的位置、速度、协方差和解状态 |

---

## 2. `prcopt_t`：处理配置

### 2.1 它是什么

`prcopt_t` 是 RTKLIB 最重要的控制结构体。  
它回答的问题是：

- 当前到底用什么定位模式
- 用哪些系统、哪些频点
- 用什么星历
- 模糊度固定怎么做
- 电离层、对流层怎么处理
- 测量噪声怎么建模
- 滤波状态怎么传播
- 站坐标、天线、基站、剔星规则怎么配置

可以把它理解成：

**解算器的“总开关面板”。**

### 2.2 主要字段可以分成哪些类

#### 1. 模式控制类

- `mode`  
  定位模式，例如单点、DGPS、Kinematic、Static、Moving-Base、PPP 等。

- `soltype`  
  解类型，前向、后向或组合。

- `nf`  
  使用的频点数，比如 L1、L1+L2、L1+L2+L5。

- `navsys`  
  使用哪些导航系统，通常是 GPS、GLO、GAL、BDS、QZS 等的组合掩码。

#### 2. 观测筛选与几何约束类

- `elmin`  
  最低高度角，低于这个高度角的卫星直接不用。

- `snrmask`  
  信噪比门限掩码，可按高度角和接收机类型分别设置 rover/base 的 SNR 筛选规则。

- `exsats[MAXSAT]`  
  手工排除或强制包含的卫星列表。

- `maxgdop`  
  GDOP 过大时拒绝当前解。

#### 3. 星历和改正模型类

- `sateph`  
  采用哪种星历/钟差来源，例如广播星历、精密星历、SSR 等。

- `ionoopt`  
  电离层处理方式，例如广播模型、双差消除、估计电离层、IONEX 等。

- `tropopt`  
  对流层处理方式，例如 Saastamoinen、估计 zenith wet delay 等。

- `tidecorr`  
  是否做固体潮、海潮负载、极潮改正。

- `sbascorr`、`sbassatsel`  
  SBAS 改正配置。

#### 4. 模糊度固定与 AR 类

- `modear`  
  整周模糊度固定模式，是否关闭、连续固定、瞬时固定、fix-and-hold、PPP-AR 等。

- `glomodear`、`gpsmodear`、`bdsmodear`  
  针对不同系统的 AR 策略。

- `arfilter`  
  是否启用 AR 过滤，剔除可疑卫星以提高固定可靠性。

- `maxout`  
  观测中断多少历元后重置偏差状态。

- `minlock`  
  卫星锁定至少多少历元才允许参与固定。

- `minfixsats`、`minholdsats`、`mindropsats`  
  固定、保持、部分剔除时需要的最少卫星数。

- `minfix`  
  至少连续多少次 fixed 才进入 hold。

- `armaxiter`  
  AR 最大迭代次数。

- `thresar[8]`  
  AR 验证阈值，典型地影响 ratio-test 一类判决。

- `elmaskar`、`elmaskhold`  
  参与固定或保持固定的高度角门限。

- `varholdamb`、`gainholdamb`  
  fix-and-hold 阶段伪观测强度和调整增益。

#### 5. 误差模型和噪声建模类

- `eratio[NFREQ]`  
  码观测和载波观测的权重比。  
  一般因为载波更精确，所以伪距噪声会被设得更大。

- `err[8]`  
  观测误差项数组。注释里给了语义：
  `[reserved, constant, elevation, baseline, doppler, snr-max, snr, rcv_std]`

  也就是说它不是一个“随便放的数组”，而是一组统一的测量误差模型系数。

- `std[3]`  
  初始状态标准差，通常对应：
  - 模糊度初值
  - 电离层状态
  - 对流层状态

- `prn[6]`  
  过程噪声标准差，控制状态传播时的不确定度增长。

- `sclkstab`  
  卫星钟稳定度，用于钟差相关建模。

- `maxinno[3]`  
  创新过大时的拒绝阈值，用于粗差控制。

- `thresslip`  
  周跳检测阈值。

#### 6. 动态模型与滤波器类

- `dynamics`  
  动力学模型，通常表示是否考虑速度、加速度传播。

- `niter`  
  每历元滤波/迭代次数。

- `intpref`  
  后处理时是否对参考站观测做插值。

- `maxtdiff`  
  流动站和基站观测时间差的最大允许值。

#### 7. 站点、天线与基线类

- `ru[3]`  
  固定模式下 rover 的已知坐标。

- `rb[3]`  
  相对定位模式下 base 的已知坐标。

- `baseline[2]`  
  基线长度约束，通常是 `{长度, sigma}`。

- `anttype[2][MAXANT]`  
  rover/base 天线型号。

- `antdel[2][3]`  
  rover/base 天线偏移。

- `pcvr[2]`  
  rover/base 天线相位中心模型。

- `rovpos`、`refpos`  
  rover/base 位置的来源方式，是配置给定、文件读入、RINEX 头读取还是 RTCM。

#### 8. 输出与兼容类

- `outsingle`  
  差分失败时是否仍输出 single 解。

- `rnxopt[2][256]`  
  rover/base 的 RINEX 相关附加选项。

- `posopt[6]`  
  定位选项数组。

- `syncsol`  
  解同步模式。

- `odisp[2][6*11]`  
  海潮负载参数。

- `freqopt`  
  一些频率相关的 AR/解算控制开关。

- `pppopt[256]`  
  PPP 相关附加字符串选项。

### 2.3 这个仓库里 `prcopt_t` 的自定义扩展

这份工程里的 `rtklib.h` 不是完全原版，`prcopt_t` 末尾增加了一批明显偏项目实验用途的字段：

- `StateModel`
- `Dop2PrRatio`
- `DopplerConstraint`
- `thres[2]`
- `DataCheck`
- `usetruth`
- `default_rb[3]`
- `default_snr_max`
- `thresdop`
- `smoothwidth`
- `smoothmode`

这些字段说明这个仓库在原始 RTKLIB 基础上额外做了：

- 多普勒速度约束实验
- 数据质量检查
- 平滑相关实验
- 默认基站位置或真值辅助

因此你在读这份工程时，不能把 `prcopt_t` 仅仅理解成“标准 RTKLIB 选项”，它已经兼具了项目定制算法的控制入口。

### 2.4 它在系统中的作用

`prcopt_t` 不是结果数据，也不是观测数据，它是“求解规则”。  
几乎所有解算函数在真正工作前，都会先看 `prcopt_t`：

- 能不能用这颗星
- 用哪个观测频点
- 创新多大算异常
- 要不要做 AR
- 动态状态是不是要扩展到速度/加速度
- 多普勒是不是要进约束

所以如果某次结果“不对”，首先不是去怪 `rtk_t`，而是先检查 `prcopt_t`。

---

## 3. `solopt_t`：输出配置

### 3.1 它是什么

`solopt_t` 决定的是：

- 解算结果以什么格式输出
- 时间怎么写
- 坐标怎么写
- 是否带表头
- 是否输出速度、统计量、调试信息

它不影响“怎么解”，主要影响“怎么写出来”。

### 3.2 主要字段说明

- `posf`  
  输出解的格式，比如 LLH、XYZ、ENU、NMEA 等。

- `times`  
  输出时间系统。

- `timef`  
  时间输出格式，例如“周内秒”还是“年月日 时分秒”。

- `timeu`  
  小数秒保留位数。

- `degf`  
  经纬度格式，是十进制度还是度分秒。

- `outhead`  
  是否输出表头。

- `outopt`  
  是否把处理选项也写进结果文件。

- `outvel`  
  是否输出速度。

- `datum`  
  使用的基准面。

- `height`  
  高程输出为椭球高还是大地高。

- `geoid`  
  使用的 geoid 模型。

- `solstatic`  
  静态模式下输出全部历元还是只输出单个最终解。

- `sstat`  
  是否输出统计信息，比如状态量、残差等。

- `trace`  
  调试 trace 级别。

- `nmeaintv[2]`  
  NMEA 输出间隔。

- `sep[64]`  
  字段分隔符。

- `prog[64]`  
  输出时写入的程序名。

- `maxsolstd`  
  结果标准差超过某阈值时不输出。

### 3.3 它在工作流里的位置

`solopt_t` 在后处理时尤其重要。  
它不决定滤波状态是否能收敛，但决定了你最终看到的结果长什么样。

例如同一个 `sol_t`，在不同 `solopt_t` 下可以被输出成：

- ECEF XYZ
- ENU baseline
- LLH
- NMEA GGA/RMC

所以它更像“结果格式化配置器”。

---

## 4. `filopt_t`：外部辅助文件配置

### 4.1 它是什么

`filopt_t` 不保存解算结果，也不保存观测。  
它只是统一记录各种外部辅助文件的路径。

### 4.2 主要字段说明

- `satantp`  
  卫星天线参数文件。

- `rcvantp`  
  接收机天线参数文件。

- `stapos`  
  站点坐标文件。

- `geoid`  
  geoid 文件。

- `iono`  
  电离层数据文件，例如 IONEX。

- `dcb`  
  DCB 偏差文件。

- `eop`  
  地球定向参数文件。

- `blq`  
  海潮负载 BLQ 文件。

- `tempdir`  
  FTP/HTTP 下载临时目录。

- `geexe`  
  Google Earth 可执行程序路径。

- `solstat`  
  解算统计信息文件。

- `trace`  
  debug trace 文件。

### 4.3 它在系统中的作用

可以把 `filopt_t` 理解为：

**“解算之外的资源清单”。**

RTKLIB 并不是只靠观测文件和导航文件就能做到所有高精度改正。  
很多更细的误差改正都依赖额外文件，而 `filopt_t` 就是这些文件的集中入口。

---

## 5. `obs_t`：观测数据总缓冲

### 5.1 它是什么

`obs_t` 是观测数据的容器。  
真正单条观测记录是 `obsd_t`，而 `obs_t` 则是“很多条 `obsd_t` 组成的数组缓冲”。

你可以把它理解成：

- `obsd_t`：单星单接收机单历元的一条原子观测
- `obs_t`：很多原子观测拼成的总观测池

### 5.2 字段说明

- `n`  
  当前已经存了多少条观测记录。

- `nmax`  
  当前分配了多大容量。

- `flag`  
  历元标志，例如正常、掉电、事件标记等。

- `rcvcount`  
  接收机事件计数。

- `tmcount`  
  time mark 计数。

- `data`  
  指向 `obsd_t` 数组的指针，真正的观测值都在这里。

### 5.3 `obsd_t` 为什么重要

因为 `obs_t` 的核心其实是 `obsd_t *data`。  
而 `obsd_t` 里保存了每条观测最核心的信息：

- `time`：观测时刻
- `sat`：卫星号
- `rcv`：接收机号
- `SNR[]`：信噪比
- `LLI[]`：失锁标志
- `code[]`：观测码类型
- `L[]`：载波相位
- `P[]`：伪距
- `D[]`：多普勒

所以 `obs_t` 本质上不是“一个历元”，而是“一个动态观测池”。  
在后处理场景中，它通常会装下整个时间段的观测；  
在实时场景中，也可能只装当前或最近几历元。

### 5.4 它在工作流中的作用

所有定位都是从观测开始的。  
没有 `obs_t`，就没有伪距、载波、多普勒，也就没有残差、模糊度、滤波更新。

`obs_t` 可以回答的问题包括：

- 当前时刻有哪些卫星被观测到了
- 每颗卫星每个频点的码、相位、多普勒是多少
- 有没有周跳或失锁
- 信号质量如何

---

## 6. `nav_t`：导航与改正数据总缓冲

### 6.1 它是什么

`nav_t` 是 RTKLIB 另一类总容器。  
如果说 `obs_t` 保存“接收机量到的东西”，那么 `nav_t` 保存的是“解释这些观测所需的导航信息和改正模型”。

### 6.2 它包含什么

`nav_t` 里面内容很多，但可以按以下几类理解：

#### 1. 各类星历

- `eph`  
  GPS/QZS/GAL/BDS/IRN 广播星历

- `geph`  
  GLONASS 星历

- `seph`  
  SBAS 星历

- `peph`  
  精密星历

- `pclk`  
  精密钟差

- `alm`  
  星历简化版 almanac

#### 2. 电离层、地球自转和时间系统模型

- `tec`  
  TEC 电离层格网

- `erp`  
  地球自转参数

- `utc_gps / utc_glo / utc_gal / utc_cmp ...`  
  各系统到 UTC 的时间参数

- `ion_gps / ion_gal / ion_qzs / ion_cmp / ion_irn`  
  各系统广播电离层模型参数

#### 3. 偏差与硬件改正

- `cbias[MAXSAT][3]`  
  卫星 DCB 偏差

- `rbias[MAXRCV][2][3]`  
  接收机 DCB 偏差

- `pcvs[MAXSAT]`  
  卫星天线相位中心改正

#### 4. 差分与增强改正

- `sbssat`
- `sbsion[]`
- `dgps[]`
- `ssr[]`

这部分是 RTKLIB 能支持 DGPS、SBAS、SSR 等增强模式的关键。

### 6.3 计数器字段的意义

像下面这些字段：

- `n, nmax`
- `ng, ngmax`
- `ns, nsmax`
- `ne, nemax`
- `nc, ncmax`
- `na, namax`
- `nt, ntmax`

本质上都是“当前数量 + 已分配容量”的配套计数器。  
因为 `nav_t` 不是一个单对象，而是一组动态数组的大容器。

### 6.4 它在工作流中的作用

`nav_t` 决定了解算器是否知道：

- 卫星在天上的什么位置
- 卫星钟差是多少
- 时间系统怎么换算
- 电离层/对流层怎么修正
- 是否有 DCB、SSR、SBAS 等改正

所以：

- `obs_t` 解决“测到了什么”
- `nav_t` 解决“这些观测该怎么解释”

没有 `nav_t`，`obs_t` 只是原始数字；  
有了 `nav_t`，这些观测才能变成几何距离、残差和状态更新。

---

## 7. `sta_t`：站点头信息

### 7.1 它是什么

`sta_t` 保存的是站点级元信息，而不是逐历元数据。  
这些信息通常来自 RINEX 头文件、站点配置或 RTCM 基站说明。

### 7.2 主要字段说明

- `name`  
  测站名。

- `marker`  
  marker 编号。

- `antdes`、`antsno`  
  天线型号和序列号。

- `rectype`、`recver`、`recsno`  
  接收机型号、版本和序列号。

- `antsetup`  
  天线安装方式或设置编号。

- `itrf`  
  所属 ITRF realization 年份。

- `deltype`  
  天线偏移是 ENU 还是 XYZ。

- `pos[3]`  
  站点参考坐标，ECEF。

- `del[3]`  
  天线偏移。

- `hgt`  
  天线高。

- `glo_cp_align`、`glo_cp_bias[4]`  
  GLONASS 码相位对齐与偏差相关信息。

### 7.3 它为什么重要

`sta_t` 虽然不像 `obs_t`、`rtk_t` 那样“每历元都在变化”，但它影响很基础：

- 天线相位中心改正
- 基站精确坐标
- 观测站点参考点到天线点的转换
- GLONASS 特殊偏差处理

因此 `sta_t` 属于“静态但非常关键”的信息。

---

## 8. `sol_t`：当前历元解结果

### 8.1 它是什么

`sol_t` 表示某一个历元的解。  
它是 RTKLIB 最直接的输出对象，也是 `rtk_t` 中 `sol` 字段的类型。

### 8.2 标准核心字段

- `time`  
  当前解时间。

- `eventime`  
  事件时间，通常用于 time-mark 或事件触发场景。

- `rr[6]`  
  位置和速度。  
  具体解释由 `type` 决定：
  - `type=0`：ECEF XYZ + 速度
  - `type=1`：ENU baseline + 速度

- `qr[6]`  
  位置协方差。

- `qv[6]`  
  速度协方差。

- `dtr[6]`  
  接收机相对不同时间系统的钟差。

- `type`  
  解类型，坐标表达方式。

- `stat`  
  解状态，比如 single、float、fix、DGPS、PPP 等。

- `ns`  
  有效卫星数。

- `age`  
  差分龄期。

- `ratio`  
  AR ratio 值，用于整周固定判别。

### 8.3 这个仓库对 `sol_t` 的扩展

这份工程里的 `sol_t` 明显加了很多与速度约束、多普勒、TDCP 相关的自定义字段：

- `prev_ratio1`、`prev_ratio2`、`thres`
- `tt`
- `ref_rr[6]`
- `dop[4]`
- `dopvel[4]`
- `sddopvel[3]`
- `tdcpvel[3]`
- `sdtdcpvel[3]`
- `checkvel[3]`
- `checkqv[6]`
- `dopqv[6]`
- `tdcpqv[6]`
- `dopstat`、`tdcpstat`、`checkstat`
- `tdcpns`、`sddopns`、`sdtdcpns`、`dopns`
- `rel_vel[3]`
- `windows_vel[10][3]`
- `predict_vel[3]`
- `para[12]`
- `windows_size`
- `init`
- `vv[3]`
- `qvv[9]`

这些字段说明这套工程不只是输出“位置解”，还在做更深入的：

- Doppler 速度估计
- TDCP 速度估计
- 单差/时差速度对比
- 窗口速度拟合与预测
- 速度一致性检查

所以在这个工程里，`sol_t` 已经从“标准定位结果”扩展成了“定位+速度诊断结果”。

### 8.4 它在系统中的作用

`sol_t` 是对外最常见的数据结构。  
当你看 `.pos` 输出、状态消息、当前解状态时，本质上通常都能追溯到 `sol_t`。

如果说 `rtk_t` 是“求解器的大脑”，那么 `sol_t` 就是“大脑这一刻给出的答案”。

---

## 9. `rtk_t`：RTK 求解状态

### 9.1 它是什么

`rtk_t` 是 RTKLIB 求解器的核心运行时对象。  
它不只是一个结果容器，而是把以下东西都集中放在一起：

- 当前解
- 滤波器状态向量
- 协方差矩阵
- 固定解状态
- 卫星状态
- 模糊度控制信息
- 错误消息
- 当前处理配置

因此可以把 `rtk_t` 理解成：

**“整个 RTK/PPP 状态机的运行时快照”。**

### 9.2 主要字段说明

#### 1. 当前解

- `sol`  
  当前解，类型就是 `sol_t`。

- `rb[6]`  
  基站位置和速度。

#### 2. 状态向量与协方差

- `nx`  
  float 状态维数。

- `na`  
  fixed 状态维数。

- `tt`  
  当前历元与上一历元时间差。

- `x`, `P`  
  float 状态向量及其协方差。

- `xa`, `Pa`  
  fixed 状态向量及其协方差。

这里的 `x` 通常是卡尔曼滤波或相关估计器真正要更新的内部状态，可能包含：

- 位置
- 速度
- 加速度
- 接收机钟差
- 电离层参数
- 对流层参数
- 各卫星各频点模糊度

#### 3. 固定相关状态

- `nfix`  
  连续固定次数。

- `excsat`  
  部分模糊度固定时，下一个待剔除卫星。

- `nb_ar`  
  上一历元参与 AR 的模糊度数量。

- `com_bias`  
  公共相位偏差。

- `holdamb`  
  是否发生过 fix-and-hold。

#### 4. 卫星级状态

- `ambc[MAXSAT]`  
  各卫星模糊度控制信息。

- `ssat[MAXSAT]`  
  各卫星状态信息。

其中 `ssat_t` 非常重要，它保存了每颗卫星的：

- 方位角/高度角
- 伪距、载波、多普勒残差
- 是否有效
- SNR
- slip
- lock/outage/reject 计数
- MW、GF 组合
- 波长
- 各类改正和标志

也就是说，`rtk_t` 不只是“接收机状态”，还包含“每颗卫星当前是否健康、是否可信、是否可用于固定”的完整信息。

#### 5. 错误与配置

- `neb`
- `errbuf`

用于保存错误消息。

- `opt`

当前使用的处理配置，类型就是 `prcopt_t`。

- `initial_mode`

初始化时使用的定位模式。

### 9.3 为什么 `rtk_t` 是最核心的结构体

因为真正“在跑”的不是 `sol_t`，而是 `rtk_t`。  
`sol_t` 只是 `rtk_t` 当前产出的结果切片。

解算器每个历元做的事，本质上都是在更新 `rtk_t`：

1. 根据 `obs_t` 取当前观测
2. 根据 `nav_t` 算卫星位置和改正
3. 用 `prcopt_t` 判断模型和参数
4. 更新 `rtk_t.x / P`
5. 进行 AR
6. 刷新 `rtk_t.ssat[]`
7. 产出 `rtk_t.sol`

所以：

- 要看当前结果，看 `rtk_t.sol`
- 要看滤波内部状态，看 `rtk_t.x`
- 要看卫星质量，看 `rtk_t.ssat`
- 要看 AR 状态，看 `nfix / holdamb / ambc`

---

## 10. 这几个结构体之间怎么配合

可以用一句更偏程序运行的描述来理解：

### 10.1 输入阶段

- `obs_t` 提供观测
- `nav_t` 提供星历和改正
- `sta_t` 提供站点和天线元信息

### 10.2 配置阶段

- `prcopt_t` 指定求解规则
- `filopt_t` 指定外部辅助资源
- `solopt_t` 指定结果输出形式

### 10.3 求解阶段

- `rtk_t` 作为运行时解算器状态被不断更新

### 10.4 输出阶段

- 当前结果写入 `sol_t`
- 再按 `solopt_t` 格式输出

---

## 11. 最容易混淆的几组区别

### 11.1 `obs_t` 和 `nav_t` 的区别

- `obs_t` 是接收机“量到的值”
- `nav_t` 是解释这些观测所需的“先验导航信息和改正信息”

### 11.2 `rtk_t` 和 `sol_t` 的区别

- `rtk_t` 是整个求解器的运行状态
- `sol_t` 是某一时刻的输出答案

### 11.3 `prcopt_t` 和 `solopt_t` 的区别

- `prcopt_t` 决定怎么解
- `solopt_t` 决定怎么写

### 11.4 `filopt_t` 和 `nav_t` 的区别

- `filopt_t` 只是文件路径配置
- `nav_t` 才是文件内容读进来后的内存数据

---

## 12. 总结

如果从“工程角色”角度压缩总结：

- `prcopt_t`：求解规则总表
- `solopt_t`：输出格式总表
- `filopt_t`：辅助文件路径总表
- `obs_t`：观测数据池
- `nav_t`：星历与改正数据池
- `sta_t`：站点/天线元信息
- `rtk_t`：求解器运行时核心状态
- `sol_t`：当前历元答案

如果从“最应该优先理解”的顺序来说，建议按下面顺序读：

1. `prcopt_t`
2. `obs_t`
3. `nav_t`
4. `rtk_t`
5. `sol_t`
6. `sta_t`
7. `solopt_t`
8. `filopt_t`

因为真正理解 RTKLIB 的关键，不在于先记住输出格式，而在于先搞清：

- 输入是什么
- 约束是什么
- 滤波器状态是什么
- 最终输出从哪里来

而这四个问题，分别主要对应：

- `obs_t`
- `prcopt_t`
- `rtk_t`
- `sol_t`
