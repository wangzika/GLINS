# GLINS 中 GNSS 原始观测如何加入因子图

## 1. 文档目的

这份文档专门解释当前 `GLINS` 工程里：

- `rover.obs / base.obs / nav` 这类 GNSS 原始观测文件
- 是如何经过 RTKLIB 前端处理
- 变成 ROS 消息 `/gnss_raw`
- 再被 `gnssContainer` 接收、筛选、同步
- 最终构造成伪距 / 载波 / 多普勒因子加入 GTSAM 因子图

如果只想记一句话，可以先记住：

**这套工程不是直接把 `rover.obs` 文本文件逐行喂进因子图，而是先由 RTKLIB 把原始观测整理成 `GNSS_Info`，再由 `gnssContainer` 按当前优化时刻同步，并构造成 GTSAM 因子。**

---

## 2. 总体数据流

从文件到因子图，主链如下：

```text
rtklibConfigPath 指向的 .conf
    ↓
gnssProcessor 读取 inpstr1/2/3-path
    ↓
RTKLIB postpos()
    ↓
rtkpos.cpp 发布 /gnss_raw (rtklib::GNSS_Info)
    ↓
IMUPreintegration 内部的 gnssContainer 订阅 /gnss_raw
    ↓
gnssInfoHandler() 把 GNSS_Info 转成队列缓存
    ↓
syncObs() 按当前优化时刻取出匹配的 GNSS 历元
    ↓
addGPSFactor()
    ↓
按模式加入：
  - 普通 GPS 位置因子
  - 双差伪距因子
  - 双差载波因子
  - 单差多普勒因子
```

---

## 3. 原始观测文件从哪里来

当前工程里，GNSS 原始文件路径不是写死在 `test_lio.cpp` 里，而是主要来自：

- launch / yaml 里的 `glins/rtklibConfigPath`
- 再由这个路径指向一份 RTKLIB 配置文件 `.conf`

例如：

- [params_20240129.yaml](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/params_20240129.yaml:26)
  ```yaml
  rtklibConfigPath: /catkin_ws/src/glins/config/conf/20240129.conf
  ```

在 [20240129.conf](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/config/conf/20240129.conf:135) 里，真正的输入文件路径是：

```text
inpstr1-path = /data/gnss/rover.obs
inpstr2-path = /data/gnss/base.obs
inpstr3-path = /data/gnss/BRDM00DLR_S_20240290000_01D_MN.rnx
outstr1-path = /output/rtklib.pos
```

也就是说：

- `inpstr1-path`：rover 观测文件
- `inpstr2-path`：base 观测文件
- `inpstr3-path`：导航/星历文件
- `outstr1-path`：RTKLIB 输出结果文件

所以主程序真正依赖的是：

**`rtklibConfigPath -> .conf -> inpstr*-path`**

而不是直接在 `test_lio.cpp` 里手工打开 `rover.obs`。

---

## 4. 主程序是怎么启动 GNSS 前端的

在 [test_lio.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_lio.cpp:92) 以后，主程序创建了：

- `gnssProcessor GP;`
- `IMUPreintegration PT;`

随后在 [test_lio.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_lio.cpp:118)：

```cpp
GP.decode(ts, te);
```

这里的作用是：

- 让 RTKLIB 在时间区间 `[ts, te]` 内先完成 GNSS 后处理解算
- 同时在解算过程中对外发布 `GNSS_Info`

GNSS 前端主入口在：

- [gnssProcessor.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:105)

```cpp
int stat = postpos(ts, te, ti, tu, &prcopt, &solopt, &filopt, infile, n, outfile, rov, base);
```

也就是：

**`gnssProcessor::decode()` 不是自己写观测解析，而是直接调用 RTKLIB 的 `postpos()`。**

---

## 5. GNSS 原始观测怎么变成 `/gnss_raw`

### 5.1 发布接口在哪注册

在 [gnssProcessor.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssProcessor.h:51)：

```cpp
rtkposRegisterPub(nh);
pntposRegisterPub(nh);
```

其中 `rtkposRegisterPub(nh)` 的实现位于：

- [rtkpos.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/src/rtkpos.cpp:113)

```cpp
pub_raw = n.advertise<rtklib::GNSS_Info>("/gnss_raw", 100);
pub_odom = n.advertise<nav_msgs::Odometry>("/rtklib_odom", 100);
```

这说明：

- RTKLIB 前端会发布 `/gnss_raw`
- 消息类型是 `rtklib::GNSS_Info`

### 5.2 `GNSS_Info` 里有什么

消息定义在：

- [GNSS_Info.msg](/Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/msg/GNSS_Info.msg:1)

关键字段包括：

- `stat`：RTKLIB 解状态
- `gpsWeek` / `weekSec`：GNSS 时间
- `pos`：ECEF 位置
- `vel`：速度
- `var`：位置方差
- `amb`：模糊度数组
- `ZD_Infos`：零差观测信息
- `SD_Infos`：双差观测信息

其中：

- [GNSS_Info_ZD.msg](/Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/msg/GNSS_Info_ZD.msg:1)
  - 保存零差观测、卫星状态等
- [GNSS_Info_SD.msg](/Users/wangzhibo/Desktop/博士研究/GLINS/rtklib/msg/GNSS_Info_SD.msg:1)
  - 保存 rover/base 双差相关观测、卫星状态等

也就是说：

**进入后端的不是“纯位置”，而是一整个经过 RTKLIB 整理过的 GNSS 历元包。**

---

## 6. 后端是怎么接收 `/gnss_raw` 的

`IMUPreintegration` 构造时会配置内部的 `gnssContainer`：

- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:210)

```cpp
container.loadrtklibConfig(rtklibConfigPath);
container.registerGnssSubscriber(nh);
container.setSystemInitialized(systemInitialized);
container.setEstimateExtGPS(estimateExtGPS);
container.setExtRot(extRot);
container.setExtGPS(extGPS);
```

其中订阅注册在：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:230)

```cpp
sub_gnss_raw = nh.subscribe<rtklib::GNSS_Info>("/gnss_raw", 100, &gnssContainer::gnssInfoHandler, this, ...);
```

所以 GNSS 后端主入口回调就是：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:798)
  - `gnssInfoHandler(...)`

---

## 7. `gnssInfoHandler()` 做了什么

`gnssInfoHandler()` 的角色不是立刻加因子，而是先做：

1. 基本合法性检查
2. 初始化 ENU 原点
3. 把 ECEF 位置转成 ENU
4. 构造一条 `nav_msgs::Odometry`
5. 把 `Odometry + GNSS_Info` 组成 `GNSSPose`
6. 压入内部队列 `gnss_queue`

关键代码在：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:840)

```cpp
GNSSPose gnss_pose_msg(odom_msg, *gnss_msg);
gnss_queue.push_back(gnss_pose_msg);
```

这里 `gnss_queue` 的类型是：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:63)

```cpp
std::deque<GNSSPose> gnss_queue;
```

而：

```cpp
typedef pair<nav_msgs::Odometry, rtklib::GNSS_Info> GNSSPose;
```

所以队列里的每个元素都同时保存了：

- 一个已经转成 ENU 的 GNSS 里程计表示
- 一份完整的 `GNSS_Info` 原始观测包

---

## 8. 为什么主循环里没显式处理 GNSS，GNSS 还是能进来

在 [test_lio.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/src/test_lio.cpp:123) 的 bag 主循环里，程序只显式处理了：

- `PointCloud2`
- `Imu`

没有看到像这样：

```cpp
m.instantiate<rtklib::GNSS_Info>()
```

原因是：

- GNSS 不走 bag 主循环手工分发
- 而是走 ROS topic 异步链：

```text
GP.decode() -> /gnss_raw -> gnssContainer::gnssInfoHandler()
```

主循环里的：

```cpp
ros::spinOnce();
```

就是让这些异步 GNSS 回调在每一轮 bag 推进时真正执行。

---

## 9. GNSS 什么时候真正加入因子图

GNSS 真正进入因子图，是在 `IMUPreintegration::addGPSFactor(...)` 里完成的。

位置在：

- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:280)

但在调用它之前，程序会先判断当前时刻附近是否有 GNSS 可用。

在 `featureHandler()` 里：

- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:734)

```cpp
if (GNSSEnable = container.isGNSSEnable(currentCorrectionTime))
{
    gpsKeyQueue.push_back(key);
    if (useGPS)
    {
        ROS_INFO("GPS KEY %d", key);
        addGPSFactor(nb, npr, ndop);
    }
}
```

这说明：

- GNSS 不是每一帧 feature 都强行加
- 而是要先判断当前优化时刻附近，是否确实有一条可同步 GNSS 历元

---

## 10. `syncObs()` 是怎么做时间同步的

进入 `addGPSFactor()` 后，首先会调用：

- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:285)

```cpp
if (container.syncObs(lastImuT_opt, 0.015))
```

`syncObs()` 的实现位于：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:885)

它会查看 `gnss_queue.front()`，判断最前面的那条 GNSS 是否：

- 太早：
  - `pop_front()` 丢掉
- 太晚：
  - 先不取，等待后续时刻
- 正好落在 `timestamp ± eps_cam` 内：
  - 取出这条 GNSS
  - 写入：
    - `gnss_odom`
    - `gnss_info`

也就是说：

**`syncObs()` 做的是“当前优化时刻和 GNSS 历元的时间对齐”。**

一旦对齐成功，后面构造因子时使用的就是：

- `gnss_odom`
- `gnss_info`

这两个当前缓存。

---

## 11. `addGPSFactor()` 是总调度入口

`IMUPreintegration::addGPSFactor(...)` 是原始观测入图的总入口。

位置：

- [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:280)

逻辑分两大类：

### 11.1 `useObs == false`

只加入普通 GPS 位置因子：

```cpp
container.addGPSFactorENU(&graphFactors, &graphValues, key);
```

也就是把 GNSS 当成“位置观测结果”来用。

---

### 11.2 `useObs == true`

加入原始观测因子链：

```cpp
npr = container.addDDPsrFactorENU(&graphFactors, &graphValues, key);
if (useCarrier)
{
    nb = container.addDDCpFactorENU(&graphFactors, &graphValues, key, lastGNSSepoch, allowCarrierTemporalLink);
}
ndop = container.addSDDopFactorENU(&graphFactors, &graphValues, key, lastGNSSepoch);
```

也就是：

- 双差伪距因子
- 双差载波因子
- 单差多普勒因子

所以原始观测真正加入图的主入口，就是这 3 个函数。

---

## 12. 双差伪距因子是怎么加的

函数：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:239)
  - `addDDPsrFactorENU(...)`

主要步骤：

1. 遍历 `gnss_info.SD_Infos`
2. 按系统和频点挑选参考星 `Info_Master`
3. 对其他卫星构造 `Infos_Else`
4. 做观测合法性筛选：
   - 伪距非零
   - 卫星高度角有效
   - `satexclude(...)`
   - `testsnr(...)`
5. 用当前 `X(key)` 初值先做一次残差预检查
6. 构造 `GNSSDDPsrFactor_ENU` 或 `..._lever`
7. 加入 `graphFactors`

这里它使用的数据源是：

- `gnss_info.SD_Infos`

也就是：

**RTKLIB 已经帮你整理好的双差观测结构。**

---

## 13. 双差载波因子是怎么加的

函数：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:338)
  - `addDDCpFactorENU(...)`

这部分最复杂，因为它除了加因子，还会引入 ambiguity 状态 `N(key)`。

主要步骤：

1. 检查 `gnss_info.amb` 是否为空
2. 用 `SD_Infos` 更新 `rtk.ssat[...]` 的卫星跟踪状态
3. 做 slip / out / lock / ambiguity 续接管理
4. 对每个系统、每个频点选参考星
5. 构造当前 epoch 的 `Infos_Else`
6. 用当前 `X(key)` 初值和临时 ambiguity 做残差预检查
7. 对残差超限观测做 outlier 剔除
8. 构造当前 ambiguity 向量 `compress_ar`
9. 给新的 ambiguity 分量建立：
   - 相位先验
   - 或与 `N(lastkey)` 的连续性连接
10. 构造：
   - `GNSSDDCpFactorCompress_ENU`
   - 或 `..._lever`
11. 如果当前 epoch 通过筛选：
   - `graphValues->insert(N(key), compress_ar)`
   - 把本 epoch 的 carrier 因子整体加入图

所以载波链的特点是：

- 不只是观测因子
- 还要额外管理 `N(key)` 这条 ambiguity 状态链

---

## 14. 单差多普勒因子是怎么加的

函数：

- [gnssContainer.h](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/gnss/gnssContainer.h:633)
  - `addSDDopFactorENU(...)`

它主要使用：

- `gnss_info.ZD_Infos`

也就是零差观测信息。

主要步骤：

1. 选参考星 `Info_Master`
2. 收集同系统、同频点下其他卫星 `Infos_Else`
3. 筛选：
   - 多普勒非零
   - `vsatD` 有效
   - 高度角足够
   - `lam[f]` 有效
4. 构造 `GNSSSDDopFactor_ENU`
5. 连接：
   - `X(lastkey)`
   - `X(key)`

它提供的是：

**基于多普勒的速度/位姿变化约束。**

---

## 15. 外参状态 `T(key)` 是怎么接进去的

如果打开：

- `estimateExtGPS == true`

那么在 [imuPreintegration.cpp](/Users/wangzhibo/Desktop/博士研究/GLINS/glins/include/imu/imuPreintegration.cpp:309) 以后，还会：

```cpp
graphValues.insert(T(key), extGPS);
PriorFactor<Vector3> priorExtGPS(T(key), extGPS, priorExtNoise);
```

并在连续时刻之间加入：

```cpp
BetweenFactor<Vector3>(T(key - 1), T(key), Vector3::Zero(), ...)
```

于是伪距/载波因子也会切换成 `..._lever` 版本，把外参一起作为优化变量。

所以：

- 不估外参时：GNSS 因子只约束 `X(key)` 或 `X(key), N(key)`
- 估外参时：GNSS 因子还会额外约束 `T(key)`

---

## 16. 模式开关怎么影响原始观测入图

这几项参数最关键：

### `useGPS`

- 总开关
- 不开则根本不加 GNSS 因子

### `useObs`

- `false`：只加普通 GPS 位置因子
- `true`：进入原始观测建图模式

### `useCarrier`

- 只有在 `useObs == true` 时才有意义
- 打开后才会接入双差载波和 ambiguity 状态 `N(key)`

### `useAmbFix`

- 只有在 `useCarrier == true` 时才有意义
- 控制是否继续做整数固定

### `estimateExtGPS`

- 控制是否把 `T(key)` 作为状态一起估计

---

## 17. 为什么说“GNSS 原始观测”不是直接等于“位置因子”

因为原始观测模式下，后端并不是只用：

- `gnss_msg->pos`

而是同时使用：

- `SD_Infos`
- `ZD_Infos`
- `amb`
- `ssat` 卫星状态
- `SNR`
- `slip`
- `azel`

这些更底层的信息来构造：

- 双差伪距因子
- 双差载波因子
- 单差多普勒因子

所以：

**普通 GPS 模式用的是“GNSS 解结果”，原始观测模式用的是“GNSS 解过程里整理出来的观测结构”。**

---

## 18. 一句话总结

在这个工程里，GNSS 原始观测加入因子图的完整链路是：

**`rover.obs/base.obs/nav -> gnssProcessor::decode() -> RTKLIB postpos() -> 发布 /gnss_raw(GNSS_Info) -> gnssContainer::gnssInfoHandler() 入队 -> syncObs() 按当前优化时刻取出 -> IMUPreintegration::addGPSFactor() -> addDDPsrFactorENU / addDDCpFactorENU / addSDDopFactorENU -> 加入 GTSAM 因子图。`**
