//
// Created by wangchuji on 2023/9/19.
//
#include "gnss/gnssProcessor.h"
#include "gnss/gnssEstimator.h"

int main(int argc, char** argv)
{
    /*
     * 纯 GNSS/RTK 入口。
     *
     * 运行链路：
     *   1. roslaunch 先加载 glins/config/params_user.yaml。
     *   2. ParamServer 从参数服务器读取 glins/rtklibConfigPath。
     *   3. gnssProcessor 构造函数读取 RTKLIB .conf 文件，拿到 rover/base/nav/out 路径。
     *   4. processor.decode() 调用 RTKLIB 的 postpos() 完成后处理解算。
     *   5. 默认只输出 RTKLIB 结果文件；如果 runFgoOptimization=true，才继续进入 GNSS FGO 优化。
     */
    ros::init(argc, argv, "gnss_estimator");
    ROS_INFO("\033[1;32m----> gnss Estimator Started.\033[0m");
    ros::NodeHandle nh;

    /*
     * gnssProcessor 负责 RTKLIB 后处理。
     * gnssEstimator 负责项目自定义的 GNSS 因子图优化，不属于“纯 RTK”必需流程。
     */
    gnssProcessor processor;
    gnssEstimator estimator;

    /*
     * 纯 RTK 模式默认不运行 FGO。
     * run_rtk.launch 中 run_fgo:=true 时，才会让 estimator 订阅 /gnss_raw 并做优化。
     */
    bool run_fgo_optimization = false;
    nh.param<bool>("glins/runFgoOptimization", run_fgo_optimization, false);
    // gtime_t ts = {0};
    // gtime_t te = {0};
//    gtime_t ts = gpst2time(2290,549500);
//    gtime_t te = gpst2time(2290,549600);
//    gtime_t te = gpst2time(2290,552600);
//    gtime_t ts = gpst2time(2192,462940);
//    gtime_t te = gpst2time(2192,463080);
//    processor.decode(ts,te);
// hk20200314
//    gtime_t ts = gpst2time(2096,549935);
//    gtime_t te = gpst2time(2096,550238);
//opensky
//    gtime_t ts = gpst2time(2188,458040);
//    gtime_t te = gpst2time(2188,458580);

//    gtime_t ts = gpst2time(2188,458270);
//    gtime_t te = gpst2time(2188,458042);

// 20240705
//    gtime_t ts = gpst2time(2321,428303.000 );
//    gtime_t te = gpst2time(2321,436150.000 );

    int start_week = 0;
    int end_week = 0;
    double start_sec = 0.0;
    double end_sec = 0.0;
    nh.param<int>("glins/startWeek", start_week, 0);
    nh.param<double>("glins/startSec", start_sec, 0.0);
    nh.param<int>("glins/endWeek", end_week, start_week);
    nh.param<double>("glins/endSec", end_sec, 0.0);

    /*
     * RTKLIB 使用 gtime_t 表示时间。
     * start_week/start_sec 为 GPS 周和周内秒；传 0 表示不限制起止时间，解算全部观测数据。
     */
    gtime_t ts = start_week > 0 ? gpst2time(start_week, start_sec) : gtime_t{0};
    gtime_t te = end_week > 0 && end_sec > 0.0 ? gpst2time(end_week, end_sec) : gtime_t{0};

    /* 调用 RTKLIB 后处理主流程，输出文件由 .conf 中 outstr1-path 决定。 */
    processor.decode(ts, te);
    if (!run_fgo_optimization)
    {
        ROS_INFO("\033[1;32m----> RTK only mode finished. Skip FGO optimization.\033[0m");
        return 0;
    }

    /*
     * GNSS FGO 调试入口。
     * 注意：这里不是普通 RTK 解算必需项；如果 RTK 结果质量差或消息队列异常，可能影响稳定性。
     */
    estimator.solveOptimization();
    ros::spin();
    return 0;
}
