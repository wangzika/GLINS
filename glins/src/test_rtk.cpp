//
// Created by wangchuji on 2023/9/19.
//
#include "gnss/gnssProcessor.h"
#include "gnss/gnssEstimator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnss_estimator");
    ROS_INFO("\033[1;32m----> gnss Estimator Started.\033[0m");
    gnssProcessor processor;
    gnssEstimator estimator;
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

//20240129
    gtime_t ts = gpst2time(2299, 111965);
    gtime_t te = gpst2time(2299, 113000);

    processor.decode(ts, te);
    estimator.solveOptimization();
    ros::spin();
    return 0;
}