//
// Created by wangchuji on 2023/8/22.
//

#include "lidar/featureExtraction.h"
#include "lidar/imageProjection.h"
#include "imu/imuPreintegration.h"
#include "gnss/gnssProcessor.h"
#include "mapping/mapOptmizationGps.h"
#include "rosbag/bag_player.h"
#include "rosgraph_msgs/Clock.h"
#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
    backward::SignalHandling sh;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio");

    string GPSfile = "/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/test.pos";
    string bagpath;
    string imu_topic;
    string lidar_topic;
    bool useRoslaunch = false;
    ros::NodeHandle nh("~");
    nh.getParam("useRoslaunch", useRoslaunch);
    if (!useRoslaunch && argv[1] && argv[2] && argv[3])
    {
        bagpath = argv[1];
        imu_topic = argv[2];
        lidar_topic = argv[3];
    }
    else
    {
        nh.getParam("bagpath", bagpath);
        nh.getParam("imu_topic", imu_topic);
        nh.getParam("lidar_topic", lidar_topic);
    }

    rosbag::Bag bag;
    std::vector<std::string> topics;
    topics.push_back(imu_topic);
    topics.push_back(lidar_topic);
    //    gtime_t ts = gpst2time(2290,551805);//551295 551812 552032
    //    gtime_t te = gpst2time(2290,550792);
    //    gtime_t te = gpst2time(2290,552300);
    //    gtime_t ts = gpst2time(2158,95630);
    //    gtime_t ts = gpst2time(2096,549935);wwwwww
    //    gtime_t te = gpst2time(2096,550236);

    // 20240129
    //    gtime_t ts = gpst2time(2299,111965);//gpst2time(2299,112705);//gpst2time(2299,111965);
    //    gtime_t te = gpst2time(2299,112280);
    // gtime_t ts = gpst2time(2299, 111972);
    // gtime_t te = gpst2time(2299, 113000);
    // gtime_t ts = gpst2time(2129,181347);
    // gtime_t te = gpst2time(2129,182154);

    gtime_t ts = gpst2time(2350, 117500);///117500
    gtime_t te = gpst2time(2350, 118300);

    // 20250120_3
    // gtime_t ts = gpst2time(2350, 120671);//120562);120710 120671
    // gtime_t te = gpst2time(2350, 122400);
    // gtime_t te = gpst2time(2350,123700);120562

    // gtime_t ts = gpst2time(2032, 273412);///117500
    // gtime_t te = gpst2time(2032, 274613);
    try
    {
        bag.open(bagpath, rosbag::bagmode::Read);
    }
    catch (std::exception& e)
    {
        ROS_ERROR("Unable to open rosbag: %s", bagpath.c_str());
        return 1;
    }

    gnssProcessor GP;

    ImageProjection IP;

    FeatureExtraction FE;

    IMUPreintegration PT;

    mapOptimization MO;

    TransformFusion TF;

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    auto clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

    //    PT.loadGPSfile(GPSfile);
    PT.setStartTime(ts);
    //    GP.ts = ts;
    GP.decode(ts, te);

    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    int i = 0;
    BOOST_FOREACH(const rosbag::MessageInstance m, view)
    {
        if (m.getTime().toSec() < ((double)ts.time + ts.sec - 18 - 0.005))
            continue;
        if (m.getTime().toSec() > ((double)te.time + te.sec - 18))
            break;
        const sensor_msgs::PointCloud2ConstPtr& cloud = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud)
        {
            //            i = i % 5;
            //            if (fabs(m.getTime().toSec() - (floor(m.getTime().toSec()) + 0.2 * i)) < 0.005){
            //                ROS_INFO("CLOUD %lf", m.getTime().toSec());
            IP.cloudHandler(cloud);
            //                i++;
            //            }
        }
        const sensor_msgs::ImuConstPtr imu = m.instantiate<sensor_msgs::Imu>();
        if (imu)
        {
            IP.imuHandler(imu);
            PT.imuHandler(imu);
        }

        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = m.getTime();
        clock_publisher.publish(clock_msg);
        ros::spinOnce();
    }

    bag.close();

    visualizeMapThread.detach();

    MO.savePath("/mnt/i/20240129/total.pos");

    MO.saveMapService();

    PT.closePosfile();
    ros::spin();

    //    ros::shutdown();

    return 0;
}