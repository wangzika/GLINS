//
// Created by wangchuji on 2023/8/16.
//

#include "utility.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

#include "gnss_tools.h"
#include "gtsam/linear/NoiseModel.h"
#include "rosbag/bag.h"
#include "rosbag/bag_player.h"
#include "gnss/gnssProcessor.h"
#include "marginalize_isam2.h"
#include "time.h"
#include "gnss/gnssContainer.h"
#include "factor/NhcFactor.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
typedef pair<nav_msgs::Odometry, rtklib::GNSS_Info> GNSSPose;
#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
    backward::SignalHandling sh;
}
// static int test_sys(int sys, int m) {
//     switch (sys) {
//         case SYS_GPS:
//             return m == 0;
//         case SYS_SBS:
//             return m == 0;
//         case SYS_GLO:
//             return m == 1;
//         case SYS_GAL:
//             return m == 2;
//         case SYS_CMP:
//             return m == 3;
//         case SYS_QZS:
//             return m == 4;
//         case SYS_IRN:
//             return m == 5;
//     }
//     return 0;
// }

typedef std::pair<double, gtsam::Pose3> Frame;

class FrameData
{
public:
    int week;
    double weeksec;
    Vector3d translation;
    Vector3d ypr;
    double timestamp;
    Pose3 pose;
    FrameData() {}
    FrameData(int week_, double weeksec_, Vector3d& translation_)
    {
        week = week_;
        weeksec = weeksec_;
        translation = translation_;
        gtime_t time = gpst2time(week, weeksec);
        timestamp = time.sec + (double)time.time - 18;
    }

    FrameData(int week_, double weeksec_, Vector3d& translation_, Vector3d& ypr_)
    {
        week = week_;
        weeksec = weeksec_;
        translation = translation_;
        ypr = ypr_;
        gtime_t time = gpst2time(week, weeksec);
        timestamp = time.sec + (double)time.time - 18;
        pose = Pose3(Rot3::RzRyRx(ypr(2), ypr(1), ypr(0)), translation);
    }

    FrameData(int week_, double weeksec_, double e, double n, double u)
    {
        week = week_;
        weeksec = weeksec_;
        translation = Vector3(e, n, u);
        gtime_t time = gpst2time(week, weeksec);
        timestamp = time.sec + (double)time.time - 18;
    }

    FrameData(int week_, double weeksec_, double e, double n, double u, double y, double p, double r)
    {
        week = week_;
        weeksec = weeksec_;
        translation = Vector3(e, n, u);
        ypr = Vector3(y, p, r);
        gtime_t time = gpst2time(week, weeksec);
        timestamp = time.sec + (double)time.time - 18;
        pose = Pose3(Rot3::RzRyRx(r, p, y), translation);
    }
};
// #define USE_COMBINED_IMU 1
class test_GINS : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Publisher pubImuOdometry;
    ros::Publisher pubGPSPath;
    ros::Publisher pubIMUPath;
    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::noiseModel::Diagonal::shared_ptr priorExtNoise;
    gtsam::Vector noiseModelBetweenBias;
#ifdef USE_COMBINED_IMU
    gtsam::PreintegratedCombinedMeasurements* imuIntegratorOpt_;
    gtsam::PreintegratedCombinedMeasurements* imuIntegratorImu_;

#else
    gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements* imuIntegratorImu_;
#endif
    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;
    Vector prevAmb_;

    gtsam::NavState prevStateOdom;//
    gtsam::imuBias::ConstantBias prevBiasOdom;//先验

    MatrixXd posCovariance;
    MatrixXd ambCovariance;

    bool doneFirstOpt = false;
    double thisImuT_imu = -1;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;
    gtsam::Values result;

    std::deque<GNSSPose> gpsMeasQueue;
    std::deque<sensor_msgs::Imu> imuQueue;
    std::deque<FrameData> lioqueue;

    double average_time = 0.0;
    const double delta_t = 0;

    rtklib::GNSS_Info gnss_info;
    int lastGNSSepoch = 0;
    double yaw = 0.0, prevYaw = 0.0;
    Eigen::Vector3d prevPos = Eigen::Vector3d(0, 0, 0);
    geometry_msgs::Quaternion yawQuat;
    nav_msgs::Path gpsOdomPath;

    int key = 0;

    nav_msgs::Path imuPath;

    // T_bl: tramsform points from lidar frame to imu frame
    gtsam::Pose3
        imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
            gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3
        lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    gtsam::Pose3
        gps2imu;
    gtsam::Pose3
        imu2gps = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extGPS.x(), -extGPS.y(), -extGPS.z()));

    gtsam::Pose3 gpsPose;
    string result_path = "";
    FILE* fp;
    FILE* fp_incre;

    std::vector<std::string> topics;
    std::vector<Frame> frames;

    Vector3 lla_origin;

    gtime_t ts, te;

    gnssContainer container;

    FrameData curlioframe;
    FrameData lastlioframe;

    LevenbergMarquardtParams lm_params;
    test_GINS()
    {

        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 2000);

        pubGPSPath = nh.advertise<nav_msgs::Path>("glins/gps/path", 1);
        pubIMUPath = nh.advertise<nav_msgs::Path>("glins/imu/fgo_path", 1);
#ifdef USE_COMBINED_IMU
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> p = gtsam::PreintegrationCombinedParams::MakeSharedU(imuGravity);
#else
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
#endif
        gps2imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extGPS.x(), extGPS.y(), extGPS.z()));

        p->accelerometerCovariance =
            gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance =
            gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance =
            gtsam::Matrix33::Identity(3, 3) *
            pow(1e-4, 2); // error committed in integrating position from velocities
#ifdef USE_COMBINED_IMU
        p->biasAccCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccBiasN, 2);
        p->biasOmegaCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrBiasN, 2);
        p->biasAccOmegaInt = gtsam::Matrix66::Identity(6, 6) * 1e-5;
#else

#endif
        // gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6)
        //         <<  -0.06, 0.13, 0.03, -5e-5, -5e-5,-2.2e-4).finished());; // assume zero initial bias
        //        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6)
        //                << 0.008587789965354,-0.005013607732023,0.03590594366906,-1.450567641497e-05,-0.0006468714529474,-0.0004370968510109).finished());; // assume zero initial bias
        // assume zero initial bias
        gtsam::imuBias::ConstantBias
            prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
        ;
        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)
            << 1e-2,
            1e-2, 1e-2, 1e-2, 1e-2, 1e-2)
            .finished()); // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);           // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);         // 1e-2 ~ 1e-3 seems to be good
        priorExtNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)
            << 0.05,
            0.05, 0.05, 0.1, 0.1, 0.1)
            .finished()); // rad,rad,rad,m, m, m
        correctionNoise2 =
            gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias =
            (gtsam::Vector(6)
                << imuAccBiasN,
                imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN)
            .finished();

        if (sensor == SensorType::HESAI)
        {
            noiseModelBetweenBias =
                (gtsam::Vector(6)
                    << imuAccBias_N[0],
                    imuAccBias_N[1], imuAccBias_N[2], imuGyrBias_N[0], imuGyrBias_N[1], imuGyrBias_N[2])
                .finished();
        }
        else
        {
            noiseModelBetweenBias =
                (gtsam::Vector(6)
                    << imuAccBiasN,
                    imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN)
                .finished();
        }

#ifdef USE_COMBINED_IMU
        imuIntegratorImu_ = new gtsam::PreintegratedCombinedMeasurements(p,
            prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedCombinedMeasurements(p,
            prior_imu_bias);
#else
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p,
            prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ =
            new gtsam::PreintegratedImuMeasurements(p,
                prior_imu_bias); // setting up the IMU integration for optimization
#endif
        // init rtk options
        container.loadrtklibConfig(rtklibConfigPath);
        container.registerGnssSubscriber(nh);
        container.setSystemInitialized(systemInitialized);
        container.setEstimateExtGPS(estimateExtGPS);
        container.setExtRot(extRot);
        container.setExtGPS(extGPS);
    }
    void set_time(gtime_t start_time = { 0 }, gtime_t end_time = { 0 })
    {
        ts = start_time;
        te = end_time;
    }

    void run()
    {
        //        string GPSfile = "/home/wangchuji/catkins_gnss/OB_GINS-main/dataset/GNSS_RTK.pos";
        //        string IMUfile = "/home/wangchuji/catkins_gnss/OB_GINS-main/dataset/ADIS16465.txt";

        string imu_topic = "/imu/data";
        topics.push_back(imu_topic);

        //        string GPSfile = "/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/test.pos";
        //        string IMUfile = "/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/imu/xsense_imu.txt";
        //        string IMUfile = "/home/wangchuji/catkins_gnss/rtklib_GSDC_6th/data/GNSS_INS_Data/urban/imu_adis16465.txt";
        string IMUfile = "/mnt/i/20240705/imu_adis16465.txt";

        result_path = fgoPath; //"/home/wangchuji/catkins_gnss/rtklib_GSDC_6th/data/GNSS_INS_Data/urban/gins_amb_ac.pos";
        //        string IMUbag = "/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/lidar_imu_backup.bag";
        //        string IMUbag = "/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Data20200314/HK20200314.bag";
        //        string IMUbag = "/media/wangchuji/T7/20231202/mems_.bag";
        //        string IMUbag = "/media/wangchuji/T7/20240129/imu/mems.bag";
        // string IMUbag = "/mnt/i/20240129/imu/mems.bag";
        // string IMUbag = "/mnt/i/GREATWHUdata/campus-02/cp_02.bag";
        // string IMUbag = "/mnt/i/20250120/urban_01/1624_filtered.bag";
        // string IMUbag = "/mnt/i/20250120/urban_03/1719_filtered.bag";
        string IMUbag = "/mnt/i/HKdataset/UrbanNav-HK_Whampoa-20210517_sensors/UrbanNav-HK_Whampoa-20210521_sensors.bag";
        //        string IMUbag = "/media/wangchuji/T7/20221029/imu_adis16465.bag";
        //        result_path = "/home/wangchuji/catkins_gnss/rtklib_GSDC_6th/data/hk1/gins_amb_ac.pos";
        fp = fopen(result_path.c_str(), "w");
        // fp_incre = fopen("/mnt/i/20240129/imu/incre.txt", "w");
        // fp_incre = fopen("/mnt/i/20250120/urban_01/incre.txt", "w");
        fp_incre = fopen("/mnt/i/20250120/urban_03/incre.txt", "w");
        fprintf(fp, "%%  GPST              x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns   sdx(m)   sdy(m)   sdz(m)  sdxy(m)  sdyz(m)  sdzx(m) age(s)  ratio\n");
        /// 改成用文件读取数据
        // loadLIOpos("/mnt/i/20240705/lidar_BLH.txt");
        // loadLIOpos("/mnt/i/GREATWHUdata/campus-02/cp_02_lidar.pos");
        // loadLIOpos("/mnt/i/20240129/final/lio_fgo.pos");
        // loadLIOpos("/mnt/i/20250120/urban_01/lio.pos");
        loadLIOpos("/mnt/i/20250120/urban_03/lio.pos");
        //        loadGPSfile(GPSfile);
        //    loadIMUfile(IMUfile);
        loadIMUbag(IMUbag);
        fclose(fp);
    }

    void loadGPSfile(string GPSfile)
    {
        fstream fp;
        fp.open(GPSfile, std::ios::in);
        string line;
        vector<string> data;
        while (!fp.eof())
        {
            getline(fp, line);
            boost::split(data, line, boost::is_any_of(" "), boost::token_compress_on);
            rtklib::GNSS_InfoPtr msgPtr(new rtklib::GNSS_Info());
            //            msgPtr->header.stamp.fromSec(stod(data.at(0)));
            //            for (int i = 0; i < 3; ++i) {
            //                msgPtr->pos[i] = stod(data.at(i + 1));
            //                msgPtr->var[i] = stod(data.at(i + 4));
            //            }

            gtime_t gtime = gpst2time(stod(data.at(0)), stod(data.at(1)));
            msgPtr->header.stamp.fromSec((double)gtime.time + gtime.sec - 18);
            Vector3 ecef(stod(data.at(2)), stod(data.at(3)), stod(data.at(4)));
            Vector3 lla = GNSS_Tools::ecef2llh(ecef); // gtools.ECEF2LLA(ecef);
            for (int i = 0; i < 3; ++i)
            {
                msgPtr->pos[i] = lla(i);
                msgPtr->var[i] = stod(data.at(i + 5));
            }
            //            gnssInforHandler(msgPtr);
        }
    }

    void loadIMUfile(string IMUfile)
    {
        fstream fp;
        fp.open(IMUfile, std::ios::in);
        string line;
        vector<string> data;
        ROS_INFO("LOAD_IMU");
        while (!fp.eof())
        {
            //            if (gpsMeasQueue.empty()) break;
            getline(fp, line);
            if (line.empty())
                continue;
            boost::split(data, line, boost::is_any_of(" "), boost::token_compress_on);
            sensor_msgs::ImuPtr imu_msgPtr(new sensor_msgs::Imu());
            //            used for increment
            //            imu_msgPtr->header.stamp.fromSec(stod(data.at(0)));
            //            imu_msgPtr->angular_velocity.x = stod(data.at(1)) * 200;
            //            imu_msgPtr->angular_velocity.y = stod(data.at(2)) * 200;
            //            imu_msgPtr->angular_velocity.z = stod(data.at(3)) * 200;
            //            imu_msgPtr->linear_acceleration.x = stod(data.at(4)) * 200;
            //            imu_msgPtr->linear_acceleration.y = stod(data.at(5)) * 200;
            //            imu_msgPtr->linear_acceleration.z = stod(data.at(6)) * 200;

            //            used for hk data
            //            imu_msgPtr->header.stamp.fromSec(stod(data.at(0))*1e-9);
            //            imu_msgPtr->angular_velocity.x = stod(data.at(1));
            //            imu_msgPtr->angular_velocity.y = stod(data.at(2));
            //            imu_msgPtr->angular_velocity.z = stod(data.at(3));
            //            imu_msgPtr->linear_acceleration.x = stod(data.at(4));
            //            imu_msgPtr->linear_acceleration.y = stod(data.at(5));
            //            imu_msgPtr->linear_acceleration.z = stod(data.at(6));
            //            used for urban data
            gtime_t gpst = gpst2time(stoi(data.at(1)), stod(data.at(2)));
            //            ROS_INFO("week %d, sec %f",stoi(data.at(1)), stod(data.at(2)));
            if ((((double)gpst.time + gpst.sec) < ((double)ts.time + ts.sec)) || (((double)gpst.time + gpst.sec) > ((double)te.time + te.sec)))
                continue;
            imu_msgPtr->header.stamp.fromSec((double)gpst.time + gpst.sec - 18);
            //            imu_msgPtr->angular_velocity.x = stod(data.at(3)) * 100;
            //            imu_msgPtr->angular_velocity.y = stod(data.at(4)) * 100;
            //            imu_msgPtr->angular_velocity.z = stod(data.at(5)) * 100;
            //            imu_msgPtr->linear_acceleration.x = stod(data.at(6)) * 100;
            //            imu_msgPtr->linear_acceleration.y = stod(data.at(7)) * 100;
            //            imu_msgPtr->linear_acceleration.z = stod(data.at(8)) * 100;

            // imu_msgPtr->angular_velocity.x = stod(data.at(4)) * 100;
            // imu_msgPtr->angular_velocity.y = stod(data.at(3)) * 100;
            // imu_msgPtr->angular_velocity.z = -stod(data.at(5)) * 100;
            // imu_msgPtr->linear_acceleration.x = stod(data.at(7)) * 100;
            // imu_msgPtr->linear_acceleration.y = stod(data.at(6)) * 100;
            // imu_msgPtr->linear_acceleration.z = -stod(data.at(8)) * 100;
            imu_msgPtr->angular_velocity.x = -stod(data.at(3)) * 100;
            imu_msgPtr->angular_velocity.y = stod(data.at(4)) * 100;
            imu_msgPtr->angular_velocity.z = -stod(data.at(5)) * 100;
            imu_msgPtr->linear_acceleration.x = -stod(data.at(6)) * 100;
            imu_msgPtr->linear_acceleration.y = stod(data.at(7)) * 100;
            imu_msgPtr->linear_acceleration.z = -stod(data.at(8)) * 100;
            imuHandler(imu_msgPtr);
        }
    }

    void loadLIOfile(string LIOfile)
    {
        fstream fp;
        fp.open(LIOfile, std::ios::in);
        string line;
        vector<string> data;
        while (!fp.eof())
        {
            getline(fp, line);
            boost::split(data, line, boost::is_any_of(" "), boost::token_compress_on);
            gtsam::Pose3 lidarPose(gtsam::Rot3::Quaternion(stod(data.at(7)), stod(data.at(4)), stod(data.at(5)), stod(data.at(6))),
                Point3(stod(data.at(1)), stod(data.at(2)), stod(data.at(3))));
            Frame lidarframe;
            lidarframe.first = stod(data.at(0));
            lidarframe.second = lidarPose;
            frames.push_back(lidarframe);
        }
    }

    void loadLIOpos(string LIOfile)
    {
        fstream fp;
        fp.open(LIOfile, std::ios::in);
        string line;
        vector<string> data;
        lla_origin = container.getOrigin();
        while (!fp.eof())
        {
            getline(fp, line);
            if (line.empty())
                continue;
            if (line[0] == '%')
                continue;
            boost::split(data, line, boost::is_any_of(" "), boost::token_compress_on);
            gtime_t gpst = gpst2time(stoi(data.at(0)), stod(data.at(1)));
            if (((double)gpst.time + gpst.sec) < ((double)ts.time + ts.sec) || (((double)gpst.time + gpst.sec) > ((double)te.time + te.sec)))
                continue;
            // Vector3 pos(stod(data.at(2)), stod(data.at(3)), stod(data.at(4)));
            // Vector3 ecef = GNSS_Tools::llh2ecef(pos);
            Vector3 ecef(stod(data.at(2)), stod(data.at(3)), stod(data.at(4)));
            Vector3 base_ecef = GNSS_Tools::llh2ecef(lla_origin);
            Vector3 ecef_vec(ecef[0] - base_ecef[0], ecef[1] - base_ecef[1], ecef[2] - base_ecef[2]);
            Vector3 enu = GNSS_Tools::ecef2enu(lla_origin, ecef);
            FrameData frame(stoi(data.at(0)), stod(data.at(1)), enu[0], enu[1], enu[2], stod(data.at(8)) * PI / 180, stod(data.at(7)) * PI / 180, stod(data.at(6)) * PI / 180);
            if (!lioqueue.empty())
            {
                Pose3 increPose = lioqueue.back().pose.between(frame.pose);
                fprintf(fp_incre, "%d %.3lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf\n", frame.week, frame.weeksec, increPose.x(), increPose.y(), increPose.z(), increPose.rotation().roll() * 180 / M_PI, increPose.rotation().pitch() * 180 / M_PI, increPose.rotation().yaw() * 180 / M_PI);
            }
            lioqueue.push_back(frame);
        }
        fclose(fp_incre);
    }

    void loadIMUbag(string IMUfile)
    {
        rosbag::Bag bag;
        bag.open(IMUfile, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            if (m.getTime().toSec() < ((double)ts.time + ts.sec - 18))
                continue;
            if (m.getTime().toSec() > ((double)te.time + te.sec - 18))
                break;
            sensor_msgs::Imu::Ptr imuMsg = m.instantiate<sensor_msgs::Imu>();
            imuMsg->header.stamp.fromSec(imuMsg->header.stamp.toSec());
            if (imuMsg)
            {
                imuHandler(imuMsg);
            }
        }
    }

    void writeGPSfile(gtime_t gpst, Vector3 ecef, int state, int nb, int npr, int ndop) const
    {
        int week;
        double weeksec = time2gpst(gpst, &week);
        fprintf(fp, "%d %.3lf %.4lf %.4lf %.4lf %d %.3lf %.3lf %.3lf %d %d %d 0.0 0.0 %.1lf %.6lf %.6lf %.6lf %.6lf %.6lf %.6lf\n", week, weeksec, ecef(0), ecef(1), ecef(2), state,
            gpsPose.rotation().roll() * 180 / PI, gpsPose.rotation().pitch() * 180 / PI, gpsPose.rotation().yaw() * 180 / PI,
            nb, npr, ndop, container.getSolRatio(),
            prevBias_.accelerometer().x(), prevBias_.accelerometer().y(), prevBias_.accelerometer().z(),
            prevBias_.gyroscope().x(), prevBias_.gyroscope().y(), prevBias_.gyroscope().z());
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        FastMap<char, Vector> threshold;
        threshold['x'] = (Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1).finished();
        threshold['b'] = (Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished();
        threshold['v'] = (Vector(3) << 0.1, 0.1, 0.1).finished();
        //        threshold['n'] = (Vector (NB(&prcopt)).setConstant(0.01));

        optParameters.optimizationParams = ISAM2DoglegParams(3.0, 1e-10, DoglegOptimizerImpl::TrustRegionAdaptationMode::SEARCH_EACH_ITERATION);
        optParameters.relinearizeThreshold = 0.01; // threshold;
        optParameters.relinearizeSkip = 1;
        optParameters.factorization = gtsam::ISAM2Params::QR;
        optParameters.enableDetailedResults = true;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;

        lm_params = LevenbergMarquardtParams();
        lm_params.setMaxIterations(30);
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3d vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3d ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3d bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu curImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(curImu);
        imuQueImu.push_back(curImu);

        double curimuTime = ROS_TIME(&curImu);
        double dt_ = (lastImuT_imu < 0) ? (1.0 / imuFrequence) : (curimuTime - lastImuT_imu);
        lastImuT_imu = curimuTime;
        int nb, npr, ndop;
        nb = 0;
        npr = 0;
        ndop = 0;
        volatile bool GNSS_enable = false;

        if ((GNSS_enable = container.isGNSSEnable(curimuTime)) ) // 125) || (systemInitialized && fabs(curimuTime - round(curimuTime)) < 0.00125)
        {
            TicToc t_epoch;
            t_epoch.tic();
            ROS_INFO("%.3lf", curimuTime);
            while (!lioqueue.empty())
            {
                if (fabs(lioqueue.front().timestamp - curimuTime) < 0.05)
                {
                    curlioframe = lioqueue.front();
                    lioqueue.pop_front();
                    break;
                }
                else
                {
                    if (lioqueue.front().timestamp < curimuTime)
                        lioqueue.pop_front();
                    else
                        break;
                }
            }
            // 0. initialize system
            if (!systemInitialized)
            {

                resetOptimization();

                // pop old IMU message
                while (!imuQueOpt.empty())
                {
                    if (ROS_TIME(&imuQueOpt.front()) <= curimuTime - delta_t)
                    {
                        lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                        imuQueOpt.pop_front();
                    }
                    else
                        break;
                }

                // initial pose
                gpsPose = gtsam::Pose3(gtsam::Rot3::RzRyRx(GroundTruthRoll * M_PI / 180, GroundTruthPitch * M_PI / 180, GroundTruthHeading * M_PI / 180),
                    gtsam::Point3(0, 0, 0));
                prevPose_ = gpsPose.compose(imu2gps); // gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0*M_PI/180,0.0*M_PI/180,0.0*M_PI/180),
                //                gtsam::Point3(0,0,0));//
                //                if (useObs) {
                //                }
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                graphFactors.add(priorPose);
                // initial velocity
                prevVel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                graphFactors.add(priorVel);
                // initial bias
                prevBias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                graphFactors.add(priorBias);

                // add values
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                prevState_ = gtsam::NavState(prevPose_, prevVel_);

                // add gps
                //    container.addGPSFactorENU(&graphFactors,&graphValues,key);
                if (useGPS)
                {
                    addGPSFactor(nb, npr, ndop);
                }
                updateAndMarginalize(graphFactors, graphValues, {}, optimizer);
                //                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();

                result = optimizer.calculateEstimate();
                prevPose_ = result.at<gtsam::Pose3>(X(key));
                prevVel_ = result.at<gtsam::Vector3>(V(key));
                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
                if (nb > 0)
                {
                    prevAmb_ = result.at<Vector>(N(key));
                }
                if (estimateExtGPS)
                {
                    extGPS = result.at<gtsam::Vector3>(T(key));
                    ROS_INFO("INITIAL EXT IMU 2 GPS %lf %lf %lf", extGPS.x(), extGPS.y(), extGPS.z());
                }
                ROS_INFO("INITIAL IMU POSE: %lf %lf %lf", prevPose_.x(), prevPose_.y(), prevPose_.z());
                imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

                lastlioframe = curlioframe;
                systemInitialized = true;
                container.setSystemInitialized(systemInitialized);
                key = 1;
                return;
            }

            // 1. integrate imu data and optimize
            while (!imuQueOpt.empty())
            {
                // pop and integrate imu data that is between two optimizations
                sensor_msgs::Imu* thisImu = &imuQueOpt.front();
                double imuTime = ROS_TIME(thisImu);
                if (imuTime <= curimuTime - delta_t)
                {
                    double dt = (lastImuT_opt < 0) ? (1.0 / imuFrequence) : (imuTime - lastImuT_opt);
                    imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x,
                            thisImu->linear_acceleration.y,
                            thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y,
                            thisImu->angular_velocity.z),
                        dt);

                    lastImuT_opt = imuTime;
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            container.tt = imuIntegratorOpt_->deltaTij();
            // add imu factor to graph
#ifdef USE_COMBINED_IMU
            const gtsam::PreintegratedCombinedMeasurements
                & preint_imu = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements&>(*imuIntegratorOpt_);
            gtsam::CombinedImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), B(key), preint_imu);
#else
            const gtsam::PreintegratedImuMeasurements
                & preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
            gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
            // add imu bias between factor
            graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1),
                B(key),
                gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(
                    sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
#endif

            graphFactors.add(imu_factor);

            Pose3 lidarPose;
            bool useable;
            Pose3 increPose = lastlioframe.pose.between(curlioframe.pose);
            lidarPose = prevState_.pose().compose(increPose);
            double d_yaw = fabs(curlioframe.pose.rotation().yaw() * 180 / M_PI - lastlioframe.pose.rotation().yaw() * 180 / M_PI);
            if (useLIO)
            {
                gtsam::noiseModel::Diagonal::shared_ptr noise_model = gtsam::noiseModel::Diagonal::Sigmas(Vector3(0.04, 0.04, 0.03));
                noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise_model);
                // graphFactors.add(BetweenTranslationFactor(X(key - 1), X(key), curlioframe.translation - lastlioframe.translation, huber));
                gtsam::noiseModel::Diagonal::shared_ptr noise_incre = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.1, 0.1, 0.3).finished());
                noiseModel::Base::shared_ptr huber_incre = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(0.5), noise_incre);
                // graphFactors.add(PriorFactor<Pose3>(X(key), lidarPose, huber_incre));
                // graphFactors.add(BetweenFactor<Pose3>(X(key - 1), X(key), increPose, huber_incre));

            }

            if (useNHC)
            {
                gtsam::Vector3 angular_velocity = Rot3::Logmap(imuIntegratorOpt_->deltaRij()) / imuIntegratorOpt_->deltaTij();
                // if (angular_velocity.norm() * 180 / M_PI < thres_angular_velocity)
                // {
                    // 定义噪声模型（设置x/z分量的约束强度）
                    // if (fabs(increPose.z()) < 0.05)
                    // {
                    //     noiseModel::Base::shared_ptr
                    //         Zaxis_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.1).finished());

                    //     // graphFactors.add(ZAxisConstraint(X(key), prevPose_.z(), Zaxis_noise));
                    // }
                if (increPose.translation().norm() < 0.05 && useLIO)
                {
                    auto staticNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)
                        << 5e-1,
                        5e-1, 5e-1, 0.1, 0.1, 0.1)
                        .finished()); // rad,rad,rad,m, m, m
                    graphFactors.add(PriorFactor<Pose3>(X(key), prevPose_, staticNoise));
                }

                {
                    // auto noiseModelX = noiseModel::Diagonal::Sigmas(Vector1(NHCnoise[0] * (useLIO ? d_yaw * fabs(increPose.x()) : 1))); // 单位：m/s
                    // noiseModel::Base::shared_ptr huberX = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(1.0), noiseModelX);
                    // auto noiseModelY = noiseModel::Diagonal::Sigmas(Vector1(NHCnoise[0] * (useLIO ? d_yaw : 1.0))); // 单位：m/s
                    // noiseModel::Base::shared_ptr huberY = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(1.0), noiseModelY);
                    // auto noiseModelZ = noiseModel::Diagonal::Sigmas(Vector1(NHCnoise[1] * (useLIO ? (d_yaw * fabs(increPose.z())) : 1.0))); // 单位：m/s
                    // noiseModel::Base::shared_ptr huberZ = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(1.0), noiseModelZ);
                    // // 添加非完整性约束因子
                    // graphFactors.add(NhcFactorX(X(key), V(key), huberX));
                    // if (useLIO)
                    // {
                    //     graphFactors.add(NhcFactorY(X(key), V(key), increPose.y(), huberY));
                    // }
                    // graphFactors.add(NhcFactorZ(X(key), V(key), huberZ));

                    if (useLIO)
                    {
                        auto noiseModel = noiseModel::Diagonal::Sigmas(Vector3(NHCnoise[0] * d_yaw * fabs(increPose.y()), NHCnoise[0], NHCnoise[1]));
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(1.0), noiseModel);
                        graphFactors.add(NhcFactor3Axis(X(key), V(key), increPose.y(), noiseModel));
                    }
                    else
                    {
                        auto noiseModel = noiseModel::Diagonal::Sigmas(Vector2(NHCnoise[0], NHCnoise[1]));
                        noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(1.0), noiseModel);
                        graphFactors.add(NhcFactor(X(key), V(key), noiseModel));
                    }



                    ROS_INFO("add NHC factor");
                }
                // }
            }
            // TODO:add lidar frame-to-frame factor
            //  insert predicted values
            gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
            //            prevState_.pose().between(propState_.pose()).print();
            // graphFactors.add(PriorFactor<Pose3>(X(key), propState_.pose(), priorPoseNoise));
            if (useLIO)
            {
                graphValues.insert(X(key), lidarPose);
            }
            else
            {
                graphValues.insert(X(key), propState_.pose());
            }
            graphValues.insert(V(key), propState_.v());
            graphValues.insert(B(key), prevBias_);
            int debugInfo = 0;
            if (!optimizer.valueExists(N(lastGNSSepoch)))
            {
                container.reset_last_ar_index();
                ROS_INFO("%d does not exist", lastGNSSepoch);
            }
            // add gps factor
            if (GNSS_enable)
            {
                if (useGPS)
                {
                    addGPSFactor(nb, npr, ndop);
                }
            }
            else
            {
                ROS_INFO("GNSS is not enabled: %lf",container.getCurGnssTime());
            }
            ROS_INFO("graph size: %d", graphFactors.size());
            ROS_INFO("value size: %d", graphValues.size());
            TicToc t_opt;
            t_opt.tic();
            // optimize
            updateAndMarginalize(graphFactors, graphValues, {}, optimizer);
            for (int i = 0; i < 10; i++)
            {
                optimizer.update();
            }
            graphFactors.resize(0);
            graphValues.clear();
            //            ROS_INFO"optimize cost: %.2fms", t_opt.toc());
            // Overwrite the beginning of the preintegration for the next step.
            result = optimizer.calculateEstimate();
            // gtsam::NonlinearFactorGraph currentFactors = optimizer.getFactorsUnsafe();
            // LevenbergMarquardtOptimizer lm_opt(graphFactors, graphValues, lm_params);
            // result = lm_opt.optimize();
            // result.print("Estimate result");
            // optimizer.print("Optimize factor");
            static int state = 6;
            static Pose3 sol_pos;
            sol_pos = result.at<gtsam::Pose3>(X(key));
            posCovariance = optimizer.marginalCovariance(X(key));
            if (useObs && GNSS_enable && useGPS)
            {
                state = container.ambiguityResolve(optimizer, result, key, posCovariance, sol_pos);
            }
            ROS_INFO("IMU POSE: %lf %lf %lf", sol_pos.x(), sol_pos.y(), sol_pos.z());
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
            lastlioframe = curlioframe;
            if (estimateExtGPS)
            {
                extGPS = result.at<gtsam::Vector3>(T(key));
                ROS_INFO("EXT IMU 2 GPS %lf %lf %lf", extGPS.x(), extGPS.y(), extGPS.z());
            }
            if (key >= 10)
            {
                KeySet marginalKeys;
                marginalKeys.insert(X(key - 10));
                marginalKeys.insert(V(key - 10));
                marginalKeys.insert(B(key - 10));
                if (optimizer.valueExists(N(key - 10)))
                {
                    marginalKeys.insert(N(key - 10));
                }
                if (estimateExtGPS)
                {
                    marginalKeys.insert(T(key - 10));
                }
                updateAndMarginalize({}, {}, marginalKeys, optimizer);
            }
            average_time = (average_time * (key - 1) + t_opt.toc()) / (key);
            // ROS_INFO("average time %.3lf", average_time);
            // ROS_INFO("opt time %.3lf",t_opt.toc());

            // write gps result
            gpsPose = prevPose_.compose(gps2imu);
            ROS_INFO("GPS POSE: %lf %lf %lf", gpsPose.x(), gpsPose.y(), gpsPose.z());
            lla_origin = container.getOrigin();
            Vector3 ecef = GNSS_Tools::enu2ecef(lla_origin, gpsPose.translation()); // gtools.ENU2ECEF(gpsPose.translation());
            gtime_t curgtime;
            curgtime.time = floor(curimuTime) + 18;
            curgtime.sec = curimuTime - floor(curimuTime);
            volatile double weeksec = time2gpst(curgtime, NULL);

            writeGPSfile(curgtime, ecef, state, nb, npr, ndop);
            ROS_INFO("total time %.3lf\n", t_epoch.toc());
            // Reset the optimization preintegration object.
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            // 2. after optiization, re-propagate imu odometry preintegration
            prevStateOdom = prevState_;
            prevBiasOdom = prevBias_;
            // first pop imu message older than current correction data
            double lastImuQT = -1;
            while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < curimuTime - delta_t)
            {
                lastImuQT = ROS_TIME(&imuQueImu.front());
                imuQueImu.pop_front();
            }
            // repropogate
            if (!imuQueImu.empty())
            {
                // reset bias use the newly optimized bias
                imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
                // integrate imu message from the beginning of this optimization
                for (int i = 0; i < (int)imuQueImu.size(); ++i)
                {
                    sensor_msgs::Imu* thisImu = &imuQueImu[i];
                    double imuTime = ROS_TIME(thisImu);
                    double dt = (lastImuQT < 0) ? (1.0 / imuFrequence) : (imuTime - lastImuQT);

                    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x,
                        thisImu->linear_acceleration.y,
                        thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,
                            thisImu->angular_velocity.y,
                            thisImu->angular_velocity.z),
                        dt);
                    lastImuQT = imuTime;
                }
            }

            ++key;
            doneFirstOpt = true;
        }

        if (!doneFirstOpt)
            return;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(curImu.linear_acceleration.x,
            curImu.linear_acceleration.y,
            curImu.linear_acceleration.z),
            gtsam::Vector3(curImu.angular_velocity.x,
                curImu.angular_velocity.y,
                curImu.angular_velocity.z),
            dt_);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        //        if (!GNSS_enable){
        //            if (fabs(curimuTime - floor(curimuTime)) < 0.005){
        //                gpsPose = currentState.pose().compose(gps2imu);
        //                ROS_INFO("GPS POSE: %lf %lf %lf",gpsPose.x(),gpsPose.y(),gpsPose.z());
        //                lla_origin = container.getOrigin();
        //                Vector3 ecef = GNSS_Tools::enu2ecef(lla_origin, extRot.transpose() * gpsPose.translation());//gtools.ENU2ECEF(gpsPose.translation());
        //                gtime_t curgtime;
        //                curgtime.time = floor(curimuTime) + 18;
        //                curgtime.sec = curimuTime - floor(curimuTime);
        //                writeGPSfile(curgtime,ecef,6,nb,npr,ndop);
        //            }
        //        }
        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = curImu.header.stamp;
        odometry.header.frame_id = "map";
        //        odometry.child_frame_id = "odom_imu";
        //        odometry.header.frame_id = odometryFrame;
        //        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = curImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = curImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = curImu.angular_velocity.z + prevBiasOdom.gyroscope().z();

        // publish path
        imuPath.header.frame_id = "map";
        imuPath.header.stamp = curImu.header.stamp;
        geometry_msgs::PoseStamped pose;
        pose.header = imuPath.header;
        pose.pose.position.x = lidarPose.translation().x();
        pose.pose.position.y = lidarPose.translation().y();
        pose.pose.position.z = lidarPose.translation().z();
        pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        imuPath.poses.push_back(pose);
        if (pubIMUPath.getNumSubscribers() != 0)
        {
            pubIMUPath.publish(imuPath);
        }
        pubImuOdometry.publish(odometry);
    }

    void addGPSFactor(int& nb, int& npr, int& ndop)
    {
        if (container.checkIsEmpty())
            return;

        if (container.syncObs(lastImuT_opt, 0.005))
        {
            if (!useObs)
            {
                container.addGPSFactorENU(&graphFactors, &graphValues, key);
            }
            else
            {
                npr = container.addDDPsrFactorENU(&graphFactors, &graphValues, key);
                nb = container.addDDCpFactorENU(&graphFactors, &graphValues, key, lastGNSSepoch);
                // ndop = container.addSDDopFactorENU(&graphFactors, &graphValues, key, lastGNSSepoch); //|| ndop < 6
                ROS_INFO("npr: %d nb: %d ndop: %d", npr, nb, ndop);
            }
            if (estimateExtGPS)
            {
                graphValues.insert(T(key), extGPS);
                gtsam::PriorFactor<Vector3> priorExtGPS(T(key), extGPS, priorExtNoise);
                graphFactors.add(priorExtGPS);
                if (systemInitialized)
                {
                    graphFactors.add(BetweenFactor<Vector3>(T(key - 1), T(key), Vector3::Zero(),
                        gtsam::noiseModel::Isotropic::Sigma(3, 0.01)));
                }
            }
            lastGNSSepoch = key;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");

    time_t t_start, t_end;

    t_start = time(0);

    gnssProcessor processor;

    test_GINS gins;
    //    gtime_t ts = gpst2time(2290,551805);//550729);
    ////    gtime_t te = gpst2time(2290,550792);
    //    gtime_t te = gpst2time(2290,552300);
    //    gtime_t ts = gpst2time(2158,95630);
    //    gtime_t ts = gpst2time(2192,462384);
    //    gtime_t te = gpst2time(2192,464583);
    //    gtime_t te = gpst2time(2192,462583);
    //    gtime_t ts = gpst2time(2096,549935);
    //    gtime_t te = gpst2time(2096,550236);
    //    gtime_t ts = gpst2time(2233,551805);//550729); 549002
    //    gtime_t te = gpst2time(2233,552300);

    // 20240129
    // gtime_t ts = gpst2time(2299, 111965);
    // gtime_t te = gpst2time(2299, 113000);

    // 20240705
    //    gtime_t ts = gpst2time(2321,429370.000 );
    //    gtime_t te = gpst2time(2321,431700.000 );

    // gtime_t ts = gpst2time(2129,181347);
    // gtime_t te = gpst2time(2129,182154);
    // 20250120_1
    // gtime_t ts = gpst2time(2350, 117500);
    // gtime_t te = gpst2time(2350, 118300);

    // 20250120_3
    // gtime_t ts = gpst2time(2350, 120671); //120562
    // gtime_t te = gpst2time(2350, 122400);

    // hk
    gtime_t ts = gpst2time(2158, 455346); //120562
    gtime_t te = gpst2time(2158, 456878); 
    gins.set_time(ts, te);

    processor.decode(ts, te);
    t_end = time(0);
    ROS_INFO("gnssProcessor took %d second\n", (int)difftime(t_end, t_start));

    gins.run();

    t_end = time(0);

    ROS_INFO("Optimizer took %d second\n", (int)difftime(t_end, t_start));
    //    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

    ros::spin();

    return 0;
}
