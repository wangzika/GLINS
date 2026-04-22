#pragma once

#ifndef IMU_H
#define IMU_H
#include "utility.h"
#include "glins/feature_info.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "factor/LidarFactor.h"
#include "marginalize_isam2.h"
#include "gnss/gnssContainer.h"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::T;

typedef pair<nav_msgs::Odometry, rtklib::GNSS_Info> GNSSPose;

class TransformFusion : public ParamServer
{
public:
        std::mutex mtx;

        ros::Subscriber subImuOdometry;
        ros::Subscriber subLaserOdometry;

        ros::Publisher pubImuOdometry;
        ros::Publisher pubImuPath;


        Eigen::Affine3f lidarOdomAffine;
        Eigen::Affine3f imuOdomAffineFront;
        Eigen::Affine3f imuOdomAffineBack;

        tf::TransformListener tfListener;
        tf::StampedTransform lidar2Baselink;

        double lidarOdomTime = -1;
        deque<nav_msgs::Odometry> imuOdomQueue;

        TransformFusion();

        Eigen::Affine3f odom2affine(nav_msgs::Odometry odom);

        void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);

        void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);
};

class IMUPreintegration : public ParamServer
{
public:

        std::mutex mtx;

        ros::Subscriber subImu;
        ros::Subscriber subObs;
        ros::Subscriber subLidarFeature;

        ros::Publisher pubImuOdometry;
        ros::Publisher pubGPSPath;
        ros::Publisher pubLaserOdometry;
        bool systemInitialized = false;

        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
        gtsam::noiseModel::Diagonal::shared_ptr priorExtNoise;
        gtsam::Vector noiseModelBetweenBias;

        gtsam::PreintegratedImuMeasurements* imuIntegratorOpt_;
        gtsam::PreintegratedImuMeasurements* imuIntegratorImu_;
        gtsam::PreintegratedImuMeasurements* imuIntegratorOdo_;

        std::deque<sensor_msgs::Imu> imuQueOpt;
        std::deque<sensor_msgs::Imu> imuQueImu;

        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;
        Vector prevAmb_;

        MatrixXd posCovariance;
        MatrixXd ambCovariance;

        gtsam::NavState prevStateOdom;
        gtsam::imuBias::ConstantBias prevBiasOdom;

        bool doneFirstOpt = false;
        double thisImuT_imu = -1;
        double lastImuT_imu = -1;
        double lastImuT_opt = -1;

        gtsam::ISAM2 optimizer;
        gtsam::NonlinearFactorGraph graphFactors;
        gtsam::Values graphValues;

        //用于保证用于lidar匹配的位姿连续
        gtsam::ISAM2 odomOptimizer;
        gtsam::NonlinearFactorGraph odomGraphFactors;
        gtsam::Values odomGraphValues;

        std::deque<sensor_msgs::Imu> imuQueue;

        const double delta_t = -0.005;

        int lastGNSSepoch = 0;
        double yaw = 0.0, prevYaw = 0.0;
        Eigen::Vector3d prevPos = Eigen::Vector3d(0, 0, 0);
        geometry_msgs::Quaternion yawQuat;
        glins::feature_info featureInfo;
        bool featureUpdate = false;

        int key = 0;

        // T_bl: tramsform points from lidar frame to imu frame
        gtsam::Pose3
                imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                        gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
        // T_lb: tramsform points from imu frame to lidar frame
        gtsam::Pose3
                lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));

        gtsam::Pose3
                gps2imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extGPS.x(), extGPS.y(), extGPS.z()));

        gtsam::Pose3
                imu2gps = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extGPS.x(), -extGPS.y(), -extGPS.z()));
        gtsam::Pose3 gpsPose;
        gtsam::Pose3 lastKeyPose;
        int lastKeyIndex;
        string result_path;
        FILE* fp, * fp_debug;
        //    FILE *fp_result;
        gtsam::Pose3 prePose;

        Vector3 lla_origin;

        gtime_t ts;

        pcl::PointCloud<PointType>::Ptr cornercloudMatch;
        pcl::PointCloud<PointType>::Ptr surfcloudMatch;

        gnssContainer container;
        std::deque<int> gpsKeyQueue;
        bool isRelative = true;
        bool isStatic = false;
        double average_time = 0.0;

        float surfNoise, edgeNoise;
        IMUPreintegration();

        void setStartTime(gtime_t& t);

        void resetOptimization();

        void resetParams();

        void addGPSFactor(int& nb, int& npr, int& ndop);

        void writeGPSfile(gtime_t gpst, Vector3 ecef, int state, FILE* file);

        void writeGPSfile2(gtime_t gpst, Vector3 ecef, int state, FILE* file);

        void closePosfile();

        void addLidarFactor();

        void featureHandler(const glins::feature_info::ConstPtr& featureMsg);

        bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur);

        void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw);

};

#endif