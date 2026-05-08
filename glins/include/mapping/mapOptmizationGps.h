#pragma once
#ifndef MAP_OPTIMIZATION
#define MAP_OPTIMIZATION
#include "utility.h"
#include "glins/cloud_info.h"
#include "glins/save_map.h"
#include "glins/feature_info.h"
#include <csignal>

#include <std_srvs/Empty.h>

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
#include <gtsam/slam/dataset.h> // gtsam
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/filters/radius_outlier_removal.h>
#include "dataSaver.h"
#include "gnss_tools.h"

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D

        PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x)(float, y, y)
    (float, z, z)(float, intensity, intensity)
    (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)
    (double, time, time))

    typedef PointXYZIRPYT PointTypePose;
typedef pair<nav_msgs::Odometry, rtklib::GNSS_Info> GNSSPose;

class DataConverter : public ParamServer
{
public:
    DataConverter();

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
        PointTypePose* transformIn);

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

    gtsam::Pose3 trans2gtsamPose(float transformIn[]);

    void gtsamPose2trans(gtsam::Pose3 pose, float transformIn[]);

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);

    Eigen::Affine3f trans2Affine3f(float transformIn[]);

    PointTypePose trans2PointTypePose(float transformIn[]);

    PointType eigen2PointType(Vector3 point);

    void transformEigen2Odom(double timestamp, nav_msgs::Odometry& laserOdometryROS, float transform[6]);

};

class mapOptimization : public DataConverter
{

public:
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2* isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubCloudRaw;
    ros::Publisher pubLoopConstraintEdge;
    ros::Publisher pubGpsConstraintEdge;
    ros::Publisher gpsOdomPathPub;

    ros::Publisher pubSLAMInfo;
    ros::Publisher pubFeatInfo;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;
    ros::Subscriber subObs;
    ros::Subscriber subRTKOdom;

    ros::ServiceServer srvSaveMap;

    // GPS里程计队列：保存已经转换成ROS Odometry形式的GPS/RTK位姿，用于按LiDAR时间戳取最近GPS约束。
    std::deque<nav_msgs::Odometry> gpsQueue;

    // GNSS原始测量队列：保存GNSSPose结构体，包含更底层的GNSS观测/解算信息，供GPS因子或伪距因子使用。
    std::deque<GNSSPose> gpsMeasQueue;

    // 当前LiDAR帧的综合信息：由前端传入，包含时间戳、特征点云、IMU初值等，是mapping本轮优化的输入缓存。
    glins::cloud_info cloudInfo;

    // LiDAR关键帧点云缓存：
    // 每当系统决定保存一个关键帧时，会把当前帧提取出的角点、平面点和原始点云存进这些vector。
    // 下标就是关键帧索引，后续构建局部地图、回环ICP、保存全局PCD时都会按索引取出对应点云。
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> laserCloudRawKeyFrames;

    // 普通关键帧的位姿和辅助信息：
    // keyframePosesOdom/keyframePosestrans保存优化后的关键帧位姿，主要用于轨迹输出和坐标变换。
    // keyframeRawOdom保存未经后端修正或更接近前端里程计的位姿，便于调试对比。
    // keyframeTimes记录每个关键帧时间戳，allResVec保存相关点云消息，keyframeDistances记录相邻关键帧累计距离。
    // keyframeGPSfactor保存与关键帧关联的GPSFactor，方便后续检查或可视化GPS约束。
    std::vector<nav_msgs::Odometry> keyframePosesOdom;
    std::vector<Eigen::Matrix4d> keyframePosestrans;
    std::vector<nav_msgs::Odometry> keyframeRawOdom;
    std::vector<double> keyframeTimes;
    std::vector<sensor_msgs::PointCloud2> allResVec;
    std::vector<double> keyframeDistances;
    std::vector<gtsam::GPSFactor> keyframeGPSfactor;

    // 普通LiDAR关键帧位姿容器：
    // cloudKeyPoses3D只保存关键帧的xyz位置，intensity通常存关键帧索引，用于KD-tree半径搜索和局部地图筛选。
    // cloudKeyPoses6D保存完整姿态，包括xyz、roll/pitch/yaw、time、intensity等，用于点云变换、图优化、回环约束。
    // mode 0的scan-to-map主要围绕这组普通LiDAR关键帧构建局部地图。
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    // GPS辅助关键帧位姿容器：
    // 这不是所有普通LiDAR关键帧的简单复制，而是在GPS可用或mode 2逻辑触发时保存的一组GPS参考关键帧。
    // cloudKeyGPSPoses3D保存GPS辅助关键帧的xyz位置，cloudKeyGPSPoses6D保存其完整姿态和时间。
    // mode 2会使用这组位姿构建laserCloudCornerFromGPSMap/laserCloudSurfFromGPSMap，
    // 然后让当前LiDAR扫描去匹配GPS map，因此GNSS误差会更直接影响前端匹配。
    // gpsIndexContainer用于记录GPS辅助关键帧索引和普通LiDAR关键帧索引之间的对应关系。
    pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyGPSPoses6D;

    // 关键帧位姿副本：
    // 回环线程、可视化线程会遍历关键帧位姿；主优化线程同时也可能在更新cloudKeyPoses。
    // 因此先把主容器复制到copy_*里，再在副本上做KD-tree搜索、回环检测和marker绘制，减少并发读写风险。
    // copy_cloudKeyPoses2D通常由3D副本复制而来，并在回环候选搜索时把z置零，只按平面距离找候选。
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses2D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudGroundLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudRaw; // giseop
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS; // giseop

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<Vector3> edgeMatchPoint_A;
    std::vector<Vector3> edgeMatchPoint_B;
    std::vector<Vector3> surfNorm;
    std::vector<double> d_plane;

    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainerGPS;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    pcl::VoxelGrid<PointType> downSizeFilterRaw; // giseop
    pcl::RadiusOutlierRemoval<PointType> radiusORFilter;

    std::unique_ptr<DataSaver> dataSaverPtr;

    int lastLoopIndex = -1;


    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];
    float tmptransformTobeMapped[6];
    Pose3 dPose;
    std::mutex mtx;
    std::mutex mtxLoopInfo;
    std::mutex mtxGpsInfo;

    Eigen::Affine3f transGPS;
    Eigen::Vector3d transLLA;
    bool systemInitialized = false;

    bool isDegenerate = false;
    Eigen::MatrixXf matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old
    map<int, int> gpsIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;
    nav_msgs::Path globalPath;
    nav_msgs::Path gpsOdomPath;
    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    Eigen::Vector3d prevPos = Eigen::Vector3d(0, 0, 0);
    geometry_msgs::Quaternion yawQuat;
    gtsam::Pose3 gps2imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extGPS.x(), extGPS.y(), extGPS.z()));
    gtsam::Pose3 optlidarPose;
    ros::Subscriber subOptlidar;


    /// build gps local map
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudGpsMapContainer;
    gtsam::Pose3 lastGPSpose;
    double lastGPSpose_time;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromGPSMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromGPSMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromGPSMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromGPSMapDS;

    pcl::PointCloud<PointType>::Ptr laserCornerCloudMatch;
    pcl::PointCloud<PointType>::Ptr laserSurfCloudMatch;

    Eigen::Affine3f incrementalTransPointAssociateToMap;

    std::deque<nav_msgs::Odometry> gnssOdomQueue;

    double para_tz[1] = { 0 };
    double para_q[4] = { 0, 0, 0, 1 };
    double para_tz_pre = 0;

    mapOptimization();

    void allocateMemory();

    void laserCloudInfoHandler(const glins::cloud_infoConstPtr& msgIn);

    void rtklibOdomHandler(const nav_msgs::OdometryConstPtr& msg);

    void lidarOdomHandler(const nav_msgs::OdometryConstPtr& odomMsg);

    void pointAssociateToMap(PointType const* const pi, PointType* const po);

    bool saveMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    bool saveMapService();

    void visualizeGlobalMapThread();

    void publishGlobalMap();

    void loopClosureThread();

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg);

    ///回环检测
    void performLoopClosure();

    bool detectLoopClosureDistance(int* latestID, int* closestID);

    bool detectLoopClosureExternal(int* latestID, int* closestID);

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum);

    void visualizeLoopClosure();

    void visualGPSConstraint();

    //初始化位姿
    /***
     * imu原始数据（cloudInfo.imuRollInit
     * imu里程计信息(cloudInfo.initialGuessX
     * 参考：LIO-SAM中mapOptmization.cpp中的updateInitialGuess()理解（https://blog.csdn.net/qq_44305240/article/details/126990259）
     */
    void updateInitialGuess();

    void extractForLoopClosure();

    void extractNearby();

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);

    void extractCloud_GPS(pcl::PointCloud<PointType>::Ptr cloudToExtract);

    void extractSurroundingKeyFrames();

    void downsampleCurrentScan();

    void updatePointAssociateToMap();

    /***
     * 查找对应的特征点
     */
    void cornerOptimization();

    void surfOptimization();

    void GroundConstraint();
    /***
 * 查找对应的特征点
 */
    void cornerOptimization_gps();

    void surfOptimization_gps();

    void combineOptimizationCoeffs();

    bool LMOptimization(int iterCount);

    /***
     * 激光扫描数据SCAN直接与地图进行匹配
     */
    void scan2MapOptimization();

    void scan2GpsMapOptimization();

    void transformUpdate();

    float constraintTransformation(float value, float limit);

    bool saveFrame();

    /***
     *
     */
    void addOdomFactor();

    void addOptOdomFactor();

    void addLoopFactor();

    bool findGPSAvail(double curTime);

    void saveKeyFramesAndFactor();

    void correctPoses();

    void updatePath(const PointTypePose& pose_in);

    void transformEiegn2Odom(double timestamp, nav_msgs::Odometry& laserOdometryROS, float transform[6]);

    void savePath(string path);

    void publishOdometry();

    void publishFrames();

    void publishLidarFeature();

};

#endif
