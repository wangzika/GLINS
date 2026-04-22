#pragma once
#ifndef IMAGE_PROJECCTION
#define IMAGE_PROJECCTION
#include "utility.h"
#include "glins/cloud_info.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include "patchworkpp/patchworkpp.hpp"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D

        PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time)
)

struct PandarPointXYZIRT
{
    PCL_ADD_POINT4D

        float intensity;
    double timestamp;
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PandarPointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    //  uint32_t time;
    uint16_t reflectivity;
    uint8_t ring;
    std::uint16_t ambient;  // additional property of p.ouster
    float time;
    uint16_t noise;
    uint32_t range;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (float, time, time)
    (uint16_t, noise, noise)
    (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;
    std::mutex cloudLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubGroundCloud;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double* imuTime = new double[queueLength];
    double* imuRotX = new double[queueLength];
    double* imuRotY = new double[queueLength];
    double* imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PandarPointXYZIRT>::Ptr tmpPandarCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;
    cv::Mat groundMat; // ground matrix for ground cloud marking

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    glins::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;

    boost::shared_ptr<PatchWorkpp<PointXYZIRT>> PatchWorkGroundSeg;
public:
    ImageProjection();

    void allocateMemory();

    void resetParameters();

    ~ImageProjection();

    /***
     * 保存到队列中
     * @param imuMsg
     */
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg);

    /***
     * imu计算的里程计增量
     * @param odometryMsg
     */
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg);

    /***
     * 接收原始点云，点云处理的主要函数，展示了点云处理的主要流程。
     * @param laserCloudMsg
     */
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

    /***
     * 激光点云数据缓存到队列中
     * @param laserCloudMsg
     * @return
     */
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

    /***
     * 从IMU数据和IMU里程计中计算去畸变信息
     * imu数据：
     *   1) 遍历当前激光帧起止时刻之间的imu数据，初始时刻对应imu的姿态角RPY设为当前帧的初始姿态角
     *   2) 用角速度、时间积分，计算每一时刻相对于初始时刻的旋转量，初始时刻旋转设为0
     * imu里程计数据：
     *   1) 遍历当前激光帧起止时刻之间的imu里程计数据，初始时刻对应imu里程计设为当前帧的初始位姿
     *   2) 用起始、终止时刻对应imu里程计，计算相对位姿变换，保存平移增量
     */
    bool deskewInfo();

    void imuDeskewInfo();

    void odomDeskewInfo();

    void findRotation(double pointTime, float* rotXCur, float* rotYCur, float* rotZCur);

    void findPosition(double relTime, float* posXCur, float* posYCur, float* posZCur);

    PointType deskewPoint(PointType* point, double relTime);

    void projectPointCloud();

    void groundRemoval_patchwork();

    void cloudExtraction();

    void publishClouds();
};

#endif //IMAGE_PROJECCTION