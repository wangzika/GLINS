#pragma once
#ifndef FEATURE_EXTRACTION
#define FEATURE_EXTRACTION
#include "utility.h"
#include "glins/cloud_info.h"
/***
 * featureExtraction 特征提取
 */
struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

class FeatureExtraction : public ParamServer {

public:

    ros::Subscriber subLaserCloudInfo;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    glins::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    FeatureExtraction();

    void initializationValue();

    /***
     * 主线程
     */
    void laserCloudInfoHandler(const glins::cloud_infoConstPtr &msgIn);

    void calculateSmoothness();

    void markOccludedPoints();

    void extractFeatures();

    void freeCloudInfoMemory();

    void publishFeatureCloud();
};

#endif //FEATURE_EXTRACTION