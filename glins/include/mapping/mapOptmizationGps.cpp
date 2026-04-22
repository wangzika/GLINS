#include "mapOptmizationGps.h"
#include "ceresFactor.hpp"

DataConverter::DataConverter()
{

}

pcl::PointCloud<PointType>::Ptr DataConverter::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
    PointTypePose* transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x,
        transformIn->y,
        transformIn->z,
        transformIn->roll,
        transformIn->pitch,
        transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto& pointFrom = cloudIn->points[i];
        cloudOut->points[i].x =
            transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z +
            transCur(0, 3);
        cloudOut->points[i].y =
            transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z +
            transCur(1, 3);
        cloudOut->points[i].z =
            transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z +
            transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

gtsam::Pose3 DataConverter::pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

gtsam::Pose3 DataConverter::trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

void DataConverter::gtsamPose2trans(gtsam::Pose3 pose, float transformIn[])
{
    transformIn[0] = pose.rotation().roll();
    transformIn[1] = pose.rotation().pitch();
    transformIn[2] = pose.rotation().yaw();
    transformIn[3] = pose.translation().x();
    transformIn[4] = pose.translation().y();
    transformIn[5] = pose.translation().z();
}

Eigen::Affine3f DataConverter::pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x,
        thisPoint.y,
        thisPoint.z,
        thisPoint.roll,
        thisPoint.pitch,
        thisPoint.yaw);
}

Eigen::Affine3f DataConverter::trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3],
        transformIn[4],
        transformIn[5],
        transformIn[0],
        transformIn[1],
        transformIn[2]);
}

PointTypePose DataConverter::trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

PointType DataConverter::eigen2PointType(Vector3 point)
{
    PointType thisPose3D;
    thisPose3D.x = point[0];
    thisPose3D.y = point[1];
    thisPose3D.z = point[2];
    thisPose3D.intensity = 1;
    return thisPose3D;
}

void DataConverter::transformEigen2Odom(double timestamp, nav_msgs::Odometry& laserOdometryROS, float transform[6])
{
    laserOdometryROS.header.stamp = ros::Time().fromSec(timestamp);
    laserOdometryROS.header.frame_id = odometryFrame;
    laserOdometryROS.child_frame_id = "odom_mapping";
    laserOdometryROS.pose.pose.position.x = transform[3];
    laserOdometryROS.pose.pose.position.y = transform[4];
    laserOdometryROS.pose.pose.position.z = transform[5];
    laserOdometryROS.pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(transform[0], transform[1],
            transform[2]);
}



mapOptimization::mapOptimization()
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    parameters.factorization = gtsam::ISAM2Params::CHOLESKY;
    isam = new ISAM2(parameters);

    pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("glins/mapping/trajectory", 100);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("glins/mapping/map_global", 100);
    pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("glins/mapping/odometry", 100);
    pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("glins/mapping/odometry_incremental", 100);
    pubPath = nh.advertise<nav_msgs::Path>("/path", 1);

    subOptlidar = nh.subscribe<nav_msgs::Odometry>("glins/lidar/odom",
        100,
        &mapOptimization::lidarOdomHandler,
        this,
        ros::TransportHints().tcpNoDelay());
    subCloud = nh.subscribe<glins::cloud_info>("glins/feature/cloud_info",
        100,
        &mapOptimization::laserCloudInfoHandler,
        this,
        ros::TransportHints().tcpNoDelay());

    subRTKOdom = nh.subscribe<nav_msgs::Odometry>("/rtklib_odom",
        100,
        &mapOptimization::rtklibOdomHandler,
        this,
        ros::TransportHints().tcpNoDelay());

    subLoop = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection",
        100,
        &mapOptimization::loopInfoHandler,
        this,
        ros::TransportHints().tcpNoDelay());

    srvSaveMap = nh.advertiseService("glins/save_map", &mapOptimization::saveMapService, this);

    pubFeatInfo = nh.advertise<glins::feature_info>("glins/mapping/feature", 100);
    pubHistoryKeyFrames =
        nh.advertise<sensor_msgs::PointCloud2>("glins/mapping/icp_loop_closure_history_cloud", 100);
    pubIcpKeyFrames =
        nh.advertise<sensor_msgs::PointCloud2>("glins/mapping/icp_loop_closure_corrected_cloud", 100);
    pubLoopConstraintEdge =
        nh.advertise<visualization_msgs::MarkerArray>("/glins/mapping/loop_closure_constraints", 100);
    pubGpsConstraintEdge =
        nh.advertise<visualization_msgs::MarkerArray>("/glins/mapping/gps_constraints", 100);

    pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("glins/mapping/map_local", 100);
    pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>("glins/mapping/cloud_registered", 100);
    pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("glins/mapping/cloud_registered_raw", 100);
    pubCloudRaw = nh.advertise<sensor_msgs::PointCloud2>("cloud_deskewed", 100);

    pubSLAMInfo = nh.advertise<glins::cloud_info>("glins/mapping/slam_info", 100);

    gpsOdomPathPub = nh.advertise<nav_msgs::Path>("/navsat/odom/path", 100);

    downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
    downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity,
        surroundingKeyframeDensity,
        surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

    const float rawMapFilterSize = 0.5; // giseop
    downSizeFilterRaw.setLeafSize(rawMapFilterSize, rawMapFilterSize, rawMapFilterSize); // giseop
    radiusORFilter.setRadiusSearch(100);


    // set log dir
    dataSaverPtr = std::make_unique<DataSaver>(saveDirectory, sequence);
    // use imu frame when saving map
    dataSaverPtr->setExtrinc(true, t_body_sensor, q_body_sensor);
    dataSaverPtr->setConfigDir(configDirectory);

    allocateMemory();


    // std::cout << savePCDDirectory << std::endl;
    // std::cout << sequence << std::endl;
}

void mapOptimization::allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyGPSPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    cloudKeyGPSPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses2D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    laserCloudGroundLast.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerLastDS.reset(
        new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
    laserCloudSurfLastDS.reset(
        new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

    laserCloudRaw.reset(new pcl::PointCloud<PointType>()); // giseop
    laserCloudRawDS.reset(new pcl::PointCloud<PointType>()); // giseop

    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
    edgeMatchPoint_A.resize(N_SCAN * Horizon_SCAN);
    edgeMatchPoint_B.resize(N_SCAN * Horizon_SCAN);
    d_plane.resize(N_SCAN * Horizon_SCAN);
    surfNorm.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
    coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
    laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

    laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

    laserCloudCornerFromGPSMap.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromGPSMap.reset(new pcl::PointCloud<PointType>());
    laserCloudCornerFromGPSMapDS.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfFromGPSMapDS.reset(new pcl::PointCloud<PointType>());

    laserCornerCloudMatch.reset(new pcl::PointCloud<PointType>());
    laserSurfCloudMatch.reset(new pcl::PointCloud<PointType>());
    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    for (int i = 0; i < 6; ++i)
    {
        transformTobeMapped[i] = 0;
    }

    matP = Eigen::MatrixXf::Zero(6, 6);
}

//main ()
void mapOptimization::laserCloudInfoHandler(const glins::cloud_infoConstPtr& msgIn)
{
    // extract time stamp
    timeLaserInfoStamp = msgIn->header.stamp;
    timeLaserInfoCur = msgIn->header.stamp.toSec();


    // extract info and feature cloud
    cloudInfo = *msgIn;
    pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
    pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
    pcl::fromROSMsg(msgIn->cloud_ground, *laserCloudGroundLast);
    pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudRaw); // deskewed data

    std::lock_guard<std::mutex> lock(mtx);

    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
    {
        timeLastProcessing = timeLaserInfoCur;
        //        ROS_INFO("timeLaserInfoCur: %.8lf",timeLaserInfoCur);
        updateInitialGuess();

        if (systemInitialized)
        {
            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            if (lidarAssociateMode == 0)
            {
                scan2MapOptimization();
            }
            else
            {
                scan2MapOptimization();
                scan2GpsMapOptimization();
            }
            publishLidarFeature();

            saveKeyFramesAndFactor();

            correctPoses();

            publishOdometry();

            publishFrames();
        }

    }
}

void mapOptimization::rtklibOdomHandler(const nav_msgs::OdometryConstPtr& msg)
{
    gnssOdomQueue.push_back(*msg);
}

void mapOptimization::lidarOdomHandler(const nav_msgs::OdometryConstPtr& odomMsg)
{
    //        ROS_INFO("opt lidar timestamp: %.8lf",odomMsg->header.stamp.toSec());
    mtx.lock();
    double optOdomTime = ROS_TIME(odomMsg);
    gtsam::Pose3 lidarPose = Pose3(
        gtsam::Rot3::Quaternion(odomMsg->pose.pose.orientation.w,
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z),
        gtsam::Point3(odomMsg->pose.pose.position.x,
            odomMsg->pose.pose.position.y,
            odomMsg->pose.pose.position.z)
    );

    optlidarPose = lidarPose;
    bool info = findGPSAvail(optOdomTime);
    if (addoptPose && info)
    {
        double delta_yaw = abs(optlidarPose.rotation().yaw() - cloudKeyPoses6D->points.back().yaw) * 180 / M_PI;
        double delta_tran = (optlidarPose.translation() - Vector3(cloudKeyPoses6D->points.back().x, cloudKeyPoses6D->points.back().y, cloudKeyPoses6D->points.back().z)).norm();
        bool enableCorrectAngle = delta_yaw < 3.0;
        bool enableCorrectTrans = delta_tran > 0.5 && delta_tran < 5;
        if (debugGps)
        {
            ROS_INFO("delta_yaw: %lf delta_tran %lf", delta_yaw, delta_tran);
        }
        Vector6 odomNoiseVector = (Vector(6) << odomMsg->twist.twist.angular.x, odomMsg->twist.twist.angular.y, odomMsg->twist.twist.angular.z,
            odomMsg->twist.twist.linear.x, odomMsg->twist.twist.linear.y, odomMsg->twist.twist.linear.z).finished();
        Vector3 tranNoiseVector = (Vector(3) << odomMsg->twist.twist.linear.x, odomMsg->twist.twist.linear.y, odomMsg->twist.twist.linear.z).finished();
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished()); // rad*rad, meter*meter
        noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector);
        noiseModel::Diagonal::shared_ptr transNoise = noiseModel::Diagonal::Variances(tranNoiseVector);
        // if (tranNoiseVector.norm() > 2)
        // {
        //     ROS_WARN("transNoiseVector is too large: %lf", tranNoiseVector.norm());
        //     mtx.unlock();
        //     return;
        // }
        if (enableCorrectAngle)
        {
            gtSAMgraph.add(PriorFactor<Pose3>(cloudKeyPoses3D->size() - 1, optlidarPose, odomNoise));
        }
        else if (enableCorrectTrans)
        {
            gtSAMgraph.add(GPSFactor(cloudKeyPoses3D->size() - 1, optlidarPose.translation(), transNoise));
        }
        else
        {
            mtx.unlock();
            return;
        }
        aLoopIsClosed = true;

        // update iSAM
        isam->update(gtSAMgraph);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        cloudKeyPoses3D->points.pop_back();
        cloudKeyGPSPoses3D->points.pop_back();
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);
        cloudKeyGPSPoses3D->push_back(thisPose3D);

        cloudKeyPoses6D->points.pop_back();
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        lastGPSpose = latestEstimate;

        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save keyframe pose odom
        //        nav_msgs::Odometry updatesOdometryROS;
        //        transformEiegn2Odom(timeLaserInfoCur, updatesOdometryROS, transformTobeMapped);
        //        keyframePosesOdom.push_back(updatesOdometryROS);

        if (!globalPath.poses.empty()) globalPath.poses.pop_back();
        // save path for visualization
        updatePath(thisPose6D);

        correctPoses();

        if (findGPSAvail(optOdomTime))
        {
            lastGPSpose = latestEstimate;
            geometry_msgs::PoseStamped pose_gps;
            pose_gps.header.stamp = ros::Time().fromSec(optOdomTime);
            pose_gps.header.frame_id = odometryFrame;
            pose_gps.pose.position.x = thisPose6D.x;
            pose_gps.pose.position.y = thisPose6D.y;
            pose_gps.pose.position.z = thisPose6D.z;
            tf::Quaternion q = tf::createQuaternionFromRPY(thisPose6D.roll, thisPose6D.pitch, thisPose6D.yaw);
            pose_gps.pose.orientation.x = q.x();
            pose_gps.pose.orientation.y = q.y();
            pose_gps.pose.orientation.z = q.z();
            pose_gps.pose.orientation.w = q.w();
            if (!gpsOdomPath.poses.empty())gpsOdomPath.poses.pop_back();
            gpsOdomPath.header.stamp.fromSec(optOdomTime);
            gpsOdomPath.header.frame_id = odometryFrame;
            gpsOdomPath.poses.push_back(pose_gps);
        }
    }
    mtx.unlock();

    //        saveKeyFramesAndFactor();

    //        correctPoses();
}

void mapOptimization::pointAssociateToMap(PointType const* const pi, PointType* const po)
{
    po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y
        + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
    po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y
        + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
    po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y
        + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
    po->intensity = pi->intensity;
}



bool mapOptimization::saveMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    if (cloudKeyPoses6D->size() < 1)
    {
        ROS_INFO("NO ENCOUGH POSE!");
        return false;
    }


    // when you shut down the terminal , we will save odom  and map
    Eigen::Vector3d optimized_lla;
    if (useGPS)
    {
        Eigen::Vector3d first_point(cloudKeyPoses6D->at(0).x, cloudKeyPoses6D->at(0).y, cloudKeyPoses6D->at(0).z);

        // we save optimized origin gps point, maybe the altitude value need to be fixes
        Eigen::Vector3d ecef_point;
        ecef_point = GNSS_Tools::enu2ecef(transLLA, first_point);//gpsTools.ENU2ECEF(first_point);
        optimized_lla = GNSS_Tools::ecef2llh(ecef_point);//gpsTools.ECEF2LLA(ecef_point);

        std::cout << std::setprecision(9) << "origin LLA: " << transLLA.transpose() << std::endl;
        std::cout << std::setprecision(9) << "optimized LLA: " << optimized_lla.transpose() << std::endl;
        dataSaverPtr->saveOriginGPS(optimized_lla);
    }


    vector<pcl::PointCloud<PointType>::Ptr> keyframePc;
    std::vector<Eigen::Vector3d> lla_vec;
    pcl::PointCloud<PointType>::Ptr temp_ptr(new pcl::PointCloud<PointType>);
    for (int i = 0; i < cloudKeyPoses6D->size(); ++i)
    {
        PointTypePose p = cloudKeyPoses6D->at(i);

        // keyframeTimes.push_back(p.time);
        //            Eigen::Translation3d tf_trans(p.x, p.y, p.z);
        //            Eigen::AngleAxisd rot_x(p.roll, Eigen::Vector3d::UnitX());
        //            Eigen::AngleAxisd rot_y(p.pitch, Eigen::Vector3d::UnitY());
        //            Eigen::AngleAxisd rot_z(p.yaw, Eigen::Vector3d::UnitZ());
        //            Eigen::Matrix4d trans = (tf_trans * rot_z * rot_y * rot_x).matrix();
        //            keyframePosestrans.push_back(trans);

        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = ros::Time().fromSec(p.time);
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = lidarFrame;
        laserOdometryROS.pose.pose.position.x = p.x;
        laserOdometryROS.pose.pose.position.y = p.y;
        laserOdometryROS.pose.pose.position.z = p.z;
        laserOdometryROS.pose.pose.orientation =
            tf::createQuaternionMsgFromRollPitchYaw(p.roll, p.pitch,
                p.yaw);
        keyframePosesOdom.push_back(laserOdometryROS);
        //            temp_ptr->clear();
        //            *temp_ptr += *surfCloudKeyFrames.at(i);
        //            *temp_ptr += *cornerCloudKeyFrames.at(i);
        //            keyframePc.push_back(temp_ptr);

        if (useGPS)
        {
            // we save optimized origin gps point, maybe the altitude value need to be fixes
            Eigen::Vector3d first_point(p.x, p.y, p.z);
            Eigen::Vector3d lla_point, ecef_point;
            ecef_point = GNSS_Tools::enu2ecef(optimized_lla, first_point);// gpsTools.ENU2ECEF(first_point);
            lla_point = GNSS_Tools::ecef2llh(ecef_point);// gpsTools.ECEF2LLA(ecef_point);
            lla_vec.push_back(lla_point);
        }
    }

    std::cout << "TImes, isam, raw_odom, pose_odom, pose3D, pose6D size: " <<
        keyframeTimes.size() << " " << isamCurrentEstimate.size() << ", " <<
        keyframeRawOdom.size() << " " << keyframePosesOdom.size() << " " <<
        cloudKeyPoses3D->size() << " " << cloudKeyPoses6D->size() << std::endl;
    std::cout << "key_cloud, surf, corner, raw_frame size: " << keyframePc.size()
        << " " << surfCloudKeyFrames.size() << " " << cornerCloudKeyFrames.size() << " "
        << laserCloudRawKeyFrames.size() << std::endl;


    dataSaverPtr->saveTimes(keyframeTimes);
    dataSaverPtr->saveGraphGtsam(gtSAMgraph, isam, isamCurrentEstimate);
    dataSaverPtr->saveOptimizedVerticesTUM(isamCurrentEstimate);
    dataSaverPtr->saveOptimizedVerticesKITTI(isamCurrentEstimate);
    dataSaverPtr->saveOdometryVerticesTUM(keyframeRawOdom);
    dataSaverPtr->saveResultBag(keyframePosesOdom, allResVec);

    if (useGPS)
        dataSaverPtr->saveKMLTrajectory(lla_vec);


    pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalRawCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalRawCloudDS(new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
    {
        *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        *globalRawCloud += *transformPointCloud(laserCloudRawKeyFrames[i], &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size()
            << " ...";
    }

    downSizeFilterCorner.setInputCloud(globalCornerCloud);
    downSizeFilterCorner.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);
    downSizeFilterCorner.filter(*globalCornerCloudDS);
    // down-sample and save surf cloud
    downSizeFilterSurf.setInputCloud(globalSurfCloud);
    downSizeFilterSurf.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);
    downSizeFilterSurf.filter(*globalSurfCloudDS);

    downSizeFilterSurf.setInputCloud(globalRawCloud);
    downSizeFilterSurf.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);
    downSizeFilterSurf.filter(*globalRawCloudDS);

    // save global point cloud map
    //        *globalMapCloud += *globalCornerCloudDS;
    //        *globalMapCloud += *globalSurfCloudDS;
    *globalMapCloud += *globalRawCloudDS;
    std::cout << "map size: " << globalMapCloud->size() << std::endl;
    dataSaverPtr->savePointCloudMap(*globalMapCloud);
    //dataSaverPtr->savePointCloudMap(keyframePosesOdom, laserCloudRawKeyFrames);

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed: " << endl;

    return true;
}

bool mapOptimization::saveMapService()
{
    if (cloudKeyPoses6D->size() < 1)
    {
        ROS_INFO("NO ENCOUGH POSE!");
        return false;
    }

    pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalRawCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalRawCloudDS(new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
    {
        *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        *globalRawCloud += *transformPointCloud(laserCloudRawKeyFrames[i], &cloudKeyPoses6D->points[i]);
        cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size()
            << " ...";
    }

    downSizeFilterCorner.setInputCloud(globalCornerCloud);
    downSizeFilterCorner.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);
    downSizeFilterCorner.filter(*globalCornerCloudDS);
    // down-sample and save surf cloud
    downSizeFilterSurf.setInputCloud(globalSurfCloud);
    downSizeFilterSurf.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);
    downSizeFilterSurf.filter(*globalSurfCloudDS);

    downSizeFilterSurf.setInputCloud(globalRawCloud);
    downSizeFilterSurf.setLeafSize(globalMapLeafSize, globalMapLeafSize, globalMapLeafSize);
    downSizeFilterSurf.filter(*globalRawCloudDS);

    // save global point cloud map
    //        *globalMapCloud += *globalCornerCloudDS;
    //        *globalMapCloud += *globalSurfCloudDS;
    *globalMapCloud += *globalRawCloud;
    std::cout << "map size: " << globalMapCloud->size() << std::endl;
    dataSaverPtr->savePointCloudMap(*globalMapCloud);
    //dataSaverPtr->savePointCloudMap(keyframePosesOdom, laserCloudRawKeyFrames);

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed: " << endl;

    return true;
}
void mapOptimization::visualizeGlobalMapThread()
{
    ros::Rate rate(0.2);
    while (ros::ok())
    {
        rate.sleep();
        publishGlobalMap();
    }
}

void mapOptimization::publishGlobalMap()
{
    if (pubLaserCloudSurround.getNumSubscribers() == 0)
        return;

    if (cloudKeyPoses3D->points.empty() == true)
        return;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

    // kd-tree to find near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(),
        globalMapVisualizationSearchRadius,
        pointSearchIndGlobalMap,
        pointSearchSqDisGlobalMap,
        0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
        globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity,
        globalMapVisualizationPoseDensity,
        globalMapVisualizationPoseDensity); // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for (auto& pt : globalMapKeyPosesDS->points)
    {
        kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
        pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
    }

    // extract visualized and downsampled key frames
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
    {
        if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) >
            globalMapVisualizationSearchRadius)
            continue;
        int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
        *globalMapKeyFrames +=
            *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
            &cloudKeyPoses6D->points[thisKeyInd]);
    }
    // downsample visualized points
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize,
        globalMapVisualizationLeafSize,
        globalMapVisualizationLeafSize); // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
}


void mapOptimization::loopClosureThread()
{
    if (loopClosureEnableFlag == false)
        return;

    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        ros::spinOnce();

        performLoopClosure();
        visualizeLoopClosure();

        if (useGPS)
            visualGPSConstraint();

        rate.sleep();
    }
}

void mapOptimization::loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
{
    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopMsg->data.size() != 2)
        return;

    loopInfoVec.push_back(*loopMsg);

    while (loopInfoVec.size() > 5)
        loopInfoVec.pop_front();
}


///回环检测
void mapOptimization::performLoopClosure()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses2D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // find keys
    int loopKeyCur;
    int loopKeyPre;
    if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
            return;

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
    {
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            return;
        if (pubHistoryKeyFrames.getNumSubscribers() != 0)
            publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }

    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    // publish corrected cloud
    if (pubIcpKeyFrames.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
        publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    }

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    Eigen::Affine3f
        tCorrect =
        correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
    lastLoopIndex = loopKeyCur;
}

bool mapOptimization::detectLoopClosureDistance(int* latestID, int* closestID)
{
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    // tricks
    // Two consecutive loop edges represent the closed loop of the same scene.
    // Adding all of them to the pose graph has little meaning and may reduce the accuracy.
    if (abs(lastLoopIndex - loopKeyCur) < 5 && lastLoopIndex != -1)
        return false;

    // tricks
    // sometimes we need to find the corressponding loop pairs
    // but we do not need to care about the z values of these poses.
    // Pls note that this is not work for stair case
    for (int i = 0; i < copy_cloudKeyPoses2D->size(); ++i)
    {
        copy_cloudKeyPoses2D->at(i).z = 0;
    }

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D);
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses2D->back(),
        historyKeyframeSearchRadius,
        pointSearchIndLoop,
        pointSearchSqDisLoop,
        0);

    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;

    // we also need to care about the accumulated distance between keyframe;
    // LOOPs that are too close together have no meaning and may reduce accuracy. For example, the lidar starts to move after being stationary for 30s in a certain place.
    // At this time, the IMU should be trusted more than the lidar.
    if (keyframeDistances.size() >= loopKeyCur)
    {
        double distance = 0.0;
        for (int j = loopKeyPre; j < loopKeyCur; ++j)
        {
            distance += keyframeDistances.at(j);
        }
        if (distance < 10)
        {
            std::cout << "CLOSE FRAME MUST FILTER OUT " << distance << std::endl;
            return false;
        }
    }

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

bool mapOptimization::detectLoopClosureExternal(int* latestID, int* closestID)
{
    // this function is not used yet, please ignore it
    int loopKeyCur = -1;
    int loopKeyPre = -1;

    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopInfoVec.empty())
        return false;

    double loopTimeCur = loopInfoVec.front().data[0];
    double loopTimePre = loopInfoVec.front().data[1];
    loopInfoVec.pop_front();

    if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
        return false;

    int cloudSize = copy_cloudKeyPoses6D->size();
    if (cloudSize < 2)
        return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i)
    {
        if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
            loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i)
    {
        if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
            loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
        else
            break;
    }

    if (loopKeyCur == loopKeyPre)
        return false;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
}

void mapOptimization::loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
{
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;
        *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear],
            &copy_cloudKeyPoses6D->points[keyNear]);
        *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],
            &copy_cloudKeyPoses6D->points[keyNear]);
    }

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

void mapOptimization::visualizeLoopClosure()
{
    if (loopIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}

void mapOptimization::visualGPSConstraint()
{
    if (gpsIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // gps nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "gps_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0.8;
    markerNode.color.g = 0;
    markerNode.color.b = 1;
    markerNode.color.a = 1;

    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "gps_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.2;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0;
    markerEdge.color.b = 0.1;
    markerEdge.color.a = 1;

    for (auto it = gpsIndexContainer.begin(); it != gpsIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;

        geometry_msgs::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);

        p.x = cloudKeyGPSPoses3D->points[key_pre].x;
        p.y = cloudKeyGPSPoses3D->points[key_pre].y;
        p.z = cloudKeyGPSPoses3D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubGpsConstraintEdge.publish(markerArray);
}

//初始化位姿
/***
 * imu原始数据（cloudInfo.imuRollInit
 * imu里程计信息(cloudInfo.initialGuessX
 * 参考：LIO-SAM中mapOptmization.cpp中的updateInitialGuess()理解（https://blog.csdn.net/qq_44305240/article/details/126990259）
 */
void mapOptimization::updateInitialGuess()
{
    // save current transformation before any processing
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);
    static Eigen::Affine3f lastImuTransformation;

    // initialization the first frame
    if (cloudKeyPoses3D->points.empty())
    {
        systemInitialized = false;
        {
            // 用imu的旋转部分初始化当前
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            if (useGroundTruthHeading)
            {
                //                    transformTobeMapped[0] = 0.4801443619*M_PI/180;
                //                    transformTobeMapped[1] = -0.6575607432*M_PI/180;
                //                    transformTobeMapped[2] = -132.9341288900*M_PI/180;
                transformTobeMapped[0] = GroundTruthRoll * M_PI / 180;
                transformTobeMapped[1] = GroundTruthPitch * M_PI / 180;
                transformTobeMapped[2] = GroundTruthHeading * M_PI / 180;
            }
            Rot3 initialAtitude = Rot3::RzRyRx(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            Point3 initialpos = initialAtitude * (-extGPS);
            lastImuTransformation = pcl::getTransformation(initialpos[0],
                initialpos[1],
                initialpos[2],
                transformTobeMapped[0],
                transformTobeMapped[1],
                transformTobeMapped[2]); // save imu before return;
            initialpos += initialAtitude * extTrans;
            transformTobeMapped[3] = initialpos[0];
            transformTobeMapped[4] = initialpos[1];
            transformTobeMapped[5] = initialpos[2];
            systemInitialized = true;
            return;
        }
    }

    if (!systemInitialized)
    {
        ROS_ERROR("sysyem need to be initialized");
        return;
    }


    // if not the first frame
    // use imu pre-integration estimation for pose guess
    static bool lastImuPreTransAvailable = false;
    if (cloudInfo.odomAvailable == true)
    {
        Eigen::Affine3f
            transBack = pcl::getTransformation(cloudInfo.initialGuessX,
                cloudInfo.initialGuessY,
                cloudInfo.initialGuessZ,
                cloudInfo.initialGuessRoll,
                cloudInfo.initialGuessPitch,
                cloudInfo.initialGuessYaw);
        if (lastImuPreTransAvailable == false)
        {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        }
        else
        {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal,
                transformTobeMapped[3],
                transformTobeMapped[4],
                transformTobeMapped[5],
                transformTobeMapped[0],
                transformTobeMapped[1],
                transformTobeMapped[2]);
            ROS_INFO("transformTobeMapped initial pose: %f, %f, %f, %f, %f, %f",
                transformTobeMapped[0],
                transformTobeMapped[1],
                transformTobeMapped[2],
                transformTobeMapped[3],
                transformTobeMapped[4],
                transformTobeMapped[5]);
            lastImuPreTransformation = transBack;

            lastImuTransformation = pcl::getTransformation(0,
                0,
                0,
                cloudInfo.imuRollInit,
                cloudInfo.imuPitchInit,
                cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    // use imu incremental estimation for pose guess (only rotation)
    if (imuType == 1 && cloudInfo.imuAvailable == true)
    {
        Eigen::Affine3f transBack =
            pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit,
                cloudInfo.imuYawInit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal,
            transformTobeMapped[3],
            transformTobeMapped[4],
            transformTobeMapped[5],
            transformTobeMapped[0],
            transformTobeMapped[1],
            transformTobeMapped[2]);

        // update last imu transformation
        lastImuTransformation = pcl::getTransformation(0,
            0,
            0,
            cloudInfo.imuRollInit,
            cloudInfo.imuPitchInit,
            cloudInfo.imuYawInit); // save imu before return;

        ROS_INFO("rpy:%.3f,%.3f,%.3f", cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        return;
    }
}

void mapOptimization::extractForLoopClosure()
{
    pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses - 1; i >= 0; --i)
    {
        if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
            cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    extractCloud(cloudToExtract);
}

void mapOptimization::extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(),
        (double)surroundingKeyframeSearchRadius,
        pointSearchInd,
        pointSearchSqDis);
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int id = pointSearchInd[i];
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }

    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    for (auto& pt : surroundingKeyPosesDS->points)
    {
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }

    // also extract some latest key frames in case the robot rotates in one position
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses - 1; i >= 0; --i)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
        {
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[i]);
        }
        else
        {
            break;
        }
    }
    if (lidarAssociateMode == 0)
    {
        extractCloud(surroundingKeyPosesDS);
    }
    else
    {
        extractCloud(surroundingKeyPosesDS);
        extractCloud_GPS(surroundingKeyPosesDS);
    }
}

void mapOptimization::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear();
    laserCloudCornerFromGPSMap->clear();
    laserCloudSurfFromGPSMap->clear();

    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
        {
            // transformed cloud available
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
        }
        else
        {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp =
                *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
                    &cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp =
                *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

    // clear map cache if too large
    if (laserCloudMapContainer.size() > 1000)
        laserCloudMapContainer.clear();
}

void mapOptimization::extractCloud_GPS(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudCornerFromGPSMap->clear();
    laserCloudSurfFromGPSMap->clear();
    ROS_INFO("last cloudKeyGPS time %.6lf", lastGPSpose_time);
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;

        if (cloudKeyGPSPoses6D->points[thisKeyInd].time >= lastGPSpose_time)
        {
            if (laserCloudMapContainerGPS.find(thisKeyInd) != laserCloudMapContainerGPS.end())
            {
                // transformed cloud available
                *laserCloudCornerFromGPSMap += laserCloudMapContainerGPS[thisKeyInd].first;
                *laserCloudSurfFromGPSMap += laserCloudMapContainerGPS[thisKeyInd].second;
            }
            else
            {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp =
                    *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyGPSPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp =
                    *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyGPSPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromGPSMap += laserCloudCornerTemp;
                *laserCloudSurfFromGPSMap += laserCloudSurfTemp;
                laserCloudMapContainerGPS[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }
    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromGPSMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromGPSMapDS);
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromGPSMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromGPSMapDS);

    // clear map cache if too large
    if (laserCloudMapContainerGPS.size() > 1000)
        laserCloudMapContainerGPS.clear();
}

void mapOptimization::extractSurroundingKeyFrames()
{
    if (cloudKeyPoses3D->points.empty() == true)
        return;

    // if (loopClosureEnableFlag == true)
    // {
    //     extractForLoopClosure();
    // } else {
    //     extractNearby();
    // }

    extractNearby();
}

void mapOptimization::downsampleCurrentScan()
{

    laserCloudRawDS->clear();
    downSizeFilterRaw.setInputCloud(laserCloudRaw);
    downSizeFilterRaw.filter(*laserCloudRawDS);

    // Downsample cloud from current scan
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}

void mapOptimization::updatePointAssociateToMap()
{
    transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
}

/***
 * 查找对应的特征点
 */
void mapOptimization::cornerOptimization()
{
    updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudCornerLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudCornerLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix3d matA1 = Matrix3d::Zero();
        Eigen::Vector3d matD1 = Vector3d::Zero();
        Eigen::Matrix3d matV1 = Matrix3d::Zero();
        //            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        //            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d pt(laserCloudCornerFromMapDS->points[pointSearchInd[j]].x,
                    laserCloudCornerFromMapDS->points[pointSearchInd[j]].y,
                    laserCloudCornerFromMapDS->points[pointSearchInd[j]].z);
                center = center + pt;
                nearCorners.push_back(pt);
            }
            center /= 5.0;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d zeroMean = nearCorners[j] - center;
                matA1 = matA1 + zeroMean * zeroMean.transpose();
            }

            matA1 /= 5.0;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(matA1);

            matD1 = eigenSolver.eigenvalues();

            matV1 = eigenSolver.eigenvectors();

            if (matD1(2, 0) > 3 * matD1(1, 0))
            {
                Eigen::Vector3d ptOnLine = center;
                Eigen::Vector3d ptA, ptB;
                ptA = ptOnLine + 0.1 * matV1.col(2);
                ptB = ptOnLine - 0.1 * matV1.col(2);

                Eigen::Vector3d lp(pointSel.x, pointSel.y, pointSel.z);
                Eigen::Vector3d a012 = (lp - ptA).cross(lp - ptB);
                Eigen::Vector3d l12 = ptA - ptB;

                Eigen::Vector3d ll = l12.cross(a012);
                ll.normalize();

                float ld2 = a012.norm() / l12.norm();

                float s = 1 - 0.9 * fabs(ld2);

                coeff.x = s * ll(0);
                coeff.y = s * ll(1);
                coeff.z = s * ll(2);
                coeff.intensity = s * ld2;

                if (s > 0.1)
                {
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    edgeMatchPoint_A[i] = ptA;
                    edgeMatchPoint_B[i] = ptB;
                    laserCloudOriCornerFlag[i] = true;
                }
            }
        }
    }
}

void mapOptimization::surfOptimization()
{
    updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0)
        {
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                    pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                    pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid)
            {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                    + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                Vector3 plane_norm(pa, pb, pc);
                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1)
                {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    surfNorm[i] = plane_norm;
                    d_plane[i] = pd;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

/***
* 查找对应的特征点
*/
void mapOptimization::cornerOptimization_gps()
{
    updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudCornerLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudCornerLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix3d matA1 = Matrix3d::Zero();
        Eigen::Vector3d matD1 = Vector3d::Zero();
        Eigen::Matrix3d matV1 = Matrix3d::Zero();
        //            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        //            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                if (pointSearchInd[j] >= laserCloudCornerFromGPSMapDS->points.size())
                {
                    ROS_ERROR("Out of bounds! pointSearchInd[%d] = %d, points size = %d\n", j, pointSearchInd[j], laserCloudCornerFromGPSMapDS->points.size());
                }
                Eigen::Vector3d pt(laserCloudCornerFromGPSMapDS->points[pointSearchInd[j]].x,
                    laserCloudCornerFromGPSMapDS->points[pointSearchInd[j]].y,
                    laserCloudCornerFromGPSMapDS->points[pointSearchInd[j]].z);
                center = center + pt;
                nearCorners.push_back(pt);
            }
            center /= 5.0;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d zeroMean = nearCorners[j] - center;
                matA1 = matA1 + zeroMean * zeroMean.transpose();
            }

            matA1 /= 5.0;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(matA1);

            matD1 = eigenSolver.eigenvalues();

            matV1 = eigenSolver.eigenvectors();

            if (matD1(2, 0) > 3 * matD1(1, 0))
            {
                Eigen::Vector3d ptOnLine = center;
                Eigen::Vector3d ptA, ptB;
                ptA = ptOnLine + 0.1 * matV1.col(2);
                ptB = ptOnLine - 0.1 * matV1.col(2);

                Eigen::Vector3d lp(pointSel.x, pointSel.y, pointSel.z);
                Eigen::Vector3d a012 = (lp - ptA).cross(lp - ptB);
                Eigen::Vector3d l12 = ptA - ptB;

                Eigen::Vector3d ll = l12.cross(a012);
                ll.normalize();

                float ld2 = a012.norm() / l12.norm();

                float s = 1 - 0.9 * fabs(ld2);

                coeff.x = s * ll(0);
                coeff.y = s * ll(1);
                coeff.z = s * ll(2);
                coeff.intensity = s * ld2;

                if (s > 0.1)
                {
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    edgeMatchPoint_A[i] = ptA;
                    edgeMatchPoint_B[i] = ptB;
                    laserCloudOriCornerFlag[i] = true;
                }
            }
        }
    }
}

void mapOptimization::surfOptimization_gps()
{
    updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < laserCloudSurfLastDSNum; i++)
    {
        PointType pointOri, pointSel, coeff;
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        pointOri = laserCloudSurfLastDS->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (pointSearchSqDis[4] < 1.0)
        {
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = laserCloudSurfFromGPSMapDS->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfFromGPSMapDS->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfFromGPSMapDS->points[pointSearchInd[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                if (fabs(pa * laserCloudSurfFromGPSMapDS->points[pointSearchInd[j]].x +
                    pb * laserCloudSurfFromGPSMapDS->points[pointSearchInd[j]].y +
                    pc * laserCloudSurfFromGPSMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid)
            {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                    + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                Vector3 plane_norm(pa, pb, pc);
                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1)
                {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    surfNorm[i] = plane_norm;
                    d_plane[i] = pd;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }
}

void mapOptimization::combineOptimizationCoeffs()
{
    // combine corner coeffs
    for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
    {
        if (laserCloudOriCornerFlag[i] == true)
        {
            laserCloudOri->push_back(laserCloudOriCornerVec[i]);
            coeffSel->push_back(coeffSelCornerVec[i]);
            laserCornerCloudMatch->push_back(laserCloudOriCornerVec[i]);
            laserCornerCloudMatch->push_back(eigen2PointType(edgeMatchPoint_A[i]));
            laserCornerCloudMatch->push_back(eigen2PointType(edgeMatchPoint_B[i]));
        }
    }
    // combine surf coeffs
    for (int i = 0; i < laserCloudSurfLastDSNum; ++i)
    {
        if (laserCloudOriSurfFlag[i] == true)
        {
            laserCloudOri->push_back(laserCloudOriSurfVec[i]);
            coeffSel->push_back(coeffSelSurfVec[i]);
            if (laserCloudOriSurfVec[i].intensity <= 0)
            {
                laserSurfCloudMatch->push_back(laserCloudOriSurfVec[i]);
                PointType tmpPoint = eigen2PointType(surfNorm[i]);
                tmpPoint.intensity = d_plane[i];
                laserSurfCloudMatch->push_back(tmpPoint);
            }
        }
    }
    if (debugLidarTimestamp)
    {
        ROS_INFO("laserCloudCornerNum Total:%d match:%d laserCloudSurfNum Total:%d match:%d", laserCloudCornerLastDSNum, laserCornerCloudMatch->size() / 3, laserCloudSurfLastDSNum, laserSurfCloudMatch->size() / 2);
    }
    // reset flag for next iteration
    std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
    std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
}

bool mapOptimization::LMOptimization(int iterCount)
{
    float srx = sin(transformTobeMapped[2]);
    float crx = cos(transformTobeMapped[2]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[0]);
    float crz = cos(transformTobeMapped[0]);

    int laserCloudSelNum = laserCloudOri->size();
    if (laserCloudSelNum < 50)
    {
        return false;
    }

    Eigen::MatrixXf matA(laserCloudSelNum, 6);
    Eigen::MatrixXf matAtA(6, 6);
    Eigen::MatrixXf matB(laserCloudSelNum, 1);
    Eigen::MatrixXf matAtB(6, 1);
    Eigen::MatrixXf matX(6, 1);

    PointType pointOri, coeff;

    for (int i = 0; i < laserCloudSelNum; i++)
    {
        // lidar -> camera
        pointOri.x = laserCloudOri->points[i].x;
        pointOri.y = laserCloudOri->points[i].y;
        pointOri.z = laserCloudOri->points[i].z;
        // lidar -> camera
        coeff.x = coeffSel->points[i].x;
        coeff.y = coeffSel->points[i].y;
        coeff.z = coeffSel->points[i].z;
        coeff.intensity = coeffSel->points[i].intensity;
        // in camera
//            float arx =
//                    (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x
//                    + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y
//                    + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) *
//                      coeff.z;
//
//            float ary = ((cry * srx * srz - crz * sry) * pointOri.x
//                         + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x
//                        + ((-cry * crz - srx * sry * srz) * pointOri.x
//                           + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;
//
//            float arz =
//                    ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) *
//                    coeff.x
//                    + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y
//                    +
//                    ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) *
//                    coeff.z;

        float arx = (-srx * cry * pointOri.x - (srx * sry * srz + crx * crz) * pointOri.y + (crx * srz - srx * sry * crz) * pointOri.z) * coeff.x
            + (crx * cry * pointOri.x - (srx * crz - crx * sry * srz) * pointOri.y + (crx * sry * crz + srx * srz) * pointOri.z) * coeff.y;

        float ary = (-crx * sry * pointOri.x + crx * cry * srz * pointOri.y + crx * cry * crz * pointOri.z) * coeff.x
            + (-srx * sry * pointOri.x + srx * sry * srz * pointOri.y + srx * cry * crz * pointOri.z) * coeff.y
            + (-cry * pointOri.x - sry * srz * pointOri.y - sry * crz * pointOri.z) * coeff.z;

        float arz = ((crx * sry * crz + srx * srz) * pointOri.y + (srx * crz - crx * sry * srz) * pointOri.z) * coeff.x
            + ((-crx * srz + srx * sry * crz) * pointOri.y + (-srx * sry * srz - crx * crz) * pointOri.z) * coeff.y
            + (cry * crz * pointOri.y - cry * srz * pointOri.z) * coeff.z;

        // camera -> lidar
        matA(i, 0) = arz;
        matA(i, 1) = ary;
        matA(i, 2) = arx;
        matA(i, 3) = coeff.x;
        matA(i, 4) = coeff.y;
        matA(i, 5) = coeff.z;
        matB(i, 0) = -coeff.intensity;
    }

    matAtA = matA.transpose() * matA;
    matAtB = matA.transpose() * matB;
    //        matX = matAtA.inverse() * matAtB;
    matX = matAtA.colPivHouseholderQr().solve(matAtB);

    if (iterCount == 0)
    {

        Eigen::MatrixXf matE(6, 1);
        Eigen::MatrixXf matV(6, 6);
        Eigen::MatrixXf matV2(6, 6);

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(matAtA);

        matE = eigenSolver.eigenvalues();

        matV = eigenSolver.eigenvectors();

        matV2 = matV;

        isDegenerate = false;
        float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
        for (int i = 5; i >= 0; i--)
        {
            if (matE(i, 0) < eignThre[i])
            {
                for (int j = 0; j < 6; j++)
                {
                    matV2(i, j) = 0;
                }
                isDegenerate = true;
            }
            else
            {
                break;
            }
        }
        matP = matV.inverse() * matV2;
    }

    if (isDegenerate)
    {
        matX = matP * matX;
    }
    transformTobeMapped[0] += matX(0, 0);
    transformTobeMapped[1] += matX(1, 0);
    transformTobeMapped[2] += matX(2, 0);
    transformTobeMapped[3] += matX(3, 0);
    transformTobeMapped[4] += matX(4, 0);
    transformTobeMapped[5] += matX(5, 0);

    float deltaR = sqrt(
        pow(pcl::rad2deg(matX(0, 0)), 2) +
        pow(pcl::rad2deg(matX(1, 0)), 2) +
        pow(pcl::rad2deg(matX(2, 0)), 2));
    float deltaT = sqrt(
        pow(matX(3, 0) * 100, 2) +
        pow(matX(4, 0) * 100, 2) +
        pow(matX(5, 0) * 100, 2));
    if (deltaR < 0.05 && deltaT < 0.05)
    {
        return true; // converged
    }
    return false; // keep optimizing
}

void mapOptimization::GroundConstraint()
{
    if (laserCloudGroundLast->points.size() > 50)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserGroundPointsDS(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::VoxelGrid<PointType> vf_ground;
        vf_ground.setLeafSize(0.4, 0.4, 0.4);
        vf_ground.setInputCloud(laserCloudGroundLast);
        vf_ground.filter(*laserGroundPointsDS);

        // 提取地面法线
        int groundPointsSize = laserGroundPointsDS->points.size();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matA0;
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matB0;

        matA0.resize(groundPointsSize, 3);
        matB0.setOnes(groundPointsSize, 1);
        matB0 = -1 * matB0;

        for (int i = 0; i < groundPointsSize; i++)
        {
            matA0(i, 0) = laserGroundPointsDS->points[i].x;
            matA0(i, 1) = laserGroundPointsDS->points[i].y;
            matA0(i, 2) = laserGroundPointsDS->points[i].z;
        }
        Eigen::Vector3f ground_normal;
        // find the norm of plane
        ground_normal = matA0.colPivHouseholderQr().solve(matB0);
        // ground_normal = this->T.block<3,3>(0,0) * ground_normal + this->T.block<3,1>(0,3);
        float negative_OA_dot_norm = 1 / ground_normal.norm();
        ground_normal.normalize();

        // calculate angle between the two vectors
        Eigen::Vector3f grav(0, 0, 1);
        Eigen::Quaternionf ground_q = Eigen::Quaternionf::FromTwoVectors(ground_normal, grav);
        ground_q.normalize();

        Eigen::Vector3f ground_ypr = ground_q.toRotationMatrix().eulerAngles(2, 1, 0);

        // Here n(pa, pb, pc) is unit norm of plane
        bool planeValid = true;
        for (int i = 0; i < groundPointsSize; i++)
        {
            // if OX * n > 0.2, then plane is not fit well
            if (fabs(ground_normal(0) * laserGroundPointsDS->points[i].x +
                ground_normal(1) * laserGroundPointsDS->points[i].y +
                ground_normal(2) * laserGroundPointsDS->points[i].z + negative_OA_dot_norm) > 0.3)
            {
                ROS_INFO("plane not fit well. %lf", fabs(ground_normal(0) * laserGroundPointsDS->points[i].x +
                    ground_normal(1) * laserGroundPointsDS->points[i].y +
                    ground_normal(2) * laserGroundPointsDS->points[i].z + negative_OA_dot_norm));
                ROS_INFO("X: %lf, Y: %lf, Z: %lf", laserGroundPointsDS->points[i].x, laserGroundPointsDS->points[i].y, laserGroundPointsDS->points[i].z);
                planeValid = false;
                break;
            }
        }
        para_tz[0] = static_cast<double>(transformTobeMapped[5]);
        Eigen::AngleAxisf rollAngle(AngleAxisf(transformTobeMapped[0], Vector3f::UnitX()));
        Eigen::AngleAxisf pitchAngle(AngleAxisf(transformTobeMapped[1], Vector3f::UnitY()));
        Eigen::AngleAxisf yawAngle(AngleAxisf(transformTobeMapped[2], Vector3f::UnitZ()));
        Eigen::Quaternionf q_tem;
        q_tem = yawAngle * pitchAngle * rollAngle;
        q_tem.normalize();
        para_q[0] = static_cast<double>(q_tem.x());
        para_q[1] = static_cast<double>(q_tem.y());
        para_q[2] = static_cast<double>(q_tem.z());
        para_q[3] = static_cast<double>(q_tem.w());

        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(para_tz, 1);
        problem.AddParameterBlock(para_q, 4);

        if (groundPointsSize)
        {
            // 利用地面法线施加约束
            if (planeValid)
            {
                ceres::CostFunction* Gravity_factor = PitchRollFactor::Create(ground_ypr[1], ground_ypr[2], 0.02); // 全局约束：不能小于0.01
                problem.AddResidualBlock(Gravity_factor, NULL, para_q);
            }

            // 添加z轴位移约束: 位移方差设置为0.01；
            double ground_cov = 40000;
            if (groundPointsSize > 10)
            {
                ground_cov = 0.01;
            }
            if (fabs(transformTobeMapped[1]) < 5.0)
            {
                ground_cov = 0.00001;
            }

            ceres::CostFunction* Ground_factor = GroundFactor::Create(ground_cov, para_tz_pre);
            problem.AddResidualBlock(Ground_factor, NULL, para_tz);
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 6;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        transformTobeMapped[5] = para_tz[0];
        q_tem.w() = static_cast<float>(para_q[3]);
        q_tem.x() = static_cast<float>(para_q[0]);
        q_tem.y() = static_cast<float>(para_q[1]);
        q_tem.z() = static_cast<float>(para_q[2]);
        q_tem.normalize();
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q_tem.w() * q_tem.x() + q_tem.y() * q_tem.z());
        double cosr_cosp = 1 - 2 * (q_tem.x() * q_tem.x() + q_tem.y() * q_tem.y());
        transformTobeMapped[0] = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q_tem.w() * q_tem.y() - q_tem.z() * q_tem.x());
        if (std::abs(sinp) >= 1)
            transformTobeMapped[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            transformTobeMapped[1] = std::asin(sinp);
        // Eigen::Vector3f opt_rpy = q_tem.toRotationMatrix().eulerAngles(2, 1, 0);

        // transformTobeMapped[1] = opt_rpy[1]; //pitch
        // transformTobeMapped[0] = opt_rpy[0]; //roll

        para_tz_pre = static_cast<double>(transformTobeMapped[5]);
    }
}

/***
 * 激光扫描数据SCAN直接与地图进行匹配
 */
void mapOptimization::scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    ROS_INFO("laserCloudSurfFromMapDS size: %zu", laserCloudSurfFromMapDS->size());

    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum &&
        laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            laserCloudOri->clear();
            coeffSel->clear();
            laserCornerCloudMatch->clear();
            laserSurfCloudMatch->clear();

            cornerOptimization();
            surfOptimization();
            combineOptimizationCoeffs();

            if (LMOptimization(iterCount) == true)
                break;
        }
        // GroundConstraint();
        transformUpdate();
    }
    else
    {
        ROS_WARN("Not enough features! Only %d edge and %d planar features available.",
            laserCloudCornerLastDSNum,
            laserCloudSurfLastDSNum);
    }
}

void mapOptimization::scan2GpsMapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    laserCornerCloudMatch->clear();
    laserSurfCloudMatch->clear();
    memcpy(tmptransformTobeMapped, transformTobeMapped, sizeof(float) * 6);
    Eigen::Affine3f gpsAffine = pcl::getTransformation(lastGPSpose.translation().x(),
        lastGPSpose.translation().y(),
        lastGPSpose.translation().z(),
        lastGPSpose.rotation().roll(),
        lastGPSpose.rotation().pitch(),
        lastGPSpose.rotation().yaw());// pclPointToAffine3f pose(lastGPSpose);
    Eigen::Affine3f curAffine = trans2Affine3f(tmptransformTobeMapped);
    Eigen::Affine3f increAffine = gpsAffine.inverse() * curAffine;

    pcl::getTranslationAndEulerAngles(increAffine,
        transformTobeMapped[3],
        transformTobeMapped[4],
        transformTobeMapped[5],
        transformTobeMapped[0],
        transformTobeMapped[1],
        transformTobeMapped[2]);
    //ROS_INFO打印tranformTobeMapped
    ROS_INFO("transformTobeMapped: %f, %f, %f, %f, %f, %f",
        transformTobeMapped[0],
        transformTobeMapped[1],
        transformTobeMapped[2],
        transformTobeMapped[3],
        transformTobeMapped[4],
        transformTobeMapped[5]);
    ROS_INFO("laserCloudSurfFromGPSMapDS size: %zu", laserCloudSurfFromGPSMapDS->size());

    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum &&
        laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromGPSMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromGPSMapDS);

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            laserCloudOri->clear();
            coeffSel->clear();
            laserCornerCloudMatch->clear();
            laserSurfCloudMatch->clear();

            cornerOptimization_gps();
            surfOptimization_gps();

            combineOptimizationCoeffs();

            if (LMOptimization(iterCount) == true)
                break;
        }
        // GroundConstraint();
        transformUpdate();
    }
    else
    {
        ROS_WARN("Not enough features! Only %d edge and %d planar features available.",
            laserCloudCornerLastDSNum,
            laserCloudSurfLastDSNum);
    }
    dPose = trans2gtsamPose(transformTobeMapped);
    Pose3 increPose = lastGPSpose.compose(dPose);
    // gtsamPose2trans(increPose, tmptransformTobeMapped);
    tmptransformTobeMapped[0] = constraintTransformation(tmptransformTobeMapped[0], rotation_tollerance);
    tmptransformTobeMapped[1] = constraintTransformation(tmptransformTobeMapped[1], rotation_tollerance);
    tmptransformTobeMapped[5] = constraintTransformation(tmptransformTobeMapped[5], z_tollerance);
    geometry_msgs::PoseStamped pose_gps;
    pose_gps.header.stamp = ros::Time().fromSec(timeLaserInfoCur);
    pose_gps.header.frame_id = odometryFrame;
    pose_gps.pose.position.x = increPose.translation().x();
    pose_gps.pose.position.y = increPose.translation().y();
    pose_gps.pose.position.z = increPose.translation().z();
    tf::Quaternion q = tf::createQuaternionFromRPY(increPose.rotation().roll(), increPose.rotation().pitch(), increPose.rotation().yaw());
    pose_gps.pose.orientation.x = q.x();
    pose_gps.pose.orientation.y = q.y();
    pose_gps.pose.orientation.z = q.z();
    pose_gps.pose.orientation.w = q.w();
    gpsOdomPath.header.stamp = timeLaserInfoStamp;
    gpsOdomPath.header.frame_id = odometryFrame;
    gpsOdomPath.poses.push_back(pose_gps);
    gpsOdomPathPub.publish(gpsOdomPath);
    memcpy(transformTobeMapped, tmptransformTobeMapped, sizeof(float) * 6);
    ROS_INFO("after transformTobeMapped: %f, %f, %f, %f, %f, %f",
        transformTobeMapped[0],
        transformTobeMapped[1],
        transformTobeMapped[2],
        transformTobeMapped[3],
        transformTobeMapped[4],
        transformTobeMapped[5]);
}

void mapOptimization::transformUpdate()
{
    if (cloudInfo.imuAvailable == true)
    {
        if (std::abs(cloudInfo.imuPitchInit) < 1.4)
        {
            double imuWeight = imuRPYWeight;
            tf::Quaternion imuQuaternion;
            tf::Quaternion transformQuaternion;
            double rollMid, pitchMid, yawMid;

            // slerp roll
            transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
            imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);

            //ROS_INFO打印cloudInfo
            ROS_INFO("imuRPYInit: %f, %f, %f",
                cloudInfo.imuRollInit,
                cloudInfo.imuPitchInit,
                cloudInfo.imuYawInit);
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid,
                yawMid);
            transformTobeMapped[0] = rollMid;

            // slerp pitch
            transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
            imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
            tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid,
                yawMid);
            transformTobeMapped[1] = pitchMid;
        }
    }

    transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
    transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
    transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

    incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
}

float mapOptimization::constraintTransformation(float value, float limit)
{
    if (value < -limit)
        value = -limit;
    if (value > limit)
        value = limit;

    return value;
}

bool mapOptimization::saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    if (lidarAssociateMode == 1) return true;
    if (sensor == SensorType::LIVOX)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
            return true;
    }

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f
        transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4],
            transformTobeMapped[5],
            transformTobeMapped[0], transformTobeMapped[1],
            transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (findGPSAvail(timeLaserInfoCur))
    {
        keyframeDistances.push_back(sqrt(x * x + y * y));
        ROS_INFO("keyframe for GPS: %.8lf", timeLaserInfoCur);
        return true;
    }

    if (fabs(timeLaserInfoCur - round(timeLaserInfoCur)) < 0.005)
        return true;

    if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;

    // std::cout << "distance gap: " << sqrt(x * x + y * y) << std::endl;
    keyframeDistances.push_back(sqrt(x * x + y * y));

    return true;
}

/***
 *
 */
void mapOptimization::addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-2, 1e-2, M_PI
                *
                M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    else
    {
        noiseModel::Diagonal::shared_ptr
            odometryNoise = noiseModel::Diagonal::Variances(
                (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1,
            cloudKeyPoses3D->size(),
            poseFrom.between(poseTo),
            odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        //            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1,
        //                                                cloudKeyPoses3D->size(),
        //                                                optlidarPose.between(poseTo),
        //                                                odometryNoise));
        //            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        //            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
        //                    (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
        //            aLoopIsClosed = true;
        //            gtSAMgraph.add(PriorFactor<Pose3>(cloudKeyPoses3D->size()-1, optlidarPose, priorNoise));
    }
}

void mapOptimization::addOptOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-2, 1e-2, M_PI
                *
                M_PI, 1e4, 1e4, 1e4).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, optlidarPose, priorNoise));
        initialEstimate.insert(0, optlidarPose);
    }
    else
    {
        noiseModel::Diagonal::shared_ptr
            odometryNoise = noiseModel::Diagonal::Variances(
                (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo = optlidarPose;//trans2gtsamPose(transformTobeMapped);
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1,
            cloudKeyPoses3D->size(),
            poseFrom.between(poseTo),
            odometryNoise));
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        aLoopIsClosed = true;
        //            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1,
        //                                                cloudKeyPoses3D->size(),
        //                                                optlidarPose.between(poseTo),
        //                                                odometryNoise));
        //            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        //            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
        //                    (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
        //            aLoopIsClosed = true;
        //            gtSAMgraph.add(PriorFactor<Pose3>(cloudKeyPoses3D->size()-1, optlidarPose, priorNoise));
    }
}

void mapOptimization::addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

bool mapOptimization::findGPSAvail(double curTime)
{
    if (lidarAssociateMode == 1) return true;
    double curgnssTime = gnssOdomQueue.front().header.stamp.toSec();
    double delta_imu2gps = curTime - curgnssTime;
    double delta_round = curTime - round(curTime);
    if (delta_imu2gps > 0.0015)
    { //0015
        while (!gnssOdomQueue.empty())
        {
            gnssOdomQueue.pop_front();
            curgnssTime = gnssOdomQueue.front().header.stamp.toSec();
            delta_imu2gps = curTime - curgnssTime;
            if (delta_imu2gps <= 0) break;
        }
    }
    return (fabs(delta_imu2gps) < 0.0015) || (fabs(delta_round) < 0.005);
}

void mapOptimization::saveKeyFramesAndFactor()
{
    if (saveFrame() == false && lidarAssociateMode != 2)
        return;

    // odom factor
    addOdomFactor();

    // gps factor
//        if (useGPS)
//            addGPSFactor();

    // loop factor
    addLoopFactor();

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // add raw odom
    nav_msgs::Odometry laserOdometryROS;
    transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS, transformTobeMapped);
    keyframeRawOdom.push_back(laserOdometryROS);

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    pcl::PointCloud<PointType>::Ptr laserCloudCornerTemp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr laserCloudSurfTemp(new pcl::PointCloud<PointType>());
    /// used for gps fusion
    if (cloudKeyPoses3D->empty() || findGPSAvail(timeLaserInfoCur))
    {
        Pose3 betweenPose_new = latestEstimate.between(lastGPSpose);
        thisPose6D.x = betweenPose_new.translation().x();
        thisPose6D.y = betweenPose_new.translation().y();
        thisPose6D.z = betweenPose_new.translation().z();
        thisPose6D.roll = betweenPose_new.rotation().roll();
        thisPose6D.pitch = betweenPose_new.rotation().pitch();
        thisPose6D.yaw = betweenPose_new.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        *laserCloudCornerTemp = *transformPointCloud(laserCloudCornerFromGPSMapDS, &thisPose6D);
        *laserCloudSurfTemp = *transformPointCloud(laserCloudSurfFromGPSMapDS, &thisPose6D);
        lastGPSpose = latestEstimate;
        lastGPSpose_time = timeLaserInfoCur;
        gpsIndexContainer[cloudKeyGPSPoses3D->size()] = cloudKeyPoses3D->size();
        cloudKeyGPSPoses3D->push_back(thisPose3D);
    }
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);
    ROS_INFO("%.6lf pose saved: %lf %lf %lf %lf %lf %lf", thisPose6D.time, thisPose6D.x, thisPose6D.y, thisPose6D.z, thisPose6D.roll, thisPose6D.pitch, thisPose6D.yaw);

    Pose3 betweenPose = lastGPSpose.between(latestEstimate);
    thisPose6D.x = betweenPose.translation().x();
    thisPose6D.y = betweenPose.translation().y();
    thisPose6D.z = betweenPose.translation().z();
    thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
    thisPose6D.roll = betweenPose.rotation().roll();
    thisPose6D.pitch = betweenPose.rotation().pitch();
    thisPose6D.yaw = betweenPose.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyGPSPoses6D->push_back(thisPose6D);
    ROS_INFO("%.6lf GPS pose saved: %lf %lf %lf %lf %lf %lf", thisPose6D.time, thisPose6D.x, thisPose6D.y, thisPose6D.z, thisPose6D.roll, thisPose6D.pitch, thisPose6D.yaw);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thislaserCloudRawKeyFrame(new pcl::PointCloud<PointType>());
    if (findGPSAvail(timeLaserInfoCur))
    {
        //        radiusORFilter.setInputCloud(laserCloudCornerTemp);
        //        radiusORFilter.filter(*thisCornerKeyFrame);
        //        radiusORFilter.setInputCloud(laserCloudSurfTemp);
        //        radiusORFilter.filter(*thisSurfKeyFrame);
        *thisCornerKeyFrame += *laserCloudCornerLastDS;
        *thisSurfKeyFrame += *laserCloudSurfLastDS;
    }
    else
    {
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
        pcl::copyPointCloud(*laserCloudRawDS, *thislaserCloudRawKeyFrame);
    }
    // save key frame cloud
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    laserCloudRawKeyFrames.push_back(thislaserCloudRawKeyFrame);
    allResVec.push_back(cloudInfo.cloud_deskewed);
    keyframeTimes.push_back(timeLaserInfoStamp.toSec());

    // save keyframe pose odom
    //        nav_msgs::Odometry updatesOdometryROS;
    //        transformEiegn2Odom(timeLaserInfoCur, updatesOdometryROS, transformTobeMapped);
    //        keyframePosesOdom.push_back(updatesOdometryROS);

    // save path for visualization
    updatePath(cloudKeyPoses6D->points.back());
}

void mapOptimization::correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // clear map cache
        laserCloudMapContainer.clear();
        laserCloudMapContainerGPS.clear();
        // clear path
        globalPath.poses.clear();
        // update key poses
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

            updatePath(cloudKeyPoses6D->points[i]);
        }

        aLoopIsClosed = false;
    }
}

void mapOptimization::updatePath(const PointTypePose& pose_in)
{
    geometry_msgs::PoseStamped pose_stamped;

    /*----------------------------------------------------------*/
    //transform Current frame to Body_imu frame
//    tf::Quaternion rot_to_bodyimu(0.003148, -0.002479, 0.000524,0.999992);
//    //tf::Quaternion rot_to_bodyimu(0,0,0,1);
//    tf::Vector3 trans_to_bodyimu(-0.047781, 0.007303, -0.026583);
//    tf::Transform current_to_bodyimu(rot_to_bodyimu,trans_to_bodyimu);
//    tf::Transform current_pose(tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw),
//                               tf::Vector3(pose_in.x, pose_in.y, pose_in.z));
//    //std::cout<< "previous pose: " << current_pose.getOrigin().x() <<"  "<<current_pose.getOrigin().y()<<"  " <<current_pose.getOrigin().z() << std::endl;
//    current_pose = current_to_bodyimu * current_pose;
    //std::cout<< "Transformed pose: " << current_pose.getOrigin().x()<<"  " <<current_pose.getOrigin().y()<<"  " <<current_pose.getOrigin().z() << "\n" << std::endl;

//    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
//    pose_stamped.header.frame_id = odometryFrame;
//    pose_stamped.pose.position.x = current_pose.getOrigin().x();
//    pose_stamped.pose.position.y = current_pose.getOrigin().y();
//    pose_stamped.pose.position.z = current_pose.getOrigin().z();
//    pose_stamped.pose.orientation.x =  current_pose.getRotation().x();
//    pose_stamped.pose.orientation.y =  current_pose.getRotation().y();
//    pose_stamped.pose.orientation.z =  current_pose.getRotation().z();
//    pose_stamped.pose.orientation.w =  current_pose.getRotation().w();
    /*----------------------------------------------------------*/

    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

void mapOptimization::transformEiegn2Odom(double timestamp, nav_msgs::Odometry& laserOdometryROS, float transform[6])
{
    laserOdometryROS.header.stamp = ros::Time().fromSec(timestamp);
    laserOdometryROS.header.frame_id = odometryFrame;
    laserOdometryROS.child_frame_id = "odom_mapping";
    laserOdometryROS.pose.pose.position.x = transform[3];
    laserOdometryROS.pose.pose.position.y = transform[4];
    laserOdometryROS.pose.pose.position.z = transform[5];
    laserOdometryROS.pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(transform[0], transform[1],
            transform[2]);
}

void mapOptimization::savePath(string path)
{
    FILE* fp = fopen(path.c_str(), "w");
    for (geometry_msgs::PoseStamped pose_stamped : globalPath.poses)
    {
        fprintf(fp, "%.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf %.8lf\n",
            pose_stamped.header.stamp.toSec(), pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z,
            pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
    }
    fclose(fp);
}

void mapOptimization::publishOdometry()
{
    // Publish odometry for ROS (global)
    nav_msgs::Odometry laserOdometryROS;
    transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS, transformTobeMapped);
    //        laserOdometryROS.header.stamp = timeLaserInfoStamp;
    //        laserOdometryROS.header.frame_id = odometryFrame;
    //        laserOdometryROS.child_frame_id = "odom_mapping";
    //        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
    //        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
    //        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
    //        laserOdometryROS.pose.pose.orientation =
    //                tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1],
    //                                                        transformTobeMapped[2]);
    pubLaserOdometryGlobal.publish(laserOdometryROS);

    // Publish TF
    static tf::TransformBroadcaster br;
    tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0],
        transformTobeMapped[1],
        transformTobeMapped[2]),
        tf::Vector3(transformTobeMapped[3],
            transformTobeMapped[4],
            transformTobeMapped[5]));
    tf::StampedTransform
        trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame,
            "lidar_link");
    br.sendTransform(trans_odom_to_lidar);
    // 发布光滑的激光里程计结果（mapping/odometry_incremental）
    /**
     * mapping/odometry_incremental里程计是只使用了点云匹配而没有使用因子图优化的里程计
     * liosam作者TixiaoShan在github回复中（https://github.com/TixiaoShan/LIO-SAM/issues/92）提到了这一点
     * 下面这部分计算incremental里程计中，incrementalOdometryAffineFront是上一帧经过因子图优化后的结果，
     * incrementalOdometryAffineBack是在点云匹配之后、因子图优化之前的缓存结果。
     * 因此，odometry_incremental是间接使用了因子图优化，相比odometry应该有一定延迟和平滑。但是根据实验的结果
     * 来看，似乎差别不大，但是为了体现作者的工作和思考，下面这部分代码依旧保留。
    */
    // Publish odometry for ROS (incremental)
    static bool lastIncreOdomPubFlag = false;
    static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
    static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
    if (lastIncreOdomPubFlag == false)
    {
        lastIncreOdomPubFlag = true;
        laserOdomIncremental = laserOdometryROS;
        increOdomAffine = trans2Affine3f(transformTobeMapped);
    }
    else
    {
        Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
        increOdomAffine = increOdomAffine * affineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = 0.1;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(roll, 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid,
                    yawMid);
                roll = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, pitch, 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid,
                    yawMid);
                pitch = pitchMid;
            }
        }
        laserOdomIncremental.header.stamp = timeLaserInfoStamp;
        laserOdomIncremental.header.frame_id = odometryFrame;
        laserOdomIncremental.child_frame_id = "odom_mapping";
        laserOdomIncremental.pose.pose.position.x = x;
        laserOdomIncremental.pose.pose.position.y = y;
        laserOdomIncremental.pose.pose.position.z = z;
        laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        if (isDegenerate)
            laserOdomIncremental.pose.covariance[0] = 1;
        else
            laserOdomIncremental.pose.covariance[0] = 0;
    }
    pubLaserOdometryIncremental.publish(laserOdomIncremental);
}

void mapOptimization::publishFrames()
{
    if (cloudKeyPoses3D->points.empty())
        return;
    // publish key poses
    publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
    // Publish surrounding key frames
    publishCloud(pubRecentKeyFrames, laserCloudSurfFromGPSMapDS, timeLaserInfoStamp, odometryFrame);
    // publish registered key frame
    if (pubRecentKeyFrame.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
        *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
        publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
    }
    // publish registered high-res raw cloud
    if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
        publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
    }
    if (pubCloudRaw.getNumSubscribers() != 0)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
        publishCloud(pubCloudRaw, cloudOut, timeLaserInfoStamp, lidarFrame);
    }

    // publish path
    if (pubPath.getNumSubscribers() != 0)
    {
        globalPath.header.stamp = timeLaserInfoStamp;
        globalPath.header.frame_id = odometryFrame;
        pubPath.publish(globalPath);
    }
    // publish SLAM infomation for 3rd-party usage
    static int lastSLAMInfoPubSize = -1;
    if (pubSLAMInfo.getNumSubscribers() != 0)
    {
        if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
        {
            glins::cloud_info slamInfo;
            slamInfo.header.stamp = timeLaserInfoStamp;
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            *cloudOut += *laserCloudCornerLastDS;
            *cloudOut += *laserCloudSurfLastDS;
            slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
            slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp,
                odometryFrame);
            pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
            *localMapOut += *laserCloudCornerFromMapDS;
            *localMapOut += *laserCloudSurfFromMapDS;
            slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp,
                odometryFrame);
            pubSLAMInfo.publish(slamInfo);
            lastSLAMInfoPubSize = cloudKeyPoses6D->size();
        }
    }
}

void mapOptimization::publishLidarFeature()
{
    //    if (saveFrame() == false)
    //        return;
    //        if (fabs(timeLaserInfoCur - floor(timeLaserInfoCur)) < 0.05 || fabs(timeLaserInfoCur - floor(timeLaserInfoCur) - 1) < 0.05){
    //            ROS_INFO("keyframe for GPS: %.8lf",timeLaserInfoCur);
    //        }else{
    //            return;
    //        }
    glins::feature_info feature;
    pcl::PointCloud<PointType>::Ptr laserCornerCloud, laserSurfCloud;
    pcl::PointCloud<PointType>::Ptr edgeP_A;
    pcl::PointCloud<PointType>::Ptr edgeP_B;
    pcl::PointCloud<PointType>::Ptr planeNorm;

    feature.isKeyFrame = saveFrame();
    feature.header.stamp = timeLaserInfoStamp;
    feature.header.frame_id = odometryFrame;
    if (!cloudKeyPoses3D->points.empty() && feature.isKeyFrame)
    {
        pcl::toROSMsg(*laserCornerCloudMatch, feature.laserCloudCornerMatch);
        pcl::toROSMsg(*laserSurfCloudMatch, feature.laserCloudSurfMatch);

        feature.laserCloudCornerMatch.header.stamp = timeLaserInfoStamp;
        feature.laserCloudCornerMatch.header.frame_id = odometryFrame;
        feature.laserCloudSurfMatch.header.stamp = timeLaserInfoStamp;
        feature.laserCloudSurfMatch.header.frame_id = odometryFrame;
    }
    nav_msgs::Odometry laserOdometryROS;
    transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS, transformTobeMapped);
    feature.laserOdom = laserOdometryROS;
    ///publish
    pubFeatInfo.publish(feature);

}
