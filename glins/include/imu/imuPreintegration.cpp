#include "imuPreintegration.h"
#include "gtsam/slam/PoseRotationPrior.h"
#include "factor/NhcFactor.h"
#include "gnss_tools.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

Eigen::Affine3f ParamServer::lastImuPreTransformation = pcl::getTransformation(0, 0, 0, 0, 0, 0);

TransformFusion::TransformFusion()
{
    if (lidarFrame != baselinkFrame)
    {
        try
        {
            tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
            tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("glins/mapping/odometry",
        5,
        &TransformFusion::lidarOdometryHandler,
        this,
        ros::TransportHints().tcpNoDelay());
    subImuOdometry = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental",
        2000,
        &TransformFusion::imuOdometryHandler,
        this,
        ros::TransportHints().tcpNoDelay());

    pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
    pubImuPath = nh.advertise<nav_msgs::Path>("glins/imu/path", 1);
}

Eigen::Affine3f TransformFusion::odom2affine(nav_msgs::Odometry odom)
{
    double x, y, z, roll, pitch, yaw;
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    return pcl::getTransformation(x, y, z, roll, pitch, yaw);
}

void TransformFusion::lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    std::lock_guard<std::mutex> lock(mtx);

    lidarOdomAffine = odom2affine(*odomMsg);

    lidarOdomTime = odomMsg->header.stamp.toSec();
}

void TransformFusion::imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // static tf
    static tf::TransformBroadcaster tfMap2Odom;
    static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
    tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

    std::lock_guard<std::mutex> lock(mtx);

    imuOdomQueue.push_back(*odomMsg);

    // get latest odometry (at current IMU stamp)
    if (lidarOdomTime == -1)
        return;
    while (!imuOdomQueue.empty())
    {
        if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
            imuOdomQueue.pop_front();
        else
            break;
    }
    Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
    Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
    Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
    Eigen::Affine3f imuOdomAffineLast = imuOdomAffineBack; // lidarOdomAffine * imuOdomAffineIncre;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

    // publish latest odometry
    nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
    laserOdometry.pose.pose.position.x = x;
    laserOdometry.pose.pose.position.y = y;
    laserOdometry.pose.pose.position.z = z;
    laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    pubImuOdometry.publish(laserOdometry);

    // publish tf
    static tf::TransformBroadcaster tfOdom2BaseLink;
    tf::Transform tCur;
    tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
    if (lidarFrame != baselinkFrame)
        tCur = tCur * lidar2Baselink;
    tf::StampedTransform
        odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
    tfOdom2BaseLink.sendTransform(odom_2_baselink);

    // publish IMU path
    static nav_msgs::Path imuPath;
    static double last_path_time = -1;
    double imuTime = imuOdomQueue.back().header.stamp.toSec();
    if (imuTime - last_path_time > 0.1)
    {
        last_path_time = imuTime;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose = laserOdometry.pose.pose;
        imuPath.poses.push_back(pose_stamped);
        while (!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
            imuPath.poses.erase(imuPath.poses.begin());
        if (pubImuPath.getNumSubscribers() != 0)
        {
            imuPath.header.stamp = imuOdomQueue.back().header.stamp;
            imuPath.header.frame_id = odometryFrame;
            pubImuPath.publish(imuPath);
        }
    }
}

IMUPreintegration::IMUPreintegration()
{
    subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic,
        2000,
        &IMUPreintegration::imuHandler,
        this,
        ros::TransportHints().tcpNoDelay());

    subLidarFeature = nh.subscribe<glins::feature_info>("glins/mapping/feature",
        20,
        &IMUPreintegration::featureHandler,
        this,
        ros::TransportHints().tcpNoDelay());
    pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 2000);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("glins/lidar/odom", 2000);

    pubGPSPath = nh.advertise<nav_msgs::Path>("glins/gps/path", 1);

    nh.param<float>("glins/surfNoise", surfNoise, 1);
    nh.param<float>("glins/edgeNoise", edgeNoise, 1);
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
    p->accelerometerCovariance =
        gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
    p->gyroscopeCovariance =
        gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
    p->integrationCovariance =
        gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); // error committed in integrating position from velocities
    //    gtsam::imuBias::ConstantBias
    //    prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias
    //        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6)
    //                << 4.902555e-1, 4.902555e-1, 4.902555e-1, 4.8481368110954e-4, 4.8481368110954e-4, 4.8481368110954e-4).finished());; // assume zero initial bias
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6)
        << -0.06,
        0.13, 0.03, -5e-5, -5e-5, -2.2e-4)
        .finished());
    ; // assume zero initial bias
    priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)
        << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
    priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);   // m/s
    priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
    priorExtNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-2);
    correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)
        << 0.05,
        0.05, 0.05, 0.1, 0.1, 1)
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
        //      p->n_gravity = gtsam::Vector3(imuGravity_N[0], imuGravity_N[1], imuGravity_N[2]);
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

    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p,
        prior_imu_bias); // setting up the IMU integration for IMU message thread
    imuIntegratorOpt_ =
        new gtsam::PreintegratedImuMeasurements(p,
            prior_imu_bias); // setting up the IMU integration for optimization
    imuIntegratorOdo_ =
        new gtsam::PreintegratedImuMeasurements(p,
            prior_imu_bias); // setting up the IMU integration for optimization
    // init rtk options
    container.loadrtklibConfig(rtklibConfigPath);
    container.registerGnssSubscriber(nh);
    container.setSystemInitialized(systemInitialized);
    container.setEstimateExtGPS(estimateExtGPS);
    container.setExtRot(extRot);
    container.setExtGPS(extGPS);

    result_path = fgoPath; //"/home/wangchuji/catkins_lidar/data/UrbanNav-HK-Medium-Urban-1/glins_ac.pos";
    fp = fopen(result_path.c_str(), "w");
    if (fp == nullptr)
    {
        std::string error = "Failed to open GLINS output file '" + result_path + "': " + std::strerror(errno);
        ROS_FATAL("%s", error.c_str());
        throw std::runtime_error(error);
    }
    // fp_debug = fopen("debug.log", "w");
    fprintf(fp, "%%  GPST              x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns   sdx(m)   sdy(m)   sdz(m)  sdxy(m)  sdyz(m)  sdzx(m) age(s)  ratio\n");
    fflush(fp);
    // fprintf(fp_debug, "%%  GPST              x-ecef(m)      y-ecef(m)      z-ecef(m)   Q  ns   sdx(m)   sdy(m)   sdz(m)  sdxy(m)  sdyz(m)  sdzx(m) age(s)  ratio\n");
    // fflush(fp_debug);
    cornercloudMatch.reset(new pcl::PointCloud<PointType>());
    surfcloudMatch.reset(new pcl::PointCloud<PointType>());
}

void IMUPreintegration::setStartTime(gtime_t& t)
{
    ts = t;
}

void IMUPreintegration::resetOptimization()
{
    gtsam::ISAM2Params optParameters;
    optParameters.optimizationParams = ISAM2DoglegParams(1.0, 1e-10, DoglegOptimizerImpl::TrustRegionAdaptationMode::SEARCH_EACH_ITERATION);
    optParameters.relinearizeThreshold = 0.01;
    optParameters.relinearizeSkip = 1;
    optParameters.factorization = gtsam::ISAM2Params::CHOLESKY;
    optParameters.findUnusedFactorSlots = true;
    optimizer = gtsam::ISAM2(optParameters);
    odomOptimizer = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newGraphFactors;
    graphFactors = newGraphFactors;
    odomGraphFactors = newGraphFactors;

    gtsam::Values NewGraphValues;
    graphValues = NewGraphValues;
    odomGraphValues = NewGraphValues;
}

void IMUPreintegration::resetParams()
{
    lastImuT_imu = -1;
    doneFirstOpt = false;
    systemInitialized = false;
}

void IMUPreintegration::addGPSFactor(int& nb, int& npr, int& ndop)
{
    if (container.checkIsEmpty())
        return;

    if (container.syncObs(lastImuT_opt, 0.015))
    {
        if (!useObs)
        {
            container.addGPSFactorENU(&graphFactors, &graphValues, key);
        }
        else
        {
            npr = container.addDDPsrFactorENU(&graphFactors, &graphValues, key);
            if (useCarrier)
            {
                nb = container.addDDCpFactorENU(&graphFactors, &graphValues, key, lastGNSSepoch);
            }
            ndop = container.addSDDopFactorENU(&graphFactors, &graphValues, key, lastGNSSepoch); //|| ndop < 6
            if (debugGps)
            {
                ROS_INFO("npr: %d nb: %d ndop: %d", npr, nb, ndop);
            }
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
        //            ROS_INFO("add gnssFactor success!");
        lastGNSSepoch = key;
    }
}

void IMUPreintegration::writeGPSfile(gtime_t gpst, Vector3 ecef, int state, FILE* file)
{
    int week;
    double weeksec = time2gpst(gpst, &week);
    fprintf(file, "%d %.3lf %.4lf %.4lf %.4lf %d %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.6lf %.6lf %.6lf %.6lf %.6lf %.6lf\n", week, weeksec, ecef(0), ecef(1), ecef(2), state,
        gpsPose.rotation().roll() * 180 / PI, gpsPose.rotation().pitch() * 180 / PI, gpsPose.rotation().yaw() * 180 / PI,
        sqrt(posCovariance(0, 0)), sqrt(posCovariance(1, 1)), sqrt(posCovariance(2, 2)),
        sqrt(posCovariance(3, 3)), sqrt(posCovariance(4, 4)), sqrt(posCovariance(5, 5)),
        prevBias_.accelerometer().x(), prevBias_.accelerometer().y(), prevBias_.accelerometer().z(),
        prevBias_.gyroscope().x(), prevBias_.gyroscope().y(), prevBias_.gyroscope().z());
}

void IMUPreintegration::writeGPSfile2(gtime_t gpst, Vector3 ecef, int state, FILE* file)
{
    int week;
    double weeksec = time2gpst(gpst, &week);
    fprintf(file, "%d %.3lf %.4lf %.4lf %.4lf %d %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.6lf %.6lf %.6lf %.6lf %.6lf %.6lf ", week, weeksec, ecef(0), ecef(1), ecef(2), state,
        gpsPose.rotation().roll() * 180 / PI, gpsPose.rotation().pitch() * 180 / PI, gpsPose.rotation().yaw() * 180 / PI,
        sqrt(posCovariance(0, 0)), sqrt(posCovariance(1, 1)), sqrt(posCovariance(2, 2)),
        sqrt(posCovariance(3, 3)), sqrt(posCovariance(4, 4)), sqrt(posCovariance(5, 5)),
        prevBias_.accelerometer().x(), prevBias_.accelerometer().y(), prevBias_.accelerometer().z(),
        prevBias_.gyroscope().x(), prevBias_.gyroscope().y(), prevBias_.gyroscope().z());
}

void IMUPreintegration::closePosfile()
{
    if (fp != nullptr)
        fclose(fp);
}

void IMUPreintegration::addLidarFactor()
{
    int laserCloudCornerNum, laserCloudSurfNum;
    laserCloudCornerNum = cornercloudMatch->size() / 3;
    laserCloudSurfNum = surfcloudMatch->size() / 2;
    int laserCornerAvailNum = 0, laserSurfAvailNum = 0;
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    Pose3 betweenPose = traits<Pose3>::Between(prevState_.pose(), propState_.pose());
    if (debugLidarTimestamp)
    {
        ROS_INFO("laserCloudCornerNum:%d laserCloudSurfNum:%d", laserCloudCornerNum, laserCloudSurfNum);
    }
    if (laserCloudSurfNum < 1 && laserCloudCornerNum < 1)
        return;
    // combine corner coeffs
    for (int i = 0; i < laserCloudCornerNum; ++i)
    {
        // if (laserCornerAvailNum > 100) break;
        Point3 cp(cornercloudMatch->points[i * 3].x, cornercloudMatch->points[i * 3].y, cornercloudMatch->points[i * 3].z);
        Point3 edA(cornercloudMatch->points[i * 3 + 1].x, cornercloudMatch->points[i * 3 + 1].y, cornercloudMatch->points[i * 3 + 1].z);
        Point3 edB(cornercloudMatch->points[i * 3 + 2].x, cornercloudMatch->points[i * 3 + 2].y, cornercloudMatch->points[i * 3 + 2].z);
        Point3 lp = betweenPose.transformFrom(cp);
        Vector3 nu = (lp - edA).cross(lp - edB);
        Vector3 de = edA - edB;
        double s = 1 - 0.9 * fabs(nu.norm() / de.norm());
        //        if (s <= 0.1) {
        //            ROS_WARN("corner s = %lf<0.1",s);
        //            continue;
        //        }
        if (lidarAssociateMode == 0)
        {
            noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(1, edgeNoise);
            noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.5), noise);
            auto lidarEdgefactor = LidarEdgeFactor(X(key), cp, edA, edB, extTrans, noise);
            graphFactors.add(lidarEdgefactor);
        }
        else
        {
            // noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(3, edgeNoise);
            noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(1, edgeNoise);
            noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(0.5), noise);
            // auto lidarEdgefactor = LidarEdgeFactorRelative(X(lastKeyIndex), X(key), cp, edA, edB, extTrans, huber);
            auto lidarEdgefactor = LidarEdgeNormFactorRelative(X(lastKeyIndex), X(key), cp, edA, edB, extTrans, noise);
            graphFactors.add(lidarEdgefactor);
        }
        laserCornerAvailNum++;
    }
    // combine surf coeffs
    for (int i = 0; i < laserCloudSurfNum; ++i)
    {
        // if (laserSurfAvailNum > 500) break;
        Point3 cp(surfcloudMatch->points[i * 2].x, surfcloudMatch->points[i * 2].y, surfcloudMatch->points[i * 2].z);
        Point3 norm(surfcloudMatch->points[i * 2 + 1].x, surfcloudMatch->points[i * 2 + 1].y, surfcloudMatch->points[i * 2 + 1].z);
        Point3 lp = betweenPose.transformFrom(cp);
        double dist = dot(lp, norm) + surfcloudMatch->points[i * 2 + 1].intensity;
        float s = fabs(1 - 0.9 * fabs(dist) / sqrt(cp.norm()));
        // if (s > 0.1) {
        if (lidarAssociateMode == 0)
        {
            noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(1, surfNoise);
            noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                noiseModel::mEstimator::Huber::Create(1.5), noise);
            auto lidarSurffactor = LidarPlaneNormFactor(X(key), cp, norm,
                surfcloudMatch->points[i * 2 + 1].intensity, extTrans,
                noise);
            graphFactors.add(lidarSurffactor);
        }
        else
        {
            noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(1, surfNoise);
            noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                noiseModel::mEstimator::Cauchy::Create(0.5), noise);
            auto lidarSurffactor = LidarPlaneNormFactorRelative(X(lastKeyIndex), X(key), cp, norm,
                surfcloudMatch->points[i * 2 + 1].intensity,
                extTrans, noise);
            graphFactors.add(lidarSurffactor);
        }
        laserSurfAvailNum++;
        // }else {
        //     if(debugLidarTimestamp){
        //         ROS_WARN("surf s = %lf<0.1",s);
        //     }
        // }
    }
    cornercloudMatch->clear();
    surfcloudMatch->clear();
}

void IMUPreintegration::featureHandler(const glins::feature_info::ConstPtr& featureMsg)
{
    std::lock_guard<std::mutex> lock(mtx);
    TicToc t_opt;
    t_opt.tic();
    ROS_INFO("feature time: %.8lf", featureMsg->header.stamp.toSec());
    featureInfo = *featureMsg;
    pcl::fromROSMsg(featureMsg->laserCloudCornerMatch, *cornercloudMatch);
    pcl::fromROSMsg(featureMsg->laserCloudSurfMatch, *surfcloudMatch);
    featureUpdate = true;
    static double lastgpstime = 0;
    volatile bool GNSSEnable = false;
    volatile double currentCorrectionTime = ROS_TIME(featureMsg);
    // make sure we have imu data to integrate
    if (imuQueOpt.empty())
        return;

    float p_x = featureMsg->laserOdom.pose.pose.position.x;
    float p_y = featureMsg->laserOdom.pose.pose.position.y;
    float p_z = featureMsg->laserOdom.pose.pose.position.z;
    float r_x = featureMsg->laserOdom.pose.pose.orientation.x;
    float r_y = featureMsg->laserOdom.pose.pose.orientation.y;
    float r_z = featureMsg->laserOdom.pose.pose.orientation.z;
    float r_w = featureMsg->laserOdom.pose.pose.orientation.w;
    bool degenerate = (int)featureMsg->laserOdom.pose.covariance[0] == 1 ? true : false;
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
        gtsam::Point3(p_x, p_y, p_z));

    int nb, npr, ndop;
    nb = 0;
    npr = 0;
    ndop = 0;

    // 0. initialize system
    if (systemInitialized == false)
    {
        resetOptimization();

        // pop old IMU message
        while (!imuQueOpt.empty())
        {
            if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
            {
                lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // initial pose
        prevPose_ = lidarPose.compose(lidar2Imu);
        if (debugImu)
        {
            ROS_INFO("predict pose: %lf %lf %lf", prevPose_.translation().x(), prevPose_.translation().y(), prevPose_.translation().z());
        }
        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
        graphFactors.add(priorPose);
        // initial velocity
        prevVel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
        graphFactors.add(priorVel);
        // initial bias
        prevBias_ = gtsam::imuBias::ConstantBias((gtsam::Vector(6)
            << 0.238860, -0.150600, 0.014712, -0.001496, -0.003197, 0.007483)
            .finished());
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
        graphFactors.add(priorBias);
        // add values
        graphValues.insert(X(0), prevPose_);
        graphValues.insert(V(0), prevVel_);
        graphValues.insert(B(0), prevBias_);

        if (GNSSEnable = container.isGNSSEnable(currentCorrectionTime))
        {
            if (useGPS)
            {
                // add gps factor
                addGPSFactor(nb, npr, ndop);
            }
        }


        //optimize for odom
        // {
        //     odomGraphFactors.add(priorPose);
        //     odomGraphFactors.add(priorVel);
        //     odomGraphFactors.add(priorBias);
        //     odomGraphValues.insert(X(0), prevPose_);
        //     odomGraphValues.insert(V(0), prevVel_);
        //     odomGraphValues.insert(B(0), prevBias_);
        //     odomOptimizer.update(odomGraphFactors, odomGraphValues);
        //     odomGraphFactors.resize(0);
        //     odomGraphValues.clear();
        // }
        // optimize once
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();

        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(0));
        prevVel_ = result.at<gtsam::Vector3>(V(0));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(0));
        if (nb > 0)
        {
            prevAmb_ = result.at<Vector>(N(0));
        }
        if (GNSSEnable)
        {
            lla_origin = container.getOrigin();
            gpsPose = prevPose_.compose(gps2imu);
            posCovariance = optimizer.marginalCovariance(X(0));
            Vector3 ecef = GNSS_Tools::enu2ecef(lla_origin,
                gpsPose.translation()); // gtools.ENU2ECEF(gpsPose.translation());
            gtime_t curgtime;
            curgtime.time = round(currentCorrectionTime) + 18;
            curgtime.sec = currentCorrectionTime - round(currentCorrectionTime);
            writeGPSfile(curgtime, ecef, 6, fp);
            fflush(fp);
        }
        imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // {
        //     result = odomOptimizer.calculateEstimate();
        //     prevStateOdom = gtsam::NavState(result.at<gtsam::Pose3>(X(0)), result.at<Vector3>(V(0)));
        //     prevBiasOdom = result.at<gtsam::imuBias::ConstantBias>(B(0));
        //     imuIntegratorOdo_->resetIntegrationAndSetBias(prevBias_);

        // }
        prePose = lidarPose.compose(lidar2Imu);
        lastKeyPose = prevPose_;
        lastGNSSepoch = 0;
        lastgpstime = currentCorrectionTime;
        lastKeyIndex = 0;
        gpsKeyQueue.push_back(0);
        key = 1;
        systemInitialized = true;
        container.setSystemInitialized(systemInitialized);
        return;
    }
    // 1. integrate imu data and optimize
    while (!imuQueOpt.empty())
    {
        // pop and integrate imu data that is between two optimizations
        sensor_msgs::Imu* thisImu = &imuQueOpt.front();
        double imuTime = ROS_TIME(thisImu);
        // std::cout << " delta_t: " << imuTime -lastImuT_opt << std::endl;
        if (fabs(imuTime - currentCorrectionTime + delta_t) > (0.5 / imuFrequence))
        {
            double dt = (lastImuT_opt < 0) ? (1.0 / imuFrequence) : (imuTime - lastImuT_opt);
            imuIntegratorOpt_->integrateMeasurement(
                gtsam::Vector3(thisImu->linear_acceleration.x,
                    thisImu->linear_acceleration.y,
                    thisImu->linear_acceleration.z),
                gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y,
                    thisImu->angular_velocity.z),
                dt);
            imuIntegratorOdo_->integrateMeasurement(
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
    // add imu factor to graph
    const gtsam::PreintegratedImuMeasurements
        & preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
    gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
    graphFactors.add(imu_factor);
    // add imu bias between factor
    graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1),
        B(key),
        gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Diagonal::Sigmas(
            sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

    //        gtsam::Pose3 delta_Pose = prePose.between(curPose);
    noiseModel::Base::shared_ptr
        odometryNoise = noiseModel::Robust::Create(noiseModel::mEstimator::Cauchy::Create(0.5),
            noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 1, 1, 3).finished()));

    // insert predicted values
    gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
    gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
    gtsam::Pose3 delta_Pose = propState_.pose().between(curPose);
    isRelative = fabs(delta_Pose.rotation().yaw() * 180 / M_PI) < 10 && delta_Pose.translation().head<2>().norm() < 2.0;
    gtsam::Pose3 delta_Pose1 = lastKeyPose.between(curPose);
    isStatic = delta_Pose1.translation().norm() < 0.02 && fabs(delta_Pose1.rotation().yaw() * 180 / M_PI) < 1.0;
    if (false)
    {
        gpsPose = propState_.pose().compose(gps2imu);
        Vector3 ecef = GNSS_Tools::enu2ecef(lla_origin, gpsPose.translation()); // gtools.ENU2ECEF(gpsPose.translation());
        gtime_t curgtime;
        curgtime.time = floor(currentCorrectionTime) + 18;
        curgtime.sec = currentCorrectionTime - floor(currentCorrectionTime);
        writeGPSfile(curgtime, ecef, 6, fp_debug);
        fflush(fp_debug);
    }
    if (debugImu)
    {
        ROS_INFO("IMU Integer time %.5lf", imuIntegratorOpt_->deltaTij());
        ROS_INFO("delta yaw %lf", fabs(delta_Pose.rotation().yaw() * 180 / M_PI));
    }
    // if (isRelative || !isStatic)
    // {
    graphValues.insert(X(key), propState_.pose());
    graphValues.insert(V(key), propState_.v());
    //     //        graphValues.insert(X(key), curPose);
    //     if (debugImu)
    //     {
    //         ROS_INFO("predict pose: %lf %lf %lf", propState_.pose().translation().x(), propState_.pose().translation().y(), propState_.pose().translation().z());
    //     }
    // }
    // else
    // {
    // graphValues.insert(X(key), curPose);
    // graphValues.insert(V(key), propState_.v());
    // graphValues.insert(V(key), Vector3(1e-4, 1e-4, 1e-4));
    //        graphValues.insert(X(key), propState_.pose());
    if (debugImu)
    {
        ROS_INFO("predict pose: %lf %lf %lf", curPose.translation().x(), curPose.translation().y(), curPose.translation().z());
        ROS_ERROR("yaw between lidar and imu is above 10 degree!");
    }
    // }
    //    prevState_.pose().between(propState_.pose()).print();

    graphValues.insert(B(key), prevBias_);

    gtime_t te;
    te.time = round(currentCorrectionTime) + 18;
    te.sec = currentCorrectionTime - round(currentCorrectionTime);
    volatile double weeksec = time2gpst(te, NULL);
    // add GPS factor (wcj)
    if (GNSSEnable = container.isGNSSEnable(currentCorrectionTime))
    {
        gpsKeyQueue.push_back(key);
        if (useGPS)
        {
            ROS_INFO("GPS KEY %d", key);
            addGPSFactor(nb, npr, ndop);
        }
    }

    // add Lidar factor
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose,
        degenerate ? correctionNoise2 : correctionNoise);
    // graphFactors.add(pose_factor);
    noiseModel::Base::shared_ptr
        correctRotNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1, 1, 1).finished()); // rad,rad,rad,m, m, m
    noiseModel::Base::shared_ptr
        correctRotNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.01, 0.01, 0.01).finished()); // rad,rad,rad,m, m, m

    PoseRotationPrior<gtsam::Pose3> pose_rotation_factor(X(key), curPose.rotation(),
        degenerate ? correctRotNoise2 : correctRotNoise);

    if (!isRelative || isStatic)
    {
        // if(!GNSSEnable) graphFactors.add(pose_factor);
        graphFactors.add(pose_rotation_factor);
    }

    volatile double delta_pitch = delta_Pose1.rotation().pitch() * 180 / PI;
    volatile double cur_pitch = curPose.rotation().pitch() * 180 / PI;
    volatile double deltat = currentCorrectionTime - lastgpstime;
    if (fabs(delta_Pose1.z() / deltat) < 0.15)
    {
        noiseModel::Base::shared_ptr
            Zaxis_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 0.01).finished());

        // graphFactors.add(ZAxisConstraint(X(key), prevPose_.z(), Zaxis_noise));
    }

    if (isStatic)
    {
        noiseModel::Base::shared_ptr
            ZUPTnoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 0.01, 0.01, 0.01).finished());
        noiseModel::Base::shared_ptr
            correctRotNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished()); // rad,rad,rad,m, m, m
        graphFactors.add(gtsam::PriorFactor<gtsam::Vector3>(V(key), (gtsam::Vector(3) << 0.00, 0.00, 0.00).finished(), ZUPTnoise));
        // graphFactors.add(gtsam::PriorFactor<gtsam::Pose3>(X(key), prevPose_, correctRotNoise));
    }

    if (lidarAssociateMode != 0)
    {
        if (!featureInfo.isKeyFrame || !GNSSEnable)
        {
            graphFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(lastKeyIndex),
                X(key),
                delta_Pose1,
                odometryNoise));
            //        graphFactors.add(pose_factor);
        }
        else
        {
            if (coupleMode == 1)
            {
                addLidarFactor();
            }
            else if (coupleMode == 0)
            {
                graphFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(lastKeyIndex),
                    X(key),
                    delta_Pose1,
                    odometryNoise));
            }
        }
    }
    else
    {
        graphFactors.add(pose_factor);
    }

    ROS_INFO("LASTKEYINDEX: %d, KEY: %d", lastKeyIndex, key);

    if (useNHC)
    {
        gtsam::Vector3 angular_velocity = Rot3::Logmap(imuIntegratorOpt_->deltaRij()) / imuIntegratorOpt_->deltaTij();
        if (angular_velocity.norm() * 180 / M_PI < thres_angular_velocity)
        {
            // 定义噪声模型（设置x/z分量的约束强度）
            auto noiseModel = noiseModel::Diagonal::Sigmas(Vector2(NHCnoise[0], NHCnoise[1])); // 单位：m/s
            noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noiseModel);
            // 添加非完整性约束因子
            graphFactors.add(NhcFactor(X(key), V(key), huber));
            ROS_INFO("add NHC factor");
        }
    }

    //optimize for odom
    // {
    //     // add imu factor to graph
    //     const gtsam::PreintegratedImuMeasurements
    //         & preint_imu_odom = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOdo_);
    //     gtsam::ImuFactor odom_imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu_odom);
    //     odomGraphFactors.add(odom_imu_factor);
    //     // add imu bias between factor
    //     odomGraphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1),
    //         B(key),
    //         gtsam::imuBias::ConstantBias(),
    //         gtsam::noiseModel::Diagonal::Sigmas(
    //             sqrt(imuIntegratorOdo_->deltaTij()) * noiseModelBetweenBias)));
    //     odomGraphFactors.add(pose_factor);
    //     gtsam::NavState odomState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
    //     odomGraphValues.insert(X(key), odomState.pose());
    //     odomGraphValues.insert(V(key), odomState.v());
    //     odomGraphValues.insert(B(key), prevBiasOdom);
    //     // optimize
    //     updateAndMarginalize(odomGraphFactors, odomGraphValues, {}, odomOptimizer);
    //     odomOptimizer.update();
    //     odomOptimizer.update();
    //     odomGraphFactors.resize(0);
    //     odomGraphValues.clear();
    //     gtsam::Values odom_result = odomOptimizer.calculateEstimate();
    //     prevStateOdom = gtsam::NavState(odom_result.at<gtsam::Pose3>(X(key)), odom_result.at<gtsam::Vector3>(V(key)));
    //     prevBiasOdom = odom_result.at<gtsam::imuBias::ConstantBias>(B(key));

    //     // if (prevBiasOdom.accelerometer().norm() > 1.0)
    //     // {
    //     //     gtsam::imuBias::ConstantBias correct_Bias = gtsam::imuBias::ConstantBias((gtsam::Vector(6)
    //     //         << 0.13,
    //     //         0.06, 0.03, -7e-5, 4e-5, -2e-4)
    //     //         .finished());
    //     //     gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(key), correct_Bias, priorBiasNoise);
    //     //     odomGraphFactors.add(priorBias);
    //     //     // optimize once
    //     //     odomOptimizer.update(odomGraphFactors, {});
    //     //     odomOptimizer.update();
    //     //     odomOptimizer.update();
    //     //     odomGraphFactors.resize(0);
    //     //     gtsam::Values odom_result = odomOptimizer.calculateEstimate();
    //     //     prevStateOdom = gtsam::NavState(odom_result.at<gtsam::Pose3>(X(key)), odom_result.at<gtsam::Vector3>(V(key)));
    //     //     prevBiasOdom = odom_result.at<gtsam::imuBias::ConstantBias>(B(key));
    //     // }
    //     if (key >= 100)
    //     {
    //         KeySet marginalKeys;
    //         marginalKeys.insert(X(key - 100));
    //         marginalKeys.insert(V(key - 100));
    //         marginalKeys.insert(B(key - 100));
    //         updateAndMarginalize({}, {}, marginalKeys, odomOptimizer);
    //     }
    // }

    if (GNSSEnable)
    {
        lastKeyIndex = key;
        lastKeyPose = curPose;
        lastgpstime = currentCorrectionTime;
    }
    else if (fabs(currentCorrectionTime - round(currentCorrectionTime)) < 0.005)
    {
        gpsKeyQueue.push_back(key);
        lastKeyIndex = key;
        lastKeyPose = curPose;
    }
    if (lidarAssociateMode == 1)
    {
        lastKeyIndex = key;
        lastKeyPose = curPose;
    }

    prePose = curPose;

    // optimize
    updateAndMarginalize(graphFactors, graphValues, {}, optimizer);

    for (int i = 0; i < 2; i++)
    {
        updateAndMarginalize({}, {}, {}, optimizer);
    }

    if (!isRelative)
    {
        optimizer.update();
        optimizer.update();
        optimizer.update();
    }
    graphFactors.resize(0);
    graphValues.clear();
    // Overwrite the beginning of the preintegration for the next step.
    // do ambiguty resolve
    gtsam::Values result = optimizer.calculateEstimate();
    static int state = 6;
    static Pose3 sol_pos;
    sol_pos = result.at<gtsam::Pose3>(X(key));
    posCovariance = optimizer.marginalCovariance(X(key));
    if (useGPS && useObs && useCarrier && GNSSEnable)
    {
        state = container.ambiguityResolve(optimizer, result, key, posCovariance, sol_pos);
    }
    if (debugImu)
    {
        ROS_INFO("IMU POSE: %lf %lf %lf", sol_pos.x(), sol_pos.y(), sol_pos.z());
    }
    prevPose_ = result.at<gtsam::Pose3>(X(key));
    prevVel_ = result.at<gtsam::Vector3>(V(key));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
    if (estimateExtGPS)
    {
        extGPS = result.at<gtsam::Vector3>(T(key));
        if (debugGps)
        {
            ROS_INFO("EXT IMU 2 GPS %lf %lf %lf", extGPS.x(), extGPS.y(), extGPS.z());
        }
    }
    if (gpsKeyQueue.size() > 2)
    {
        KeySet marginalKeys;
        int startKey = gpsKeyQueue.front();
        gpsKeyQueue.pop_front();
        int endKey = gpsKeyQueue.front();
        for (int i = startKey; i < endKey; i++)
        {
            marginalKeys.insert(X(i));
            marginalKeys.insert(V(i));
            marginalKeys.insert(B(i));
            if (optimizer.valueExists(N(i)))
            {
                marginalKeys.insert(N(i));
            }
            if (optimizer.valueExists(T(i)))
            {
                marginalKeys.insert(T(i));
            }
        }
        updateAndMarginalize({}, {}, marginalKeys, optimizer);
    }
    if (!optimizer.valueExists(N(lastGNSSepoch)))
    {
        container.reset_last_ar_index();
        ROS_INFO("%d does not exist", lastGNSSepoch);
    }
    average_time = (average_time * (key - 1) + t_opt.toc()) / (key);
    ROS_INFO("average time %.3lf", average_time);
    ROS_INFO("optize and margin time %.3lf", t_opt.toc());

    GNSSEnable = (fabs(currentCorrectionTime - round(currentCorrectionTime)) < 0.005);
    // write gps result
    if (GNSSEnable)
    {
        lla_origin = container.getOrigin();
        gpsPose = prevPose_.compose(gps2imu); // prevPose_
        Pose3 tmppose = prevState_.pose().compose(gps2imu);
        ROS_INFO("GNSS POSE: %lf %lf %lf", gpsPose.translation().x(), gpsPose.translation().y(), gpsPose.translation().z());
        Vector3 ecef = GNSS_Tools::enu2ecef(lla_origin, tmppose.translation()); // gtools.ENU2ECEF(gpsPose.translation());
        gtime_t curgtime;
        curgtime.time = round(currentCorrectionTime) + 18;
        curgtime.sec = currentCorrectionTime - round(currentCorrectionTime);
        volatile double weeksec = time2gpst(curgtime, NULL);
        int laserCloudCornerNum, laserCloudSurfNum;
        laserCloudCornerNum = featureMsg->laserCloudCornerMatch.data.size() / 3;
        laserCloudSurfNum = featureMsg->laserCloudSurfMatch.data.size() / 2;
        writeGPSfile2(curgtime, ecef, state, fp);
        fprintf(fp, "%d %d\n", laserCloudCornerNum, laserCloudSurfNum);
        fflush(fp);
    }
    lidarPose = prevPose_.compose(imu2Lidar);
    // publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp.fromSec(currentCorrectionTime);
    odometry.header.frame_id = odometryFrame;
    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
    odometry.twist.twist.angular.x = posCovariance(0, 0);
    odometry.twist.twist.angular.y = posCovariance(1, 1);
    odometry.twist.twist.angular.z = posCovariance(2, 2);
    odometry.twist.twist.linear.x = posCovariance(3, 3);
    odometry.twist.twist.linear.y = posCovariance(4, 4);
    odometry.twist.twist.linear.z = posCovariance(5, 5);
    odometry.twist.covariance.elems[0] = state;
    odometry.pose.covariance.elems[0] = featureInfo.isKeyFrame;
    if (GNSSEnable || featureInfo.isKeyFrame)
    {
        pubLaserOdometry.publish(odometry);
    }

    // Reset the optimization preintegration object.
    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

    // 2. after optiization, re-propagate imu odometry preintegration
    prevStateOdom = prevState_;
    prevBiasOdom = prevBias_;
    // first pop imu message older than current correction data
    double lastImuQT = -1;
    while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
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
    doneFirstOpt = true;
    ++key;
}

bool IMUPreintegration::failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
{
    Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
    if (vel.norm() > 30)
    {
        ROS_WARN("Large velocity, reset IMU-preintegration!");
        return true;
    }

    Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
    Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
    if (ba.norm() > 1.0 || bg.norm() > 1.0)
    {
        ROS_WARN("Large bias, reset IMU-preintegration!");
        return true;
    }

    return false;
}

void IMUPreintegration::imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
{
    std::lock_guard<std::mutex> lock(mtx);

    sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

    imuQueOpt.push_back(thisImu);
    imuQueImu.push_back(thisImu);

    if (doneFirstOpt == false)
        return;

    double imuTime = ROS_TIME(&thisImu);
    double dt = (lastImuT_imu < 0) ? (1.0 / imuFrequence) : (imuTime - lastImuT_imu);
    lastImuT_imu = imuTime;

    // integrate this single imu message
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x,
        thisImu.linear_acceleration.y,
        thisImu.linear_acceleration.z),
        gtsam::Vector3(thisImu.angular_velocity.x,
            thisImu.angular_velocity.y,
            thisImu.angular_velocity.z),
        dt);

    // predict odometry
    gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

    // publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = thisImu.header.stamp;
    odometry.header.frame_id = odometryFrame;
    odometry.child_frame_id = "odom_imu";

    // transform imu pose to ldiar1
    gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
    gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);
    static int count = 0;
    if (false)
    {
        gpsPose = imuPose.compose(gps2imu);
        Vector3 ecef = GNSS_Tools::enu2ecef(lla_origin, gpsPose.translation()); // gtools.ENU2ECEF(gpsPose.translation());
        gtime_t curgtime;
        curgtime.time = floor(imuTime) + 18;
        curgtime.sec = imuTime - floor(imuTime);
        writeGPSfile(curgtime, ecef, 6, fp_debug);
        count++;
        if (count % 100 == 0)
        {
            fflush(fp_debug);
            count = 0;
        }
    }
    odometry.pose.pose.position.x = lidarPose.translation().x();
    odometry.pose.pose.position.y = lidarPose.translation().y();
    odometry.pose.pose.position.z = lidarPose.translation().z();
    odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

    if (debugImu)
    {
        ROS_INFO("initial pose: %f, %f, %f, %f, %f, %f", odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z,
            lidarPose.rotation().roll(), lidarPose.rotation().pitch(), lidarPose.rotation().yaw());

    }
    odometry.twist.twist.linear.x = currentState.velocity().x();
    odometry.twist.twist.linear.y = currentState.velocity().y();
    odometry.twist.twist.linear.z = currentState.velocity().z();
    odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    pubImuOdometry.publish(odometry);
}
