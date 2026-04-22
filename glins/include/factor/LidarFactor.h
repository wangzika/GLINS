//
// Created by wangchuji on 2023/9/7.
//
#pragma once
#include <utility>

#include "gtsam/global_includes.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/geometry/Pose3.h"

using namespace gtsam;
#ifndef GLINS_LIDARFACTOR_H
#define GLINS_LIDARFACTOR_H
class LidarEdgeFactor : public NoiseModelFactor1<Pose3>
{
private:
    Point3 curr_point;
    Point3 last_point_A;
    Point3 last_point_B;
    Vector3 ext_lb;
    gtsam::Pose3 imu2lidar;

public:
    LidarEdgeFactor(Key key, Point3 cp, Point3 lpA, Point3 lpB, Vector3 lb, const SharedNoiseModel& model) :
        NoiseModelFactor1<Pose3>(model, key)
    {
        curr_point = cp;
        last_point_A = lpA;
        last_point_B = lpB;
        ext_lb = lb;
        imu2lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(ext_lb.x(), ext_lb.y(), ext_lb.z()));
    }

    Vector evaluateError(const Pose3& RxPose,
        boost::optional<gtsam::Matrix&>H1 = boost::none) const override
    {
        //        RxPose.compose(imu2lidar);
        gtsam::Matrix H_pose(3, 6);
        gtsam::Matrix H_lb(6, 6);

        Pose3 lidar_pose = RxPose.compose(imu2lidar, H_lb);

        Point3 lp = lidar_pose.transformFrom(curr_point, H_pose);//.rotation(H_pose) * (curr_point+ext_lb) + RxPose.translation(H_pose);

        Vector3 nu = (lp - last_point_A).cross(lp - last_point_B);
        Vector3 de = last_point_A - last_point_B;
        Matrix3 de_hat, cp_hat;
        double norm_de = de.norm();
        double norm_nu = nu.norm();
        de.normalize();
        nu.normalize();
        de_hat << 0, -de(2), de(1),
            de(2), 0, -de(0),
            -de(1), de(0), 0;
        cp_hat << 0, -curr_point(2), curr_point(1),
            curr_point(2), 0, -curr_point(0),
            -curr_point(1), curr_point(0), 0;
        double s = 1 - 0.9 * fabs(norm_nu / norm_de);
        //        cout << H_pose << endl;
        if (H1)
        {
            H1->resize(1, 6);
            *H1 = -s * nu.transpose() * de_hat * H_pose * H_lb;
            //            H1->block(0,0,3,3) = -de_hat * H_pose.block(0,0,3,3) * cp_hat;
            //            H1->block(0,3,3,3) = de_hat * H_pose.block(0,3,3,3); //H_pose.block(0,3,3,3)
        }

        return Vector1(s * norm_nu / norm_de);
    }

};

class LidarEdgeFactorRelative : public NoiseModelFactor2<Pose3, Pose3>
{
private:
    Point3 curr_point;
    Point3 last_point_A;
    Point3 last_point_B;
    gtsam::Pose3 imu2lidar;

public:
    LidarEdgeFactorRelative(Key key1, Key key2, Point3 cp, Point3 lpA, Point3 lpB, Vector3 lb, const SharedNoiseModel& model) :
        NoiseModelFactor2<Pose3, Pose3>(model, key1, key2)
    {
        curr_point = cp;
        last_point_A = lpA;
        last_point_B = lpB;
        imu2lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(lb.x(), lb.y(), lb.z()));
    }

    Vector evaluateError(const Pose3& RxPose, const Pose3& curPose,
        boost::optional<gtsam::Matrix&>H1 = boost::none,
        boost::optional<gtsam::Matrix&>H2 = boost::none) const
    {
        gtsam::Matrix H_cur, H_pre;
        gtsam::Matrix H_pose(3, 6);
        gtsam::Matrix H_lb_last(6, 6);
        gtsam::Matrix H_lb_curr(6, 6);
        gtsam::Pose3 last_lidarpose = RxPose.compose(imu2lidar, H_lb_last);
        gtsam::Pose3 cur_lidarpose = curPose.compose(imu2lidar, H_lb_curr);
        Matrix3 de_hat;
        double norm_de, norm_nu;
        Pose3 betweenPose = traits<Pose3>::Between(last_lidarpose, cur_lidarpose, H_pre, H_cur);//RxPose.between(curPose,H_pre,H_cur);
        Point3 lp = betweenPose.transformFrom((curr_point), H_pose);//.rotation(H_pose) * (curr_point+ext_lb) + RxPose.translation(H_pose);

        Vector3 nu = (lp - last_point_A).cross(lp - last_point_B);
        Vector3 de = last_point_A - last_point_B;

        norm_de = de.norm(); norm_nu = nu.norm();
        de.normalize();
        de_hat << skewSymmetric(-de.x(), -de.y(), -de.z());
        double s = 1 - 0.9 * fabs(norm_nu / norm_de);
        if (H1)
        {
            H1->resize(3, 6);
            *H1 = s * de_hat * H_pose * H_pre * H_lb_last;
        }
        if (H2)
        {
            H2->resize(3, 6);
            *H2 = s * de_hat * H_pose * H_cur * H_lb_curr;
        }
        //        cout << s * de_hat * H_pose << endl << endl;
        //
        //        cout << s * nu / norm_de << endl << endl;

        return (s * nu / norm_de);
    }
};

class LidarEdgeNormFactorRelative : public NoiseModelFactor2<Pose3, Pose3>
{
private:
    Point3 curr_point;
    Point3 last_point_A;
    Point3 last_point_B;
    gtsam::Pose3 imu2lidar;

public:
    LidarEdgeNormFactorRelative(Key key1, Key key2, Point3 cp, Point3 lpA, Point3 lpB, Vector3 lb, const SharedNoiseModel& model) :
        NoiseModelFactor2<Pose3, Pose3>(model, key1, key2)
    {
        curr_point = cp;
        last_point_A = lpA;
        last_point_B = lpB;
        imu2lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(lb.x(), lb.y(), lb.z()));
    }

    Vector evaluateError(const Pose3& RxPose, const Pose3& curPose,
        boost::optional<gtsam::Matrix&>H1 = boost::none,
        boost::optional<gtsam::Matrix&>H2 = boost::none) const
    {
        gtsam::Matrix H_cur, H_pre;
        gtsam::Matrix H_pose(3, 6);
        gtsam::Matrix H_lb_last(6, 6);
        gtsam::Matrix H_lb_curr(6, 6);
        gtsam::Pose3 last_lidarpose = RxPose.compose(imu2lidar, H_lb_last);
        gtsam::Pose3 cur_lidarpose = curPose.compose(imu2lidar, H_lb_curr);
        Matrix3 de_hat;
        double norm_de, norm_nu;
        Pose3 betweenPose = traits<Pose3>::Between(last_lidarpose, cur_lidarpose, H_pre, H_cur);//RxPose.between(curPose,H_pre,H_cur);
        Point3 lp = betweenPose.transformFrom((curr_point), H_pose);//.rotation(H_pose) * (curr_point+ext_lb) + RxPose.translation(H_pose);

        Vector3 nu = (lp - last_point_A).cross(lp - last_point_B);
        Vector3 de = last_point_A - last_point_B;

        norm_de = de.norm(); norm_nu = nu.norm();
        de.normalize();
        nu.normalize();
        de_hat << skewSymmetric(-de.x(), -de.y(), -de.z());
        double s = 1 - 0.9 * fabs(norm_nu / norm_de);
        if (H1)
        {
            H1->resize(1, 6);
            *H1 = s * nu.transpose() * de_hat * H_pose * H_pre * H_lb_last;
        }
        if (H2)
        {
            H2->resize(1, 6);
            *H2 = s * nu.transpose() * de_hat * H_pose * H_cur * H_lb_curr;
        }
        //        cout << s * de_hat * H_pose << endl << endl;
        //
        // cout << s * norm_nu / norm_de << endl << endl;

        return Vector1(s * norm_nu / norm_de);
    }
};


class LidarPlaneFactor : public NoiseModelFactor1<Pose3>
{
private:
    Point3 curr_point;
    Point3 last_point_A;
    Point3 last_point_B;
    Point3 last_point_C;
    Eigen::Vector3d plane_norm;
    Vector3 ext_lb;

public:
    LidarPlaneFactor(Key key, const Point3& cp, const Point3& lpA, const Point3& lpB, const Point3& lpC, const SharedNoiseModel& model) :
        NoiseModelFactor1<Pose3>(model, key), curr_point(cp), last_point_A(lpA), last_point_B(lpB), last_point_C(lpC)
    {
        plane_norm = (lpA - lpB).cross(lpA - lpC);
        plane_norm.normalize();
    }

    Vector evaluateError(const Pose3& RxPose,
        boost::optional<gtsam::Matrix&>H1 = boost::none) const
    {
        gtsam::Matrix H_pose(3, 6);

        Point3 lp = RxPose.rotation(H_pose) * curr_point + RxPose.translation(H_pose);

        double dist = (lp - last_point_A).dot(plane_norm);

        Matrix3 cp_hat;
        cp_hat << 0, -curr_point(2), curr_point(1),
            curr_point(2), 0, -curr_point(0),
            -curr_point(1), curr_point(0), 0;
        if (H1)
        {
            H1->resize(1, 6);
            H1->block(0, 0, 1, 3) = -plane_norm.transpose() * H_pose.block(0, 0, 3, 3) * cp_hat;
            H1->block(0, 3, 1, 3) = plane_norm.transpose() * H_pose.block(0, 3, 3, 3);
        }

        return Vector1(dist);
    }

};

class LidarPlaneNormFactor : public NoiseModelFactor1<Pose3>
{
private:
    Point3 curr_point;
    Vector3 plane_norm;
    double d;
    Vector3 ext_lb;
    gtsam::Pose3 imu2lidar;

public:
    LidarPlaneNormFactor(Key key, Point3 cp, Vector3 pn, const double d_s, Vector3 lb, const SharedNoiseModel& model) :
        NoiseModelFactor1<Pose3>(model, key)
    {
        curr_point = cp;
        plane_norm = pn;
        d = d_s;
        ext_lb = lb;
        imu2lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(ext_lb.x(), ext_lb.y(), ext_lb.z()));
    }

    Vector evaluateError(const Pose3& pose,
        boost::optional<gtsam::Matrix&>H1 = boost::none) const
    {
        gtsam::Matrix H_pose(3, 6);
        gtsam::Matrix H_lb(6, 6);

        Pose3 lidar_pose = pose.compose(imu2lidar, H_lb);

        Point3 lp = lidar_pose.transformFrom(curr_point, H_pose);//pose.rotation(H_pose) * (curr_point+ext_lb) + pose.translation(H_pose);

        //        ROS_INFO("lidar pose: %lf %lf %lf %lf %lf %lf", lidar_pose.translation().x(), lidar_pose.translation().y(), lidar_pose.translation().z(), lidar_pose.rotation().roll(), lidar_pose.rotation().pitch(), lidar_pose.rotation().yaw());
        //        ROS_INFO("curr point: %lf %lf %lf", curr_point.x(), curr_point.y(), curr_point.z());
        //        ROS_INFO("lp: %lf %lf %lf", lp.x(), lp.y(), lp.z());
        //        ROS_INFO("plane norm: %lf %lf %lf %lf", plane_norm.x(), plane_norm.y(), plane_norm.z(), d);
        gtsam::Matrix H_norm(1, 3);
        double dist = dot(lp, plane_norm, H_norm) + d;
        float s = 1 - 0.9 * fabs(dist) / sqrt(curr_point.norm());
        if (H1)
        {
            H1->resize(1, 6);
            *H1 = s * H_norm * H_pose * H_lb;
        }

        return Vector1(s * dist);
    }

};

class LidarPlaneNormFactorRelative : public NoiseModelFactor2<Pose3, Pose3>
{
private:
    Point3 curr_point;
    Vector3 plane_norm;
    double d;
    gtsam::Pose3 imu2lidar;
public:
    LidarPlaneNormFactorRelative(Key key1, Key key2, Point3 cp, Vector3 pn, const double d_s, Vector3 lb, const SharedNoiseModel& model) :
        NoiseModelFactor2<Pose3, Pose3>(model, key1, key2)
    {
        curr_point = cp;
        plane_norm = pn;
        d = d_s;
        imu2lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(lb.x(), lb.y(), lb.z()));
    }

    Vector evaluateError(const Pose3& RxPose, const Pose3& curPose,
        boost::optional<gtsam::Matrix&>H1 = boost::none,
        boost::optional<gtsam::Matrix&>H2 = boost::none) const
    {
        gtsam::Matrix H_cur, H_pre;
        gtsam::Matrix H_pose(3, 6);
        gtsam::Matrix H_lb_last(6, 6);
        gtsam::Matrix H_lb_curr(6, 6);
        gtsam::Pose3 last_lidarpose = RxPose.compose(imu2lidar, H_lb_last);
        gtsam::Pose3 cur_lidarpose = curPose.compose(imu2lidar, H_lb_curr);

        Pose3 betweenPose = last_lidarpose.between(cur_lidarpose, H_pre, H_cur);
        Point3 lp = betweenPose.transformFrom((curr_point), H_pose);

        gtsam::Matrix H_norm(1, 3);
        double dist = dot(lp, plane_norm, H_norm) + d;
        float s = 1 - 0.9 * fabs(dist) / sqrt(curr_point.norm());
        //    ROS_INFO("last lidar pose: %lf %lf %lf %lf %lf %lf", last_lidarpose.translation().x(), last_lidarpose.translation().y(), last_lidarpose.translation().z(), last_lidarpose.rotation().roll(), last_lidarpose.rotation().pitch(), last_lidarpose.rotation().yaw());
        //    ROS_INFO("current lidar pose: %lf %lf %lf %lf %lf %lf", cur_lidarpose.translation().x(), cur_lidarpose.translation().y(), cur_lidarpose.translation().z(), cur_lidarpose.rotation().roll(), cur_lidarpose.rotation().pitch(), cur_lidarpose.rotation().yaw());
        //    ROS_INFO("curr point: %lf %lf %lf", curr_point.x(), curr_point.y(), curr_point.z());
        //    ROS_INFO("lp: %lf %lf %lf", lp.x(), lp.y(), lp.z());
        //    ROS_INFO("plane norm: %lf %lf %lf %lf", plane_norm.x(), plane_norm.y(), plane_norm.z(), d);
        if (H1)
        {
            H1->resize(1, 6);
            *H1 = s * H_norm * H_pose * H_pre * H_lb_last;
        }
        if (H2)
        {
            H2->resize(1, 6);
            *H2 = s * H_norm * H_pose * H_cur * H_lb_curr;
        }
        //        cout << s * H_norm * H_pose << endl << endl;
        //
        // cout << s * dist << endl << endl;

        return Vector1(s * dist);
    }

};

#endif //GLINS_LIDARFACTOR_H
