#ifndef NONHOLONOMIC_CONSTRAINT_FACTOR_H
#define NONHOLONOMIC_CONSTRAINT_FACTOR_H

#include <gtsam/global_includes.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose2.h>
using namespace gtsam;


class NhcFactor : public NoiseModelFactor2<Pose3, Vector3>
{
public:
    NhcFactor(Key poseKey, Key velocityKey, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Vector3>(noiseModel, poseKey, velocityKey)
    {

    }

    Vector evaluateError(const Pose3& pose, const Vector3& velocity,
        boost::optional<gtsam::Matrix&>H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        // 获取旋转矩阵（ENU到载体坐标系的旋转）
        const Rot3 R = pose.rotation();

        // 将全局速度转换到载体坐标系
        const Vector3 body_vel = R.transpose() * velocity;  // R是ENU->Body的旋转

        // 非完整性约束：横向和垂直速度分量为零（仅保留x方向）
        // 残差计算：x和z分量应该为0
        Vector2 residual;
        residual << body_vel.x(), body_vel.z();

        // 雅可比矩阵计算（如果需要）
        if (H1 || H2)
        {
            // 对姿态的导数
            if (H1)
            {
                Matrix26 H_pose = gtsam::Matrix::Zero(2, 6);
                // 计算速度在body系下的导数
                gtsam::Matrix H_rot = R.matrix().transpose() * skewSymmetric(velocity);
                H_pose.block<2, 3>(0, 0) << H_rot.row(0), H_rot.row(2);

                *H1 = H_pose;
            }

            // 对速度的导数
            if (H2)
            {
                Matrix23 H_vel;
                H_vel << R.matrix().transpose().row(0),  // dy/dv
                    R.matrix().transpose().row(2);  // dz/dv
                *H2 = H_vel;
            }
        }

        return residual;
    }
};

class NhcFactor3Axis : public NoiseModelFactor2<Pose3, Vector3>
{
    double vel_Y;
public:
    NhcFactor3Axis(Key poseKey, Key velocityKey, double vel_y, const SharedNoiseModel& noiseModel)
        : vel_Y(vel_y), NoiseModelFactor2<Pose3, Vector3>(noiseModel, poseKey, velocityKey)
    {

    }

    Vector evaluateError(const Pose3& pose, const Vector3& velocity,
        boost::optional<gtsam::Matrix&>H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        // 获取旋转矩阵（ENU到载体坐标系的旋转）
        const Rot3 R = pose.rotation();

        // 将全局速度转换到载体坐标系
        const Vector3 body_vel = R.transpose() * velocity;  // R是ENU->Body的旋转

        // 非完整性约束：横向和垂直速度分量为零（仅保留x方向）
        // 残差计算：x和z分量应该为0
        Vector3 residual;
        residual << body_vel.x(), body_vel.y() - vel_Y, body_vel.z();

        // 雅可比矩阵计算（如果需要）
        if (H1 || H2)
        {
            // 对姿态的导数
            if (H1)
            {
                Matrix36 H_pose = gtsam::Matrix::Zero(3, 6);
                // 计算速度在body系下的导数
                gtsam::Matrix H_rot = R.matrix().transpose() * skewSymmetric(velocity);
                H_pose.block<3, 3>(0, 0) << H_rot;

                *H1 = H_pose;
            }

            // 对速度的导数
            if (H2)
            {
                Matrix33 H_vel;
                H_vel = R.matrix().transpose();
                *H2 = H_vel;
            }
        }

        return residual;
    }
};

class NhcFactorX : public NoiseModelFactor2<Pose3, Vector3>
{
public:
    NhcFactorX(Key poseKey, Key velocityKey, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Vector3>(noiseModel, poseKey, velocityKey)
    {

    }

    Vector evaluateError(const Pose3& pose, const Vector3& velocity,
        boost::optional<gtsam::Matrix&>H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        // 获取旋转矩阵（ENU到载体坐标系的旋转）
        const Rot3 R = pose.rotation();

        // 将全局速度转换到载体坐标系
        const Vector3 body_vel = R.transpose() * velocity;  // R是ENU->Body的旋转

        // 非完整性约束：横向和垂直速度分量为零（仅保留x方向）
        // 残差计算：x和z分量应该为0
        Vector1 residual;
        residual << body_vel.x();

        // 雅可比矩阵计算（如果需要）
        if (H1 || H2)
        {
            // 对姿态的导数
            if (H1)
            {
                Matrix16 H_pose = gtsam::Matrix::Zero(1, 6);
                // 计算速度在body系下的导数
                gtsam::Matrix H_rot = R.matrix().transpose() * skewSymmetric(velocity);
                H_pose.block<1, 3>(0, 0) << H_rot.row(0);

                *H1 = H_pose;
            }

            // 对速度的导数
            if (H2)
            {
                Matrix13 H_vel;
                H_vel << R.matrix().transpose().row(0);  // dy/dv
                *H2 = H_vel;
            }
        }

        return residual;
    }
};
class NhcFactorY : public NoiseModelFactor2<Pose3, Vector3>
{
    double vel_Y;
public:
    NhcFactorY(Key poseKey, Key velocityKey, double vel_y, const SharedNoiseModel& noiseModel)
        : vel_Y(vel_y), NoiseModelFactor2<Pose3, Vector3>(noiseModel, poseKey, velocityKey)
    {

    }

    Vector evaluateError(const Pose3& pose, const Vector3& velocity,
        boost::optional<gtsam::Matrix&>H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        // 获取旋转矩阵（ENU到载体坐标系的旋转）
        const Rot3 R = pose.rotation();

        // 将全局速度转换到载体坐标系
        const Vector3 body_vel = R.transpose() * velocity;  // R是ENU->Body的旋转

        // 非完整性约束：横向和垂直速度分量为零（仅保留x方向）
        // 残差计算：x和z分量应该为0
        Vector1 residual;
        residual << body_vel.y() - vel_Y;

        // 雅可比矩阵计算（如果需要）
        if (H1 || H2)
        {
            // 对姿态的导数
            if (H1)
            {
                Matrix16 H_pose = gtsam::Matrix::Zero(1, 6);
                // 计算速度在body系下的导数
                gtsam::Matrix H_rot = R.matrix().transpose() * skewSymmetric(velocity);
                H_pose.block<1, 3>(0, 0) << H_rot.row(1);
                *H1 = H_pose;
            }

            // 对速度的导数
            if (H2)
            {
                Matrix13 H_vel;
                H_vel << R.matrix().transpose().row(1);  // dy/dv
                *H2 = H_vel;
            }
        }

        return residual;
    }
};

class NhcFactorZ : public NoiseModelFactor2<Pose3, Vector3>
{
public:
    NhcFactorZ(Key poseKey, Key velocityKey, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Vector3>(noiseModel, poseKey, velocityKey)
    {

    }

    Vector evaluateError(const Pose3& pose, const Vector3& velocity,
        boost::optional<gtsam::Matrix&>H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const override
    {
        // 获取旋转矩阵（ENU到载体坐标系的旋转）
        const Rot3 R = pose.rotation();

        // 将全局速度转换到载体坐标系
        const Vector3 body_vel = R.transpose() * velocity;  // R是ENU->Body的旋转

        // 非完整性约束：横向和垂直速度分量为零（仅保留x方向）
        // 残差计算：x和z分量应该为0
        Vector1 residual;
        residual << body_vel.z();

        // 雅可比矩阵计算（如果需要）
        if (H1 || H2)
        {
            // 对姿态的导数
            if (H1)
            {
                Matrix16 H_pose = gtsam::Matrix::Zero(1, 6);
                // 计算速度在body系下的导数
                gtsam::Matrix H_rot = R.matrix().transpose() * skewSymmetric(velocity);
                H_pose.block<1, 3>(0, 0) << H_rot.row(2);
                *H1 = H_pose;
            }

            // 对速度的导数
            if (H2)
            {
                Matrix13 H_vel;
                H_vel << R.matrix().transpose().row(2);  // dz/dv
                *H2 = H_vel;
            }
        }

        return residual;
    }
};


class ZAxisConstraint : public NoiseModelFactor1<Pose3>
{
    double z_axis_constraint;

public:
    ZAxisConstraint(Key key, double z_constraint, const SharedNoiseModel& model)
        : z_axis_constraint(z_constraint), NoiseModelFactor1<Pose3>(model, key) {}

    Vector evaluateError(const Pose3& pose, boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        if (H)
        {
            // 雅可比矩阵 - 只对Z轴平移和旋转敏感
            *H = gtsam::Matrix::Zero(1, 6);
            (*H)(0, 5) = 1.0; // 对Z轴平移的导数
            // 如果需要也可以添加对旋转的约束
        }
        return Vector1(pose.z() - z_axis_constraint); // 返回Z轴高度误差
    }
};

#endif // NONHOLONOMIC_CONSTRAINT_FACTOR_H