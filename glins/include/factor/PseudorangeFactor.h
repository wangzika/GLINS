//
// Created by wangchuji on 2023/2/14.
//
#pragma once
#ifndef LIO_SAM_PSEUDORANGEFACTOR_H
#define LIO_SAM_PSEUDORANGEFACTOR_H
#include "gtsam/base/Vector.h"
#include "gtsam/base/Matrix.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/ExpressionFactor.h"
#include "gtsam/geometry/Point3.h"
#include "gnss_tools.h"
#include <queue>
#include <map>
#include <gtsam/geometry/Pose3.h>
using namespace gtsam;
#if 0
class DDPseudorangeFactor : public NoiseModelFactor1<Point3>
{
private:

    DDMeasurement ddMeasurement;
    Point3 base_pos;
    typedef NoiseModelFactor1<Point3> Base;
    typedef DDPseudorangeFactor This;

public:
    DDPseudorangeFactor() = default;

    DDPseudorangeFactor(Key key1, DDMeasurement measurement, Point3 base, const SharedNoiseModel& model) :Base(model, key1), ddMeasurement(measurement), base_pos(base) {}

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new DDPseudorangeFactor(*this)));
    }

    Vector evaluateError(const Point3& RxPos,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {
        double master_Geodist_rover, master_Geodist_base;
        double i_Geodist_rover, i_Geodist_base;

        Vector3 rover_satpos_m(ddMeasurement.u_master_SV.sat_pos_x, ddMeasurement.u_master_SV.sat_pos_y, ddMeasurement.u_master_SV.sat_pos_z);
        Vector3 rover_satpos_i(ddMeasurement.u_iSV.sat_pos_x, ddMeasurement.u_iSV.sat_pos_y, ddMeasurement.u_iSV.sat_pos_z);
        Vector3 base_satpos_m(ddMeasurement.r_master_SV.sat_pos_x, ddMeasurement.r_master_SV.sat_pos_y, ddMeasurement.r_master_SV.sat_pos_z);
        Vector3 base_satpos_i(ddMeasurement.r_iSV.sat_pos_x, ddMeasurement.r_iSV.sat_pos_y, ddMeasurement.r_iSV.sat_pos_z);

        Vector3 delta_rover_m = rover_satpos_m - RxPos;
        Vector3 delta_rover_i = rover_satpos_i - RxPos;
        Vector3 delta_base_m = base_satpos_m - base_pos;
        Vector3 delta_base_i = base_satpos_i - base_pos;

        master_Geodist_rover = delta_rover_m.norm();
        master_Geodist_base = delta_base_m.norm();
        i_Geodist_rover = delta_rover_i.norm();
        i_Geodist_base = delta_base_i.norm();

        double delta_dist_master_rover = ddMeasurement.u_master_SV.pseudorange - master_Geodist_rover;
        double delta_dist_master_base = ddMeasurement.r_master_SV.pseudorange - master_Geodist_base;
        double delta_dist_i_rover = ddMeasurement.u_iSV.pseudorange - i_Geodist_rover;
        double delta_dist_i_base = ddMeasurement.r_iSV.pseudorange - i_Geodist_base;
        Vector3 h = (-delta_rover_i / i_Geodist_rover + delta_rover_m / master_Geodist_rover);
        *H1 = h.transpose();
        Vector1 res;
        res << ((delta_dist_master_rover - delta_dist_master_base) - (delta_dist_i_rover - delta_dist_i_base));

        return res;
    }
};

class DDPseudorangeFactor_sys : public NoiseModelFactor1<Point3>
{
private:
    //    std::map<int,vector<DDMeasurement>> measures;
    vector<DDMeasurement> measures;
    int meas_size;
    Point3 base_pos;
    typedef NoiseModelFactor1<Point3> Base;
    typedef DDPseudorangeFactor_sys This;
    Key key_;

public:
    DDPseudorangeFactor_sys() = default;

    //    DDPseudorangeFactor_sys(Key key,map<int,vector<DDMeasurement>> measure_,Point3 base, const SharedNoiseModel& model):
    //    Base(model,key),base_pos(base),measures(measure_){}

    DDPseudorangeFactor_sys(Key key, vector<DDMeasurement> measure_, Point3 base, int size, const SharedNoiseModel& model) :
        Base(model, key), base_pos(base), measures(measure_), meas_size(size)
    {
        cout << "add DDPseudorangeFactor_sys:" << Symbol(key) << endl;
        key_ = key;
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new DDPseudorangeFactor_sys(*this)));
    }

    Vector evaluateError(const Point3& RxPos,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(meas_size, 3);
        VectorXd residual = VectorXd::Zero(meas_size);
        rtklib::GNSS_Raw u_master, r_master;
        u_master = measures.begin()->u_master_SV;
        r_master = measures.begin()->r_master_SV;
        Vector3 u_master_sat(u_master.sat_pos_x, u_master.sat_pos_y, u_master.sat_pos_z);
        Vector3 r_master_sat(r_master.sat_pos_x, r_master.sat_pos_y, r_master.sat_pos_z);


        double estdist_u_master = (u_master_sat - RxPos).norm();
        double V_u_master = u_master.pseudorange - estdist_u_master;
        double estdist_r_master = (r_master_sat - base_pos).norm();
        double V_r_master = r_master.pseudorange - estdist_r_master;

        Vector3 H_master = (RxPos - u_master_sat).normalized();

        Eigen::Matrix<double, Dynamic, Dynamic> R = Eigen::Matrix<double, Dynamic, Dynamic>::Identity(meas_size, meas_size);

        int i = 0;
        for (auto measure : measures)
        {
            rtklib::GNSS_Raw u_else, r_else;
            u_else = measure.u_iSV;
            r_else = measure.r_iSV;
            Vector3 u_else_sat(u_else.sat_pos_x, u_else.sat_pos_y, u_else.sat_pos_z);
            Vector3 r_else_sat(r_else.sat_pos_x, r_else.sat_pos_y, r_else.sat_pos_z);
            double estdist_u_else = (u_else_sat - RxPos).norm();
            double V_u_else = u_else.pseudorange - estdist_u_else;
            double estdist_r_else = (r_else_sat - base_pos).norm();
            double V_r_else = r_else.pseudorange - estdist_r_else;

            Vector3 H_else = (RxPos - u_else_sat).normalized();

            H.block(i, 0, 1, 3) = (H_else - H_master).transpose();
            residual[i] = ((V_u_master - V_u_else) - (V_r_master - V_r_else));
            //            if(fabs(residual[i])>10){
            //                R(i,i) = 0;
            //                cout << residual[i] << endl;
            //            }
            i++;
        }

        //        std::cout<<"size->"<<meas_size<<std::endl;
        std::cout << Symbol(key_) << std::endl;
        std::cout << "H->\n" << H << std::endl;
        if (H1)
        {
            H1->resize(meas_size, 3);

            *H1 = R * H;
            std::cout << "H1->\n" << *H1 << std::endl;
        }
        //        std::cout << "test\n";

        return R * residual;
        //        for (auto iter = measures.begin(); iter != measures.end(); ++iter) {
        //
        //            int sys = iter ->first;
        //            vector<DDMeasurement> meas = iter->second;
        //            lio_sam::GNSS_Raw u_master,r_master;
        //            u_master = meas.begin()->u_master_SV;
        //            r_master = meas.begin() ->r_master_SV;
        //            Vector3 u_master_sat(u_master.sat_pos_x,u_master.sat_pos_y,u_master.sat_pos_z);
        //            Vector3 r_master_sat(r_master.sat_pos_x,r_master.sat_pos_y,r_master.sat_pos_z);
        //
        //
        //            double estdist_u_master = (u_master_sat - RxPos).norm();
        //            double V_u_master = u_master.pseudorange-estdist_u_master;
        //            double estdist_r_master = (r_master_sat - base_pos).norm();
        //            double V_r_master = r_master.pseudorange-estdist_r_master;
        //
        //            Vector3 H_master = (RxPos - u_master_sat).normalized();
        //
        //            for(auto measure : meas){
        //                lio_sam::GNSS_Raw u_else,r_else;
        //                u_else = measure.u_iSV;
        //                r_else = measure.r_iSV;
        //                Vector3 u_else_sat(u_else.sat_pos_x,u_else.sat_pos_y,u_else.sat_pos_z);
        //                Vector3 r_else_sat(r_else.sat_pos_x,r_else.sat_pos_y,r_else.sat_pos_z);
        //                double estdist_u_else = (u_else_sat - RxPos).norm();
        //                double V_u_else = u_else.pseudorange-estdist_u_else;
        //                double estdist_r_else = (r_else_sat - base_pos).norm();
        //                double V_r_else = r_else.pseudorange-estdist_r_else;
        //
        //                Vector3 H_else = (RxPos - u_else_sat).normalized();
        //
        //                H << (H_else - H_master).transpose();
        //                residual<<((V_u_master-V_u_else)-(V_r_master-V_r_else));
        //
        //                num_mea++;
        //            }
        //        }
    }

};
#endif

// 双差伪距因子，直接以 ECEF 坐标下的接收机位置 Point3 作为优化变量。
//
// 作用:
// - 使用一颗参考星 Master 和多颗从卫星 Else 构造双差伪距残差。
// - 残差形式:
//   DD_ij = [(P_r_i - rho_r_i) - (P_b_i - rho_b_i)]
//         - [(P_r_j - rho_r_j) - (P_b_j - rho_b_j)]
//
// 状态变量:
// - key -> Point3，表示 Rover/GNSS 天线在 ECEF 坐标系下的位置。
//
// 特点:
// - 不涉及局部 ENU 坐标、Pose3 姿态、外参旋转或杆臂。
// - 适合纯 GNSS 位置估计，或者已经把状态直接定义为 ECEF 三维点的场景。
// - 相比下面的 ENU/Pose3 版本，这个类更简单，但无法直接约束 LIO/GINS 中的完整位姿。
class GNSSDDPsrFactor : public NoiseModelFactor1<Point3>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;

public:
    GNSSDDPsrFactor() = default;

    GNSSDDPsrFactor(Key key, const rtklib::GNSS_Info_SD& _Info_Master, const std::vector<rtklib::GNSS_Info_SD>& _Infos_Else,
        const prcopt_t _opt, const int _f, const bool _UnitWeight, const SharedNoiseModel& model)
        : NoiseModelFactor1<Point3>(model, key),
        opt(_opt), Info_Master(_Info_Master), Infos_Else(_Infos_Else),
        f(_f), UnitWeight(_UnitWeight)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDPsrFactor(*this)));
    }

    Vector evaluateError(const Point3& RxPos,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - RxPos).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 1);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(RxPos, Info_Master, TYPE_Rover_Master); // 计算卫地距
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(RxPos, Info_Master, TYPE_Rover_Master, H_Rover_Master);
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);
        double V_Rover_Master = Info_Master.Mea_Rover.P[f] - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.P[f] - Est_Geodist_Base_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;

            double Est_Geodist_Base_Else;

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(RxPos, Info_Else, TYPE_Rover_Else);
            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(RxPos, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double V_Rover_Else = Info_Else.Mea_Rover.P[f] - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.P[f] - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else);
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 1);

            //            cout << residual(i,0) << "  " << R(i,i) << std::endl;
        }

        if (UnitWeight) // 单位矩阵，即等权
            R.setIdentity();
        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;
        if (H1)
        {
            H1->resize(residual_size, 3);
            *H1 = llt * H;
        }
        //        cout << "H1\n";
        //        cout << *H1 << endl;

        return residual;
    }
};

// 双差伪距因子，以局部坐标系下的 Pose3 作为优化变量，并通过固定杆臂换算到 GNSS 天线位置。
//
// 作用:
// - 和 GNSSDDPsrFactor 一样构造双差伪距残差，但优化变量不再是 ECEF Point3，
//   而是 LIO/GINS 因子图中的 Pose3。
// - 内部会执行:
//   1. RxPos.transformFrom(lb_) 得到天线在局部坐标系下的位置；
//   2. 乘 extRot 和 ENU->ECEF 旋转矩阵；
//   3. 加上 lla_origin 对应的 ECEF 原点；
//   4. 得到 GNSS 计算需要的 Rover ECEF 坐标。
//
// 状态变量:
// - key -> Pose3，表示当前 IMU/LiDAR/body 在局部 ENU/map 坐标系下的位姿。
//
// 特点:
// - lb_ 和 extRot 是已知常量，不参与优化。
// - 适合 GNSS 作为位置观测约束 LIO/GINS 当前位姿的情况。
// - 这是和融合系统最常用、最直接对应的双差伪距因子版本。
class GNSSDDPsrFactor_ENU : public NoiseModelFactor1<Pose3>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;
    Point3 lla_origin;
    Vector3 lb_;
    Matrix3 extRot;

public:
    GNSSDDPsrFactor_ENU() = default;

    GNSSDDPsrFactor_ENU(Key key, const rtklib::GNSS_Info_SD& _Info_Master, const std::vector<rtklib::GNSS_Info_SD>& _Infos_Else,
        const prcopt_t _opt, const int _f, const bool _UnitWeight, const Point3 _lla_origin, const Vector3 lb, const Matrix3 _extRot, const SharedNoiseModel& model)
        : NoiseModelFactor1<Pose3>(model, key),
        opt(_opt), Info_Master(_Info_Master), Infos_Else(_Infos_Else),
        f(_f), UnitWeight(_UnitWeight), lla_origin(_lla_origin), lb_(lb), extRot(_extRot)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDPsrFactor_ENU(*this)));
    }

    Vector evaluateError(const Pose3& RxPos,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());
        double sinp = sin(lat), cosp = cos(lat), sinl = sin(lon), cosl = cos(lon);
        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sinl, -cosl * sinp, cosl* cosp,
            cosl, -sinl * sinp, sinl* cosp,
            0, cosp, sinp;
        Eigen::Vector3d ecef;

        //        ecef = r * (RxPos.translation(H_pos) + RxPos.rotation(H_pos) * lb_) + t;
        ecef = r * extRot * RxPos.transformFrom(lb_, H_pos) + t;
        //        H_pos.block<3,3>(0,0) = skewSymmetric(-lb_.x(),-lb_.y(),-lb_.z());
        //        H_pos.block<3,3>(0,3) = Matrix3::Identity();

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - ecef).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 1);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(ecef, Info_Master, TYPE_Rover_Master); // 计算卫地距
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(ecef, Info_Master, TYPE_Rover_Master, H_Rover_Master);
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);
        double V_Rover_Master = Info_Master.Mea_Rover.P[f] - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.P[f] - Est_Geodist_Base_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;

            double Est_Geodist_Base_Else;

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(ecef, Info_Else, TYPE_Rover_Else);
            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(ecef, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double V_Rover_Else = Info_Else.Mea_Rover.P[f] - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.P[f] - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else);
            double scale = 1;
            if (fabs(residual(i, 0)) > 10) scale = 100;
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 1) * scale;

            //            cout << residual(i,0) << "  " << R(i,i) << std::endl;
        }

        if (UnitWeight) // 单位矩阵，即等权
            R.setIdentity();
        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;
        if (H1)
        {
            H1->resize(residual_size, 6);
            *H1 = llt * H * r * extRot * H_pos;
        }

        return residual;
    }

    Vector debugEvaluateError(const Pose3& RxPos,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {

        // H_pos 是 GTSAM 返回的局部天线点对 Pose3 扰动的雅可比:
        // p_local = RxPos.transformFrom(lb_)
        // H_pos = d p_local / d Pose3, size = 3 x 6.
        Eigen::Matrix<double, 3, 6> H_pos;

        // lla_origin 是局部 ENU 坐标系原点，经纬度单位通常为 degree。
        // 这里转成 rad 后构造 ENU -> ECEF 的旋转矩阵。
        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());
        double sinp = sin(lat), cosp = cos(lat), sinl = sin(lon), cosl = cos(lon);

        // t: ENU 原点在 ECEF 坐标系下的位置。
        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);

        // r: ENU -> ECEF 旋转矩阵。
        // p_ecef = r * p_enu + t
        Eigen::Matrix3d r;
        r << -sinl, -cosl * sinp, cosl* cosp,
            cosl, -sinl * sinp, sinl* cosp,
            0, cosp, sinp;

        // 将因子图中的当前位姿 RxPos 转成 GNSS 使用的天线 ECEF 坐标。
        // lb_ 是 GNSS 天线相对 IMU/LiDAR 坐标系的杆臂。
        //
        // p_ant^ecef = R_enu^ecef * R_ext * T_body^enu * lb_ + p_origin^ecef
        Eigen::Vector3d ecef;
        ecef = r * extRot * RxPos.transformFrom(lb_, H_pos) + t;

        // H:     双差伪距残差对 Rover 天线 ECEF 位置的雅可比，size = n_dd x 3。
        // residual: 每颗从卫星对应一条双差伪距残差，size = n_dd。
        // R:     残差协方差矩阵，后续用于白化 residual 和 H。
        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        // BasePos 是基准站坐标，RTK 双差里通常认为基准站位置已知。
        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);

        // 参考星 Master 的方位角/高度角、Rover/Base 时间差、当前基线长度。
        // 方差模型 varerr_sdobs 会用这些量估计观测噪声。
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - ecef).norm();//bl是基线长度，GNSS_Tools::CalGeodist(BasePos, ecef)也可以计算基线长度，但前者已经在计算残差时用过了，这里直接用两点距离的方式避免重复计算。

        // 初始方差来自参考星的单差伪距误差模型。
        // 最后一个参数为 1，表示 pseudorange/code 观测；载波相位因子里通常传 0。
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 1);

        // 先把参考星单差方差填到整块 R 中，后面每条双差残差再加上从卫星的单差方差。
        // 直观上:
        // Var(DD_i_j) ~= Var(SD_i) + Var(SD_j)
        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master 指参考星。先计算参考星 i 的 Rover/Base 几何距离:
        // rho_r_i = || sat_i - rover ||
        // rho_b_i = || sat_i - base  ||
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(ecef, Info_Master, TYPE_Rover_Master); // 计算卫地距

        // H_Rover_Master = d rho_r_i / d rover_ecef
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(ecef, Info_Master, TYPE_Rover_Master, H_Rover_Master);
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);

        // 参考星的单差残差项:
        // v_r_i = P_r_i - rho_r_i
        // v_b_i = P_b_i - rho_b_i
        // SD_i  = v_r_i - v_b_i
        double V_Rover_Master = Info_Master.Mea_Rover.P[f] - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.P[f] - Est_Geodist_Base_Master;

        // 每一颗 Else 从卫星 j 和 Master 参考星 i 构造一条双差伪距残差:
        //
        // residual_j = [(P_r_i - rho_r_i) - (P_b_i - rho_b_i)]
        //            - [(P_r_j - rho_r_j) - (P_b_j - rho_b_j)]
        //
        // 简写:
        // residual_j = SD_i - SD_j
        //
        // 伪距没有整周模糊度项，所以不像载波相位因子那样需要减 N * lambda。
        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            double Est_Geodist_Rover_Else;

            double Est_Geodist_Base_Else;

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(ecef, Info_Else, TYPE_Rover_Else);

            // 从卫星 j 的几何距离雅可比:
            // H_Rover_Else = d rho_r_j / d rover_ecef
            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(ecef, Info_Else, TYPE_Rover_Else, H_Rover_Else);

            // 双差残差对 Rover 位置的雅可比只包含 Rover 端的几何距离项。
            // Base 位置固定，所以 Base 相关距离对当前状态没有导数。
            //
            // residual_j = ... - rho_r_i + rho_r_j + const
            // d residual_j / d rover_ecef = d rho_r_j/d rover_ecef - d rho_r_i/d rover_ecef
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            // 从卫星 j 的单差残差项:
            // v_r_j = P_r_j - rho_r_j
            // v_b_j = P_b_j - rho_b_j
            // SD_j  = v_r_j - v_b_j
            double V_Rover_Else = Info_Else.Mea_Rover.P[f] - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.P[f] - Est_Geodist_Base_Else;

            // 双差伪距残差:
            // DD_i_j = SD_i - SD_j
            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else);

            // 简单的降权策略: 残差超过 10 m 时，将该观测方差放大 100 倍。
            // 方差变大意味着信息量变小，优化时这条观测的影响会降低。
            double scale = 1;
            if (fabs(residual(i, 0)) > 10) scale = 100;

            // 双差方差追加从卫星 j 的单差方差:
            // Var(DD_i_j) ~= Var(SD_i) + Var(SD_j)
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 1) * scale;

            cout << residual(i, 0) << "  " << R(i, i) << std::endl;
        }

        // 如果 UnitWeight 为 true，则忽略观测方差，所有双差残差等权。
        if (UnitWeight) // 单位矩阵，即等权
            R.setIdentity();

        // 对残差和雅可比进行白化。
        // GTSAM 因子返回的应是:
        // e_white = sqrt_info * e
        //噪声大的观测 -> R 大 -> R⁻¹ 小 -> 白化后残差变小 -> 权重低
        // 噪声小的观测 -> R 小 -> R⁻¹ 大 -> 白化后残差变大 -> 权重高
        // 这里:
        // sqrt_info = chol(R^-1)^T
        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;
        if (H1)
        {
            H1->resize(residual_size, 6);

            // 链式法则，把 d residual / d rover_ecef 转成 d residual / d Pose3:
            //
            // d e_white / d Pose3
            // = sqrt_info
            // * d e / d p_ecef
            // * d p_ecef / d p_enu
            // * d p_enu / d p_body
            // * d p_body / d Pose3
            //
            // 对应代码:
            // llt * H * r * extRot * H_pos
            *H1 = llt * H * r * extRot * H_pos;
        }

        return residual;
    }
};

// 双差伪距因子，以 Pose3 和 GNSS 杆臂 Vector3 共同作为优化变量。
//
// 作用:
// - 残差仍然是双差伪距残差，但 GNSS 天线相对 body/IMU 的杆臂 lb 不再固定，
//   而是作为第二个变量一起估计。
// - 内部会用 RxPos.transformFrom(lb, H_pos, H_lb) 同时计算:
//   1. 天线位置；
//   2. 残差对 Pose3 的雅可比 H1；
//   3. 残差对杆臂 lb 的雅可比 H2。
//
// 状态变量:
// - key1 -> Pose3，当前 body/IMU 位姿。
// - key2 -> Vector3，GNSS 天线相对 body/IMU 的杆臂。
//
// 特点:
// - 适合外参/杆臂不够准确，希望在线标定 GNSS 天线杆臂的场景。
// - 比 GNSSDDPsrFactor_ENU 多估计 3 个自由度，灵活但也更依赖运动激励和观测几何。
// - 这里没有 extRot 参数，默认杆臂与 Pose3 使用同一局部/body 变换关系。
class GNSSDDPsrFactor_ENU_lever : public NoiseModelFactor2<Pose3, Vector3>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;
    Point3 lla_origin;

public:
    GNSSDDPsrFactor_ENU_lever() = default;

    GNSSDDPsrFactor_ENU_lever(Key key1, Key key2, const rtklib::GNSS_Info_SD& _Info_Master, const std::vector<rtklib::GNSS_Info_SD>& _Infos_Else,
        const prcopt_t _opt, const int _f, const bool _UnitWeight, const Point3 _lla_origin, const SharedNoiseModel& model)
        : NoiseModelFactor2<Pose3, Vector3>(model, key1, key2),
        opt(_opt), Info_Master(_Info_Master), Infos_Else(_Infos_Else),
        f(_f), UnitWeight(_UnitWeight), lla_origin(_lla_origin)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDPsrFactor_ENU_lever(*this)));
    }

    Vector evaluateError(const Pose3& RxPos, const Vector3& lb,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos;
        Eigen::Matrix<double, 3, 3> H_lb;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());
        double sinp = sin(lat), cosp = cos(lat), sinl = sin(lon), cosl = cos(lon);
        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        //        r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat),
        //                cos(lon), -sin(lon) * sin(lat), sin(lon) * cos(lat),
        //                0, cos(lat), sin(lat);
        r << -sinl, -cosl * sinp, cosl* cosp,
            cosl, -sinl * sinp, sinl* cosp,
            0, cosp, sinp;
        Eigen::Vector3d ecef;

        //        ecef = r * (RxPos.translation(H_pos) + RxPos.rotation(H_pos) * lb_) + t;
        ecef = r * RxPos.transformFrom(lb, H_pos, H_lb) + t;

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - ecef).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 1);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(ecef, Info_Master, TYPE_Rover_Master); // 计算卫地距
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(ecef, Info_Master, TYPE_Rover_Master, H_Rover_Master);
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);
        double V_Rover_Master = Info_Master.Mea_Rover.P[f] - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.P[f] - Est_Geodist_Base_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;

            double Est_Geodist_Base_Else;

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(ecef, Info_Else, TYPE_Rover_Else);
            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(ecef, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double V_Rover_Else = Info_Else.Mea_Rover.P[f] - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.P[f] - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else);
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 1);

            //            cout << residual(i,0) << "  " << R(i,i) << std::endl;
        }

        if (UnitWeight) // 单位矩阵，即等权
            R.setIdentity();
        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;
        if (H1)
        {
            H1->resize(residual_size, 6);
            *H1 = llt * H * r * H_pos;
        }
        if (H2)
        {
            H2->resize(residual_size, 3);
            *H2 = llt * H * r * H_lb;
        }

        return residual;
    }
};

#endif // LIO_SAM_PSEUDORANGEFACTOR_H
