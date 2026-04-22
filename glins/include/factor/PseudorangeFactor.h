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
        ecef = r * extRot * RxPos.transformFrom(lb_, H_pos) + t;

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
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 1) * scale;

            cout << residual(i, 0) << "  " << R(i, i) << std::endl;
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
};

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
