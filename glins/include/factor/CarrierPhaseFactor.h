//
// Created by wangchuji on 2023/8/19.
//
#pragma once
#ifndef glins_CARRIERPHASEFACTOR_H
#define glins_CARRIERPHASEFACTOR_H
#include "gtsam/base/Vector.h"
#include "gtsam/base/Matrix.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/ExpressionFactor.h"
#include "gtsam/geometry/Point3.h"
#include "gnss_tools.h"
#include <map>
#include <gtsam/geometry/Pose3.h>
#include "utility.h"
using namespace gtsam;
class GNSSDDCpFactor : public NoiseModelFactor2<Point3, Vector>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;

public:
    GNSSDDCpFactor() = default;

    GNSSDDCpFactor(Key key1, Key key2, const rtklib::GNSS_Info_SD& _Info_Master, const std::vector<rtklib::GNSS_Info_SD>& _Infos_Else,
        const prcopt_t _opt, const int _f, const bool _UnitWeight, const SharedNoiseModel& model)
        : NoiseModelFactor2<Point3, Vector>(model, key1, key2),
        opt(_opt), Info_Master(_Info_Master), Infos_Else(_Infos_Else),
        f(_f), UnitWeight(_UnitWeight)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDCpFactor(*this)));
    }

    Vector evaluateError(const Point3& RxPos, const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        //        Eigen::Matrix<double,Dynamic,1> H_ambi = Eigen::Matrix<double,Dynamic,1>::Zero(residual_size, 1);
        //        Eigen::Matrix<double,Dynamic,Dynamic> H_ambj = Eigen::Matrix<double,Dynamic,Dynamic>::Zero(residual_size,residual_size);
        Eigen::Matrix<double, Dynamic, Dynamic> H_amb = Eigen::Matrix<double, Dynamic, Dynamic>::Zero(residual_size, Amb.size());

        H.setZero();
        H_amb.setZero();

        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - RxPos).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 0);

        R.setConstant(pr_vari);
        //白化矩阵：白化矩阵就是一个 把带协方差的残差变成标准化残差的权重矩阵。
        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(RxPos, Info_Master, TYPE_Rover_Master); // 计算卫地距
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);

        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(RxPos, Info_Master, TYPE_Rover_Master, H_Rover_Master);

        double wave_lengthi = Info_Master.ssat.lam[f];
        double V_Rover_Master = Info_Master.Mea_Rover.L[f] * wave_lengthi - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.L[f] * wave_lengthi - Est_Geodist_Base_Master;

        double ambi = Amb[MAXSAT * f + Info_Master.Mea_Rover.sat - 1];
        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;
            double Est_Geodist_Base_Else;
            double ambj = Amb[MAXSAT * f + Info_Else.Mea_Rover.sat - 1];

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(RxPos, Info_Else, TYPE_Rover_Else);
            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(RxPos, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            double wave_lengthj = Info_Else.ssat.lam[f];
            double V_Rover_Else = Info_Else.Mea_Rover.L[f] * wave_lengthj - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.L[f] * wave_lengthj - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else) - (ambi * wave_lengthi - ambj * wave_lengthj);
            //            H_ambj(i,i)=wave_lengthj;
            //            H_ambi(i,0)=-wave_lengthi;
            H_amb(i, MAXSAT * f + Info_Else.Mea_Rover.sat - 1) = wave_lengthj;
            H_amb(i, MAXSAT * f + Info_Master.Mea_Rover.sat - 1) = -wave_lengthi;
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 0);

            //            cout << "wave_lengthj: " << wave_lengthj << " "
            //            << "wave_lengthi: " << wave_lengthi << " "
            //            << "L_rover " << Info_Else.Mea_Rover.L[f] << " "
            //            << "L_base " << Info_Else.Mea_Base.L[f] << " "
            //            << residual(i,0) << "  " << R(i,i) << std::endl;
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
        if (H2)
        {
            H2->resize(residual_size, Amb.size());
            *H2 = llt * H_amb;
            //            H2->block(0,0,residual_size,1) = llt * H_ambi;
            //            H2->block(0,1,residual_size,residual_size) = llt * H_ambj;
        }

        return residual;
    }
};

class GNSSDDCpFactorCompress : public NoiseModelFactor2<Point3, Vector>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;
    const map<int, int> ar_index;

public:
    GNSSDDCpFactorCompress() = default;

    GNSSDDCpFactorCompress(Key key1, Key key2, const rtklib::GNSS_Info_SD& _Info_Master, const std::vector<rtklib::GNSS_Info_SD>& _Infos_Else, const map<int, int>& _ar_index,
        const prcopt_t _opt, const int _f, const bool _UnitWeight, const SharedNoiseModel& model)
        : NoiseModelFactor2<Point3, Vector>(model, key1, key2),
        opt(_opt), Info_Master(_Info_Master), Infos_Else(_Infos_Else), f(_f),
        UnitWeight(_UnitWeight), ar_index(_ar_index)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDCpFactorCompress(*this)));
    }

    Vector evaluateError(const Point3& RxPos, const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        //        Eigen::Matrix<double,Dynamic,1> H_ambi = Eigen::Matrix<double,Dynamic,1>::Zero(residual_size, 1);
        //        Eigen::Matrix<double,Dynamic,Dynamic> H_ambj = Eigen::Matrix<double,Dynamic,Dynamic>::Zero(residual_size,residual_size);
        Eigen::Matrix<double, Dynamic, Dynamic> H_amb = Eigen::Matrix<double, Dynamic, Dynamic>::Zero(residual_size, Amb.size());

        H.setZero();
        H_amb.setZero();

        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - RxPos).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 0);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(RxPos, Info_Master, TYPE_Rover_Master); // 计算卫地距
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);

        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(RxPos, Info_Master, TYPE_Rover_Master, H_Rover_Master);

        double wave_lengthi = Info_Master.ssat.lam[f];
        double V_Rover_Master = Info_Master.Mea_Rover.L[f] * wave_lengthi - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.L[f] * wave_lengthi - Est_Geodist_Base_Master;

        double ambi = Amb[ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second];
        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;
            double Est_Geodist_Base_Else;
            double ambj = Amb[ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second];

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(RxPos, Info_Else, TYPE_Rover_Else);
            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(RxPos, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            double wave_lengthj = Info_Else.ssat.lam[f];
            double V_Rover_Else = Info_Else.Mea_Rover.L[f] * wave_lengthj - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.L[f] * wave_lengthj - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else) - (ambi * wave_lengthi - ambj * wave_lengthj);
            //            H_ambj(i,i)=wave_lengthj;
            //            H_ambi(i,0)=-wave_lengthi;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second) = wave_lengthj;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second) = -wave_lengthi;
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 0);

            //            cout << "wave_lengthj: " << wave_lengthj << " "
            //            << "wave_lengthi: " << wave_lengthi << " "
            //            << "L_rover " << Info_Else.Mea_Rover.L[f] << " "
            //            << "L_base " << Info_Else.Mea_Base.L[f] << " "
            //            << residual(i,0) << "  " << R(i,i) << std::endl;
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
        if (H2)
        {
            H2->resize(residual_size, Amb.size());
            *H2 = llt * H_amb;
            //            H2->block(0,0,residual_size,1) = llt * H_ambi;
            //            H2->block(0,1,residual_size,residual_size) = llt * H_ambj;
        }

        return residual;
    }
};

class GNSSDDCpFactorSingle : public NoiseModelFactor3<Point3, double, double>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const rtklib::GNSS_Info_SD Info_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;

public:
    GNSSDDCpFactorSingle() = default;

    GNSSDDCpFactorSingle(Key key1, Key key2, Key key3, const rtklib::GNSS_Info_SD& _Info_Master, const rtklib::GNSS_Info_SD& _Info_Else,
        const prcopt_t _opt, const int _f, const bool _UnitWeight, const SharedNoiseModel& model)
        : NoiseModelFactor3<Point3, double, double>(model, key1, key2, key3),
        opt(_opt), Info_Master(_Info_Master), Info_Else(_Info_Else),
        f(_f), UnitWeight(_UnitWeight)
    {
        residual_size = 1;
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDCpFactorSingle(*this)));
    }

    Vector evaluateError(const Point3& RxPos, const double& ambi, const double& ambj,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none) const
    {

        Eigen::Matrix<double, Dynamic, 3> H(residual_size, 3);
        Eigen::Matrix<double, Dynamic, 1> H_ambi(residual_size, 1);
        Eigen::Matrix<double, Dynamic, 1> H_ambj(residual_size, 1);

        H.setZero();
        H_ambi.setZero();
        H_ambj.setZero();

        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - RxPos).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 0);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(RxPos, Info_Master, TYPE_Rover_Master); // 计算卫地距
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);

        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(RxPos, Info_Master, TYPE_Rover_Master, H_Rover_Master);

        double wave_lengthi = Info_Master.ssat.lam[f];
        double V_Rover_Master = Info_Master.Mea_Rover.L[f] * wave_lengthi - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.L[f] * wave_lengthi - Est_Geodist_Base_Master;

        for (int i = 0; i < residual_size; i++)
        {

            double Est_Geodist_Rover_Else;
            double Est_Geodist_Base_Else;

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(RxPos, Info_Else, TYPE_Rover_Else);
            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(RxPos, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            double wave_lengthj = Info_Else.ssat.lam[f];
            double V_Rover_Else = Info_Else.Mea_Rover.L[f] * wave_lengthj - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.L[f] * wave_lengthj - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else) - (ambi * wave_lengthi - ambj * wave_lengthj);
            H_ambj(i, 0) = wave_lengthj;
            H_ambi(i, 0) = -wave_lengthi;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 0);
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
        if (H2)
        {
            H2->resize(residual_size, 1);
            *H2 = llt * H_ambi;
        }
        if (H3)
        {
            H3->resize(residual_size, 1);
            *H3 = llt * H_ambj;
        }

        return residual;
    }
};

class GNSSDDCpFactor_ENU : public NoiseModelFactor2<Pose3, Vector>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;
    Point3 lla_origin;
    Vector3 lb;

public:
    GNSSDDCpFactor_ENU() = delete;

    GNSSDDCpFactor_ENU(Key key1, Key key2, const rtklib::GNSS_Info_SD& Info_Master, const std::vector<rtklib::GNSS_Info_SD>& Infos_Else,
        const prcopt_t _opt, const int _f, const bool UnitWeight, const Point3& _lla_origin, const Vector3 lb, const SharedNoiseModel& model)
        : NoiseModelFactor2<Pose3, Vector>(model, key1, key2),
        opt(_opt), Info_Master(Info_Master), Infos_Else(Infos_Else),
        f(_f), UnitWeight(UnitWeight), lla_origin(_lla_origin), lb(lb)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDCpFactor_ENU(*this)));
    }

    Vector evaluateError(const Pose3& RxPos, const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon)* cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon)* cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d ecef;
        //        ecef = r * (RxPos.translation(H_pos) + RxPos.rotation(H_pos) * lb) + t;

        ecef = r * RxPos.transformFrom(lb, H_pos) + t;

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        //        Eigen::Matrix<double,Dynamic,1> H_ambi = Eigen::Matrix<double,Dynamic,1>::Zero(residual_size, 1);
        //        Eigen::Matrix<double,Dynamic,Dynamic> H_ambj = Eigen::Matrix<double,Dynamic,Dynamic>::Zero(residual_size,residual_size);
        //        Eigen::Matrix<double,Dynamic,Dynamic> H_amb = Eigen::Matrix<double,Dynamic,Dynamic>::Zero(residual_size,Amb.size());

        Eigen::SparseMatrix<double> H_amb(residual_size, Amb.size());
        H_amb.reserve(residual_size * 2);
        H.setZero();
        //        H_amb.setZero();

        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - ecef).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 0);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(ecef, Info_Master, TYPE_Rover_Master); // 计算卫地距
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(ecef, Info_Master, TYPE_Rover_Master, H_Rover_Master);

        double wave_lengthi = Info_Master.ssat.lam[f];
        double V_Rover_Master = Info_Master.Mea_Rover.L[f] * wave_lengthi - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.L[f] * wave_lengthi - Est_Geodist_Base_Master;

        double ambi = Amb[MAXSAT * f + Info_Master.Mea_Rover.sat - 1];
        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;
            double Est_Geodist_Base_Else;
            double ambj = Amb[MAXSAT * f + Info_Else.Mea_Rover.sat - 1];

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(ecef, Info_Else, TYPE_Rover_Else);
            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(ecef, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            double wave_lengthj = Info_Else.ssat.lam[f];
            double V_Rover_Else = Info_Else.Mea_Rover.L[f] * wave_lengthj - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.L[f] * wave_lengthj - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else) - (ambi * wave_lengthi - ambj * wave_lengthj);
            //            H_ambj(i,i)=wave_lengthj;
            //            H_ambi(i,0)=-wave_lengthi;
            //            H_amb(i,MAXSAT*f+Info_Else.Mea_Rover.sat-1)=wave_lengthj;
            //            H_amb(i,MAXSAT*f+Info_Master.Mea_Rover.sat-1)=-wave_lengthi;
            H_amb.insert(i, MAXSAT * f + Info_Else.Mea_Rover.sat - 1) = wave_lengthj;
            H_amb.insert(i, MAXSAT * f + Info_Master.Mea_Rover.sat - 1) = -wave_lengthi;
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 0);

            //            cout << "wave_lengthj: " << wave_lengthj << " "
            //            << "wave_lengthi: " << wave_lengthi << " "
            //            << "L_rover " << Info_Else.Mea_Rover.L[f] << " "
            //            << "L_base " << Info_Else.Mea_Base.L[f] << " "
            //            << residual(i,0) << "  " << R(i,i) << std::endl;
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
            H2->resize(residual_size, Amb.size());
            *H2 = llt * H_amb;
            //            H2->block(0,0,residual_size,1) = llt * H_ambi;
            //            H2->block(0,1,residual_size,residual_size) = llt * H_ambj;
        }
        return residual;
    }
};

class GNSSDDCpFactorCompress_ENU : public NoiseModelFactor2<Pose3, Vector>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;
    Point3 lla_origin;
    Vector3 lb;
    const map<int, int> ar_index;
    Matrix3 extRot;

public:
    GNSSDDCpFactorCompress_ENU() = delete;

    GNSSDDCpFactorCompress_ENU(Key key1, Key key2, const rtklib::GNSS_Info_SD& Info_Master, const std::vector<rtklib::GNSS_Info_SD>& Infos_Else, const map<int, int>& _ar_index,
        const prcopt_t _opt, const int _f, const bool UnitWeight, const Point3& _lla_origin, const Vector3 lb, const Matrix3 _extRot, const SharedNoiseModel& model)
        : NoiseModelFactor2<Pose3, Vector>(model, key1, key2),
        opt(_opt), Info_Master(Info_Master), Infos_Else(Infos_Else),
        f(_f), UnitWeight(UnitWeight), lla_origin(_lla_origin), lb(lb), ar_index(_ar_index), extRot(_extRot)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDCpFactorCompress_ENU(*this)));
    }

    Vector evaluateError(const Pose3& RxPos, const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon)* cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon)* cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d ecef;
        //        ecef = r * (RxPos.translation(H_pos) + RxPos.rotation(H_pos) * lb) + t;
        ecef = r * extRot * RxPos.transformFrom(lb, H_pos) + t;
        //        H_pos.block<3,3>(0,0) = skewSymmetric(-lb.x(),-lb.y(),-lb.z());
        //        H_pos.block<3,3>(0,3) = Matrix3::Identity();

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        Eigen::Matrix<double, Dynamic, Dynamic> H_amb = Eigen::Matrix<double, Dynamic, Dynamic>::Zero(residual_size, Amb.size());

        H.setZero();
        H_amb.setZero();

        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - ecef).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 0);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(ecef, Info_Master, TYPE_Rover_Master); // 计算卫地距
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(ecef, Info_Master, TYPE_Rover_Master, H_Rover_Master);

        double wave_lengthi = Info_Master.ssat.lam[f];
        double V_Rover_Master = Info_Master.Mea_Rover.L[f] * wave_lengthi - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.L[f] * wave_lengthi - Est_Geodist_Base_Master;

        double ambi = Amb[ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second];
        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;
            double Est_Geodist_Base_Else;
            double ambj = Amb[ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second];

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(ecef, Info_Else, TYPE_Rover_Else);
            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(ecef, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            double wave_lengthj = Info_Else.ssat.lam[f];
            double V_Rover_Else = Info_Else.Mea_Rover.L[f] * wave_lengthj - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.L[f] * wave_lengthj - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else) - (ambi * wave_lengthi - ambj * wave_lengthj);
            //            H_ambj(i,i)=wave_lengthj;
            //            H_ambi(i,0)=-wave_lengthi;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second) = wave_lengthj;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second) = -wave_lengthi;
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            double scale = 1;
            if (fabs(residual(i, 0)) > 10) scale = 100;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 0) * scale;

            //            cout << "wave_lengthj: " << wave_lengthj << " "
            //            << "wave_lengthi: " << wave_lengthi << " "
            //            << "L_rover " << Info_Else.Mea_Rover.L[f] << " "
            //            << "L_base " << Info_Else.Mea_Base.L[f] << " "
            //            << residual(i,0) << "  " << R(i,i) << std::endl;
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
        if (H2)
        {
            H2->resize(residual_size, Amb.size());
            *H2 = llt * H_amb;
            //            H2->block(0,0,residual_size,1) = llt * H_ambi;
            //            H2->block(0,1,residual_size,residual_size) = llt * H_ambj;
        }
        return residual;
    }

    Vector debugEvaluateError(const Pose3& RxPos, const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {
        Eigen::Matrix<double, 3, 6> H_pos;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon)* cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon)* cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d ecef;
        ecef = r * extRot * RxPos.transformFrom(lb, H_pos) + t;

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        Eigen::Matrix<double, Dynamic, Dynamic> H_amb = Eigen::Matrix<double, Dynamic, Dynamic>::Zero(residual_size, Amb.size());

        H.setZero();
        H_amb.setZero();

        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - ecef).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 0);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(ecef, Info_Master, TYPE_Rover_Master); // 计算卫地距
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(ecef, Info_Master, TYPE_Rover_Master, H_Rover_Master);

        double wave_lengthi = Info_Master.ssat.lam[f];
        double V_Rover_Master = Info_Master.Mea_Rover.L[f] * wave_lengthi - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.L[f] * wave_lengthi - Est_Geodist_Base_Master;

        double ambi = Amb[ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second];
        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            double Est_Geodist_Rover_Else;
            double Est_Geodist_Base_Else;
            double ambj = Amb[ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second];

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(ecef, Info_Else, TYPE_Rover_Else);
            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(ecef, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            double wave_lengthj = Info_Else.ssat.lam[f];
            double V_Rover_Else = Info_Else.Mea_Rover.L[f] * wave_lengthj - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.L[f] * wave_lengthj - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else) - (ambi * wave_lengthi - ambj * wave_lengthj);
            double factor = fabs(residual(i, 0)) < 1.0 ? 1.0 : 10.0;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second) = wave_lengthj;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second) = -wave_lengthi;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 0) *
                factor;

            cout << "wave_lengthj: " << wave_lengthj << " "
                << "wave_lengthi: " << wave_lengthi << " "
                << "L_rover " << Info_Else.Mea_Rover.L[f] << " "
                << "L_base " << Info_Else.Mea_Base.L[f] << " "
                << residual(i, 0) << "  " << R(i, i) << std::endl;
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
        if (H2)
        {
            H2->resize(residual_size, Amb.size());
            *H2 = llt * H_amb;
        }
        return residual;
    }
};

class GNSSDDCpFactorCompress_ENU_lever : public NoiseModelFactor3<Pose3, Vector, Vector3>
{
private:
    const prcopt_t opt;
    const rtklib::GNSS_Info_SD Info_Master;
    const std::vector<rtklib::GNSS_Info_SD> Infos_Else;
    int residual_size;
    const int f;
    const bool UnitWeight;
    Point3 lla_origin;
    const map<int, int> ar_index;

public:
    GNSSDDCpFactorCompress_ENU_lever() = delete;

    GNSSDDCpFactorCompress_ENU_lever(Key key1, Key key2, Key key3, const rtklib::GNSS_Info_SD& Info_Master, const std::vector<rtklib::GNSS_Info_SD>& Infos_Else, const map<int, int>& _ar_index,
        const prcopt_t _opt, const int _f, const bool UnitWeight, const Point3& _lla_origin, const SharedNoiseModel& model)
        : NoiseModelFactor3<Pose3, Vector, Vector3>(model, key1, key2, key3),
        opt(_opt), Info_Master(Info_Master), Infos_Else(Infos_Else),
        f(_f), UnitWeight(UnitWeight), lla_origin(_lla_origin), ar_index(_ar_index)
    {
        residual_size = Infos_Else.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSDDCpFactorCompress_ENU_lever(*this)));
    }

    Vector evaluateError(const Pose3& RxPos, const Vector& Amb, const Vector3& lb,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos;
        Eigen::Matrix<double, 3, 3> H_lb;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon)* cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon)* cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d ecef;
        //        ecef = r * (RxPos.translation(H_pos) + RxPos.rotation(H_pos) * lb) + t;

        ecef = r * RxPos.transformFrom(lb, H_pos, H_lb) + t;

        Eigen::Matrix<double, Dynamic, 3> H = Eigen::Matrix<double, Dynamic, 3>::Zero(residual_size, 3);
        Eigen::Matrix<double, Dynamic, Dynamic> H_amb = Eigen::Matrix<double, Dynamic, Dynamic>::Zero(residual_size, Amb.size());

        H.setZero();
        H_amb.setZero();

        VectorXd residual = VectorXd::Zero(residual_size);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Eigen::Vector3d BasePos(opt.rb[0], opt.rb[1], opt.rb[2]);
        double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
        double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
        double bl = (BasePos - ecef).norm();
        double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1], Info_Master.Mea_Rover.SNR[f] * SNR_UNIT, Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
            bl, dt, f, &opt, &(Info_Master.Mea_Rover), 0);

        R.setConstant(pr_vari);

        Eigen::MatrixXd llt(residual_size, residual_size);

        // Master指参考星
        double Est_Geodist_Rover_Master;
        double Est_Geodist_Base_Master;
        Est_Geodist_Rover_Master = GNSS_Tools::CalGeodist(ecef, Info_Master, TYPE_Rover_Master); // 计算卫地距
        Est_Geodist_Base_Master = GNSS_Tools::CalGeodist(BasePos, Info_Master, TYPE_Base_Master);
        double H_Rover_Master[3];
        GNSS_Tools::CalJacobian(ecef, Info_Master, TYPE_Rover_Master, H_Rover_Master);

        double wave_lengthi = Info_Master.ssat.lam[f];
        double V_Rover_Master = Info_Master.Mea_Rover.L[f] * wave_lengthi - Est_Geodist_Rover_Master;
        double V_Base_Master = Info_Master.Mea_Base.L[f] * wave_lengthi - Est_Geodist_Base_Master;

        double ambi = Amb[ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second];
        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

            //            double factor=Info_Else.ssat.outlier_obs[1][f]?1e10:1.0;

            double Est_Geodist_Rover_Else;
            double Est_Geodist_Base_Else;
            double ambj = Amb[ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second];

            Est_Geodist_Rover_Else = GNSS_Tools::CalGeodist(ecef, Info_Else, TYPE_Rover_Else);
            Est_Geodist_Base_Else = GNSS_Tools::CalGeodist(BasePos, Info_Else, TYPE_Base_Else);

            double H_Rover_Else[3];
            GNSS_Tools::CalJacobian(ecef, Info_Else, TYPE_Rover_Else, H_Rover_Else);
            for (int j = 0; j < 3; j++)
                H(i, j) = H_Rover_Else[j];
            for (int j = 0; j < 3; j++)
                H(i, j) -= H_Rover_Master[j];

            double wave_lengthj = Info_Else.ssat.lam[f];
            double V_Rover_Else = Info_Else.Mea_Rover.L[f] * wave_lengthj - Est_Geodist_Rover_Else;
            double V_Base_Else = Info_Else.Mea_Base.L[f] * wave_lengthj - Est_Geodist_Base_Else;

            residual(i, 0) = (V_Rover_Master - V_Base_Master) - (V_Rover_Else - V_Base_Else) - (ambi * wave_lengthi - ambj * wave_lengthj);
            //            H_ambj(i,i)=wave_lengthj;
            //            H_ambi(i,0)=-wave_lengthi;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Else.Mea_Rover.sat)->second) = wave_lengthj;
            H_amb(i, ar_index.find(MAXSAT * f + Info_Master.Mea_Rover.sat)->second) = -wave_lengthi;
            //            R(i,i)+=varerr_sdobs(Info_Else.ssat.sys,Info_Else.ssat.azel[1],Info_Else.Mea_Rover.SNR[f]*SNR_UNIT,Info_Else.Mea_Base.SNR[f]*SNR_UNIT,
            //                                 bl,rb_dt,f,&opt,&(Info_Else.Mea_Rover),1) * factor;
            R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea_Rover.SNR[f] * SNR_UNIT, Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                bl, dt, f, &opt, &(Info_Else.Mea_Rover), 0);

            //            cout << "wave_lengthj: " << wave_lengthj << " "
            //            << "wave_lengthi: " << wave_lengthi << " "
            //            << "L_rover " << Info_Else.Mea_Rover.L[f] << " "
            //            << "L_base " << Info_Else.Mea_Base.L[f] << " "
            //            << residual(i,0) << "  " << R(i,i) << std::endl;
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
            H2->resize(residual_size, Amb.size());
            *H2 = llt * H_amb;
            //            H2->block(0,0,residual_size,1) = llt * H_ambi;
            //            H2->block(0,1,residual_size,residual_size) = llt * H_ambj;
        }
        if (H3)
        {
            H3->resize(residual_size, 3);
            *H3 = llt * H * r * H_lb;
        }
        return residual;
    }
};

class GNSSAmbConstraint : public NoiseModelFactor2<Vector, Vector>
{
private:
    vector<int> sat;
    vector<int> f;
    double delta_amb;
    int residual_size;

public:
    GNSSAmbConstraint() = delete;

    GNSSAmbConstraint(Key key1, Key key2, const double delta_amb_, const vector<int>& sat_, const vector<int>& _f,
        const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector, Vector>(model, key1, key2), sat(sat_), f(_f), delta_amb(delta_amb_)
    {
        residual_size = sat.size();
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSAmbConstraint(*this)));
    }

    Vector evaluateError(const Vector& Amb_k, const Vector& Amb_k_1,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {
        //        MatrixXd H_k(1,Amb_k.size());
        //        MatrixXd H_k_1(1,Amb_k.size());

        Eigen::SparseMatrix<double> H_k(residual_size, Amb_k.size());
        Eigen::SparseMatrix<double> H_k_1(residual_size, Amb_k.size());
        Vector d_amb(residual_size);
        H_k.reserve(residual_size);
        H_k_1.reserve(residual_size);
        //        H_k.setZero();
        //        H_k_1.setZero();
        //        H_k(0,MAXSAT*f+sat-1) = 1;
        //        H_k_1(0,MAXSAT*f+sat-1) = -1;

        for (int i = 0; i < residual_size; i++)
        {
            H_k.insert(i, MAXSAT * f[i] + sat[i] - 1) = 1;
            H_k_1.insert(i, MAXSAT * f[i] + sat[i] - 1) = -1;
            d_amb(i) = Amb_k(MAXSAT * f[i] + sat[i] - 1) - Amb_k_1(MAXSAT * f[i] + sat[i] - 1) - delta_amb;
        }

        if (H1)
        {
            H1->resize(residual_size, Amb_k.size());
            *H1 = H_k;
        }

        if (H2)
        {
            H2->resize(residual_size, Amb_k_1.size());
            *H2 = H_k_1;
        }

        return d_amb;
    };
};

class GNSSAmbConstraintCompress : public NoiseModelFactor2<Vector, Vector>
{
private:
    vector<int> sat;// 卫星编号
    vector<int> f;// 频率编号
    double delta_amb;// 模型值
    int residual_size;// 约束数量
    map<int, int> ar_index;// 模糊度在状态向量中的索引
    map<int, int> last_ar_index;// 上一时刻模糊度在状态向量中的索引
    mutable bool debugInfo;// 调试信息输出开关

public:
    GNSSAmbConstraintCompress() = delete;

    GNSSAmbConstraintCompress(Key key1, Key key2, const double delta_amb_, const vector<int>& sat_, const vector<int>& _f,
        const map<int, int>& _ar_index, const map<int, int>& _last_ar_index,
        const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector, Vector>(model, key1, key2), sat(sat_), f(_f), delta_amb(delta_amb_),
        ar_index(_ar_index), last_ar_index(_last_ar_index)
    {
        residual_size = sat.size();
        debugInfo = true;
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSAmbConstraintCompress(*this)));
    }

    Vector evaluateError(const Vector& Amb_k, const Vector& Amb_k_1,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {
        //        MatrixXd H_k(1,Amb_k.size());
        //        MatrixXd H_k_1(1,Amb_k.size());

        Eigen::SparseMatrix<double> H_k(residual_size, Amb_k.size());
        Eigen::SparseMatrix<double> H_k_1(residual_size, Amb_k_1.size());
        Vector d_amb(residual_size);
        H_k.reserve(residual_size);
        H_k_1.reserve(residual_size);
        //        H_k.setZero();
        //        H_k_1.setZero();
        //        H_k(0,MAXSAT*f+sat-1) = 1;
        //        H_k_1(0,MAXSAT*f+sat-1) = -1;

        for (int i = 0; i < residual_size; i++)
        {
            H_k.insert(i, ar_index.find(MAXSAT * f[i] + sat[i])->second) = 1;
            H_k_1.insert(i, last_ar_index.find(MAXSAT * f[i] + sat[i])->second) = -1;
            d_amb(i) = Amb_k(ar_index.find(MAXSAT * f[i] + sat[i])->second) - Amb_k_1(last_ar_index.find(MAXSAT * f[i] + sat[i])->second) - delta_amb;
            if (debugInfo)
            {
                debugInfo = false;
                cout << d_amb(i) << " " << Amb_k(ar_index.find(MAXSAT * f[i] + sat[i])->second) << " " << Amb_k_1(last_ar_index.find(MAXSAT * f[i] + sat[i])->second) << endl;
            }
        }

        if (H1)
        {
            H1->resize(residual_size, Amb_k.size());
            *H1 = H_k;
        }
        //        cout << "H1\n" ;
        //        cout << *H1 << endl;
        if (H2)
        {
            H2->resize(residual_size, Amb_k_1.size());
            *H2 = H_k_1;
        }
        //        cout << "H2\n" ;
        //        cout << *H2 << endl;
        return d_amb;
    };
};

class GNSSAmbConstraintSingle : public NoiseModelFactor2<double, double>
{
private:
    double delta_amb;
    int residual_size;

public:
    GNSSAmbConstraintSingle() = delete;

    GNSSAmbConstraintSingle(Key key1, Key key2, const double delta_amb_, const SharedNoiseModel& model)
        : NoiseModelFactor2<double, double>(model, key1, key2), delta_amb(delta_amb_)
    {
        residual_size = 1;
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSAmbConstraintSingle(*this)));
    }

    Vector evaluateError(const double& Amb_k, const double& Amb_k_1,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {

        Eigen::MatrixXd H_k(residual_size, 1);
        Eigen::MatrixXd H_k_1(residual_size, 1);
        Vector d_amb(residual_size);

        for (int i = 0; i < residual_size; i++)
        {
            H_k(0, 0) = 1;
            H_k_1(0, 0) = -1;
            d_amb(i) = Amb_k - Amb_k_1 - delta_amb;
        }

        if (H1)
        {
            H1->resize(residual_size, 1);
            *H1 = H_k;
        }
        if (H2)
        {
            H2->resize(residual_size, 1);
            *H2 = H_k_1;
        }

        return d_amb;
    };
};

class GNSSFixConstraint : public NoiseModelFactor1<Vector>
{
private:
    int nb;
    MatrixXi ix;
    VectorXd bias;

public:
    GNSSFixConstraint() = delete;

    GNSSFixConstraint(Key key1, const int nb_, const MatrixXi& ix_, const VectorXd& bias_,
        const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector>(model, key1), nb(nb_), ix(ix_), bias(bias_)
    {
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSFixConstraint(*this)));
    }

    Vector evaluateError(const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {

        Eigen::SparseMatrix<double> H_da(nb, Amb.size());
        Vector d_amb(nb);
        H_da.reserve(nb * 2);

        for (int i = 0; i < nb; i++)
        {
            H_da.insert(i, ix(0, i)) = 1;
            H_da.insert(i, ix(1, i)) = -1;
            d_amb(i) = Amb(ix(0, i)) - Amb(ix(1, i)) - bias(i);
        }

        if (H1)
        {
            H1->resize(nb, Amb.size());
            *H1 = H_da;
        }
        //        cout << "H1\n" ;
        //        cout << *H1 << endl;

        return d_amb;
    };
};

class GNSSPhasePriorConstraint : public NoiseModelFactor1<Vector>
{
private:
    int sat;
    int f;
    double amb_prior;
    mutable bool debugInfo;

public:
    GNSSPhasePriorConstraint() = delete;

    GNSSPhasePriorConstraint(Key key1, const double& amb_prior_, const int& sat_, const int& f_,
        const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector>(model, key1), amb_prior(amb_prior_), sat(sat_), f(f_)
    {
        debugInfo = true;
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSPhasePriorConstraint(*this)));
    }

    Vector evaluateError(const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {
        MatrixXd H_a(1, Amb.size());
        Vector d_amb(1);
        H_a.setZero();
        H_a(0, MAXSAT * f + sat - 1) = 1;
        d_amb(0) = Amb(MAXSAT * f + sat - 1) - amb_prior;
        if (debugInfo)
        {
            debugInfo = false;
            cout << d_amb(0) << endl;
        }
        if (H1)
        {
            H1->resize(1, Amb.size());
            *H1 = H_a;
        }

        return d_amb;
    };
};

class GNSSPhasePriorConstraintCompress : public NoiseModelFactor1<Vector>
{
private:
    int index;
    double amb_prior;

public:
    GNSSPhasePriorConstraintCompress() = delete;

    GNSSPhasePriorConstraintCompress(Key key1, const double& amb_prior_, const int& _index,
        const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector>(model, key1), index(_index), amb_prior(amb_prior_)
    {
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSPhasePriorConstraintCompress(*this)));
    }

    Vector evaluateError(const Vector& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {
        MatrixXd H_a(1, Amb.size());
        Vector d_amb(1);
        H_a.setZero();
        H_a(0, index) = 1;
        d_amb(0) = Amb(index) - amb_prior;

        if (H1)
        {
            H1->resize(1, Amb.size());
            *H1 = H_a;
        }

        return d_amb;
    };
};

class GNSSPhasePriorConstraintSingle : public NoiseModelFactor1<double>
{
private:
    int sat;
    int f;
    double amb_prior;

public:
    GNSSPhasePriorConstraintSingle() = delete;

    GNSSPhasePriorConstraintSingle(Key key1, const double& amb_prior_, const int& sat_, const int& f_,
        const SharedNoiseModel& model)
        : NoiseModelFactor1<double>(model, key1), amb_prior(amb_prior_), sat(sat_), f(f_)
    {
    }

    virtual NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<NonlinearFactor>(NonlinearFactor::shared_ptr(new GNSSPhasePriorConstraintSingle(*this)));
    }

    Vector evaluateError(const double& Amb,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
    {
        MatrixXd H_a(1, 1);
        Vector d_amb(1);
        H_a(0, 0) = 1;
        d_amb(0) = Amb - amb_prior;

        if (H1)
        {
            H1->resize(1, 1);
            *H1 = H_a;
        }

        return d_amb;
    };
};
#endif // glins_CARRIERPHASEFACTOR_H
