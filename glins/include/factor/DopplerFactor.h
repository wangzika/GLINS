//
// Created by wangchuji on 2023/2/24.
//
#pragma once
#ifndef LIO_SAM_DOPPLERFACTOR_H
#define LIO_SAM_DOPPLERFACTOR_H
#include "gtsam/base/Vector.h"
#include "gtsam/base/Matrix.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/geometry/Point3.h"
#include <gtsam/nonlinear/Symbol.h>
#include "gnss_tools.h"
#include "utility.h"
using namespace gtsam;
#if 0
class DopplerFactor : public NoiseModelFactor2<Point3,double>{
private:
    Point3 SatPos;
    Point3 SatVel;
    Point3 Pos;
    double Doppler;
    double SatClkDrift;
    typedef NoiseModelFactor2<Point3,double> Base;
    typedef DopplerFactor This;

public:
    DopplerFactor();

    DopplerFactor(Key key1, Key key2,rtklib::GNSS_Raw obs, Point3 Pos,const SharedNoiseModel& model):
            Base(model,key1,key2),Pos(Pos){
        SatPos=Point3(obs.sat_pos_x,obs.sat_pos_y,obs.sat_pos_z);
        SatVel=Point3(obs.sat_vel_x,obs.sat_vel_y,obs.sat_vel_z);
        Doppler=-obs.doppler*obs.lamda;
        SatClkDrift=obs.sat_clk_drift;
        cout << "add dopplerFactor:" << Symbol(key1) << Symbol(key2) << endl;
    }

    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                gtsam::NonlinearFactor::shared_ptr(new DopplerFactor(*this)));
    }

    Vector1 computeErrorAndJacobians(const Point3& Vel,
                                     const double& ClockDrift,
                                     OptionalJacobian<1, 3> H1,
                                     OptionalJacobian<1, 1> H2) const {
        double Geodist;
        double delta_x = Pos[0] - SatPos[0];
        double delta_y = Pos[1] - SatPos[1];
        double delta_z = Pos[2] - SatPos[2];
        Geodist = sqrt(pow(delta_x,2)+ pow(delta_y,2) + pow(delta_z,2));

        if(H1)
        {
            *H1<<delta_x/Geodist,delta_y/Geodist,delta_z/Geodist;
//                printf("H：%lf %lf %lf\n",delta_x/Geodist,delta_y/Geodist,delta_z/Geodist);
        }
        if(H2)
        {
            *H2<<1.0;
        }

        const double OMGE_ = 7.2921151467E-5;
        const double CLIGHT_ = 299792458.0;

        Vector3 RelativeVel(Vel[0]-SatVel[0],Vel[1]-SatVel[1],Vel[2]-SatVel[2]);
        Vector3 e(delta_x/Geodist,delta_y/Geodist,delta_z/Geodist);

        double Rate=RelativeVel.dot(e)-OMGE_*(SatVel[1]*Pos[0]+SatPos[1]*Vel[0]-
                                              SatVel[0]*Pos[1]-SatPos[0]*Vel[1])/CLIGHT_;

        double EstimatedValue=Rate+ClockDrift-SatClkDrift;

        Vector1 vec;
        vec<<EstimatedValue-Doppler;

//        printf("err:%lf %lf %lf\n",Doppler,EstimatedValue,vec[0]);

        return vec;
    }

    Vector evaluateError(const Point3& rr,
                         const double& clk_bias,
                         boost::optional<gtsam::Matrix&> H1 = boost::none,
                         boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {
        return computeErrorAndJacobians(rr,clk_bias,H1,H2);
    }



};  // \ RangeFactorWithTransform
#endif
class GNSSSDDopFactor : public NoiseModelFactor2<Point3, Point3>
{
private:
    const rtklib::GNSS_Info_ZD Info_Master;
    const std::vector<rtklib::GNSS_Info_ZD> Infos_Else;
    const prcopt_t opt;
    const int f;
    double delta_t;
    const bool UnitWeight;
    int residual_size;

public:
    GNSSSDDopFactor(Key key1, Key key2, const rtklib::GNSS_Info_ZD &_Info_Master, const std::vector<rtklib::GNSS_Info_ZD> &_Infos_Else,
                    const prcopt_t _opt, const int _f, const double _delta_t,
                    const bool _UnitWeight, const SharedNoiseModel &model) : NoiseModelFactor2<Point3, Point3>(model, key1, key2),
                                                                             Info_Master(_Info_Master), Infos_Else(_Infos_Else),
                                                                             opt(_opt), f(_f), delta_t(_delta_t), UnitWeight(_UnitWeight)
    {
        residual_size = Infos_Else.size();
    }

    virtual gtsam::NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new GNSSSDDopFactor(*this)));
    }

    Vector evaluateError(const Point3 &pre_RxPos, const Point3 &cur_RxPos,
                         boost::optional<gtsam::Matrix &> H1 = boost::none,
                         boost::optional<gtsam::Matrix &> H2 = boost::none) const
    {
        Eigen::MatrixXd H_vel(residual_size, 3);
        Eigen::MatrixXd residual(residual_size, 1);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Vector3 RxVel = (cur_RxPos - pre_RxPos) / delta_t;

        R.setConstant(GNSS_Tools::varerr_zdobs(Info_Master.ssat.sys, Info_Master.ssat.azel[1], Info_Master.Mea.SNR[f] * SNR_UNIT,
                                               f, &opt, &(Info_Master.Mea), 2));

        Eigen::MatrixXd llt(residual_size, residual_size);

        double Est_Dop_Master = GNSS_Tools::CalRate(cur_RxPos, RxVel, Info_Master.Sat, 0.0);
        double H_Master[3];
        GNSS_Tools::CalVisionVector(cur_RxPos, Info_Master.Sat, H_Master);
        double WaveLeni = Info_Master.ssat.lam[f];
        double Mea_Dop_Master = -(Info_Master.Mea.D[f] * WaveLeni);
        double V_Master = Est_Dop_Master - Mea_Dop_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);
            double Est_Dop_Else = GNSS_Tools::CalRate(cur_RxPos, RxVel, Info_Else.Sat, 0.0);
            double H_Else[3];
            GNSS_Tools::CalVisionVector(cur_RxPos, Info_Else.Sat, H_Else);
            double WaveLenj = Info_Else.ssat.lam[f];
            double Mea_Dop_Else = -(Info_Else.Mea.D[f] * WaveLenj);
            double V_Else = Est_Dop_Else - Mea_Dop_Else;
            residual(i, 0) = V_Master - V_Else;
            for (int j = 0; j < 3; j++)
                H_vel(i, j) = H_Master[j] - H_Else[j];

            R(i, i) += GNSS_Tools::varerr_zdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea.SNR[f] * SNR_UNIT,
                                                f, &opt, &(Info_Else.Mea), 2);
            //            cout << residual(i,0) << "  " << R(i,i) << std::endl;
        }
        if (UnitWeight)
            R.setIdentity();

        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;

        if (H1)
        {
            H1->resize(residual_size, 3);
            Matrix3 H_pre = -(1.0 / delta_t) * Matrix3::Identity();
            *H1 = llt * H_vel * H_pre;
        }
        if (H2)
        {
            H2->resize(residual_size, 3);
            Matrix3 H_cur = (1.0 / delta_t) * Matrix3::Identity();
            *H2 = llt * H_vel * H_cur;
        }

        return residual;
    }
};

class GNSSSDDopFactor_ENU : public NoiseModelFactor2<Pose3, Pose3>
{
private:
    const rtklib::GNSS_Info_ZD Info_Master;
    const std::vector<rtklib::GNSS_Info_ZD> Infos_Else;
    const prcopt_t opt;
    const int f;
    const bool UnitWeight;
    int residual_size;
    double delta_t;
    Point3 lla_origin;
    Vector3 lb_;

public:
    GNSSSDDopFactor_ENU(Key key1, Key key2, const rtklib::GNSS_Info_ZD &_Info_Master, const std::vector<rtklib::GNSS_Info_ZD> &_Infos_Else,
                        const prcopt_t _opt, const int _f, const double _delta_t,
                        const bool _UnitWeight, const Point3 _lla_origin, const Vector3 lb, const SharedNoiseModel &model) : NoiseModelFactor2<Pose3, Pose3>(model, key1, key2),
                                                                                                                             Info_Master(_Info_Master), Infos_Else(_Infos_Else),
                                                                                                                             opt(_opt), f(_f), delta_t(_delta_t), UnitWeight(_UnitWeight), lla_origin(_lla_origin), lb_(lb)
    {
        residual_size = Infos_Else.size();
    }

    virtual gtsam::NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new GNSSSDDopFactor_ENU(*this)));
    }

    Vector evaluateError(const Pose3 &pre_RxPos, const Pose3 &cur_RxPos,
                         boost::optional<gtsam::Matrix &> H1 = boost::none,
                         boost::optional<gtsam::Matrix &> H2 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos_pre, H_pos_cur;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon) * cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d pre_ecef, cur_ecef;
        pre_ecef = r * pre_RxPos.transformFrom(lb_, H_pos_pre) + t;
        cur_ecef = r * cur_RxPos.transformFrom(lb_, H_pos_cur) + t;

        Eigen::MatrixXd H_vel(residual_size, 3);
        Eigen::MatrixXd residual(residual_size, 1);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Vector3 RxVel = (cur_ecef - pre_ecef) / delta_t;

        R.setConstant(GNSS_Tools::varerr_zdobs(Info_Master.ssat.sys, Info_Master.ssat.azel[1], Info_Master.Mea.SNR[f] * SNR_UNIT,
                                               f, &opt, &(Info_Master.Mea), 2));

        Eigen::MatrixXd llt(residual_size, residual_size);

        double Est_Dop_Master = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Master.Sat, 0.0);
        double H_Master[3];
        GNSS_Tools::CalVisionVector(cur_ecef, Info_Master.Sat, H_Master);
        double WaveLeni = Info_Master.ssat.lam[f];
        double Mea_Dop_Master = -(Info_Master.Mea.D[f] * WaveLeni);
        double V_Master = Est_Dop_Master - Mea_Dop_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);
            double Est_Dop_Else = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Else.Sat, 0.0);
            double H_Else[3];
            GNSS_Tools::CalVisionVector(cur_ecef, Info_Else.Sat, H_Else);
            double WaveLenj = Info_Else.ssat.lam[f];
            double Mea_Dop_Else = -(Info_Else.Mea.D[f] * WaveLenj);
            double V_Else = Est_Dop_Else - Mea_Dop_Else;
            residual(i, 0) = V_Master - V_Else;
            for (int j = 0; j < 3; j++)
                H_vel(i, j) = H_Master[j] - H_Else[j];

            R(i, i) += GNSS_Tools::varerr_zdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea.SNR[f] * SNR_UNIT,
                                                f, &opt, &(Info_Else.Mea), 2);
        }
        if (UnitWeight)
            R.setIdentity();

        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;

        if (H1)
        {
            H1->resize(residual_size, 6);
            Matrix3 H_pre = -(1.0 / delta_t) * Matrix3::Identity();
            *H1 = llt * H_vel * H_pre * (-1) * r * H_pos_pre;
        }
        if (H2)
        {
            H2->resize(residual_size, 6);
            Matrix3 H_cur = (1.0 / delta_t) * Matrix3::Identity();
            *H2 = llt * H_vel * H_cur * (-1) * r * H_pos_cur;
        }

        return residual;
    }

    Vector debugEvaluateError(const Pose3 &pre_RxPos, const Pose3 &cur_RxPos,
                              boost::optional<gtsam::Matrix &> H1 = boost::none,
                              boost::optional<gtsam::Matrix &> H2 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos_pre, H_pos_cur;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon) * cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d pre_ecef, cur_ecef;
        pre_ecef = r * pre_RxPos.transformFrom(lb_, H_pos_pre) + t;
        cur_ecef = r * cur_RxPos.transformFrom(lb_, H_pos_cur) + t;

        Eigen::MatrixXd H_vel(residual_size, 3);
        Eigen::MatrixXd residual(residual_size, 1);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        Vector3 RxVel = (cur_ecef - pre_ecef) / delta_t;

        R.setConstant(GNSS_Tools::varerr_zdobs(Info_Master.ssat.sys, Info_Master.ssat.azel[1], Info_Master.Mea.SNR[f] * SNR_UNIT,
                                               f, &opt, &(Info_Master.Mea), 2));

        Eigen::MatrixXd llt(residual_size, residual_size);

        double Est_Dop_Master = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Master.Sat, 0.0);
        double H_Master[3];
        GNSS_Tools::CalVisionVector(cur_ecef, Info_Master.Sat, H_Master);
        double WaveLeni = Info_Master.ssat.lam[f];
        double Mea_Dop_Master = -(Info_Master.Mea.D[f] * WaveLeni);
        double V_Master = Est_Dop_Master - Mea_Dop_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);
            double Est_Dop_Else = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Else.Sat, 0.0);
            double H_Else[3];
            GNSS_Tools::CalVisionVector(cur_ecef, Info_Else.Sat, H_Else);
            double WaveLenj = Info_Else.ssat.lam[f];
            double Mea_Dop_Else = -(Info_Else.Mea.D[f] * WaveLenj);
            double V_Else = Est_Dop_Else - Mea_Dop_Else;
            residual(i, 0) = V_Master - V_Else;
            for (int j = 0; j < 3; j++)
                H_vel(i, j) = H_Master[j] - H_Else[j];

            R(i, i) += GNSS_Tools::varerr_zdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea.SNR[f] * SNR_UNIT,
                                                f, &opt, &(Info_Else.Mea), 2);

            cout << residual(i, 0) << " " << R(i, i) << endl;
        }
        if (UnitWeight)
            R.setIdentity();

        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;

        if (H1)
        {
            H1->resize(residual_size, 6);
            Matrix3 H_pre = -(1.0 / delta_t) * Matrix3::Identity();
            *H1 = llt * H_vel * H_pre * (-1) * r * H_pos_pre;
        }
        if (H2)
        {
            H2->resize(residual_size, 6);
            Matrix3 H_cur = (1.0 / delta_t) * Matrix3::Identity();
            *H2 = llt * H_vel * H_cur * (-1) * r * H_pos_cur;
        }

        return residual;
    }
};

class GNSSSDDopFactor_ENUVel : public NoiseModelFactor2<Vector3, Pose3>
{
private:
    const rtklib::GNSS_Info_ZD Info_Master;
    const std::vector<rtklib::GNSS_Info_ZD> Infos_Else;
    const prcopt_t opt;
    const int f;
    const bool UnitWeight;
    int residual_size;
    Point3 lla_origin;
    Vector3 lb_;

public:
    GNSSSDDopFactor_ENUVel(Key key1, Key key2, const rtklib::GNSS_Info_ZD &_Info_Master, const std::vector<rtklib::GNSS_Info_ZD> &_Infos_Else,
                           const prcopt_t _opt, const int _f,
                           const bool _UnitWeight, const Point3 _lla_origin, const Vector3 lb, const SharedNoiseModel &model) : NoiseModelFactor2<Vector3, Pose3>(model, key1, key2),
                                                                                                                                Info_Master(_Info_Master), Infos_Else(_Infos_Else),
                                                                                                                                opt(_opt), f(_f), UnitWeight(_UnitWeight), lla_origin(_lla_origin), lb_(lb)
    {
        residual_size = Infos_Else.size();
    }

    virtual gtsam::NonlinearFactor::shared_ptr clone() const
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new GNSSSDDopFactor_ENUVel(*this)));
    }

    Vector evaluateError(const Vector3 &RxVel, const Pose3 &RxPos,
                         boost::optional<gtsam::Matrix &> H1 = boost::none,
                         boost::optional<gtsam::Matrix &> H2 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos_pre, H_pos_cur;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon) * cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d cur_ecef;
        cur_ecef = r * RxPos.transformFrom(lb_, H_pos_cur) + t;

        Eigen::MatrixXd H_vel(residual_size, 3);
        Eigen::MatrixXd residual(residual_size, 1);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        R.setConstant(GNSS_Tools::varerr_zdobs(Info_Master.ssat.sys, Info_Master.ssat.azel[1], Info_Master.Mea.SNR[f] * SNR_UNIT,
                                               f, &opt, &(Info_Master.Mea), 2));

        Eigen::MatrixXd llt(residual_size, residual_size);

        double Est_Dop_Master = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Master.Sat, 0.0);
        double H_Master[3];
        GNSS_Tools::CalVisionVector(cur_ecef, Info_Master.Sat, H_Master);
        double WaveLeni = Info_Master.ssat.lam[f];
        double Mea_Dop_Master = -(Info_Master.Mea.D[f] * WaveLeni);
        double V_Master = Est_Dop_Master - Mea_Dop_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);
            double Est_Dop_Else = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Else.Sat, 0.0);
            double H_Else[3];
            GNSS_Tools::CalVisionVector(cur_ecef, Info_Else.Sat, H_Else);
            double WaveLenj = Info_Else.ssat.lam[f];
            double Mea_Dop_Else = -(Info_Else.Mea.D[f] * WaveLenj);
            double V_Else = Est_Dop_Else - Mea_Dop_Else;
            residual(i, 0) = V_Master - V_Else;
            for (int j = 0; j < 3; j++)
                H_vel(i, j) = H_Master[j] - H_Else[j];

            R(i, i) += GNSS_Tools::varerr_zdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea.SNR[f] * SNR_UNIT,
                                                f, &opt, &(Info_Else.Mea), 2);
        }
        if (UnitWeight)
            R.setIdentity();

        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;

        if (H1)
        {
            H1->resize(residual_size, 3);
            *H1 = llt * H_vel * r;
        }
        if (H2)
        {
            H2->resize(residual_size, 6);
            H2->setZero();
        }

        return residual;
    }

    Vector debugEvaluateError(const Vector3 &RxVel, const Pose3 &RxPos,
                              boost::optional<gtsam::Matrix &> H1 = boost::none,
                              boost::optional<gtsam::Matrix &> H2 = boost::none) const
    {

        Eigen::Matrix<double, 3, 6> H_pos_pre, H_pos_cur;

        double lat = DEG2RAD(lla_origin.x());
        double lon = DEG2RAD(lla_origin.y());

        Eigen::Vector3d t;
        t = GNSS_Tools::llh2ecef(lla_origin);
        Eigen::Matrix3d r;
        r << -sin(lon), -cos(lon) * sin(lat), cos(lon) * cos(lat),
            cos(lon), -sin(lon) * sin(lat), sin(lon) * cos(lat),
            0, cos(lat), sin(lat);
        Eigen::Vector3d cur_ecef;
        cur_ecef = r * RxPos.transformFrom(lb_, H_pos_cur) + t;

        Eigen::MatrixXd H_vel(residual_size, 3);
        Eigen::MatrixXd residual(residual_size, 1);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(residual_size, residual_size);

        R.setConstant(GNSS_Tools::varerr_zdobs(Info_Master.ssat.sys, Info_Master.ssat.azel[1], Info_Master.Mea.SNR[f] * SNR_UNIT,
                                               f, &opt, &(Info_Master.Mea), 2));

        Eigen::MatrixXd llt(residual_size, residual_size);

        double Est_Dop_Master = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Master.Sat, 0.0);
        double H_Master[3];
        GNSS_Tools::CalVisionVector(cur_ecef, Info_Master.Sat, H_Master);
        double WaveLeni = Info_Master.ssat.lam[f];
        double Mea_Dop_Master = -(Info_Master.Mea.D[f] * WaveLeni);
        double V_Master = Est_Dop_Master - Mea_Dop_Master;

        for (int i = 0; i < residual_size; i++)
        {
            rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);
            double Est_Dop_Else = GNSS_Tools::CalRate(cur_ecef, RxVel, Info_Else.Sat, 0.0);
            double H_Else[3];
            GNSS_Tools::CalVisionVector(cur_ecef, Info_Else.Sat, H_Else);
            double WaveLenj = Info_Else.ssat.lam[f];
            double Mea_Dop_Else = -(Info_Else.Mea.D[f] * WaveLenj);
            double V_Else = Est_Dop_Else - Mea_Dop_Else;
            residual(i, 0) = V_Master - V_Else;
            for (int j = 0; j < 3; j++)
                H_vel(i, j) = H_Master[j] - H_Else[j];

            R(i, i) += GNSS_Tools::varerr_zdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1], Info_Else.Mea.SNR[f] * SNR_UNIT,
                                                f, &opt, &(Info_Else.Mea), 2);
            cout << residual(i, 0) << " " << R(i, i) << endl;
        }
        if (UnitWeight)
            R.setIdentity();

        llt = R.inverse().llt().matrixL().transpose();

        residual = llt * residual;

        if (H1)
        {
            H1->resize(residual_size, 3);
            *H1 = llt * H_vel * r;
        }
        if (H2)
        {
            H2->resize(residual_size, 6);
            H2->setZero();
        }

        return residual;
    }
};

#endif // LIO_SAM_DOPPLERFACTOR_H
