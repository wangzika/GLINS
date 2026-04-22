//
// Created by wangchuji on 2023/11/21.
//

#ifndef GLINS_MOTIONFACTOR_H
#define GLINS_MOTIONFACTOR_H
#include "gtsam/base/Vector.h"
#include "gtsam/base/Matrix.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/nonlinear/ExpressionFactor.h"
#include "gtsam/geometry/Point3.h"
#include <gtsam/nonlinear/Symbol.h>
#include "rtklib/gnss_tools.h"
#include <queue>
#include <map>
#include "utility.h"

using namespace gtsam;
class GNSSCVFactor : public NoiseModelFactor3<Point3, Point3, Point3>
{
private:
    const rtklib::GNSS_Info_ZD Info_Master;
    const std::vector<rtklib::GNSS_Info_ZD> Infos_Else;
    const prcopt_t opt;
    const int f;
    const bool UnitWeight;
    int residual_size;
    double delta_t;
public:
    GNSSCVFactor(Key key1, Key key2, Key key3, const rtklib::GNSS_Info_ZD& _Info_Master, const std::vector<rtklib::GNSS_Info_ZD>& _Infos_Else,
        const prcopt_t _opt, const int _f, const double _delta_t,
        const bool _UnitWeight, const SharedNoiseModel& model) :
        NoiseModelFactor3<Point3, Point3, Point3>(model, key1, key2, key3),
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

    Vector evaluateError(const Point3& pre_RxPos, const Point3& cur_RxPos,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none) const
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

        for (int i = 0;i < residual_size;i++)
        {
            rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);
            double Est_Dop_Else = GNSS_Tools::CalRate(cur_RxPos, RxVel, Info_Else.Sat, 0.0);
            double H_Else[3];
            GNSS_Tools::CalVisionVector(cur_RxPos, Info_Else.Sat, H_Else);
            double WaveLenj = Info_Else.ssat.lam[f];
            double Mea_Dop_Else = -(Info_Else.Mea.D[f] * WaveLenj);
            double V_Else = Est_Dop_Else - Mea_Dop_Else;
            residual(i, 0) = V_Master - V_Else;
            for (int j = 0;j < 3;j++)
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

#endif //GLINS_MOTIONFACTOR_H
