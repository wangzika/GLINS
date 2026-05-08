//
// Created by wangchuji on 2023/12/20.
//
#pragma once

#include <iostream>
#include <string>
#include "ros/ros.h"

#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "factor/PseudorangeFactor.h"
#include "factor/DopplerFactor.h"
#include "factor/CarrierPhaseFactor.h"
#include "factor/GnssFactor.h"

#ifndef GLINS_GNSSCONTAINER_H
#define GLINS_GNSSCONTAINER_H

using namespace std;
using namespace ros;
using namespace gtsam;
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::N;
using gtsam::symbol_shorthand::T;
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

typedef pair<nav_msgs::Odometry, rtklib::GNSS_Info> GNSSPose;

static int test_sys(int sys, int m)
{
    switch (sys)
    {
    case SYS_GPS:
        return m == 0;
    case SYS_SBS:
        return m == 0;
    case SYS_GLO:
        return m == 1;
    case SYS_GAL:
        return m == 2;
    case SYS_CMP:
        return m == 3;
    case SYS_QZS:
        return m == 4;
    case SYS_IRN:
        return m == 5;
    }
    return 0;
}

class gnssContainer
{
private:
    std::deque<GNSSPose> gnss_queue;
    rtklib::GNSS_Info gnss_info;
    nav_msgs::Odometry gnss_odom;
    GNSS_Tools gnssTools;

    ros::Subscriber sub_gnss_raw;

    Vector3 lla_origin;
    Eigen::Vector3d extGPS;
    Eigen::Matrix3d extRot;

    Vector prevAmb;
    Vector AmbFullArray;
    MatrixXd ambCovariance;

    int epoch = 0;

    prcopt_t prcopt;
    solopt_t solopt;
    filopt_t filopt = { "" };
    double pre_gpstime[2];

    rtk_t rtk;

    bool init_origin;
    bool systemInitialized;
    bool estimateExtGPS;

    nav_msgs::Path gpsOdomPath;

    std::mutex mtxGpsInfo;

    map<int, int> last_ar_index;

    MatrixXi ix;

    VectorXd bias;

    bool useGroundTruthPos;
    vector<double> GroundTruthPosV;
    Vector3 GroundTruthPos;

public:
    double tt = 0;

    gnssContainer()
    {
        init_origin = false;
        prcopt = prcopt_default;
        solopt = solopt_default;
        bool systemInitialized_ = false;
        bool estimateExtGPS_ = false;
        getsysopts(&prcopt, &solopt, &filopt);

        Vector3 ecef_ref(prcopt.rb[0], prcopt.rb[1], prcopt.rb[2]);

        rtkinit(&rtk, &prcopt);
        prevAmb = VectorXd(NB(&prcopt)).setZero();
        AmbFullArray = VectorXd(NB(&prcopt)).setZero();
        gnss_queue.clear();
    }

    gnssContainer(const string& rtklibConfigPath, NodeHandle& nh, bool systemInitialized_ = false, bool estimateExtGPS_ = false) : systemInitialized(systemInitialized_), estimateExtGPS(estimateExtGPS_)
    {
        init_origin = false;
        prcopt = prcopt_default;
        solopt = solopt_default;
        if (!loadopts(rtklibConfigPath.c_str(), sysopts))
        {
            exit(1);
        }
        getsysopts(&prcopt, &solopt, &filopt);

        /*subscriber*/
        sub_gnss_raw = nh.subscribe<rtklib::GNSS_Info>("gnss_raw", 100, &gnssContainer::gnssInfoHandler, this,
            ros::TransportHints().tcpNoDelay());

        nh.param<bool>("glins/useGroundTruthPos", useGroundTruthPos, false);
        nh.param<vector<double>>("glins/GroundTruthPos", GroundTruthPosV, vector<double>(3, 0));
        GroundTruthPos = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(GroundTruthPosV.data(), 3, 1);
        Vector3 ecef_ref(prcopt.rb[0], prcopt.rb[1], prcopt.rb[2]);

        rtkinit(&rtk, &prcopt);
        prevAmb = VectorXd(NB(&prcopt)).setZero();
        AmbFullArray = VectorXd(NB(&prcopt)).setZero();
        gnss_queue.clear();
    }

    void reset_last_ar_index()
    {
        last_ar_index.clear();
    }
    void setSystemInitialized(bool initialized)
    {
        systemInitialized = initialized;
    }

    void setEstimateExtGPS(bool estimated)
    {
        estimateExtGPS = estimated;
    }

    void setExtRot(Matrix3d& ext)
    {
        extRot = ext;
    }

    void setExtGPS(Vector3d& ext)
    {
        extGPS = ext;
    }

    double getSolRatio() const
    {
        return rtk.sol.ratio;
    }

    Vector3 getOrigin() const
    {
        return lla_origin;
    }
    void loadrtklibConfig(const string& rtklibConfigPath)
    {
        if (!loadopts(rtklibConfigPath.c_str(), sysopts))
        {
            exit(1);
        }
        getsysopts(&prcopt, &solopt, &filopt);
        rtkinit(&rtk, &prcopt);
        prevAmb = VectorXd(NB(&prcopt)).setZero();
        AmbFullArray = VectorXd(NB(&prcopt)).setZero();
        ix = MatrixXi(2, NB(&prcopt));
        bias = VectorXd(NB(&prcopt));
    }

    void registerGnssSubscriber(NodeHandle& nh)
    {
        sub_gnss_raw = nh.subscribe<rtklib::GNSS_Info>("/gnss_raw", 100, &gnssContainer::gnssInfoHandler, this,
            ros::TransportHints().tcpNoDelay());
        nh.param<bool>("glins/useGroundTruthPos", useGroundTruthPos, false);
        nh.param<vector<double>>("glins/GroundTruthPos", GroundTruthPosV, vector<double>(3, 0));
        GroundTruthPos = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(GroundTruthPosV.data(), 3, 1);
        Vector3 ecef_ref(prcopt.rb[0], prcopt.rb[1], prcopt.rb[2]);
    }

    int addDDPsrFactorENU(gtsam::NonlinearFactorGraph* graphFactors, gtsam::Values* graphValues, int key)
    {
        int validSat[MAXSAT] = { 0 };
        int npr = 0, ns;
        ns = gnss_info.SD_Infos.size();
        for (int m = 0; m < 6; m++)
        {
            for (int f = 0; f < 3; f++)
            {
                int sysi, sysj;
                int i, j;
                for (i = -1, j = 0; j < ns; j++)
                {
                    rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                    sysi = ssatj.sys;
                    // if (!ssatj.vsatP[f])
                    //     continue;
                    if (!test_sys(sysi, m) || sysi == SYS_SBS)
                        continue;
                    if (gnss_info.SD_Infos[j].Mea_Rover.P[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.P[f] == 0.0 || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                        continue;
                    if (satexclude(gnss_info.SD_Infos[j].Mea_Rover.sat, ssatj.ephvar, ssatj.svh, &prcopt))
                        continue;
                    if (i >= 0 && gnss_info.SD_Infos[j].ssat.slip[f] & LLI_SLIP)
                        continue;
                    if (testsnr(0, f, ssatj.azel[1], gnss_info.SD_Infos[j].Mea_Rover.SNR[f] * SNR_UNIT, &prcopt.snrmask))
                        continue;
                    if (i < 0 || ssatj.azel[1] >= gnss_info.SD_Infos[i].ssat.azel[1])
                        i = j;
                }
                if (i < 0)
                    continue;

                rtklib::GNSS_Info_SD Info_Master = gnss_info.SD_Infos[i];
                vector<rtklib::GNSS_Info_SD> Infos_Else;
                for (j = 0; j < ns; j++)
                {
                    if (i == j)
                        continue; /* skip ref sat */
                    rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                    sysj = ssatj.sys;
                    // if (!ssatj.vsatP[f])
                    //     continue;
                    if (!test_sys(sysj, m))
                        continue;
                    if (gnss_info.SD_Infos[j].Mea_Rover.P[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.P[f] == 0.0 || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                        continue;
                    if (satexclude(gnss_info.SD_Infos[j].Mea_Rover.sat, ssatj.ephvar, ssatj.svh, &prcopt))
                        continue;
                    if (testsnr(0, f, ssatj.azel[1], gnss_info.SD_Infos[j].Mea_Rover.SNR[f] * SNR_UNIT, &prcopt.snrmask))
                        continue;
                    vector<rtklib::GNSS_Info_SD> tmpInfo;
                    tmpInfo.push_back(gnss_info.SD_Infos[j]);
                    noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(1, 1);
                    GNSSDDPsrFactor_ENU dd_psr_factor = GNSSDDPsrFactor_ENU(X(key), Info_Master, tmpInfo, prcopt,
                        f, true, lla_origin, extGPS, extRot.transpose(), noise);
                    double residual = dd_psr_factor.debugEvaluateError(graphValues->at<Pose3>(X(key)))[0];
                    if ((fabs(residual) > (15)) && tt < 2.0)
                    {
                        ROS_WARN("pseudorange residual out of range %.3lf", residual);
                        continue;
                    }
                    Infos_Else.push_back(gnss_info.SD_Infos[j]);
                    if (f == 0)
                        validSat[gnss_info.SD_Infos[j].Mea_Rover.sat - 1] = 1;
                }

                if (!Infos_Else.empty())
                {
                    npr += Infos_Else.size();
#if 0
                    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(Infos_Else.size(), Infos_Else.size());
                    Eigen::Vector3d BasePos(prcopt.rb[0], prcopt.rb[1], prcopt.rb[2]);
                    Eigen::Vector3d RxPos(gnss_info.pos[0], gnss_info.pos[1], gnss_info.pos[2]);
                    double azeli[2] = { Info_Master.ssat.azel[0], Info_Master.ssat.azel[1] };
                    double dt = Info_Master.Mea_Rover.time - Info_Master.Mea_Base.time;
                    double bl = (BasePos - RxPos).norm();
                    double pr_vari = GNSS_Tools::varerr_sdobs(Info_Master.ssat.sys, azeli[1],
                        Info_Master.Mea_Rover.SNR[f] * SNR_UNIT,
                        Info_Master.Mea_Base.SNR[f] * SNR_UNIT,
                        bl, dt, f, &prcopt, &(Info_Master.Mea_Rover), 1);
                    R.setConstant(pr_vari);
                    for (i = 0; i < Infos_Else.size(); i++)
                    {
                        rtklib::GNSS_Info_SD Info_Else = Infos_Else.at(i);

                        R(i, i) += GNSS_Tools::varerr_sdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1],
                            Info_Else.Mea_Rover.SNR[f] * SNR_UNIT,
                            Info_Else.Mea_Base.SNR[f] * SNR_UNIT,
                            bl, dt, f, &prcopt, &(Info_Else.Mea_Rover), 1);

                    }
                    noiseModel::Base::shared_ptr noise = noiseModel::Gaussian::Covariance(R);
                    noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.0), noise);

                    auto* dd_psr_factor = new GNSSDDPsrFactor_ENU(X(key), Info_Master, Infos_Else, prcopt,
                        f, true, lla_origin, extGPS, huber);
                    graphFactors.add(*dd_psr_factor);

#endif

#if 1
                    gtsam::Vector var(Infos_Else.size());
                    var.setConstant(0.1);
                    noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(var);
                    noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);

                    if (estimateExtGPS)
                    {
                        GNSSDDPsrFactor_ENU_lever::shared_ptr dd_psr_factor(
                            new GNSSDDPsrFactor_ENU_lever(X(key), T(key), Info_Master, Infos_Else, prcopt,
                                f, false, lla_origin, huber));
                        graphFactors->add(dd_psr_factor);
                    }
                    else
                    {
                        GNSSDDPsrFactor_ENU::shared_ptr dd_psr_factor(
                            new GNSSDDPsrFactor_ENU(X(key), Info_Master, Infos_Else, prcopt,
                                f, false, lla_origin, extGPS, extRot.transpose(), huber));
                        graphFactors->add(dd_psr_factor);
                    }
#endif
                }
            }
        }
        return npr;
    }

    int addDDCpFactorENU(gtsam::NonlinearFactorGraph* graphFactors, gtsam::Values* graphValues, int key, int lastkey)
    {
        int ncp, ns;
        ncp = 0;
        ns = gnss_info.SD_Infos.size();
        int slip, sat, reset, nf = NF(&rtk.opt);
        vector<int> sats;
        vector<int> freqs;
        VectorXi udbias = VectorXi(NB(&prcopt)).setZero();
        VectorXd compress_ar(NB(&prcopt));
        compress_ar.setZero();
        map<int, int> ar_index;
        if (gnss_info.amb.empty()) return 0;
        for (int n = 0; n < MAXSAT; ++n)
        {
            for (int f = 0; f < 3; ++f)
            {
                rtk.ssat[n].vsatL[f] = 0;
                rtk.ssat[n].vsat[f] = 0;
                rtk.ssat[n].vs = 0;
                rtk.ssat[n].sys = 0;
            }
            rtk.ssat[n].azel[0] = rtk.ssat[n].azel[1] = 0.0;
        }
        for (int k = 0; k < ns; k++)
        {
            sat = gnss_info.SD_Infos[k].Mea_Rover.sat;
            memcpy(rtk.ssat[sat - 1].vsat, gnss_info.SD_Infos[k].ssat.vsat.data(), sizeof(uint8_t) * 3);
            memcpy(rtk.ssat[sat - 1].vsatL, gnss_info.SD_Infos[k].ssat.vsatL.data(), sizeof(uint8_t) * 3);
            memcpy(rtk.ssat[sat - 1].slip, gnss_info.SD_Infos[k].ssat.slip.data(), sizeof(uint8_t) * 3);
            memcpy(rtk.ssat[sat - 1].azel, gnss_info.SD_Infos[k].ssat.azel.data(), sizeof(double) * 2);
            rtk.ssat[sat - 1].vs = 1;
            rtk.ssat[sat - 1].sys = gnss_info.SD_Infos[k].ssat.sys;
        }
        /** update phase-bias **/
        for (int f = 0; f < nf; ++f)
        {
            for (int n = 0; n < MAXSAT; ++n)
            {

                reset = ++rtk.ssat[n].outc[f] > (uint32_t)rtk.opt.maxout;
                if (reset && AmbFullArray[MAXSAT * f + n] != 0.0)
                {
                    if (last_ar_index.count(MAXSAT * f + n + 1) != 0)
                    {
                        prevAmb(last_ar_index.find(MAXSAT * f + n + 1)->second) = 0;
                    }
                    AmbFullArray[MAXSAT * f + n] = 0.0;
                    rtk.ssat[n].outc[f] = 0;
                }
                if (reset)
                {
                    rtk.ssat[n].lock[f] = -rtk.opt.minlock;
                }
            }
            for (int n = 0; n < MAXSAT; ++n)
            {
                if (rtk.ssat[n].vs)
                {
                    if (!(rtk.ssat[n].slip[f] & 1) && rtk.ssat[n].rejc[f] < 2)
                        continue;
                    if (last_ar_index.count(MAXSAT * f + n + 1) != 0)
                    {
                        prevAmb(last_ar_index.find(MAXSAT * f + n + 1)->second) = 0;
                    }
                    AmbFullArray[MAXSAT * f + n] = 0.0;
                    rtk.ssat[n].rejc[f] = 0;
                    rtk.ssat[n].lock[f] = -rtk.opt.minlock;
                }
            }

            for (int n = 0; n < MAXSAT; ++n)
            {
                if (rtk.ssat[n].vs)
                {
                    if (gnss_info.amb[MAXSAT * f + n] == 0.0 || AmbFullArray[MAXSAT * f + n] != 0.0)
                        continue;
                    rtk.ssat[n].lock[f] = -rtk.opt.minlock;
                }
            }
        }

        for (int m = 0; m < 6; m++)
        {
            for (int f = 0; f < 3; f++)
            {
                int sysi, sysj;
                int sati, satj;
                int i, j;
                for (i = -1, j = 0; j < ns; j++)
                {
                    rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                    sysi = ssatj.sys;
                    satj = gnss_info.SD_Infos[j].Mea_Rover.sat;
                    if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0 || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                        continue;
                    // if (!ssatj.vsatL[f])
                    //     continue;
                    if (!test_sys(sysi, m) || sysi == SYS_SBS)
                        continue;
                    if (satexclude(satj, ssatj.ephvar, ssatj.svh, &prcopt))
                        continue;
                    if (testsnr(0, f, ssatj.azel[1], gnss_info.SD_Infos[j].Mea_Rover.SNR[f] * SNR_UNIT, &prcopt.snrmask))
                        continue;
                    if (i >= 0 && gnss_info.SD_Infos[j].ssat.slip[f] & LLI_SLIP)
                        continue;
                    if (i < 0 || ssatj.azel[1] >= gnss_info.SD_Infos[i].ssat.azel[1])
                        i = j;
                }
                if (i < 0)
                    continue;

                rtklib::GNSS_Info_SD Info_Master = gnss_info.SD_Infos[i];

                vector<rtklib::GNSS_Info_SD> Infos_Else;

                map<int, int> part_ar_index;
                for (j = 0; j < ns; j++)
                {
                    if (i == j)
                        continue; /* skip ref sat */
                    rtklib::sat_state ssatj = gnss_info.SD_Infos[j].ssat;
                    sysj = ssatj.sys;
                    satj = gnss_info.SD_Infos[j].Mea_Rover.sat;
                    if (gnss_info.SD_Infos[j].Mea_Rover.L[f] == 0.0 || gnss_info.SD_Infos[j].Mea_Base.L[f] == 0.0 || ssatj.azel[1] == 0.0 || ssatj.azel_b[1] == 0.0)
                        continue;
                    // if (!ssatj.vsatL[f])
                    //     continue;
                    if (!test_sys(sysj, m) || sysj == SYS_SBS)
                        continue;
                    if (satexclude(satj, ssatj.ephvar, ssatj.svh, &prcopt))
                        continue;
                    if (testsnr(0, f, ssatj.azel[1], gnss_info.SD_Infos[j].Mea_Rover.SNR[f] * SNR_UNIT, &prcopt.snrmask))
                        continue;
                    vector<rtklib::GNSS_Info_SD> tmpInfo;
                    map<int, int> tmp_ar_index;
                    double ambi = AmbFullArray[f * MAXSAT + Info_Master.Mea_Rover.sat - 1] != 0.0 ? AmbFullArray[f * MAXSAT + Info_Master.Mea_Rover.sat - 1] : gnss_info.amb[f * MAXSAT + Info_Master.Mea_Rover.sat - 1];
                    double ambj = AmbFullArray[f * MAXSAT + satj - 1] != 0.0 ? AmbFullArray[f * MAXSAT + satj - 1] : gnss_info.amb[f * MAXSAT + satj - 1];
                    tmpInfo.push_back(gnss_info.SD_Infos[j]);
                    tmp_ar_index[f * MAXSAT + satj] = 0;
                    tmp_ar_index[f * MAXSAT + Info_Master.Mea_Rover.sat] = 1;

                    Vector2 tmp_amb(ambj, ambi);
                    noiseModel::Base::shared_ptr noise = noiseModel::Isotropic::Sigma(1, 1);
                    GNSSDDCpFactorCompress_ENU dd_cp_factor = GNSSDDCpFactorCompress_ENU(X(key), N(key), Info_Master, tmpInfo, tmp_ar_index, prcopt,
                        f, true, lla_origin, extGPS, extRot.transpose(), noise);
                    double residual = fabs(dd_cp_factor.debugEvaluateError(graphValues->at<Pose3>(X(key)), tmp_amb)[0]);
                    if (fabs(residual) > (prcopt.maxinno[0] * 0.1))
                    {
                        rtk.ssat[satj - 1].vsat[f] = 0;
                        ROS_WARN("phase residual out of range %.3lf\n", residual);
                        continue;
                    }
                    // else
                    // {
                    //     // ROS_INFO("phase residual in range %.3lf\n",residual);
                    // }

                    Infos_Else.push_back(gnss_info.SD_Infos[j]);
                    slip = gnss_info.SD_Infos[j].ssat.slip[f];
                    part_ar_index[f * MAXSAT + satj] = ncp;
                    compress_ar[ncp] = ambj; // gnss_info.amb[f * MAXSAT + satj - 1];
                    ncp++;

                    if (systemInitialized)
                    {
                        double delta_amb = AmbFullArray[f * MAXSAT + satj - 1] - gnss_info.amb[f * MAXSAT + satj - 1];
                        if ((!(slip & LLI_SLIP)) && last_ar_index.count(f * MAXSAT + satj) != 0 && fabs(delta_amb) < 1 && prcopt.modear > 2)
                        {
                            sats.push_back(gnss_info.SD_Infos[j].Mea_Rover.sat);
                            freqs.push_back(f);
                            udbias(f * MAXSAT + gnss_info.SD_Infos[j].Mea_Rover.sat - 1) = 1;
                        }
                        else
                        {
                            graphFactors->add(GNSSPhasePriorConstraintCompress(N(key), compress_ar[ncp - 1], ncp - 1,
                                noiseModel::Diagonal::Sigmas(
                                    Vector(1).setConstant(30))));
                        }
                    }
                }

                if (!Infos_Else.empty())
                {
                    part_ar_index[f * MAXSAT + Info_Master.Mea_Rover.sat] = ncp;
                    compress_ar[ncp] = AmbFullArray[f * MAXSAT + Info_Master.Mea_Rover.sat - 1] != 0.0 ? AmbFullArray[f * MAXSAT + Info_Master.Mea_Rover.sat - 1] : gnss_info.amb[f * MAXSAT + Info_Master.Mea_Rover.sat - 1];
                    ncp++;
                    if (systemInitialized)
                    {
                        double delta_amb = AmbFullArray[f * MAXSAT + Info_Master.Mea_Rover.sat - 1] - gnss_info.amb[f * MAXSAT + Info_Master.Mea_Rover.sat - 1];
                        if (last_ar_index.count(f * MAXSAT + Info_Master.Mea_Rover.sat) != 0 && fabs(delta_amb) < 1 && prcopt.modear > 2)
                        {
                            sats.push_back(Info_Master.Mea_Rover.sat);
                            freqs.push_back(f);
                            udbias(f * MAXSAT + Info_Master.Mea_Rover.sat - 1) = 1;
                        }
                        else
                        {
                            graphFactors->add(GNSSPhasePriorConstraintCompress(N(key), compress_ar[ncp - 1], ncp - 1,
                                noiseModel::Diagonal::Sigmas(
                                    Vector(1).setConstant(30))));
                        }
                    }
                    gtsam::Vector var(Infos_Else.size());
                    var.setConstant(0.1);
                    noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(var);
                    noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);
                    if (estimateExtGPS)
                    {
                        GNSSDDCpFactorCompress_ENU_lever::shared_ptr dd_cp_factor(new GNSSDDCpFactorCompress_ENU_lever(X(key), N(key), T(key), Info_Master, Infos_Else, part_ar_index, prcopt,
                            f, false, lla_origin, huber));
                        graphFactors->add(dd_cp_factor);
                    }
                    else
                    {
                        GNSSDDCpFactorCompress_ENU::shared_ptr dd_cp_factor(new GNSSDDCpFactorCompress_ENU(X(key), N(key), Info_Master, Infos_Else, part_ar_index, prcopt,
                            f, false, lla_origin, extGPS, extRot.transpose(), huber));
                        graphFactors->add(dd_cp_factor);
                    }

                    ar_index.insert(part_ar_index.begin(), part_ar_index.end());
                }
            }
        }
        if (!sats.empty() && !freqs.empty() && prcopt.modear > 2)
        { // fgo.num_factor[1]!=0
            noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(Vector(sats.size()).setConstant(1e-3));
            noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), noise);

            GNSSAmbConstraintCompress::shared_ptr amb_constraint_factor(new GNSSAmbConstraintCompress(N(key), N(lastkey), 0.0, sats, freqs, ar_index, last_ar_index,
                noise));
            graphFactors->add(amb_constraint_factor);
        }
        compress_ar.conservativeResize(ncp);
        if (ncp > 0)
        {
            graphValues->insert(N(key), compress_ar);
        }
        //        noiseModel::Diagonal::Sigmas(Vector(compress_ar.size()).setConstant(30)
        if (!systemInitialized && compress_ar.size() > 0)
        {
            gtsam::Matrix priorNoiseMatrix = gtsam::Matrix::Identity(compress_ar.size(), compress_ar.size()) * 900;
            graphFactors->add(PriorFactor<Vector>(N(key), (Vector)compress_ar,
                noiseModel::Gaussian::Covariance(priorNoiseMatrix)));
        }
        last_ar_index = ar_index;
        return ncp;
    }

    int addSDDopFactorENU(gtsam::NonlinearFactorGraph* graphFactors, gtsam::Values* graphValues, int key, int lastkey)
    {
        int validSat[MAXSAT] = { 0 };
        int ns = gnss_info.ZD_Infos.size();
        int ndop = 0;
        double delta_t = gnss_info.weekSec - pre_gpstime[1];
        for (int m = 0; m < 6; m++)
        {
            for (int f = 0; f < 3; f++)
            {
                int i, j;
                for (i = -1, j = 0; j < ns; j++)
                {
                    rtklib::sat_state ssatj = gnss_info.ZD_Infos[j].ssat;
                    int sys = gnss_info.ZD_Infos[j].ssat.sys;
                    if (!test_sys(sys, m) || sys == SYS_SBS)
                        continue;
                    if (gnss_info.ZD_Infos[j].Mea.D[f] == 0.0 || ssatj.azel[1] == 0.0)
                        continue;
                    if (!ssatj.vsatD[f])
                        continue;
                    if (ssatj.azel[1] < prcopt.elmin)
                        continue;
                    if (ssatj.lam[f] == 0.0)
                        continue;
                    if (satexclude(gnss_info.ZD_Infos[j].Mea.sat, ssatj.ephvar, ssatj.svh, &prcopt))
                        continue;
                    if (i < 0 || ssatj.azel[1] >= gnss_info.ZD_Infos[i].ssat.azel[1])
                        i = j;
                }
                if (i < 0)
                    continue;
                rtklib::GNSS_Info_ZD Info_Master = gnss_info.ZD_Infos[i];
                std::vector<rtklib::GNSS_Info_ZD> Infos_Else;
                for (j = 0; j < ns; j++)
                {
                    if (i == j)
                        continue; /* skip ref sat */
                    rtklib::sat_state ssatj = gnss_info.ZD_Infos[j].ssat;
                    int sys = ssatj.sys;
                    if (!test_sys(sys, m) || sys == SYS_SBS)
                        continue;
                    if (gnss_info.ZD_Infos[j].Mea.D[f] == 0.0 || ssatj.azel[1] == 0.0)
                        continue;
                    if (!ssatj.vsatD[f])
                        if (gnss_info.stat != SOLQ_DGPS)
                            continue;
                    if (ssatj.lam[f] == 0.0)
                        continue;
                    if (ssatj.azel[1] < prcopt.elmin)
                        continue;
                    if (satexclude(gnss_info.ZD_Infos[j].Mea.sat, ssatj.ephvar, ssatj.svh, &prcopt))
                        continue;
                    Infos_Else.push_back(gnss_info.ZD_Infos[j]);
                    if (f == 0)
                        validSat[gnss_info.ZD_Infos[j].Mea.sat - 1] = 1;
                }

                if (!Infos_Else.empty())
                {
                    ndop += Infos_Else.size();
#if 0
                    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(Infos_Else.size(), Infos_Else.size());

                    R.setConstant(GNSS_Tools::varerr_zdobs(Info_Master.ssat.sys, Info_Master.ssat.azel[1],
                        Info_Master.Mea.SNR[f] * SNR_UNIT,
                        f, &prcopt, &(Info_Master.Mea), 2));
                    for (int i = 0; i < Infos_Else.size(); i++)
                    {
                        rtklib::GNSS_Info_ZD Info_Else = Infos_Else.at(i);

                        R(i, i) += GNSS_Tools::varerr_zdobs(Info_Else.ssat.sys, Info_Else.ssat.azel[1],
                            Info_Else.Mea.SNR[f] * SNR_UNIT,
                            f, &prcopt, &(Info_Else.Mea), 2);
                    }

                    noiseModel::Base::shared_ptr noise = noiseModel::Gaussian::Covariance(R);
                    noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.0), noise);

                    auto* sd_dop_factor = new GNSSSDDopFactor_ENU(X(lastkey), X(key), Info_Master, Infos_Else,
                        prcopt,
                        f, delta_t, true, lla_origin, extGPS, huber);

                    graphFactors.add(*sd_dop_factor);

#endif

#if 1
                    gtsam::Vector var(Infos_Else.size());
                    var.setConstant(1);
                    noiseModel::Base::shared_ptr noise = noiseModel::Diagonal::Sigmas(var);
                    noiseModel::Base::shared_ptr huber = noiseModel::Robust::Create(
                        noiseModel::mEstimator::Huber::Create(1.0), noise);
                    // GNSSSDDopFactor_ENUVel::shared_ptr sd_dop_factor(new GNSSSDDopFactor_ENUVel(V(key), X(key), Info_Master, Infos_Else, prcopt,
                    //                                                                       f, false, lla_origin, extGPS,huber));

                    GNSSSDDopFactor_ENU::shared_ptr sd_dop_factor(new GNSSSDDopFactor_ENU(X(lastkey), X(key), Info_Master, Infos_Else, prcopt,
                        f, delta_t, false, lla_origin, extGPS, huber));

                    GNSSSDDopFactor_ENUVel debug_sd_dop_factor = GNSSSDDopFactor_ENUVel(V(key), X(key), Info_Master, Infos_Else, prcopt,
                        f, false, lla_origin, extGPS, huber);
                    debug_sd_dop_factor.debugEvaluateError(graphValues->at<Vector3>(V(key)), graphValues->at<Pose3>(X(key)));
                    graphFactors->add(sd_dop_factor);
#endif
                }
            }
        }
        return ndop;
    }

    int addGPSFactorENU(gtsam::NonlinearFactorGraph* graphFactors, gtsam::Values* graphValues, int key)
    {
        // GPS too noisy, skip
        float noise_x = fabs(gnss_odom.pose.covariance[0]);
        float noise_y = fabs(gnss_odom.pose.covariance[7]);
        float noise_z = fabs(gnss_odom.pose.covariance[14]);

        float gps_x = gnss_odom.pose.pose.position.x;
        float gps_y = gnss_odom.pose.pose.position.y;
        float gps_z = gnss_odom.pose.pose.position.z;

        ROS_INFO("GPS Noise: %f, %f, %f", noise_x, noise_y, noise_z);
        ROS_INFO("GPS Position: %f, %f, %f", gps_x, gps_y, gps_z);
        gtsam::Vector Vector3(3);
        Vector3 << noise_x, noise_y, noise_z;
        noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances((Vector3));
        noiseModel::Robust::shared_ptr robust_gps_noise = noiseModel::Robust::Create(
            noiseModel::mEstimator::Huber::Create(1.5), gps_noise);
        GnssFactor gnss_factor = GnssFactor(X(key), gtsam::Point3(gps_x, gps_y, gps_z), extGPS, robust_gps_noise);
        graphFactors->add(gnss_factor);
        return 1;
    }

    void gnssInfoHandler(const rtklib::GNSS_InfoConstPtr& gnss_msg)
    {
        mtxGpsInfo.lock();
        double currentGpsTime = gnss_msg->header.stamp.toSec();
        if (std::isnan(gnss_msg->pos[0] + gnss_msg->pos[1] + gnss_msg->pos[2]))
        {
            ROS_ERROR("POS LLA NAN...");
            mtxGpsInfo.unlock();
            return;
        }
        if (!init_origin)//如果没有初始化原点坐标，那么就用第一条有效的GNSS消息来初始化原点坐标
        {
            if (!useGroundTruthPos && gnss_msg->stat != 1)
            {
                mtxGpsInfo.unlock();
                return;
            }
            if (useGroundTruthPos)//如果使用地面真实位置来初始化原点坐标，那么就用GroundTruthPos来初始化原点坐标
            {
                Eigen::Vector3d lla = GNSS_Tools::ecef2llh(GroundTruthPos);
                lla_origin << lla[0], lla[1], lla[2];
            }
            else
            {
                ROS_INFO("mapOptimized Init Orgin GPS xyz  %f, %f, %f", gnss_msg->pos[0], gnss_msg->pos[1], gnss_msg->pos[2]);
                Eigen::Vector3d ecef(gnss_msg->pos[0], gnss_msg->pos[1], gnss_msg->pos[2]);
                Eigen::Vector3d lla = GNSS_Tools::ecef2llh(ecef);
                lla_origin << lla[0], lla[1], lla[2];
            }
            init_origin = true;
        }
        if (!init_origin)
        {
            ROS_ERROR("waiting init origin axis");
            mtxGpsInfo.unlock();
            return;
        }
        Eigen::Vector3d ecef(gnss_msg->pos[0], gnss_msg->pos[1], gnss_msg->pos[2]);
        Eigen::Vector3d llh = GNSS_Tools::ecef2llh(ecef);
        Eigen::Vector3d enu = GNSS_Tools::ecef2enu(lla_origin, ecef); // gtools.ECEF2ENU(ecef);
        ROS_INFO("TimeStamp: %.3lf(%d/%.3lf) GPS ENU to Origin: %f, %f, %f", currentGpsTime, gnss_msg->gpsWeek, gnss_msg->weekSec, enu(0), enu(1), enu(2));

        // maybe you need to get the extrinsics between your gnss and imu
        // most of the time, they are in the same frame
        Eigen::Vector3d calib_enu = enu;
        {
            // pub gps odometry
            nav_msgs::Odometry odom_msg;
            odom_msg.header.stamp = gnss_msg->header.stamp;
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "gps";

            // ----------------- 1. use utm -----------------------
            //        odom_msg.pose.pose.position.x = utm_x - origin_utm_x;
            //        odom_msg.pose.pose.position.y = utm_y - origin_utm_y;
            //        odom_msg.pose.pose.position.z = msg->altitude - origin_al;

            // ----------------- 2. use enu -----------------------
            odom_msg.pose.pose.position.x = calib_enu(0);
            odom_msg.pose.pose.position.y = calib_enu(1);
            odom_msg.pose.pose.position.z = calib_enu(2);
            odom_msg.pose.covariance[0] = gnss_msg->var[0];
            odom_msg.pose.covariance[7] = gnss_msg->var[1];
            odom_msg.pose.covariance[14] = gnss_msg->var[2];
            odom_msg.pose.covariance[1] = llh[0];
            odom_msg.pose.covariance[2] = llh[1];
            odom_msg.pose.covariance[3] = llh[2];

            // publish path
            gpsOdomPath.header.frame_id = "map";
            gpsOdomPath.header.stamp = gnss_msg->header.stamp;
            geometry_msgs::PoseStamped pose;
            pose.header = gpsOdomPath.header;
            pose.pose.position.x = calib_enu(0);
            pose.pose.position.y = calib_enu(1);
            pose.pose.position.z = calib_enu(2);
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            gpsOdomPath.poses.push_back(pose);
            GNSSPose gnss_pose_msg(odom_msg, *gnss_msg);
            gnss_queue.push_back(gnss_pose_msg);
        }
        mtxGpsInfo.unlock();
    }

    bool syncObs(double timestamp, double eps_cam)
    {
        bool hasGNSS = false;
        while (!gnss_queue.empty())
        {
            mtxGpsInfo.lock();
            ROS_INFO("gnss_info_front timestamp %lf lastimuopt time: %lf", gnss_queue.front().second.header.stamp.toSec(), timestamp);

            if (gnss_queue.front().second.header.stamp.toSec() < timestamp - eps_cam)
            {
                gnss_queue.pop_front();
                mtxGpsInfo.unlock();
            }
            else if (gnss_queue.front().second.header.stamp.toSec() > timestamp + eps_cam)
            {
                mtxGpsInfo.unlock();
                break;
            }
            else
            {
                hasGNSS = true;
                gnss_odom = gnss_queue.front().first;
                gnss_info = gnss_queue.front().second;
                ROS_INFO("gnss_info timestamp %lf", gnss_info.header.stamp.toSec());
                gnss_queue.pop_front();
                mtxGpsInfo.unlock();
                break;
            }
        }
        return hasGNSS;
    }

    bool checkIsEmpty()
    {
        return gnss_queue.empty();
    }

    bool isGNSSEnable(double curimuTime)
    {
        double curgnssTime = gnss_queue.front().first.header.stamp.toSec();
        double delta_imu2gps = curimuTime - curgnssTime;
        if (delta_imu2gps > 0.00125)
        { // 0015
            while (!gnss_queue.empty())
            {
                gnss_queue.pop_front();
                curgnssTime = gnss_queue.front().first.header.stamp.toSec();
                delta_imu2gps = curimuTime - curgnssTime;
                if (delta_imu2gps <= 0)
                    break;
            }
        }
        // ROS_INFO("delta_imu2gps:%.5lf",delta_imu2gps);
        // ROS_INFO("optimization timestamp: curIMUtime: %.8lf curGNSStime: %.8lf",curimuTime,curgnssTime);
        return (fabs(delta_imu2gps) < 0.00125);
    };

    double getCurGnssTime(){
        return gnss_queue.front().first.header.stamp.toSec();
    }
    void updateSatState()
    {
        for (int i = 0; i < MAXSAT; i++)
            for (int f = 0; f < NFREQ; f++)
            {
                /* inc lock count if this sat used for good fix */
                if (!rtk.ssat[i].vsat[f])
                    continue;
                rtk.ssat[i].outc[f] = 0;
                if (rtk.ssat[i].lock[f] < 0 || (rtk.nfix > 0 && rtk.ssat[i].fix[f] == 2))
                    rtk.ssat[i].lock[f]++;
            }

        int ns = gnss_info.SD_Infos.size();
        for (int k = 0; k < ns; k++)
        {
            int sat = gnss_info.SD_Infos[k].Mea_Rover.sat;
            memcpy(rtk.ssat[sat - 1].rejc, gnss_info.SD_Infos[k].ssat.rejc.data(), sizeof(uint32_t) * 3);
        }
    }

    void updateAmbguity(int nb, Vector Amb)
    {
        memcpy(AmbFullArray.data(), gnss_info.amb.data(), sizeof(double) * NB(&prcopt));
        if (nb > 0)
        {
            for (std::map<int, int>::iterator it = last_ar_index.begin(); it != last_ar_index.end(); it++)
            {
                AmbFullArray[it->first - 1] = Amb[it->second];
            }
        }
    }

    int ambiguityResolve(ISAM2& graphFactors, gtsam::Values& graphValues, int key, MatrixXd& posCovariance, Pose3& sol_pos)
    {
        int nb, state = 6, nx;
        double var;
        nx = posCovariance.cols();
        var = posCovariance.diagonal().sum() / nx;

        if (!graphValues.exists(N(key)))
        {
            rtk.nfix = 0;
            updateSatState();
            ROS_INFO("ambiguties don't exist!");
            return state;
        }
        if ((var >= prcopt.thresar[1]))
        {
            rtk.nfix = 0;
            updateSatState();
            updateAmbguity(0, prevAmb);
            prevAmb = graphValues.at<Vector>(N(key));
            ambCovariance = graphFactors.marginalCovariance(N(key));
            ROS_INFO("position variance too large:  %.4f pos_ocv %.4f %.4f %.4f", var, posCovariance(3, 3), posCovariance(4, 4), posCovariance(5, 5));
            return state;
        }
        VectorXd dx(nx);

        prevAmb = graphValues.at<Vector>(N(key));
        ambCovariance = graphFactors.marginalCovariance(N(key));

        gtsam::ISAM2::sharedFactorGraph amb_graph = graphFactors.joint(X(key), N(key));
        KeyVector variables{ X(key), N(key) };
        Marginals marginals(*amb_graph, graphValues, Marginals::CHOLESKY);
        JointMarginal pos_amb_cov = marginals.jointMarginalCovariance(variables);
        MatrixXd joint_pos_amb = pos_amb_cov(X(key), N(key));

        if ((nb = GNSS_Tools::manage_Amb_LAMBDA(&rtk, ix, prevAmb, ambCovariance, joint_pos_amb, bias, dx, last_ar_index)) > 1)
        {
            Pose3 delta_pose = Pose3(Rot3::RzRyRx(dx.block(0, 0, 3, 1)), dx.block(3, 0, 3, 1));
            sol_pos = sol_pos.compose(delta_pose);
            state = 1;
            if (++rtk.nfix >= rtk.opt.minfix)
            {
                NonlinearFactorGraph graph;
                graph.add(GNSSFixConstraint(N(key), nb, ix, bias, noiseModel::Diagonal::Sigmas(Vector(nb).setConstant(sqrt(0.1)))));
                graphFactors.update(graph);
                graph.resize(0);
                graphValues = graphFactors.calculateEstimate();
                //                        ROS_INFO("ambiguity fix before: position variance %.4f pos_ocv %.4f %.4f %.4f\n",var,posCovariance(3,3),posCovariance(4,4),posCovariance(5,5));
                posCovariance = graphFactors.marginalCovariance(X(key));
                ambCovariance = graphFactors.marginalCovariance(N(key));
                //                        var = (posCovariance(0,0) + posCovariance(1,1) + posCovariance(2,2))/3;
                //                        ROS_INFO("ambiguity fix after: position variance %.4f pos_ocv %.4f %.4f %.4f\n",var,posCovariance(3,3),posCovariance(4,4),posCovariance(5,5));
                //                prevPose_ = sol_pos;//currentEstimate.at<Point3>(X(epoch));
                prevAmb = graphValues.at<Vector>(N(key));
                //                prevVel_ = result.at<gtsam::Vector3>(V(key));
                //                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                //                prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
                //                extGPS = result.at<gtsam::Vector3>(T(key));
            }
        }
        else
        {
            if (nb == -1)
                state = 4;
            if (nb == 0)
                state = 3;
            rtk.nfix = 0;
        }
        updateSatState();
        updateAmbguity(nb, prevAmb);
        return state;
    }
};

#endif // GLINS_GNSSCONTAINER_H
