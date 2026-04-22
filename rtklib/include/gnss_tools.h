/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 *
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 *******************************************************/
#pragma once
#ifndef GNSS_Tools_HPP
#define GNSS_Tools_HPP

#include "ros/ros.h"
#include "rtklib/obsdt.h"
#include "rtklib/sat_state.h"
#include "rtklib/GNSS_Info.h"
#include "rtklib/GNSS_Info_ZD.h"
#include "rtklib/GNSS_Info_SD.h"
 // google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
#include "rtklib.h"

using namespace Eigen;
/* number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP(opt)     ((opt)->dynamics==0?3:9)
#define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))
#define NL(opt)     ((opt)->glomodear!=GLO_ARMODE_AUTOCAL?0:NFREQGLO)
#define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt))
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))
#define NX(opt)     (NR(opt)+NB(opt))
#define useEleVar 1
#define TYPE_Rover_Else 0
#define TYPE_Base_Else 1
#define TYPE_Rover_Master 2
#define TYPE_Base_Master 3
#define use_fixed_cov_ar 1
#ifndef MIN
#  define MIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif
#define SQR(x)   ((x)*(x))


typedef struct
{
    rtk_t rtk;
    int num_factor[3]; // 0:code 1:phase 2:doppler
    int stat;         // 6:fgo_rtk_float 4:dgps 1:fgo_rtk_fix
}fgo_t;

/**
 * @brief GNSS Tools
 * @note  GNSS related functions
 */
class GNSS_Tools
{
public:
    GNSS_Tools() {}

public:

    /***
     *
     * @param data 经纬高
     * @return ecef坐标向量
     */
    static Eigen::MatrixXd llh2ecef(Eigen::MatrixXd data) // transform the llh to ecef
    {
        Eigen::MatrixXd ecef; // the ecef for output
        ecef.resize(3, 1);
        double lat = (double)data(0) * M_PI / 180.0; // lat to radis
        double lon = (double)data(1) * M_PI / 180.0; // lon to radis
        double alt = (double)data(2); // altitude
        double sinp = sin(lat), cosp = cos(lat), sinl = sin(lon), cosl = cos(lon);
        double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
        ecef(0) = (v + alt) * cosp * cosl;
        ecef(1) = (v + alt) * cosp * sinl;
        ecef(2) = (v * (1.0 - e2) + alt) * sinp;
        return ecef;

        //        double a = 6378137.0;
        //        double b = 6356752.314;
        //        double n, Rx, Ry, Rz;
        //        double lat = (double) data(0) * 3.1415926 / 180.0; // lat to radis
        //        double lon = (double) data(1) * 3.1415926 / 180.0; // lon to radis
        //        double alt = (double) data(2); // altitude
        //        n = a * a / sqrt(a * a * cos(lat) * cos(lat) + b * b * sin(lat) * sin(lat));
        //        Rx = (n + alt) * cos(lat) * cos(lon);
        //        Ry = (n + alt) * cos(lat) * sin(lon);
        //        Rz = (b * b / (a * a) * n + alt) * sin(lat);
        //        ecef(0) = Rx; // return value in ecef
        //        ecef(1) = Ry; // return value in ecef
        //        ecef(2) = Rz; // return value in ecef
        //        return ecef;

                /**************for test purpose*************************
                Eigen::MatrixXd llh;
                llh.resize(3, 1);
                Eigen::MatrixXd ecef;
                ecef.resize(3, 1);
                llh(0) = 114.1772621294604;
                llh(1) = 22.29842880200087;
                llh(2) = 58;
                ecef = llh2ecef(llh);
                cout << "ecef ->: " << ecef << "\n";
                */
    }


    /***
    *
    * @param data ecef坐标
    * @return 经纬高坐标向量
    */
    static Eigen::MatrixXd ecef2llh(Eigen::MatrixXd data) // transform the ecef to llh
    {
        Eigen::MatrixXd llh; // the ecef for output
        llh.resize(3, 1);
        double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(data.data(), data.data(), 2), z, zk, v = RE_WGS84, sinp;

        for (z = data(2), zk = 0.0;fabs(z - zk) >= 1E-4;)
        {
            zk = z;
            sinp = z / sqrt(r2 + z * z);
            v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
            z = data(2) + v * e2 * sinp;
        }
        llh(0) = (r2 > 1E-12 ? atan(z / sqrt(r2)) : (data(2) > 0.0 ? PI / 2.0 : -PI / 2.0)) * 180 / M_PI;
        llh(1) = (r2 > 1E-12 ? atan2(data(1), data(0)) : 0.0) * 180 / M_PI;
        llh(2) = sqrt(r2 + z * z) - v;
        return llh;

        //        Eigen::MatrixXd llh; // the ecef for output
        //        double pi = 3.1415926; // pi
        //        llh.resize(3, 1);
        //        double x = data(0); // obtain ecef
        //        double y = data(1);
        //        double z = data(2);
        //        double x2 = pow(x, 2);
        //        double y2 = pow(y, 2);
        //        double z2 = pow(z, 2);
        //
        //        double a = 6378137.0000; //earth radius in meters
        //        double b = 6356752.3142; // earth semiminor in meters
        //        double e = sqrt(1 - (b / a) * (b / a));
        //        double b2 = b * b;
        //        double e2 = e * e;
        //        double ep = e * (a / b);
        //        double r = sqrt(x2 + y2);
        //        double r2 = r * r;
        //        double E2 = a * a - b * b;
        //        double F = 54 * b2 * z2;
        //        double G = r2 + (1 - e2) * z2 - e2 * E2;
        //        double c = (e2 * e2 * F * r2) / (G * G * G);
        //        double s = (1 + c + sqrt(c * c + 2 * c));
        //        s = pow(s, 1 / 3);
        //        double P = F / (3 * ((s + 1 / s + 1) * (s + 1 / s + 1)) * G * G);
        //        double Q = sqrt(1 + 2 * e2 * e2 * P);
        //        double ro = -(P * e2 * r) / (1 + Q) +
        //                    sqrt((a * a / 2) * (1 + 1 / Q) - (P * (1 - e2) * z2) / (Q * (1 + Q)) - P * r2 / 2);
        //        double tmp = (r - e2 * ro) * (r - e2 * ro);
        //        double U = sqrt(tmp + z2);
        //        double V = sqrt(tmp + (1 - e2) * z2);
        //        double zo = (b2 * z) / (a * V);
        //
        //        double height = U * (1 - b2 / (a * V));
        //
        //        double lat = atan((z + ep * ep * zo) / r);
        //
        //        double temp = atan(y / x);
        //        double long_;
        //        if (x >= 0)
        //            long_ = temp;
        //        else if ((x < 0) && (y >= 0))
        //            long_ = pi + temp;
        //        else
        //            long_ = temp - pi;
        //        llh(0) = (lat) * (180 / pi);
        //        llh(1) = (long_) * (180 / pi);
        //        llh(2) = height;
        //        return llh;

                /**************for test purpose*************************
                Eigen::MatrixXd ecef;
                ecef.resize(3, 1);
                Eigen::MatrixXd llh;
                llh.resize(3, 1);
                ecef(0) = -2418080.9387265667;
                ecef(1) = 5386190.3905763263;
                ecef(2) = 2405041.9305451373;
                llh = ecef2llh(ecef);
                cout << "llh ->: " << llh << "\n";
                */
    }

    /***
     * ecef to enu
     * @param originllh 站心坐标
     * @param ecef ecef坐标
     * @return enu坐标向量
     */
    static Eigen::MatrixXd ecef2enu(Eigen::MatrixXd originllh, Eigen::MatrixXd ecef) // transform the ecef to enu
    {
        double pi = 3.1415926; // pi
        double DEG2RAD = pi / 180.0;
        double RAD2DEG = 180.0 / pi;

        Eigen::MatrixXd enu; // the enu for output
        enu.resize(3, 1); // resize to 3X1
        Eigen::MatrixXd oxyz; // the original position
        oxyz.resize(3, 1); // resize to 3X1

        double x, y, z; // save the x y z in ecef
        x = ecef(0);
        y = ecef(1);
        z = ecef(2);

        double ox, oy, oz; // save original reference position in ecef
        oxyz = llh2ecef(originllh);
        ox = oxyz(0); // obtain x in ecef
        oy = oxyz(1); // obtain y in ecef
        oz = oxyz(2); // obtain z in ecef

        double dx, dy, dz;
        dx = x - ox;
        dy = y - oy;
        dz = z - oz;

        double lonDeg, latDeg; // save the origin lon alt in llh
        latDeg = originllh(0);
        lonDeg = originllh(1);
        double lon = lonDeg * DEG2RAD;
        double lat = latDeg * DEG2RAD;

        //save ENU
        enu(0) = -sin(lon) * dx + cos(lon) * dy;
        enu(1) = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
        enu(2) = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
        return enu;

        /**************for test purpose*****suqare distance is about 37.4 meters********************
        Eigen::MatrixXd llh;  //original
        llh.resize(3, 1);
        llh(0) = 114.1775072541416;
        llh(1) = 22.29817969722738;
        llh(2) = 58;
        Eigen::MatrixXd ecef;
        ecef.resize(3, 1);
        ecef(0) = -2418080.9387265667;
        ecef(1) = 5386190.3905763263;
        ecef(2) = 2405041.9305451373;
        Eigen::MatrixXd enu;
        enu.resize(3, 1);
        enu = ecef2enu(llh, ecef);
        cout << "enu ->: " << enu << "\n";
        */
    }

    /***
    *
    * @param originllh 站心坐标
    * @param enu enu向量
    * @return ecef坐标
    */
    static Eigen::MatrixXd enu2ecef(Eigen::MatrixXd originllh, Eigen::MatrixXd enu) // transform the ecef to enu
    {
        // enu to ecef
        double e = enu(0);
        double n = enu(1);
        double u = enu(2);
        double lon = (double)originllh(1) * D2R;
        double lat = (double)originllh(0) * D2R;
        Eigen::MatrixXd oxyz; // the original position
        oxyz.resize(3, 1); // resize to 3X1
        oxyz = llh2ecef(originllh);
        double ox = oxyz(0);
        double oy = oxyz(1);
        double oz = oxyz(2);

        oxyz(0) = ox - sin(lon) * e - cos(lon) * sin(lat) * n + cos(lon) * cos(lat) * u;
        oxyz(1) = oy + cos(lon) * e - sin(lon) * sin(lat) * n + cos(lat) * sin(lon) * u;
        oxyz(2) = oz + cos(lat) * n + sin(lat) * u;
        return oxyz;
    }

    static double getDistanceFrom2Points(Eigen::Vector3d p1, Eigen::Vector3d p2)
    {
        double xMod = pow((p1.x() - p2.x()), 2);
        double yMod = pow((p1.y() - p2.y()), 2);
        double zMod = pow((p1.z() - p2.z()), 2);
        double mod = sqrt(xMod + yMod + zMod);
        return mod;
    }

    /* getFullRankARCovMatrix
    * defi_cov_ar: the input defi_cov_ar is the deficient matrix from ceres-solver
    * n: number of phase-bias
    * r: reference satellite
    * u: use end satellite (GNSS receiver)
    */
    static void getFullRankARCovMatrix(Eigen::MatrixXd defi_cov_ar, Eigen::MatrixXd& fi_cov_ar, int& n)
    {
        bool find_zero = false;
        for (int i = 0; i < defi_cov_ar.rows(); i++)
        {
            if ((defi_cov_ar(i, 0) == 0) && (find_zero == 0))
            {
                find_zero = 1;
                n = i;
            }
        }

        fi_cov_ar.resize(n, n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
            {
                fi_cov_ar(i, j) = defi_cov_ar(i, j);
            }
    }


    /* check the valid epoch based on gps time span*/
    bool checkValidEpoch(double gps_sec)
    {
        if ((gps_sec >= start_gps_sec) && (gps_sec <= end_gps_sec))
        {
            return true;
        }
        else return false;
    }

    /* single-differenced measurement error variance -----------------------------*/
    static double varerr_zdobs(int sys, double el, double snr_rover, int f, const prcopt_t* opt, const rtklib::obsdt* obs,
        const int ObsType)
    {
        double a, b, e;
        double snr_max = opt->err[5];
        double fact = 1.0;
        double sinel = sin(el), var;
        int nf = 3, frq;

        frq = f % nf;
        /* include err ratio and measurement std (P or L) from receiver */
//    if (ObsType==1) fact=opt->eratio[frq];
//    else if(ObsType==0) fact=1.0;
//    else if(ObsType==2) fact=opt->eratio[frq]/3.0;
        if (ObsType == 1) fact = opt->eratio[frq];
        else if (ObsType == 0) fact = opt->eratio[frq] / opt->eratio[0];
        else if (ObsType == 2) fact = opt->eratio[frq] / opt->Dop2PrRatio;
        //    } else if (ObsType==1) fact=opt->eratio[frq]; /* use err ratio only */
        //    else if(ObsType==2) fact=opt->eratio[frq]/3.0;

                /* increase variance for pseudoranges */
        //    if (code) fact=opt->eratio[frq];
        if (fact <= 0.0) fact = opt->eratio[0];
        /* adjust variances for constellation */
        switch (sys)
        {
        case SYS_GPS:
            fact *= EFACT_GPS;
            break;
        case SYS_GLO:
            fact *= EFACT_GLO;
            break;
        case SYS_GAL:
            fact *= EFACT_GAL;
            break;
        case SYS_SBS:
            fact *= EFACT_SBS;
            break;
        case SYS_QZS:
            fact *= EFACT_QZS;
            break;
        case SYS_CMP:
            fact *= EFACT_CMP;
            break;
        case SYS_IRN:
            fact *= EFACT_IRN;
            break;
        default:
            fact *= EFACT_GPS;
            break;
        }
        /* adjust variance for config parameters */
        a = fact * opt->err[1];  /* base term */
        b = fact * opt->err[2];  /* el term */
        /* calculate variance */
        var = 1.0 * (a * a + b * b / sinel / sinel);
        if (opt->err[6] > 0)
        {  /* add SNR term *///wcj
            e = fact * opt->err[6];
            var += e * e * (pow(10, 0.1 * MAX(snr_max - snr_rover, 0)));
        }
        //        if (opt->err[7]>0.0) {   /* add rcvr stdevs term */
        //            if (ObsType==1) var+=SQR(opt->err[7]*0.01*(1<<(obs->Pstd[frq]+5))); /* 0.01*2^(n+5) */
        //            else if (ObsType==0)var+=SQR(opt->err[7]*obs->Lstd[frq]*0.004*0.2); /* 0.004 cycles -> m) */
        //        }

        var *= (opt->ionoopt == IONOOPT_IFLC) ? SQR(3.0) : 1.0;
        return var;
    }

    /* single-differenced measurement error variance -----------------------------*/
    static double varerr_sdobs(int sys, double el, double snr_rover, double snr_base,
        double bl, double dt, int f, const prcopt_t* opt, const rtklib::obsdt* obs,
        const int ObsType)
    {
        double a, b, c, d, e;
        double snr_max = opt->err[5];
        double fact = 1.0;
        double sinel = sin(el), var;
        int nf = NF(opt), frq, code;

        frq = f % nf;
        code = f < nf ? 0 : 1;
        /* include err ratio and measurement std (P or L) from receiver */
//        if (ObsType==1) fact=opt->eratio[frq];
//        else if(ObsType==0) fact=1.0;
//        else if(ObsType==2) fact=opt->eratio[frq]/3.0;
        if (ObsType == 1) fact = opt->eratio[frq];
        else if (ObsType == 0) fact = opt->eratio[frq] / opt->eratio[0];
        else if (ObsType == 2) fact = opt->eratio[frq] / opt->Dop2PrRatio;
        //    } else if (ObsType==1) fact=opt->eratio[frq]; /* use err ratio only */
        //    else if(ObsType==2) fact=opt->eratio[frq]/3.0;

                /* increase variance for pseudoranges */
        //    if (code) fact=opt->eratio[frq];
        //    if (fact<=0.0) fact=opt->eratio[0];
                /* adjust variances for constellation */
        switch (sys)
        {
        case SYS_GPS:
            fact *= EFACT_GPS;
            break;
        case SYS_GLO:
            fact *= EFACT_GLO;
            break;
        case SYS_GAL:
            fact *= EFACT_GAL;
            break;
        case SYS_SBS:
            fact *= EFACT_SBS;
            break;
        case SYS_QZS:
            fact *= EFACT_QZS;
            break;
        case SYS_CMP:
            fact *= EFACT_CMP;
            break;
        case SYS_IRN:
            fact *= EFACT_IRN;
            break;
        default:
            fact *= EFACT_GPS;
            break;
        }
        /* adjust variance for config parameters */
        a = fact * opt->err[1];  /* base term */
        b = fact * opt->err[2];  /* el term */
        c = opt->err[3] * bl / 1E4; /* baseline term */
        d = CLIGHT * opt->sclkstab * dt; /* clock term */
        /* calculate variance */
        //LCY
//        var=(snr_base!=0?2.0:1.0)*opt->err[8]*(a*a+b*b/sinel/sinel+c*c)+d*d;
//        if (opt->err[6]>0) {  /* add SNR term *///wcj
//            e=fact*opt->err[6];
//            if(snr_base!=0) {
//                var += e * e * (pow(10, 0.1 * MAX(snr_max - snr_rover, 0)) +
//                                pow(10, 0.1 * MAX(snr_max - snr_base, 0)));
//            }else{
//                var += e * e * (pow(10, 0.1 * MAX(snr_max - snr_rover, 0)));
//            }
//        }
//        if (opt->err[7]>0.0) {   /* add rcvr stdevs term */
//            if (ObsType==1) var+=SQR(opt->err[7]*0.01*(1<<(obs->Pstd[frq]+5))); /* 0.01*2^(n+5) */
//            else if(ObsType==0)var+=SQR(opt->err[7]*obs->Lstd[frq]*0.004*0.2); /* 0.004 cycles -> m) */
//        }
        if (ObsType == 2)
        {
            var = 1.0 * (a * a + b * b / sinel / sinel + c * c) + d * d;
            if (opt->err[6] > 0)
            {  /* add SNR term */
                e = fact * opt->err[6];
                var += e * e * (pow(10, 0.1 * MAX(snr_max - snr_rover, 0)));
                /*** ***/
//            g=fact*0.05;
//            e=fact*0.75;
//            var+=g+e*pow(10,0.1*(-snr_rover));
            }
        }
        else
        {
            var = 2.0 * (a * a + b * b / sinel / sinel + c * c) + d * d;
            if (opt->err[6] > 0)
            {  /* add SNR term */
                e = fact * opt->err[6];
                var += e * e * (pow(10, 0.1 * MAX(snr_max - snr_rover, 0)) +
                    pow(10, 0.1 * MAX(snr_max - snr_base, 0)));
                /*** ***/
//            g=fact*0.05;
//            e=fact*0.75;
//            var+=g+g+e*(pow(10,0.1*(-snr_rover))+pow(10,0.1*(-snr_base)));
            }
        }

        var *= (opt->ionoopt == IONOOPT_IFLC) ? SQR(3.0) : 1.0;
        return var;
    }

    static double CalGeodist(const Eigen::Vector3d& rx_pos, const rtklib::GNSS_Info_SD& obs, int type_DD)
    {
        const double OMGE_ = 7.2921151467E-5;
        const double CLIGHT_ = 299792458.0;
        double pos[3] = { 0 }, dts = obs.Sat_Rover.clk_bias;
        double zhd, zazel[] = { 0.0, 90.0 * D2R };
        double Est_Geodist = 0;
        double azel[2];
        Eigen::Vector3d sv_pos;
        gtime_t time;

        if (type_DD % 2 == 0)
        {
            dts = obs.Sat_Rover.clk_bias;
            azel[0] = obs.ssat.azel[0];
            azel[1] = obs.ssat.azel[1];
            time.time = (int)obs.Mea_Rover.time;
            time.sec = obs.Mea_Rover.time - (int)obs.Mea_Rover.time;
            memcpy(sv_pos.data(), obs.Sat_Rover.pos.data(), sizeof(double) * 3);
        }
        else if (type_DD % 2 == 1)
        {
            memcpy(sv_pos.data(), obs.Sat_Base.pos.data(), sizeof(double) * 3);
            azel[0] = obs.ssat.azel_b[0];
            azel[1] = obs.ssat.azel_b[1];
            time.time = (int)obs.Mea_Base.time;
            time.sec = obs.Mea_Base.time - (int)obs.Mea_Rover.time;
            dts = obs.Sat_Base.clk_bias;
        }

        double rr[3] = { rx_pos(0), rx_pos(1), rx_pos(2) };
        ecef2pos(rr, pos);

        Est_Geodist = (sv_pos - rx_pos).norm();
        const double sagnac = OMGE_ * (sv_pos(0) * rx_pos(1) - sv_pos(1) * rx_pos(0)) / CLIGHT_;
        Est_Geodist += sagnac;
        Est_Geodist += -CLIGHT_ * dts;
        zhd = tropmodel(time, pos, zazel, 0.0);
        Est_Geodist += tropmapf(time, pos, azel, NULL) * zhd;
        return Est_Geodist;
    }

    static void CalJacobian(const Eigen::Vector3d& RxPos, const rtklib::GNSS_Info_SD& OBS, int type_DD, double* H)
    {
        Eigen::Vector3d SatPos(type_DD % 2 == 0 ? OBS.Sat_Rover.pos.data() : OBS.Sat_Base.pos.data());
        Eigen::Vector3d DeltaPos = RxPos - SatPos;
        Eigen::Vector3d LoS = DeltaPos.normalized();
        memcpy(H, LoS.data(), sizeof(double) * 3);
    }

    static void CalVisionVector(const Eigen::Vector3d& RxPos, const rtklib::satdt& Sat, double e[3])
    {
        Vector3d Pos(Sat.pos[0], Sat.pos[1], Sat.pos[2]);
        Eigen::Vector3d DeltaPos = RxPos - Pos;
        Eigen::Vector3d LoS = DeltaPos.normalized();
        memcpy(e, LoS.data(), sizeof(double) * 3);
    }

    static double CalRate(Eigen::Vector3d RxPosVec, Eigen::Vector3d RxVelVec, rtklib::satdt Sat, double RxClkDrift)
    {
        const double OMGE_ = 7.2921151467E-5;
        const double CLIGHT_ = 299792458.0;
        Eigen::Vector3d vel(Sat.vel[0], Sat.vel[1], Sat.vel[2]);
        Eigen::Vector3d RelativeVel = RxVelVec - vel;
        Eigen::Vector3d LoS;
        double los[3];
        CalVisionVector(RxPosVec, Sat, los);
        memcpy(LoS.data(), los, sizeof(double) * 3);

        double Rate = RelativeVel.dot(LoS) - OMGE_ * (Sat.vel[1] * RxPosVec[0] + Sat.pos[1] * RxVelVec[0] -
            Sat.vel[0] * RxPosVec[1] - Sat.pos[0] * RxVelVec[1]) / CLIGHT_;

        return Rate + RxClkDrift - CLIGHT_ * Sat.clk_drift;
    }

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

    /* index for single to double-difference transformation matrix (D') --------------------*/
    static int ddidx(rtk_t* rtk, const VectorXd& amb, MatrixXi& ix, int gps, int glo, int sbs, int bds)
    {
        int i, j, k, m, f, n, nb = 0, nf = NF(&rtk->opt), nofix;
        double fix[MAXSAT], ref[MAXSAT];

        /* clear fix flag for all sats (1=float, 2=fix) */
        for (i = 0;i < MAXSAT;i++) for (j = 0;j < NFREQ;j++)
        {
            rtk->ssat[i].fix[j] = 0;
        }
        for (m = 0;m < 6;m++)
        { /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */

/* skip if ambiguity resolution turned off for this sys */
            nofix = (m == 0 && gps == 0) || (m == 1 && glo == 0) || (m == 3 && bds == 0);

            /* step through freqs */
            for (f = 0, k = 0;f < nf;f++, k += MAXSAT)
            {

                /* look for first valid sat (i=state index, i-k=sat index) */
                for (i = k;i < k + MAXSAT;i++)
                {
                    /* skip if sat not active */
                    if (amb[i] == 0.0 || !test_sys(rtk->ssat[i - k].sys, m) ||
                        !rtk->ssat[i - k].vsat[f])
                    {
                        continue;
                    }
                    /* set sat to use for fixing ambiguity if meets criteria */
                    if (rtk->ssat[i - k].lock[f] >= 0 && !(rtk->ssat[i - k].slip[f] & 2) &&
                        rtk->ssat[i - k].azel[1] >= rtk->opt.elmaskar && !nofix)
                    {
                        rtk->ssat[i - k].fix[f] = 2; /* fix */
                        break;/* break out of loop if find good sat */
                    }
                    /* else don't use this sat for fixing ambiguity */
                    else rtk->ssat[i - k].fix[f] = 1;
                }
                if (rtk->ssat[i - k].fix[f] != 2) continue;  /* no good sat found */
                /* step through all sats (j=state index, j-k=sat index, i-k=first good sat) */
                for (n = 0, j = k;j < k + MAXSAT;j++)
                {
                    if (i == j || amb[j] == 0.0 || !test_sys(rtk->ssat[j - k].sys, m) ||
                        !rtk->ssat[j - k].vsat[f])
                    {
                        continue;
                    }
                    if (sbs == 0 && satsys(j - k + 1, NULL) == SYS_SBS) continue;
                    if (rtk->ssat[j - k].lock[f] >= 0 && !(rtk->ssat[j - k].slip[f] & 2) &&
                        rtk->ssat[j - k].vsat[f] &&
                        rtk->ssat[j - k].azel[1] >= rtk->opt.elmaskar && !nofix)
                    {
                        /* set D coeffs to subtract sat j from sat i */
                        ix(0, nb) = i; /* state index of ref bias */
                        ix(1, nb) = j; /* state index of target bias */
                        /* inc # of sats used for fix */
                        ref[nb] = i - k + 1;
                        fix[nb++] = j - k + 1;
                        if (timediff(rtk->sol.time, gpst2time(2290, 549505)) <= 0.01)
                        {
                            printf("%lf ", fix[nb - 1]);
                        }
                        rtk->ssat[j - k].fix[f] = 2; /* fix */
                        n++; /* count # of sat pairs for this freq/constellation */
                    }
                    /* else don't use this sat for fixing ambiguity */
                    else rtk->ssat[j - k].fix[f] = 1;
                }
                /* don't use ref sat if no sat pairs */
                if (n == 0) rtk->ssat[i - k].fix[f] = 1;
            }
        }




        //        if (fabs(timediff(rtk->sol.time, gpst2time(2188,458273)))==0.00)
        //        {
        //            printf("%d %d %d %d %d %d\n",rtk->ssat[90-1].lock[0],rtk->ssat[90-1].lock[1],rtk->ssat[90-1].lock[2],
        //                   rtk->ssat[132-1].lock[0],rtk->ssat[132-1].lock[1],rtk->ssat[132-1].lock[2]
        //            );
        //        }

        return nb;
    }

    static int ddidx(rtk_t* rtk, const VectorXd& amb, MatrixXi& ix, const std::map<int, int>& ar_index, int gps, int glo, int sbs, int bds)
    {
        int i, j, k, m, f, n, nb = 0, nf = NF(&rtk->opt), nofix;
        double fix[MAXSAT], ref[MAXSAT];

        /* clear fix flag for all sats (1=float, 2=fix) */
        for (i = 0;i < MAXSAT;i++) for (j = 0;j < NFREQ;j++)
        {
            rtk->ssat[i].fix[j] = 0;
        }
        for (m = 0;m < 6;m++)
        { /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */

/* skip if ambiguity resolution turned off for this sys */
            nofix = (m == 0 && gps == 0) || (m == 1 && glo == 0) || (m == 3 && bds == 0);

            /* step through freqs */
            for (f = 0, k = 0;f < nf;f++, k += MAXSAT)
            {

                /* look for first valid sat (i=state index, i-k=sat index) */
                for (i = k;i < k + MAXSAT;i++)
                {
                    /* skip if sat not active */
                    if (ar_index.count(i + 1) == 0 || !test_sys(rtk->ssat[i - k].sys, m) ||
                        !rtk->ssat[i - k].vsat[f])
                    {
                        continue;
                    }
                    /* set sat to use for fixing ambiguity if meets criteria */
                    if (rtk->ssat[i - k].lock[f] >= 0 && !(rtk->ssat[i - k].slip[f] & 2) &&
                        rtk->ssat[i - k].azel[1] >= rtk->opt.elmaskar && !nofix)
                    {
                        rtk->ssat[i - k].fix[f] = 2; /* fix */
                        break;/* break out of loop if find good sat */
                    }
                    /* else don't use this sat for fixing ambiguity */
                    else rtk->ssat[i - k].fix[f] = 1;
                }
                if (rtk->ssat[i - k].fix[f] != 2) continue;  /* no good sat found */
                /* step through all sats (j=state index, j-k=sat index, i-k=first good sat) */
                for (n = 0, j = k;j < k + MAXSAT;j++)
                {
                    if (i == j || ar_index.count(j + 1) == 0 || !test_sys(rtk->ssat[j - k].sys, m) ||
                        !rtk->ssat[j - k].vsat[f])
                    {
                        continue;
                    }
                    if (sbs == 0 && satsys(j - k + 1, NULL) == SYS_SBS) continue;
                    //                    if (fabs(timediff(rtk->sol.time, gpst2time(2290,549505)))<=0.01)
                    //                    {
                    //                        printf("%d %d %d %d %d %d\n",i-k+1,j-k+1,rtk->ssat[j-k].lock[f],rtk->ssat[j-k].slip[f],rtk->ssat[j-k].vsat[f],f);
                    //                    }
                    if (rtk->ssat[j - k].lock[f] >= 0 && !(rtk->ssat[j - k].slip[f] & 2) &&
                        rtk->ssat[j - k].vsat[f] &&
                        rtk->ssat[j - k].azel[1] >= rtk->opt.elmaskar && !nofix)
                    {
                        /* set D coeffs to subtract sat j from sat i */
                        ix(0, nb) = ar_index.find(i + 1)->second; /* state index of ref bias */
                        ix(1, nb) = ar_index.find(j + 1)->second; /* state index of target bias */
                        /* inc # of sats used for fix */
                        ref[nb] = i - k + 1;
                        fix[nb++] = j - k + 1;
                        if (fabs(timediff(rtk->sol.time, gpst2time(2290, 549519))) <= 0.01)
                        {
                            printf("%lf %lf %d %d %d\n", ref[nb - 1], fix[nb - 1], rtk->ssat[j - k].lock[f], rtk->ssat[j - k].slip[f], rtk->ssat[j - k].vsat[f]);
                        }
                        rtk->ssat[j - k].fix[f] = 2; /* fix */
                        n++; /* count # of sat pairs for this freq/constellation */
                    }
                    /* else don't use this sat for fixing ambiguity */
                    else rtk->ssat[j - k].fix[f] = 1;
                }
                /* don't use ref sat if no sat pairs */
                if (n == 0) rtk->ssat[i - k].fix[f] = 1;
            }
        }

        return nb;
    }


    /***
     *
     * @param ssat
     * @param opt
     * @param ix
     * @param amb
     * @param amb_cov
     * @param joint_pos_amb
     * @param bias
     * @param dx
     * @return
     */
    static int manage_Amb_LAMBDA(rtk_t* rtk, MatrixXi& ix,
        const VectorXd& amb, const MatrixXd& amb_cov, const MatrixXd& joint_pos_amb,
        VectorXd& bias, VectorXd& dx)
    {
        //        TicToc t_margin;
        //        t_margin.tic();
        //        gtsam::ISAM2::sharedFactorGraph amb_graph = optimizer.joint(X(epoch),N(epoch));
        //        KeyVector variables {X(epoch),N(epoch)};
        //        Marginals marginals(*amb_graph, currentEstimate,Marginals::QR);
        //
        //        JointMarginal pos_amb_cov = marginals.jointMarginalCovariance(variables);
        //        MatrixXd joint_pos_amb = pos_amb_cov(X(epoch),N(epoch));
        //        ROS_INFO("marginal time %.3lf",t_margin.toc());
        int gps1 = -1, glo1 = -1, bds1 = -1, nb, na, nf, rerun = 0, dly;
        int i, f;
        float ratio1;
        double elmin[3] = { 90 * PI / 180,90 * PI / 180,90 * PI / 180 };
        int ar = 0, excflag = 0, arsats[MAXOBS] = { 0 }, lockc[NFREQ] = { 0 }, ns;
        na = dx.size();nf = NF(&rtk->opt);


        if (rtk->sol.prev_ratio2 < rtk->opt.thresar[0] && rtk->nb_ar >= rtk->opt.mindropsats)
        {
            /* find and count sats used last time for AR */
            for (f = 0;f < nf;f++)
            {
                for (i = 0;i < MAXSAT;i++)
                {
                    if (!rtk->ssat[i].vs) continue;
                    if (rtk->ssat[i].vsat[f] && rtk->ssat[i].lock[f] >= 0 && rtk->ssat[i].azel[1] >= rtk->opt.elmin)
                    {
                        arsats[ar++] = i;
                    }
                }
            }
            if (rtk->excsat < ar)
            {
                i = arsats[rtk->excsat];
                for (f = 0;f < nf;f++)
                {
                    lockc[f] = rtk->ssat[i - 1].lock[f];  /* save lock count */
                    /* remove sat from AR long enough to enable hold if stays fixed */
                    rtk->ssat[i - 1].lock[f] = -rtk->nb_ar;
                }
                excflag = 1;
                ROS_INFO("excflag = 1");
            }
            else rtk->excsat = 0; /* exclude none and reset to beginning of list */
        }
        rtk->sol.ratio = 0.0;
        rtk->nb_ar = 0;

        /** first try for ambiguty resolution **/
        gps1 = 1;glo1 = 0;bds1 = 1;
        if ((nb = ddidx(rtk, amb, ix, gps1, glo1, glo1, bds1)) < (rtk->opt.minfixsats - 1))
        {  /* nb is sat pairs */
            ROS_INFO("not enough valid double-differences");
            return -1; /* flag abort */
        }
        rtk->nb_ar = nb;
        if (!resamb_LAMBDA(rtk, ix, nb, na, amb, amb_cov, joint_pos_amb, bias, dx))
        {
            nb = 0;
        }
        ratio1 = rtk->sol.ratio;

        /** partial ambiguty resolution **/
        if (rtk->sol.prev_ratio2 >= rtk->opt.thresar[0] && ((rtk->sol.ratio < rtk->opt.thresar[0]) || (rtk->sol.ratio < rtk->opt.thresar[0] * 1.1 && rtk->sol.ratio < rtk->sol.prev_ratio1 / 2.0)))
        {
            dly = 2;
            for (i = 0;i < MAXSAT;i++)
            {
                if (!rtk->ssat[i].vs) continue;
                for (f = 0; f < nf; ++f)
                {
                    if (rtk->ssat[i].fix[f] != 2) continue;
                    if (rtk->ssat[i].azel[1] < elmin[f]) elmin[f] = rtk->ssat[i].azel[1];
                    /* check for new sats */
                    if (rtk->ssat[i].lock[f] == 0)
                    {
                        rtk->ssat[i].lock[f] = -rtk->opt.minlock - dly;  /* delay use of this sat with stagger */
                        dly += 2;  /* stagger next try of new sats */
                        rerun = 1;
                    }
                }
            }

            if (!rerun)
            {
                ROS_INFO("delete the low elevation satlite");
                for (i = 0;i < MAXSAT;i++)
                {
                    if (!rtk->ssat[i].vs) continue;
                    for (f = 0; f < nf; ++f)
                    {
                        if (rtk->ssat[i].fix[f] != 2) continue;
                        if (rtk->ssat[i].azel[1] == elmin[f])
                        {
                            rtk->ssat[i].lock[f] = -rtk->opt.minlock - dly;
                            dly += 2;  /* stagger next try of new sats */
                            rerun = 1;
                        }
                    }
                }
            }
            if (rerun)
            {
                rtk->sol.ratio = 0.0;
                rtk->nb_ar = 0;
                if ((nb = ddidx(rtk, amb, ix, gps1, glo1, glo1, bds1)) < (rtk->opt.minfixsats - 1))
                {  /* nb is sat pairs */
                    ROS_INFO("not enough valid double-differences");
                    return -1; /* flag abort */
                }
                rtk->nb_ar = nb;
                if (!resamb_LAMBDA(rtk, ix, nb, na, amb, amb_cov, joint_pos_amb, bias, dx))
                {
                    nb = 0;
                }
            }
            else
            {
                ROS_INFO("no rerun");
            }
        }
        /* restore excluded sat if still no fix or significant increase in ar ratio */
        if (excflag && (rtk->sol.ratio < rtk->opt.thresar[0]) && (rtk->sol.ratio < (1.5 * rtk->sol.prev_ratio2)))
        {
            i = arsats[rtk->excsat++];
            for (f = 0;f < nf;f++) rtk->ssat[i - 1].lock[f] = lockc[f];
        }

        rtk->sol.prev_ratio1 = ratio1 > 0 ? ratio1 : rtk->sol.ratio;
        rtk->sol.prev_ratio2 = rtk->sol.ratio;
        return nb;

    }

    static int manage_Amb_LAMBDA(rtk_t* rtk, MatrixXi& ix,
        const VectorXd& amb, const MatrixXd& amb_cov, const MatrixXd& joint_pos_amb,
        VectorXd& bias, VectorXd& dx, const std::map<int, int>& ar_index)
    {
        int gps1 = -1, glo1 = -1, bds1 = -1, nb, na, nf, rerun = 0, dly;
        int i, f;
        float ratio1;
        double elmin[3] = { 90 * PI / 180,90 * PI / 180,90 * PI / 180 };
        int ar = 0, excflag = 0, arsats[MAXOBS] = { 0 }, lockc[NFREQ] = { 0 }, ns;
        na = dx.size();nf = NF(&rtk->opt);


        if (rtk->sol.prev_ratio2 < rtk->opt.thresar[0] && rtk->nb_ar >= rtk->opt.mindropsats)
        {
            /* find and count sats used last time for AR */
            for (f = 0;f < nf;f++)
            {
                for (i = 0;i < MAXSAT;i++)
                {
                    if (!rtk->ssat[i].vs) continue;
                    if (rtk->ssat[i].vsat[f] && rtk->ssat[i].lock[f] >= 0 && rtk->ssat[i].azel[1] >= rtk->opt.elmin)
                    {
                        arsats[ar++] = i;
                    }
                }
            }
            if (rtk->excsat < ar)
            {
                i = arsats[rtk->excsat];
                for (f = 0;f < nf;f++)
                {
                    lockc[f] = rtk->ssat[i].lock[f];  /* save lock count */
                    /* remove sat from AR long enough to enable hold if stays fixed */
                    rtk->ssat[i].lock[f] = -rtk->nb_ar;
                }
                excflag = 1;
                ROS_INFO("excflag = 1");
            }
            else rtk->excsat = 0; /* exclude none and reset to beginning of list */
        }
        rtk->sol.ratio = 0.0;
        rtk->nb_ar = 0;

        /** first try for ambiguty resolution **/
        gps1 = 1;glo1 = 0;bds1 = 1;
        if ((nb = ddidx(rtk, amb, ix, ar_index, gps1, glo1, glo1, bds1)) < (rtk->opt.minfixsats - 1))
        {  /* nb is sat pairs */
            ROS_INFO("not enough valid double-differences");
            rtk->sol.prev_ratio1 = ratio1 > 0 ? ratio1 : rtk->sol.ratio;
            rtk->sol.prev_ratio2 = rtk->sol.ratio;
            return -1; /* flag abort */
        }
        rtk->nb_ar = nb;
        if (!resamb_LAMBDA(rtk, ix, nb, na, amb, amb_cov, joint_pos_amb, bias, dx))
        {
            nb = 0;
        }
        ratio1 = rtk->sol.ratio;

        /** partial ambiguty resolution **/
        if (rtk->sol.prev_ratio2 >= rtk->opt.thresar[0] && ((rtk->sol.ratio < rtk->opt.thresar[0]) || (rtk->sol.ratio < rtk->opt.thresar[0] * 1.1 && rtk->sol.ratio < rtk->sol.prev_ratio1 / 2.0)))
        {
            dly = 2;
            for (i = 0;i < MAXSAT;i++)
            {
                if (!rtk->ssat[i].vs) continue;
                for (f = 0; f < nf; ++f)
                {
                    if (rtk->ssat[i].fix[f] != 2) continue;
                    if (rtk->ssat[i].azel[1] < elmin[f]) elmin[f] = rtk->ssat[i].azel[1];
                    /* check for new sats */
                    if (rtk->ssat[i].lock[f] == 0)
                    {
                        rtk->ssat[i].lock[f] = -rtk->opt.minlock - dly;  /* delay use of this sat with stagger */
                        dly += 2;  /* stagger next try of new sats */
                        rerun = 1;
                    }
                }
            }
            if (rerun)
            {
                rtk->sol.ratio = 0.0;
                rtk->nb_ar = 0;
                if ((nb = ddidx(rtk, amb, ix, ar_index, gps1, glo1, glo1, bds1)) < (rtk->opt.minfixsats - 1))
                {  /* nb is sat pairs */
                    ROS_INFO("not enough valid double-differences");
                    rtk->sol.prev_ratio1 = ratio1 > 0 ? ratio1 : rtk->sol.ratio;
                    rtk->sol.prev_ratio2 = rtk->sol.ratio;
                    return -1; /* flag abort */
                }
                rtk->nb_ar = nb;
                if (!resamb_LAMBDA(rtk, ix, nb, na, amb, amb_cov, joint_pos_amb, bias, dx))
                {
                    nb = 0;
                }
            }
            else
            {
                ROS_INFO("no rerun");
            }
        }
        /* restore excluded sat if still no fix or significant increase in ar ratio */
        if (excflag && (rtk->sol.ratio < rtk->opt.thresar[0]) && (rtk->sol.ratio < (1.5 * rtk->sol.prev_ratio2)))
        {
            i = arsats[rtk->excsat++];
            for (f = 0;f < nf;f++) rtk->ssat[i].lock[f] = lockc[f];
        }

        rtk->sol.prev_ratio1 = ratio1 > 0 ? ratio1 : rtk->sol.ratio;
        rtk->sol.prev_ratio2 = rtk->sol.ratio;
        return nb;

    }

    static int resamb_LAMBDA(rtk_t* rtk, const MatrixXi& ix, int nb, int na,
        const VectorXd& amb, const MatrixXd& amb_cov, const MatrixXd& joint_pos_amb,
        VectorXd& bias, VectorXd& dx)
    {
        double* y, * DP, * b, * db, * Qb, * Qab, s[2], * xa;
        int nAmb = amb.size();
        y = mat(nb, 1); DP = mat(nb, nAmb); b = mat(nb, 2); db = mat(nb, 1); Qb = mat(nb, nb);
        Qab = mat(na, nb);xa = zeros(na, 1);
        int i, j, info;
        float ratio;



        for (i = 0;i < nb;i++)
        {
            y[i] = amb[ix(0, i)] - amb[ix(1, i)];
        }
        //        Eigen::Map<VectorXd> ddamb(y,nb, 1);
        //        std::cout << ddamb << std::endl;
        for (j = 0;j < nAmb;j++) for (i = 0;i < nb;i++)
        {
            DP[i + j * nb] = amb_cov(ix(0, i), j) - amb_cov(ix(1, i), j);
        }
        for (j = 0;j < nb;j++) for (i = 0;i < nb;i++)
        {
            Qb[i + j * nb] = DP[i + (ix(0, j)) * nb] - DP[i + (ix(1, j)) * nb];
        }
        for (j = 0;j < nb;j++) for (i = 0;i < na;i++)
        {
            Qab[i + j * na] = joint_pos_amb(i, ix(0, j)) - joint_pos_amb(i, ix(1, j));
        }
        if (!(info = lambda(nb, 2, y, Qb, b, s)))
        {
            ratio = s[0] > 0 ? (float)(s[1] / s[0]) : 0.0f;
            if (ratio > 999.9) ratio = 999.9f;
            rtk->sol.ratio = ratio;
            for (i = 0;i < nb;i++)
            {
                bias[i] = b[i];
                y[i] -= b[i];
            }
            if (s[0] <= 0.0 || s[1] / s[0] >= rtk->opt.thresar[0])
            {
                if (!matinv(Qb, nb))
                {
                    ROS_INFO("ambiguity validation successed! (nb=%d ratio=%.2f s=%.2f/%.2f)",
                        nb, s[1] / s[0], s[0], s[1]);
                    matmul("NN", nb, 1, nb, 1.0, Qb, y, 0.0, db); /* db = Qb^-1*(b0-b) */
                    matmul("NN", na, 1, nb, -1.0, Qab, db, 1.0, xa); /* rtk->xa = rtk->x-Qab*db */
                    memcpy(dx.data(), xa, sizeof(double) * na);
                    free(y); free(DP); free(b); free(db); free(Qb); free(Qab);
                    return 1;
                }
            }
            else
            {
                ROS_ERROR("ambiguity validation failed (nb=%d ratio=%.2f s=%.2f/%.2f)",
                    nb, s[1] / s[0], s[0], s[1]);
            }
        }
        else
        { /* validation failed */
            ROS_ERROR("lambda error (info=%d)", info);
            //            nb=0;
        }
        free(y); free(DP); free(b); free(db); free(Qb); free(Qab);
        return 0;

    }

    static void fgoinit(fgo_t* fgo, prcopt_t* prcopt)
    {
        rtkinit(&fgo->rtk, prcopt);
        for (int i = 0;i < 3;i++)
        {
            fgo->num_factor[i] = 0;
        }
        fgo->stat = 0;
    }
};


#endif // POSE_SYSTEM_HPP
