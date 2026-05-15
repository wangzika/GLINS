/*------------------------------------------------------------------------------
* pntpos.c : standard positioning
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2010/07/28 1.0  moved from rtkcmn.c
*                           changed api:
*                               pntpos()
*                           deleted api:
*                               pntvel()
*           2011/01/12 1.1  add option to include unhealthy satellite
*                           reject duplicated observation data
*                           changed api: ionocorr()
*           2011/11/08 1.2  enable snr mask for single-mode (rtklib_2.4.1_p3)
*           2012/12/25 1.3  add variable snr mask
*           2014/05/26 1.4  support galileo and beidou
*           2015/03/19 1.5  fix bug on ionosphere correction for GLO and BDS
*           2018/10/10 1.6  support api change of satexclude()
*           2020/11/30 1.7  support NavIC/IRNSS in pntpos()
*                           no support IONOOPT_LEX option in ioncorr()
*                           improve handling of TGD correction for each system
*                           use E1-E5b for Galileo dual-freq iono-correction
*                           use API sat2freq() to get carrier frequency
*                           add output of velocity estimation error in estvel()
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
double para_Vel[1][3];
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
/* constants/macros ----------------------------------------------------------*/

#define SQR(x)      ((x)*(x))
#define MAX(x,y)    ((x)>=(y)?(x):(y))

#if 0 /* enable GPS-QZS time offset estimation */
#define NX          (4+5)       /* # of estimated parameters */
#else
#define NX          (4+4)       /* # of estimated parameters */
#endif
#define MAXITR      10          /* max number of iteration for point pos */
#define ERR_ION     5.0         /* ionospheric delay Std (m) */
#define ERR_TROP    3.0         /* tropspheric delay Std (m) */
#define ERR_SAAS    0.3         /* Saastamoinen model error Std (m) */
#define ERR_BRDCI   0.5         /* broadcast ionosphere model error factor */
#define ERR_CBIAS   0.3         /* code bias error Std (m) */
#define REL_HUMI    0.7         /* relative humidity for Saastamoinen model */
#define MIN_EL      (5.0*D2R)   /* min elevation for measurement error (rad) */

//ros::Publisher pub_gnss_raw;

extern void pntposRegisterPub(ros::NodeHandle& n)
{
    //    pub_gnss_raw = n.advertise<lio_sam::GNSS_Raw_Array>("GNSSPsrCarRov1", 1000);
}


/* pseudorange measurement error variance ------------------------------------*/
/* single-differenced measurement error variance -----------------------------*/
#define NF(opt)     ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
/* pseudorange measurement error variance ------------------------------------*/
static double varerr(const prcopt_t* opt, const ssat_t* ssat, const obsd_t* obs, double el, int sys)
{
    double fact = 1.0, varr, snr_rover;

    switch (sys)
    {
    case SYS_GPS: fact *= EFACT_GPS; break;
    case SYS_GLO: fact *= EFACT_GLO; break;
    case SYS_SBS: fact *= EFACT_SBS; break;
    case SYS_CMP: fact *= EFACT_CMP; break;
    case SYS_QZS: fact *= EFACT_QZS; break;
    case SYS_IRN: fact *= EFACT_IRN; break;
    default:      fact *= EFACT_GPS; break;
    }
    if (el < MIN_EL) el = MIN_EL;
    /* var = R^2*(a^2 + (b^2/sin(el) + c^2*(10^(0.1*(snr_max-snr_rover)))) + (d*rcv_std)^2) */
    varr = SQR(opt->err[1]) + SQR(opt->err[2]) / sin(el);
    if (opt->err[6] > 0.0)
    {  /* if snr term not zero */
        snr_rover = (ssat) ? SNR_UNIT * ssat->snr_rover[0] : opt->err[5];
        varr += SQR(opt->err[6]) * pow(10, 0.1 * MAX(opt->err[5] - snr_rover, 0));
    }
    varr *= SQR(opt->eratio[0]);
    if (opt->err[7] > 0.0)
    {
        varr += SQR(opt->err[7] * 0.01 * (1 << (obs->qualP[0] + 5)));  /* 0.01*2^(n+5) m */
    }
    if (opt->ionoopt == IONOOPT_IFLC) varr *= SQR(3.0); /* iono-free */
    return SQR(fact) * varr;
}

static double varerr(int sys, double el, double snr_rover, int f, const prcopt_t* opt,
    int ObsType)
{
    double a, b, e;
    double snr_max = opt->err[5];
    double fact;
    double sinel = sin(el), var;
    int nf = NF(opt), frq, code;
    double g;

    frq = f % nf;code = f < nf ? 0 : 1;
    /* increase variance for pseudoranges */
    if (ObsType == 1) fact = opt->eratio[frq];
    else if (ObsType == 2) fact = opt->eratio[frq] / opt->Dop2PrRatio;
    /* else adjust variance between freqs */
    else fact = opt->eratio[frq] / opt->eratio[0];

    /* adjust variance for constellation */
    switch (sys)
    {
    case SYS_GPS: fact *= EFACT_GPS;break;
    case SYS_GLO: fact *= EFACT_GLO;break;
    case SYS_GAL: fact *= EFACT_GAL;break;
    case SYS_SBS: fact *= EFACT_SBS;break;
    case SYS_QZS: fact *= EFACT_QZS;break;
    case SYS_CMP: fact *= EFACT_CMP;break;
    case SYS_IRN: fact *= EFACT_IRN;break;
    default:      fact *= EFACT_GPS;break;
    }
    /* adjust variance for config parameters */
    a = fact * opt->err[1];  /* base term */
    b = fact * opt->err[2];  /* el term */
    /* calculate variance */

    var = 1.0 * (a * a + b * b / sinel / sinel);
    if (opt->err[6] > 0)
    {  /* add SNR term */
        e = fact * opt->err[6];
        var += e * e * (pow(10, 0.1 * MAX(snr_max - snr_rover, 0)));
    }

    var *= (opt->ionoopt == IONOOPT_IFLC) ? SQR(3.0) : 1.0;
    return var;
}
/* get group delay parameter (m) ---------------------------------------------*/
static double gettgd(int sat, const nav_t* nav, int type)
{
    int i, sys = satsys(sat, NULL);

    if (sys == SYS_GLO)
    {
        for (i = 0;i < nav->ng;i++)
        {
            if (nav->geph[i].sat == sat) break;
        }
        return (i >= nav->ng) ? 0.0 : -nav->geph[i].dtaun * CLIGHT;
    }
    else
    {
        for (i = 0;i < nav->n;i++)
        {
            if (nav->eph[i].sat == sat) break;
        }
        return (i >= nav->n) ? 0.0 : nav->eph[i].tgd[type] * CLIGHT;
    }
}
/* test SNR mask -------------------------------------------------------------*/
static int snrmask(const obsd_t* obs, const double* azel, const prcopt_t* opt)
{
    if (testsnr(0, 0, azel[1], obs->SNR[0] * SNR_UNIT, &opt->snrmask))
    {
        return 0;
    }
    if (opt->ionoopt == IONOOPT_IFLC)
    {
        if (testsnr(0, 1, azel[1], obs->SNR[1] * SNR_UNIT, &opt->snrmask)) return 0;
    }
    return 1;
}
/* iono-free or "pseudo iono-free" pseudorange with code bias correction -----*/
static double prange(const obsd_t* obs, const nav_t* nav, const prcopt_t* opt,
    double* var)
{
    double P1, P2, gamma, b1, b2;
    int sat, sys;

    sat = obs->sat;
    sys = satsys(sat, NULL);
    P1 = obs->P[0];
    P2 = obs->P[1];
    *var = 0.0;

    if (P1 == 0.0 || (opt->ionoopt == IONOOPT_IFLC && P2 == 0.0)) return 0.0;

    /* P1-C1,P2-C2 DCB correction */
    if (sys == SYS_GPS || sys == SYS_GLO)
    {
        if (obs->code[0] == CODE_L1C) P1 += nav->cbias[sat - 1][1]; /* C1->P1 */
        if (obs->code[1] == CODE_L2C) P2 += nav->cbias[sat - 1][2]; /* C2->P2 */
    }
    if (opt->ionoopt == IONOOPT_IFLC)
    { /* dual-frequency */

        if (sys == SYS_GPS || sys == SYS_QZS)
        { /* L1-L2,G1-G2 */
            gamma = SQR(FREQL1 / FREQL2);
            return (P2 - gamma * P1) / (1.0 - gamma);
        }
        else if (sys == SYS_GLO)
        { /* G1-G2 */
            gamma = SQR(FREQ1_GLO / FREQ2_GLO);
            return (P2 - gamma * P1) / (1.0 - gamma);
        }
        else if (sys == SYS_GAL)
        { /* E1-E5b */
            gamma = SQR(FREQL1 / FREQE5b);
            if (getseleph(SYS_GAL))
            { /* F/NAV */
                P2 -= gettgd(sat, nav, 0) - gettgd(sat, nav, 1); /* BGD_E5aE5b */
            }
            return (P2 - gamma * P1) / (1.0 - gamma);
        }
        else if (sys == SYS_CMP)
        { /* B1-B2 */
            gamma = SQR(((obs->code[0] == CODE_L2I) ? FREQ1_CMP : FREQL1) / FREQ2_CMP);
            if (obs->code[0] == CODE_L2I) b1 = gettgd(sat, nav, 0); /* TGD_B1I */
            else if (obs->code[0] == CODE_L1P) b1 = gettgd(sat, nav, 2); /* TGD_B1Cp */
            else b1 = gettgd(sat, nav, 2) + gettgd(sat, nav, 4); /* TGD_B1Cp+ISC_B1Cd */
            b2 = gettgd(sat, nav, 1); /* TGD_B2I/B2bI (m) */
            return ((P2 - gamma * P1) - (b2 - gamma * b1)) / (1.0 - gamma);
        }
        else if (sys == SYS_IRN)
        { /* L5-S */
            gamma = SQR(FREQL5 / FREQs);
            return (P2 - gamma * P1) / (1.0 - gamma);
        }
    }
    else
    { /* single-freq (L1/E1/B1) */
        *var = SQR(ERR_CBIAS);

        if (sys == SYS_GPS || sys == SYS_QZS)
        { /* L1 */
            b1 = gettgd(sat, nav, 0); /* TGD (m) */
            return P1 - b1;
        }
        else if (sys == SYS_GLO)
        { /* G1 */
            gamma = SQR(FREQ1_GLO / FREQ2_GLO);
            b1 = gettgd(sat, nav, 0); /* -dtaun (m) */
            return P1 - b1 / (gamma - 1.0);
        }
        else if (sys == SYS_GAL)
        { /* E1 */
            if (getseleph(SYS_GAL)) b1 = gettgd(sat, nav, 0); /* BGD_E1E5a */
            else                    b1 = gettgd(sat, nav, 1); /* BGD_E1E5b */
            return P1 - b1;
        }
        else if (sys == SYS_CMP)
        { /* B1I/B1Cp/B1Cd */
            if (obs->code[0] == CODE_L2I) b1 = gettgd(sat, nav, 0); /* TGD_B1I */
            else if (obs->code[0] == CODE_L1P) b1 = gettgd(sat, nav, 2); /* TGD_B1Cp */
            else b1 = gettgd(sat, nav, 2) + gettgd(sat, nav, 4); /* TGD_B1Cp+ISC_B1Cd */
            return P1 - b1;
        }
        else if (sys == SYS_IRN)
        { /* L5 */
            gamma = SQR(FREQs / FREQL5);
            b1 = gettgd(sat, nav, 0); /* TGD (m) */
            return P1 - gamma * b1;
        }
    }
    return P1;
}
/* ionospheric correction ------------------------------------------------------
* compute ionospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          int    sat       I   satellite number
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
*          double *ion      O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric delay (L1) variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int ionocorr(gtime_t time, const nav_t* nav, int sat, const double* pos,
    const double* azel, int ionoopt, double* ion, double* var)
{
    int err = 0;

    trace(4, "ionocorr: time=%s opt=%d sat=%2d pos=%.3f %.3f azel=%.3f %.3f\n",
        time_str(time, 3), ionoopt, sat, pos[0] * R2D, pos[1] * R2D, azel[0] * R2D,
        azel[1] * R2D);

    /* SBAS ionosphere model */
    if (ionoopt == IONOOPT_SBAS)
    {
        if (sbsioncorr(time, nav, pos, azel, ion, var)) return 1;
        err = 1;
    }
    /* IONEX TEC model */
    if (ionoopt == IONOOPT_TEC)
    {
        if (iontec(time, nav, pos, azel, 1, ion, var)) return 1;
        err = 1;
    }
    /* QZSS broadcast ionosphere model */
    if (ionoopt == IONOOPT_QZS && norm(nav->ion_qzs, 8) > 0.0)
    {
        *ion = ionmodel(time, nav->ion_qzs, pos, azel);
        *var = SQR(*ion * ERR_BRDCI);
        return 1;
    }
    /* GPS broadcast ionosphere model */
    if (ionoopt == IONOOPT_BRDC || err == 1)
    {
        *ion = ionmodel(time, nav->ion_gps, pos, azel);
        *var = SQR(*ion * ERR_BRDCI);
        return 1;
    }
    *ion = 0.0;
    *var = ionoopt == IONOOPT_OFF ? SQR(ERR_ION) : 0.0;
    return 1;
}
/* tropospheric correction -----------------------------------------------------
* compute tropospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    tropopt   I   tropospheric correction option (TROPOPT_???)
*          double *trp      O   tropospheric delay (m)
*          double *var      O   tropospheric delay variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int tropcorr(gtime_t time, const nav_t* nav, const double* pos,
    const double* azel, int tropopt, double* trp, double* var)
{
    trace(4, "tropcorr: time=%s opt=%d pos=%.3f %.3f azel=%.3f %.3f\n",
        time_str(time, 3), tropopt, pos[0] * R2D, pos[1] * R2D, azel[0] * R2D,
        azel[1] * R2D);

    /* Saastamoinen model */
    if (tropopt == TROPOPT_SAAS || tropopt == TROPOPT_EST || tropopt == TROPOPT_ESTG)
    {
        *trp = tropmodel(time, pos, azel, REL_HUMI);
        *var = SQR(ERR_SAAS / (sin(azel[1]) + 0.1));
        return 1;
    }
    /* SBAS (MOPS) troposphere model */
    if (tropopt == TROPOPT_SBAS)
    {
        *trp = sbstropcorr(time, pos, azel, var);
        return 1;
    }
    /* no correction */
    *trp = 0.0;
    *var = tropopt == TROPOPT_OFF ? SQR(ERR_TROP) : 0.0;
    return 1;
}
/* pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, const obsd_t* obs, int n, const double* rs,
    const double* dts, const double* vare, const int* svh,
    const nav_t* nav, const double* x, const prcopt_t* opt,
    const ssat_t* ssat, double* v, double* H, double* var,
    double* azel, int* vsat, double* resp, int* ns)
{
    gtime_t time;
    double r, freq, dion = 0.0, dtrp = 0.0, vmeas, vion = 0.0, vtrp = 0.0, rr[3], pos[3], dtr, e[3], P;
    double snr_rover;
    int i, j, nv = 0, sat, sys, mask[NX - 3] = { 0 };

    for (i = 0;i < 3;i++) rr[i] = x[i];
    dtr = x[3];

    ecef2pos(rr, pos);
    for (i = *ns = 0;i < n && i < MAXOBS;i++)
    {
        vsat[i] = 0;
        azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;
        time = obs[i].time;
        sat = obs[i].sat;
        if (!(sys = satsys(sat, NULL))) continue;

        /* reject duplicated observation data */
        if (i < n - 1 && i < MAXOBS - 1 && sat == obs[i + 1].sat)
        {
            trace(2, "duplicated obs data %s sat=%d\n", time_str(time, 3), sat);
            i++;
            continue;
        }
        /* excluded satellite? */
        if (satexclude(sat, vare[i], svh[i], opt)) continue;

        /* geometric distance and elevation mask*/
        if ((r = geodist(rs + i * 6, rr, e)) <= 0.0) continue;
        if (satazel(pos, e, azel + i * 2) < opt->elmin) continue;

        if (iter > 0)
        {
            /* test SNR mask */
            if (!snrmask(obs + i, azel + i * 2, opt)) continue;

            /* ionospheric correction */
            if (!ionocorr(time, nav, sat, pos, azel + i * 2, opt->ionoopt, &dion, &vion))
            {
                continue;
            }
            if ((freq = sat2freq(sat, obs[i].code[0], nav)) == 0.0) continue;
            dion *= SQR(FREQL1 / freq);
            vion *= SQR(FREQL1 / freq);

            /* tropospheric correction */
            if (!tropcorr(time, nav, pos, azel + i * 2, opt->tropopt, &dtrp, &vtrp))
            {
                continue;
            }
        }
        /* psendorange with code bias correction */
        if ((P = prange(obs + i, nav, opt, &vmeas)) == 0.0) continue;

        snr_rover = (ssat) ? SNR_UNIT * ssat[sat - 1].snr_rover[0] : opt->err[5];
        /* pseudorange residual */
        v[nv] = P - (r + dtr - CLIGHT * dts[i * 2] + dion + dtrp);

        //        if (iter>2&& fabs(v[nv]) > 100) continue;

                /* design matrix */
        for (j = 0;j < NX;j++)
        {
            H[j + nv * NX] = j < 3 ? -e[j] : (j == 3 ? 1.0 : 0.0);
        }
        /* time system offset and receiver bias correction */
        if (sys == SYS_GLO) { v[nv] -= x[4]; H[4 + nv * NX] = 1.0; mask[1] = 1; }
        else if (sys == SYS_GAL) { v[nv] -= x[5]; H[5 + nv * NX] = 1.0; mask[2] = 1; }
        else if (sys == SYS_CMP) { v[nv] -= x[6]; H[6 + nv * NX] = 1.0; mask[3] = 1; }
        else if (sys == SYS_IRN) { v[nv] -= x[7]; H[7 + nv * NX] = 1.0; mask[4] = 1; }
#if 0 /* enable QZS-GPS time offset estimation */
        else if (sys == SYS_QZS) { v[nv] -= x[8]; H[8 + nv * NX] = 1.0; mask[5] = 1; }
#endif
        else mask[0] = 1;

        vsat[i] = 1; resp[i] = v[nv]; (*ns)++;

        /* variance of pseudorange error */
        var[nv++] = varerr(opt, &ssat[i], &obs[i], azel[1 + i * 2], sys) + vare[i] + vmeas + vion + vtrp;

        //        var[nv++]= varerr(sys,azel[1+i*2],snr_rover,0,opt,1)+vare[i]+vmeas+vion+vtrp;
        //        printf("sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n",obs[i].sat,
        //              azel[i*2]*R2D,azel[1+i*2]*R2D,resp[i],sqrt(var[nv-1]));
        trace(4, "sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n", obs[i].sat,
            azel[i * 2] * R2D, azel[1 + i * 2] * R2D, resp[i], sqrt(var[nv - 1]));
    }
    /* constraint to avoid rank-deficient */
    for (i = 0;i < NX - 3;i++)
    {
        if (mask[i]) continue;
        v[nv] = 0.0;
        for (j = 0;j < NX;j++) H[j + nv * NX] = j == i + 3 ? 1.0 : 0.0;
        var[nv++] = 0.01;
    }
    return nv;
}
static int zdrescode(int iter, const obsd_t* obs, int n, const double* rs,
    const double* dts, const double* vare, const int* svh,
    const nav_t* nav, const double* x, const prcopt_t* opt,
    const ssat_t* ssat, double* y, double* var, double* e,
    double* azel, int* vsat, double* resp, int* ns)
{
    gtime_t time;
    double r, freq, dion = 0.0, dtrp = 0.0, vmeas, vion = 0.0, vtrp = 0.0, rr[3], pos[3], dtr = 0, P;
    double snr_rover;
    int i, j, sat, sys, mask[NX - 3] = { 0 };

    for (i = 0;i < 3;i++) rr[i] = x[i];
    ecef2pos(rr, pos);

    for (i = *ns = 0;i < n && i < MAXOBS;i++)
    {
        vsat[i] = 0;
        azel[i * 2] = azel[1 + i * 2] = resp[i] = 0.0;
        time = obs[i].time;
        sat = obs[i].sat;
        snr_rover = (ssat) ? SNR_UNIT * ssat[sat - 1].snr_rover[0] : opt->err[5];
        if (!(sys = satsys(sat, NULL))) continue;

        if (snr_rover == 0.0)continue;
        /* reject duplicated observation data */
        if (i < n - 1 && i < MAXOBS - 1 && sat == obs[i + 1].sat)
        {
            trace(2, "duplicated obs data %s sat=%d\n", time_str(time, 3), sat);
            i++;
            continue;
        }
        /* excluded satellite? */
        if (satexclude(sat, vare[i], svh[i], opt)) continue;

        /* geometric distance and elevation mask*/
        if ((r = geodist(rs + i * 6, rr, e + i * 3)) <= 0.0) continue;
        if (satazel(pos, e + i * 3, azel + i * 2) < opt->elmin) continue;

        if (iter > 0)
        {
            /* test SNR mask */
            if (!snrmask(obs + i, azel + i * 2, opt)) continue;

            /* ionospheric correction */
            if (!ionocorr(time, nav, sat, pos, azel + i * 2, opt->ionoopt, &dion, &vion))
            {
                continue;
            }
            if ((freq = sat2freq(sat, obs[i].code[0], nav)) == 0.0) continue;
            dion *= SQR(FREQL1 / freq);
            vion *= SQR(FREQL1 / freq);

            /* tropospheric correction */
            if (!tropcorr(time, nav, pos, azel + i * 2, opt->tropopt, &dtrp, &vtrp))
            {
                continue;
            }
        }
        /* psendorange with code bias correction */
        if ((P = prange(obs + i, nav, opt, &vmeas)) == 0.0) continue;

        /* pseudorange residual */
        y[i] = P - (r + dtr - CLIGHT * dts[i * 2] + dion + dtrp);


        vsat[i] = 1;  (*ns)++;
        //        resp[i]=y[i];
                /* variance of pseudorange error */
        //        var[nv++]=varerr(opt,azel[1+i*2],snr_rover,sys)+vare[i]+vmeas+vion+vtrp;

        var[i] = varerr(sys, azel[1 + i * 2], snr_rover, 0, opt, 1) + vare[i] + vmeas + vion + vtrp;

    }
    return 0;
}
/* test satellite system (m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN) ---------*/
static int test_sys(int sys, int m)
{
    switch (sys)
    {
    case SYS_GPS: return m == 0;
    case SYS_SBS: return m == 0;
    case SYS_GLO: return m == 1;
    case SYS_GAL: return m == 2;
    case SYS_CMP: return m == 3;
    case SYS_QZS: return m == 4;
    case SYS_IRN: return m == 5;
    }
    return 0;
}

static int sdrescode(int iter, const nav_t* nav, const obsd_t* obs, int n,
    const double* y, double* v, double* H, double* e,
    double* azel, int* vsat, double* resp,
    double* var, const ssat_t* ssat)
{
    double* Ri, * Rj, freqi, freqj, * Hi;
    int i, j, k, m, sysi, sysj, b = 0, nv = 0, nb[6 + 2] = { 0 };
    int ns = 0;
    double* R;
    Ri = mat(n + 2, 1); Rj = mat(n + 2, 1);

    for (m = 0; m < 6; m++)
    {
        for (i = -1, j = 0;j < n;j++)
        {
            sysi = ssat[obs[j].sat - 1].sys;
            if (!test_sys(sysi, m) || sysi == SYS_SBS) continue;
            if (y[j] == 0.0) continue;
            if (i < 0 || azel[1 + j * 2] >= azel[1 + i * 2]) i = j;
        }

        if (i < 0) continue;

        for (j = 0;j < n;j++)
        {
            if (i == j)continue;
            sysi = ssat[obs[i].sat - 1].sys;
            sysj = ssat[obs[j].sat - 1].sys;
            freqi = sat2freq(obs[i].sat, obs[i].code[0], nav);
            freqj = sat2freq(obs[j].sat, obs[j].code[0], nav);
            if (freqi <= 0.0 || freqj <= 0.0) continue;
            if (!test_sys(sysj, m)) continue;
            if (y[j] == 0.0) continue;

            if (H)
            {
                Hi = H + nv * 3;
                for (k = 0;k < 3;k++) Hi[k] = 0.0;
            }

            v[nv] = y[i] - y[j];

            if (iter > 3 && fabs(v[nv]) > 50) continue;
            resp[j] = v[nv];
            if (H)
            {
                for (k = 0;k < 3;k++)
                {
                    Hi[k] = -e[k + i * 3] + e[k + j * 3];
                }
            }

            Ri[nv] = var[i];
            Rj[nv] = var[j];

            nv++;
            nb[ns]++;
        }
        ns++;
    }

    R = mat(nv, nv);
    for (i = 0;i < nv * nv;i++) R[i] = 0.0;
    for (b = 0, k = 0;b < ns;k += nb[b++])
    {  /* loop through each system */
        for (i = 0;i < nb[b];i++) for (j = 0;j < nb[b];j++)
        {
            R[k + i + (k + j) * nv] = Ri[k + i] + (i == j ? Rj[k + i] : 0.0);
        }
    }

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>> res(v, nv, 1);

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>> jacobian(H, nv, 3);

    //    std::cout << "H\n" << jacobian << std::endl;

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>> variance(R, nv, nv);

    //    std::cout << "R\n" << variance << std::endl;

    Eigen::MatrixXd llt(nv, nv);
    llt = variance.inverse().llt().matrixL().transpose();

    res = llt * res;

    jacobian = llt * jacobian;

    free(Ri);free(Rj);free(R);
    return nv;
}
/* pseudorange residuals -----------------------------------------------------*/
static int rescode_sd(int iter, const obsd_t* obs, int n, const double* rs,
    const double* dts, const double* vare, const int* svh,
    const nav_t* nav, const double* x, const prcopt_t* opt,
    const ssat_t* ssat, double* v, double* H, double* var,
    double* azel, int* vsat, double* resp, int* ns)
{
    gtime_t time;
    double r, freq, dion = 0.0, dtrp = 0.0, vmeas, vion = 0.0, vtrp = 0.0, rr[3], pos[3], dtr, P;
    double snr_rover = (ssat) ? SNR_UNIT * ssat->snr_rover[0] : opt->err[5];
    int i, j, nv = 0, sat, sys, mask[NX - 3] = { 0 };
    double* y = zeros(n, 1);
    double* e = zeros(3, n);

    trace(3, "resprng : n=%d\n", n);

    for (i = 0;i < 3;i++) rr[i] = x[i];

    ecef2pos(rr, pos);

    zdrescode(iter, obs, n, rs, dts, vare, svh, nav, x, opt, ssat, y, var, e, azel, vsat, resp, ns);

    nv = sdrescode(iter, nav, obs, n, y, v, H, e, azel, vsat, resp, var, ssat);

    free(y);free(e);
    return nv;
}
/* validate solution ---------------------------------------------------------*/
static int valsol(const double* azel, const int* vsat, int n,
    const prcopt_t* opt, const double* v, int nv, int nx,
    char* msg)
{
    double azels[MAXOBS * 2], dop[4], vv;
    int i, ns;

    trace(3, "valsol  : n=%d nv=%d\n", n, nv);

    /* Chi-square validation of residuals */
    vv = dot(v, v, nv);
    if (nv > nx && vv > chisqr[nv - nx - 1])
    {
        sprintf(msg, "Warning: large chi-square error nv=%d vv=%.1f cs=%.1f", nv, vv, chisqr[nv - nx - 1]);
        /* return 0; */ /* threshold too strict for all use cases, report error but continue on */
    }
    /* large GDOP check */
    for (i = ns = 0;i < n;i++)
    {
        if (!vsat[i]) continue;
        azels[ns * 2] = azel[i * 2];
        azels[1 + ns * 2] = azel[1 + i * 2];
        ns++;
    }
    dops(ns, azels, opt->elmin, dop);
    if (dop[0] <= 0.0 || dop[0] > opt->maxgdop)
    {
        sprintf(msg, "gdop error nv=%d gdop=%.1f", nv, dop[0]);
        return 0;
    }
    return 1;
}
/* estimate receiver position ------------------------------------------------*/
static int estpos(const obsd_t* obs, int n, const double* rs, const double* dts,
    const double* vare, const int* svh, const nav_t* nav,
    const prcopt_t* opt, const ssat_t* ssat, sol_t* sol, double* azel,
    int* vsat, double* resp, char* msg)
{
    double x[NX] = { 0 }, dx[NX], Q[NX * NX], * v, * H, * var, sig;
    int i, j, k, info, stat, nv, ns;

    trace(3, "estpos  : n=%d\n", n);

    v = mat(n + 4, 1); H = mat(NX, n + 4); var = mat(n + 4, 1);

    for (i = 0;i < 3;i++) x[i] = sol->rr[i];

    for (i = 0;i < MAXITR;i++)
    {

        /* pseudorange residuals (m) */
        nv = rescode(i, obs, n, rs, dts, vare, svh, nav, x, opt, ssat, v, H, var, azel, vsat, resp,
            &ns);
        if (nv < NX)
        {
            sprintf(msg, "lack of valid sats ns=%d", nv);
            break;
        }
        /* weighted by Std */
        for (j = 0;j < nv;j++)
        {
            sig = sqrt(var[j]);
            v[j] /= sig;
            for (k = 0;k < NX;k++) H[k + j * NX] /= sig;
        }
        /* least square estimation */
        if ((info = lsq(H, v, NX, nv, dx, Q)))
        {
            sprintf(msg, "lsq error info=%d", info);
            break;
        }
        for (j = 0;j < NX;j++)
        {
            x[j] += dx[j];
        }
        //        nv= rescode_sd(i,obs,n,rs,dts,vare,svh,nav,x,opt,ssat,v,H,var,azel,vsat,resp,
        //                       &ns);
        //
        //        if (nv<3) {
        //            sprintf(msg,"lack of valid sats ns=%d",nv);
        //            break;
        //        }
        //        if ((info=lsq(H,v,3,nv,dx,Q))) {
        //            sprintf(msg,"lsq error info=%d",info);
        //            break;
        //        }
        //        for (j=0;j<3;j++) {
        //            x[j]+=dx[j];
        //        }
        if (norm(dx, NX) < 1E-4)
        {
            sol->type = 0;
            sol->time = timeadd(obs[0].time, -x[3] / CLIGHT);
            sol->dtr[0] = x[3] / CLIGHT; /* receiver clock bias (s) */
            sol->dtr[1] = x[4] / CLIGHT; /* GLO-GPS time offset (s) */
            sol->dtr[2] = x[5] / CLIGHT; /* GAL-GPS time offset (s) */
            sol->dtr[3] = x[6] / CLIGHT; /* BDS-GPS time offset (s) */
            sol->dtr[4] = x[7] / CLIGHT; /* IRN-GPS time offset (s) */
            for (j = 0;j < 6;j++) sol->rr[j] = j < 3 ? x[j] : 0.0;
            for (j = 0;j < 3;j++) sol->qr[j] = (float)Q[j + j * NX];
            sol->qr[3] = (float)Q[1];    /* cov xy */
            sol->qr[4] = (float)Q[2 + NX]; /* cov yz */
            sol->qr[5] = (float)Q[2];    /* cov zx */
            sol->ns = (uint8_t)ns;
            sol->age = sol->ratio = 0.0;

            /* validate solution */
            if ((stat = valsol(azel, vsat, n, opt, v, nv, NX, msg)))
            {
                sol->stat = opt->sateph == EPHOPT_SBAS ? SOLQ_SBAS : SOLQ_SINGLE;
            }
            free(v); free(H); free(var);
            return stat;
        }
    }
    if (i >= MAXITR) sprintf(msg, "iteration divergent i=%d", i);

    free(v); free(H); free(var);
    return 0;
}
/* RAIM FDE (failure detection and exclution) -------------------------------*/
static int raim_fde(const obsd_t* obs, int n, const double* rs,
    const double* dts, const double* vare, const int* svh,
    const nav_t* nav, const prcopt_t* opt, const ssat_t* ssat,
    sol_t* sol, double* azel, int* vsat, double* resp, char* msg)
{
    obsd_t* obs_e;
    sol_t sol_e = { {0} };
    char tstr[32], name[16], msg_e[128];
    double* rs_e, * dts_e, * vare_e, * azel_e, * resp_e, rms_e, rms = 100.0;
    int i, j, k, nvsat, stat = 0, * svh_e, * vsat_e, sat = 0;

    trace(3, "raim_fde: %s n=%2d\n", time_str(obs[0].time, 0), n);

    if (!(obs_e = (obsd_t*)malloc(sizeof(obsd_t) * n))) return 0;
    rs_e = mat(6, n); dts_e = mat(2, n); vare_e = mat(1, n); azel_e = zeros(2, n);
    svh_e = imat(1, n); vsat_e = imat(1, n); resp_e = mat(1, n);

    for (i = 0;i < n;i++)
    {

        /* satellite exclusion */
        for (j = k = 0;j < n;j++)
        {
            if (j == i) continue;
            obs_e[k] = obs[j];
            matcpy(rs_e + 6 * k, rs + 6 * j, 6, 1);
            matcpy(dts_e + 2 * k, dts + 2 * j, 2, 1);
            vare_e[k] = vare[j];
            svh_e[k++] = svh[j];
        }
        /* estimate receiver position without a satellite */
        if (!estpos(obs_e, n - 1, rs_e, dts_e, vare_e, svh_e, nav, opt, ssat, &sol_e, azel_e,
            vsat_e, resp_e, msg_e))
        {
            trace(3, "raim_fde: exsat=%2d (%s)\n", obs[i].sat, msg);
            continue;
        }
        for (j = nvsat = 0, rms_e = 0.0;j < n - 1;j++)
        {
            if (!vsat_e[j]) continue;
            rms_e += SQR(resp_e[j]);
            nvsat++;
        }
        if (nvsat < 5)
        {
            trace(3, "raim_fde: exsat=%2d lack of satellites nvsat=%2d\n",
                obs[i].sat, nvsat);
            continue;
        }
        rms_e = sqrt(rms_e / nvsat);

        trace(3, "raim_fde: exsat=%2d rms=%8.3f\n", obs[i].sat, rms_e);

        if (rms_e > rms) continue;

        /* save result */
        for (j = k = 0;j < n;j++)
        {
            if (j == i) continue;
            matcpy(azel + 2 * j, azel_e + 2 * k, 2, 1);
            vsat[j] = vsat_e[k];
            resp[j] = resp_e[k++];
        }
        stat = 1;
        sol_e.eventime = sol->eventime;
        *sol = sol_e;
        sat = obs[i].sat;
        rms = rms_e;
        vsat[i] = 0;
        strcpy(msg, msg_e);
    }
    if (stat)
    {
        time2str(obs[0].time, tstr, 2); satno2id(sat, name);
        trace(2, "%s: %s excluded by raim\n", tstr + 11, name);
    }
    free(obs_e);
    free(rs_e); free(dts_e); free(vare_e); free(azel_e);
    free(svh_e); free(vsat_e); free(resp_e);
    return stat;
}
/* range rate residuals ------------------------------------------------------*/
static int resdop(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const double* rr, const double* x,
    const double* azel, const int* vsat, double err, double* v,
    double* H, const prcopt_t* opt)
{
    double freq, rate, pos[3], E[9], a[3], e[3], vs[3], cosel, sig;
    int i, j, nv = 0;

    trace(3, "resdop  : n=%d\n", n);

    ecef2pos(rr, pos); xyz2enu(pos, E);

    for (i = 0;i < n && i < MAXOBS;i++)
    {

        freq = sat2freq(obs[i].sat, obs[i].code[0], nav);

        if (obs[i].D[0] == 0.0 || freq == 0.0 || !vsat[i] || norm(rs + 3 + i * 6, 3) <= 0.0)
        {
            continue;
        }
        /* LOS (line-of-sight) vector in ECEF */
        cosel = cos(azel[1 + i * 2]);
        a[0] = sin(azel[i * 2]) * cosel;
        a[1] = cos(azel[i * 2]) * cosel;
        a[2] = sin(azel[1 + i * 2]);
        matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);

        /* satellite velocity relative to receiver in ECEF */
        for (j = 0;j < 3;j++)
        {
            vs[j] = rs[j + 3 + i * 6] - x[j];
        }
        /* range rate with earth rotation correction */
        rate = dot(vs, e, 3) + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] -
            rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);

        /* Std of range rate error (m/s) */
        sig = (err <= 0.0) ? 1.0 : err * CLIGHT / freq;
        //        sig= varerr(satsys(obs[i].sat,NULL),azel[1+i*2],obs[i].SNR[0]*SNR_UNIT,0,opt,2);
                /* range rate residual (m/s) */
        v[nv] = (-obs[i].D[0] * CLIGHT / freq - (rate + x[3] - CLIGHT * dts[1 + i * 2])) / sig;

        /* design matrix */
        for (j = 0;j < 4;j++)
        {
            H[j + nv * 4] = ((j < 3) ? -e[j] : 1.0) / sig;
        }
        nv++;
    }
    return nv;
}
///* test satellite system (m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN) ---------*/
//static int test_sys(int sys, int m)
//{
//    switch (sys) {
//        case SYS_GPS: return m==0;
//        case SYS_SBS: return m==0;
//        case SYS_GLO: return m==1;
//        case SYS_GAL: return m==2;
//        case SYS_CMP: return m==3;
//        case SYS_QZS: return m==4;
//        case SYS_IRN: return m==5;
//    }
//    return 0;
//}
/* double-differenced measurement error covariance ---------------------------
*
*   nb[n]:  # of sat pairs in group
*   n:      # of groups (2 for each system, phase and code)
*   Ri[nv]: variances of first sats in double diff pairs
*   Rj[nv]: variances of 2nd sats in double diff pairs
*   nv:     total # of sat pairs
*   R[nv][nv]:  double diff measurement err covariance matrix       */
static void ddcov(const int* nb, int n, const double* Ri, const double* Rj,
    int nv, double* R)
{
    int i, j, k = 0, b;

    trace(3, "ddcov   : n=%d\n", n);

    for (i = 0;i < nv * nv;i++) R[i] = 0.0;
    for (b = 0;b < n;k += nb[b++])
    {  /* loop through each system */

        for (i = 0;i < nb[b];i++) for (j = 0;j < nb[b];j++)
        {
            R[k + i + (k + j) * nv] = Ri[k + i] + (i == j ? Rj[k + i] : 0.0);
        }
    }
    trace(5, "R=\n"); tracemat(5, R, nv, nv, 8, 6);
}

/* UD (undifferenced) phase/code residual for satellite ----------------------*/
static void zdres_sat_dop(int base, const obsd_t* obs, const nav_t* nav, const double* azel,
    const prcopt_t* opt, double* y, double* freq, double rate, double rcv_clk_drift,
    const double* dts)
{
    int i, nf = NF(opt);

    for (i = 0;i < nf;i++)
    {
        if ((freq[i] = sat2freq(obs->sat, obs->code[i], nav)) == 0.0) continue;

        /* check SNR mask */
        if (testsnr(base, i, azel[1], obs->SNR[i] * SNR_UNIT, &opt->snrmask))
        {
            continue;
        }
        /* residuals = observable - estimated range */
        if (obs->D[i] != 0.0 && base == 0)
            y[i] = -CLIGHT / freq[i] * obs->D[i] - (rate + rcv_clk_drift - CLIGHT * dts[1 + i * 2]);
    }
}

static int zdres_dop(int base, const obsd_t* obs, int n, const double* rs,
    const double* dts, const double* var, const int* svh,
    const nav_t* nav, const double* rr, const double* x, const prcopt_t* opt,
    double* y, double* e, const double* azel, double* freq)
{
    double rr_[3], pos[3];
    int i, nf = NF(opt);

    trace(3, "zdres   : n=%d rr=%.2f %.2f %.2f\n", n, rr[0], rr[1], rr[2]);

    double E[9], a[3], e_doppler[3], vs[3], cosel;
    int j;
    double rate = 0;

    /* init residuals to zero */
    for (i = 0; i < n * nf; i++) y[i] = 0.0;

    if (norm(rr, 3) <= 0.0) return 0; /* no receiver position */

    /* rr_ = local copy of rcvr pos */
    for (i = 0; i < 3; i++) rr_[i] = rr[i];

    /* translate rcvr pos from ecef to geodetic */
    ecef2pos(rr_, pos);
    xyz2enu(pos, E);

    /* loop through satellites */
    for (i = 0; i < n; i++)
    {
        /* compute geometric-range and azimuth/elevation angle */
        if ((geodist(rs + i * 6, rr_, e + i * 3)) <= 0.0) continue;

        /* excluded satellite? */
        if (satexclude(obs[i].sat, var[i], svh[i], opt)) continue;

        if (norm(rs + 3 + i * 6, 3) <= 0.0)
        {
            continue;
        }

        /* line-of-sight vector in ecef */
        cosel = cos(azel[1 + i * 2]);
        a[0] = sin(azel[i * 2]) * cosel;
        a[1] = cos(azel[i * 2]) * cosel;
        a[2] = sin(azel[1 + i * 2]);
        matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e_doppler);
        /* satellite velocity relative to receiver in ecef */
        for (j = 0; j < 3; j++) vs[j] = rs[j + 3 + i * 6] - x[j];
        /* range rate with earth rotation correction */
        rate = dot(vs, e_doppler, 3) + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] -
            rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);

        /* calc undifferenced phase/code residual for satellite */
        zdres_sat_dop(base, obs + i, nav, azel + i * 2, opt, y + i * nf, freq + i * nf, rate, 0, dts);
    }

    return 1;

}

static int ddres_dop(const prcopt_t* opt, const obsd_t* obs, double* y, double* e,
    const double* azel, double* freq, int ns, double* v, double* H, double* R, int iter)
{
    double threshadj;
    double* Ri, * Rj, freqi, freqj, * Hi = NULL;
    int i, j, k, m, f, nv = 0, nb[NFREQ * 6 * 3 + 2] = { 0 }, b = 0, sysi, sysj, nf = NF(opt);
    int frq;


    Ri = mat(ns * nf + 2, 1); Rj = mat(ns * nf + 2, 1);
    /* step through sat systems: m=0:gps/sbs,1:glo,2:gal,3:bds 4:qzs 5:irn*/
    for (m = 0;m < 6;m++)
    {
        /* step through phases/codes */
        for (f = 0;f < nf;f++)
        {
            frq = f % nf;

            /* find reference satellite with highest elevation, set to i */
            for (i = -1, j = 0;j < ns;j++)
            {
                sysi = satsys(obs[j].sat, NULL);
                if (!test_sys(sysi, m) || sysi == SYS_SBS) continue;
                if (y[f + i * nf] == 0.0 && y[f + j * nf] == 0.0) continue;
                /* skip sat with slip unless no other valid sat */
                if (i < 0 || azel[1 + j * 2] >= azel[1 + i * 2]) i = j;
            }
            if (i < 0) continue;

            /* calculate double differences of residuals (code/phase) for each sat */
            for (j = 0;j < ns;j++)
            {
                if (i == j) continue;  /* skip ref sat */
                sysi = satsys(obs[i].sat, NULL);
                sysj = satsys(obs[j].sat, NULL);
                freqi = freq[frq + i * nf];
                freqj = freq[frq + j * nf];
                if (freqi <= 0.0 || freqj <= 0.0) continue;
                if (!test_sys(sysj, m)) continue;
                if (y[f + i * nf] == 0.0 && y[f + j * nf] == 0.0) continue;
                if (H)
                {
                    Hi = H + nv * 3;
                    for (k = 0;k < 3;k++) Hi[k] = 0.0;
                }

                /* SD residual Doppler*/
                v[nv] = (y[f + i * nf] - y[f + j * nf]);
                /* partial derivatives by rover position, combine unit vectors from two sats */
                if (H)
                {
                    for (k = 0; k < 3; k++)
                    {
                        Hi[k] = -e[k + i * 3] + e[k + j * 3];
                    }
                }

                threshadj = (iter == 0 ? 10 : 1);
                //                /* adjust threshold by error stdev ratio unless one of the phase biases was just initialized*/
                //                bool init=(rtk->P[ii+rtk->nx*ii]==SQR(rtk->opt.std[0]))||
                //                          (rtk->P[jj+rtk->nx*jj]==SQR(rtk->opt.std[0]));


                                /* if residual too large, flag as outlier */
                                /* adjust threshold by error stdev ratio unless one of the phase biases was just initialized*/
                //                threshadj=code||(P[ii+rtk->nx*ii]==SQR(rtk->opt.std[0]))||
                //                        (P[jj+rtk->nx*jj]==SQR(rtk->opt.std[0]))?opt->eratio[frq]:1;
                //                if ((opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno*threshadj)
                if ((fabs(v[nv]) > opt->maxinno[2] * threshadj))
                {
                    continue;
                }

                /* single-differenced measurement error variances (m) */
                else
                {
                    Ri[nv] = varerr(satsys(obs[i].sat, NULL), azel[1 + i * 2], obs[i].SNR[frq] * SNR_UNIT, frq, opt, 2);
                    Rj[nv] = varerr(satsys(obs[j].sat, NULL), azel[1 + j * 2], obs[j].SNR[frq] * SNR_UNIT, frq, opt, 2);
                }

                nv++;
                nb[b]++;
            }
            b++;
        }
    }  /* end of system loop */

    /* double-differenced measurement error covariance */
    ddcov(nb, b, Ri, Rj, nv, R);

    free(Ri); free(Rj);
    return nv;
}

/* range rate residuals ------------------------------------------------------*/
static int resdop_P(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const double* rr, const double* x,
    const double* azel, const int* vsat, double err, double* v,
    double* H, double* P, const prcopt_t* opt)
{
    double freq, rate, pos[3], E[9], a[3], e[3], vs[3], cosel, sig;
    int i, j, nv = 0;

    trace(3, "resdop  : n=%d\n", n);

    ecef2pos(rr, pos); xyz2enu(pos, E);

    for (i = 0;i < n && i < MAXOBS;i++)
    {

        freq = sat2freq(obs[i].sat, obs[i].code[0], nav);

        if (obs[i].D[0] == 0.0 || freq == 0.0 || !vsat[i] || norm(rs + 3 + i * 6, 3) <= 0.0)
        {
            continue;
        }
        /* LOS (line-of-sight) vector in ECEF */
        cosel = cos(azel[1 + i * 2]);
        a[0] = sin(azel[i * 2]) * cosel;
        a[1] = cos(azel[i * 2]) * cosel;
        a[2] = sin(azel[1 + i * 2]);
        matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);

        /* satellite velocity relative to receiver in ECEF */
        for (j = 0;j < 3;j++)
        {
            vs[j] = rs[j + 3 + i * 6] - x[j];
        }
        /* range rate with earth rotation correction */
        rate = dot(vs, e, 3) + OMGE / CLIGHT * (rs[4 + i * 6] * rr[0] + rs[1 + i * 6] * x[0] -
            rs[3 + i * 6] * rr[1] - rs[i * 6] * x[1]);

        /* Std of range rate error (m/s) */
//        P[nv+nv*n]=pow((err<=0.0)?1.0:err*CLIGHT/freq,2);
        P[nv + nv * n] = varerr(satsys(obs[i].sat, NULL), azel[1 + i * 2], obs[i].SNR[0] * SNR_UNIT, 0, opt, 2);
        /* range rate residual (m/s) */
        v[nv] = (-obs[i].D[0] * CLIGHT / freq - (rate + x[3] - CLIGHT * dts[1 + i * 2]));

        /* design matrix */
        for (j = 0;j < 4;j++)
        {
            H[j + nv * 4] = ((j < 3) ? -e[j] : 1.0);
        }
        nv++;
    }
    return nv;
}


/* td residuals -------------------------------------------------------------*/
static int td_res(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const double* rr, const double* prr, const double* x,
    const double* azel, const int* vsat, double err, double* v,
    double* H, ssat_t* ssat, int iter, double* vare, const prcopt_t* opt)
{
    double freq, rate, pos[3], E[9], a[3], e[3], cosel, sig;
    int i, j, nv = 0;
    double cvs[3], pvs[3];
    int sys;
    double dion = 0.0, vion = 0.0, dtrp = 0.0, vtrp = 0.0, vmeas = 0.0;
    trace(3, "resdop  : n=%d\n", n);

    ecef2pos(rr, pos); xyz2enu(pos, E);

    /** init previous measurements before save**/
    if (iter == 11)
    {
        for (i = 0;i < MAXSAT;i++)
        {
            ssat[i].ref_y[0] = 0.0;
            for (j = 0; j < 3; j++)
            {
                ssat[i].ref_rs[j] = 0.0;
                ssat[i].ref_e[j] = 0.0;
            }
            ssat[i].ref_dts = 0.0;
            ssat[i].ref_dtrp[0] = 0.0;
            ssat[i].ref_dion[0] = 0.0;
        }
    }

    for (i = 0;i < n && i < MAXOBS;i++)
    {

        freq = sat2freq(obs[i].sat, obs[i].code[0], nav);
        sys = satsys(obs[i].sat, NULL);

        if (obs[i].L[0] == 0.0 || freq == 0.0 || !vsat[i] || norm(rs + 3 + i * 6, 3) <= 0.0 || (ssat[obs[i].sat - 1].slip[0] & 1))
        {
            continue;
        }
        if (iter < 10)
        {
            if (ssat[obs[i].sat - 1].ref_y[0] == 0.0) continue;
        }

        /* LOS (line-of-sight) vector in ECEF */
        cosel = cos(azel[1 + i * 2]);
        a[0] = sin(azel[i * 2]) * cosel;
        a[1] = cos(azel[i * 2]) * cosel;
        a[2] = sin(azel[1 + i * 2]);
        matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);

        /* satellite velocity relative to receiver in ECEF */
        for (j = 0;j < 3;j++)
        {
            cvs[j] = rs[j + i * 6] - prr[j];
            pvs[j] = ssat[obs[i].sat - 1].ref_rs[j] - prr[j];
        }
        ///TODO:TDCP也要改正地球自转吗？
        /* range rate with earth rotation correction */
//        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*x[0]-
//                                      rs[3+i*6]*rr[1]-rs[  i*6]*x[1]);

        /* variance calculate */
//        if ((prange(obs + i, nav, opt, &vmeas)) == 0.0) continue;
        if (iter >= 0)
        {
            /* test SNR mask */
            if (!snrmask(obs + i, azel + i * 2, opt)) continue;

            /* ionospheric correction */
            if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, opt->ionoopt, &dion, &vion))
            {
                continue;
            }
            dion *= SQR(FREQL1 / freq);
            vion *= SQR(FREQL1 / freq);

            /* tropospheric correction */
            if (!tropcorr(obs[i].time, nav, pos, azel + i * 2, opt->tropopt, &dtrp, &vtrp))
            {
                continue;
            }
        }

        if (iter == 11)
        {
            ssat[obs[i].sat - 1].ref_y[0] = obs[i].L[0];
            for (j = 0;j < 3;j++)
            {
                ssat[obs[i].sat - 1].ref_rs[j] = rs[j + i * 6];
                ssat[obs[i].sat - 1].ref_e[j] = e[j];
            }
            ssat[obs[i].sat - 1].ref_dts = dts[i * 2];
            ssat[obs[i].sat - 1].ref_dtrp[0] = dtrp;
            ssat[obs[i].sat - 1].ref_dion[0] = dion;
            continue;
            nv++;
        }
        /* Std of range rate error (m/s) */
//        sig=varerr(opt,&ssat[i],&obs[i],azel[1+i*2],sys) + vare[i] + vmeas + vion + vtrp;
        sig = sqrt(varerr(sys, azel[1 + i * 2], obs[i].SNR[0] * SNR_UNIT, 0, opt, 0));// + vare[i] + vmeas + vion + vtrp;
        /* range rate residual (m/s) */
        v[nv] = ((obs[i].L[0] - ssat[obs[i].sat - 1].ref_y[0]) * CLIGHT / freq -
            (
                -dot(e, x, 3) + x[3]
                + dot(cvs, e, 3) - dot(pvs, ssat[obs[i].sat - 1].ref_e, 3)
                - CLIGHT * (dts[i * 2] - ssat[obs[i].sat - 1].ref_dts)
                + dtrp - ssat[obs[i].sat - 1].ref_dtrp[0] - dion + ssat[obs[i].sat - 1].ref_dion[0]
                )) / sig;

        if (fabs(v[nv]) > 50) continue;
        //        printf("sig: %15.8lf v[nv]: %15.8lf iter: %d\n",sig,v[nv],iter);
                /* design matrix */
        for (j = 0;j < 4;j++)
        {
            H[j + nv * 4] = ((j < 3) ? -e[j] : 1.0) / sig;
        }
        nv++;
    }
    return nv;
}

/* td residuals 提取P矩阵-------------------------------------------------------------*/
static int td_res_P(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const double* rr, const double* prr, const double* x,
    const double* azel, const int* vsat, double err, double* v,
    double* H, ssat_t* ssat, int iter, double* vare, const prcopt_t* opt,
    double* P)
{
    double freq, rate, pos[3], E[9], a[3], e[3], cosel, sig;
    int i, j, nv = 0;
    double cvs[3], pvs[3];
    int sys;
    double dion = 0.0, vion = 0.0, dtrp = 0.0, vtrp = 0.0, vmeas = 0.0;
    trace(3, "resdop  : n=%d\n", n);

    ecef2pos(rr, pos); xyz2enu(pos, E);

    /** init previous measurements before save**/
    if (iter == 11)
    {
        for (i = 0;i < MAXSAT;i++)
        {
            ssat[i].ref_y[0] = 0.0;
            for (j = 0; j < 3; j++)
            {
                ssat[i].ref_rs[j] = 0.0;
                ssat[i].ref_e[j] = 0.0;
            }
            ssat[i].ref_dts = 0.0;
            ssat[i].ref_dtrp[0] = 0.0;
            ssat[i].ref_dion[0] = 0.0;
        }
    }

    for (i = 0;i < n && i < MAXOBS;i++)
    {

        freq = sat2freq(obs[i].sat, obs[i].code[0], nav);
        sys = satsys(obs[i].sat, NULL);

        if (obs[i].L[0] == 0.0 || freq == 0.0 || !vsat[i] || norm(rs + 3 + i * 6, 3) <= 0.0 || (ssat[obs[i].sat - 1].slip[0] & 1))
        {
            //            printf("obs[i].L[0]==0.0 :%d freq==0.0 :%d !vsat[i] :%d norm(rs+3+i*6,3)<=0.0 :%d (ssat[obs[i].sat-1].slip[0]&1 :%d",obs[i].L[0]==0.0,freq==0.0,!vsat[i],norm(rs+3+i*6,3)<=0.0,(ssat[obs[i].sat-1].slip[0]&1));
            continue;
        }
        if (iter < 10)
        {
            if (ssat[obs[i].sat - 1].ref_y[0] == 0.0) continue;
        }

        /* LOS (line-of-sight) vector in ECEF */
        cosel = cos(azel[1 + i * 2]);
        a[0] = sin(azel[i * 2]) * cosel;
        a[1] = cos(azel[i * 2]) * cosel;
        a[2] = sin(azel[1 + i * 2]);
        matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);

        /* satellite velocity relative to receiver in ECEF */
        for (j = 0;j < 3;j++)
        {
            cvs[j] = rs[j + i * 6] - prr[j];
            pvs[j] = ssat[obs[i].sat - 1].ref_rs[j] - prr[j];
        }
        ///TODO:TDCP也要改正地球自转吗？
        /* range rate with earth rotation correction */
//        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*x[0]-
//                                      rs[3+i*6]*rr[1]-rs[  i*6]*x[1]);

        /* variance calculate */
        if ((prange(obs + i, nav, opt, &vmeas)) == 0.0) continue;
        if (iter >= 0)
        {
            /* test SNR mask */
            if (!snrmask(obs + i, azel + i * 2, opt)) continue;

            /* ionospheric correction */
            if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, opt->ionoopt, &dion, &vion))
            {
                continue;
            }
            dion *= SQR(FREQL1 / freq);
            vion *= SQR(FREQL1 / freq);

            /* tropospheric correction */
            if (!tropcorr(obs[i].time, nav, pos, azel + i * 2, opt->tropopt, &dtrp, &vtrp))
            {
                continue;
            }
        }

        if (iter == 11)
        {
            ssat[obs[i].sat - 1].ref_y[0] = obs[i].L[0];
            for (j = 0;j < 3;j++)
            {
                ssat[obs[i].sat - 1].ref_rs[j] = rs[j + i * 6];
                ssat[obs[i].sat - 1].ref_e[j] = e[j];
            }
            ssat[obs[i].sat - 1].ref_dts = dts[i * 2];
            ssat[obs[i].sat - 1].ref_dtrp[0] = dtrp;
            ssat[obs[i].sat - 1].ref_dion[0] = dion;
            continue;
            nv++;
        }
        /* Std of range rate error (m/s) */
//        sig=varerr(opt,&ssat[i],&obs[i],azel[1+i*2],sys) + vare[i] + vmeas + vion + vtrp;
        P[nv + nv * n] = varerr(sys, azel[1 + i * 2], obs[i].SNR[0] * SNR_UNIT, 0, opt, 0);// + vare[i] + vmeas + vion + vtrp;
        /* range rate residual (m/s) */
        v[nv] = ((obs[i].L[0] - ssat[obs[i].sat - 1].ref_y[0]) * CLIGHT / freq -
            (
                -dot(e, x, 3) + x[3]
                + dot(cvs, e, 3) - dot(pvs, ssat[obs[i].sat - 1].ref_e, 3)
                - CLIGHT * (dts[i * 2] - ssat[obs[i].sat - 1].ref_dts)
                //                       +dtrp-ssat[obs[i].sat - 1].ref_dtrp[0]-dion+ssat[obs[i].sat - 1].ref_dion[0]
                ));
        ssat[obs[i].sat - 1].restdcp[0] = v[nv];

        if (iter > 0 && fabs(v[nv]) > 100) continue;
        //        printf("sig: %15.8lf v[nv]: %15.8lf iter: %d\n",sig,v[nv],iter);
                /* design matrix */
        for (j = 0;j < 4;j++)
        {
            H[j + nv * 4] = ((j < 3) ? -e[j] : 1.0);
        }
        nv++;
    }
    return nv;
}

/***
 * 采用多頻數據
 * @param obs
 * @param n
 * @param rs
 * @param dts
 * @param nav
 * @param rr
 * @param prr
 * @param x
 * @param azel
 * @param vsat
 * @param err
 * @param v
 * @param H
 * @param ssat
 * @param iter
 * @param vare
 * @param opt
 * @param P
 * @return
 */
static int td_res_P_nf(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const double* rr, const double* prr, const double* x,
    const double* azel, const int* vsat, double err, double* v,
    double* H, ssat_t* ssat, int iter, double* vare, const prcopt_t* opt,
    double* P)
{
    double freq, rate, pos[3], E[9], a[3], e[3], cosel, sig;
    int i, j, f, nv = 0;
    double cvs[3], pvs[3];
    int sys;
    double dion = 0.0, vion = 0.0, dtrp = 0.0, vtrp = 0.0, vmeas = 0.0;
    trace(3, "resdop  : n=%d\n", n);

    ecef2pos(rr, pos); xyz2enu(pos, E);

    /** init previous measurements before save**/
    if (iter == 11)
    {
        for (i = 0;i < MAXSAT;i++)
        {
            for (f = 0;f < NFREQ;f++)
            {
                ssat[i].ref_y[f] = 0.0;
                ssat[i].ref_dtrp[f] = 0.0;
                ssat[i].ref_dion[f] = 0.0;
            }
            for (j = 0; j < 3; j++)
            {
                ssat[i].ref_rs[j] = 0.0;
                ssat[i].ref_e[j] = 0.0;
            }
            ssat[i].ref_dts = 0.0;

        }
    }

    for (i = 0;i < n && i < MAXOBS;i++)
    {

        for (f = 0;f < NFREQ;f++)
        {
            freq = sat2freq(obs[i].sat, obs[i].code[f], nav);
            sys = satsys(obs[i].sat, NULL);

            if (obs[i].L[f] == 0.0 || freq == 0.0 || !vsat[i] || norm(rs + 3 + i * 6, 3) <= 0.0 ||
                (ssat[obs[i].sat - 1].slip[f] & 1))
            {
                //                printf("obs[i].L[0]==0.0 :%d freq==0.0 :%d !vsat[i] :%d norm(rs+3+i*6,3)<=0.0 :%d (ssat[obs[i].sat-1].slip[0]&1 :%d\n",
                //                       obs[i].L[f] == 0.0, freq == 0.0, !vsat[i], norm(rs + 3 + i * 6, 3) <= 0.0,
                //                       (ssat[obs[i].sat - 1].slip[f] & 1));
                continue;
            }
            if (iter < 10)
            {
                if (ssat[obs[i].sat - 1].ref_y[f] == 0.0) continue;
            }

            /* LOS (line-of-sight) vector in ECEF */
            cosel = cos(azel[1 + i * 2]);
            a[0] = sin(azel[i * 2]) * cosel;
            a[1] = cos(azel[i * 2]) * cosel;
            a[2] = sin(azel[1 + i * 2]);
            matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e);

            /* satellite velocity relative to receiver in ECEF */
            for (j = 0; j < 3; j++)
            {
                cvs[j] = rs[j + i * 6] - prr[j];
                pvs[j] = ssat[obs[i].sat - 1].ref_rs[j] - prr[j];
            }
            ///TODO:TDCP也要改正地球自转吗？
            /* range rate with earth rotation correction */
//        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*x[0]-
//                                      rs[3+i*6]*rr[1]-rs[  i*6]*x[1]);

            /* variance calculate */
//        if ((prange(obs + i, nav, opt, &vmeas)) == 0.0) continue;
            if (iter >= 0)
            {
                /* test SNR mask */
                if (!snrmask(obs + i, azel + i * 2, opt)) continue;

                /* ionospheric correction */
                if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, opt->ionoopt, &dion, &vion))
                {
                    continue;
                }
                dion *= SQR(FREQL1 / freq);
                vion *= SQR(FREQL1 / freq);

                /* tropospheric correction */
                if (!tropcorr(obs[i].time, nav, pos, azel + i * 2, opt->tropopt, &dtrp, &vtrp))
                {
                    continue;
                }
            }
            if (iter == 11)
            {
                ssat[obs[i].sat - 1].ref_y[f] = obs[i].L[f];
                for (j = 0; j < 3; j++)
                {
                    ssat[obs[i].sat - 1].ref_rs[j] = rs[j + i * 6];
                    ssat[obs[i].sat - 1].ref_e[j] = e[j];
                }
                ssat[obs[i].sat - 1].ref_dts = dts[i * 2];
                ssat[obs[i].sat - 1].ref_dtrp[f] = dtrp;
                ssat[obs[i].sat - 1].ref_dion[f] = dion;
                nv++;
                continue;
            }
            P[nv + nv * n] = varerr(sys, azel[1 + i * 2], obs[i].SNR[f] * SNR_UNIT, f, opt, 0);
            /* Std of range rate error (m/s) */
//        sig=varerr(opt,&ssat[i],&obs[i],azel[1+i*2],sys) + vare[i] + vmeas + vion + vtrp;
            // + vare[i] + vmeas + vion + vtrp;
            /* range rate residual (m/s) */
            v[nv] = ((obs[i].L[f] - ssat[obs[i].sat - 1].ref_y[f]) * CLIGHT / freq -
                (
                    -dot(e, x, 3) + x[3]
                    + dot(cvs, e, 3) - dot(pvs, ssat[obs[i].sat - 1].ref_e, 3)
                    - CLIGHT * (dts[i * 2] - ssat[obs[i].sat - 1].ref_dts)
                    //                             + dtrp - ssat[obs[i].sat - 1].ref_dtrp[f] - dion + ssat[obs[i].sat - 1].ref_dion[f]
                    ));

            if (iter > 0 && fabs(v[nv]) > 15) continue;
            //        printf("sig: %15.8lf v[nv]: %15.8lf iter: %d\n",sig,v[nv],iter);
                        /* design matrix */
            for (j = 0; j < 4; j++)
            {
                H[j + nv * 4] = ((j < 3) ? -e[j] : 1.0);
            }
            nv++;
        }
    }
    return nv;
}

/* UD (undifferenced) phase/code residual for satellite ----------------------*/
static void zdres_sat_td(int base, const obsd_t* obs, const nav_t* nav, const double* azel,
    const prcopt_t* opt, double* y, double* freq, double rcv_clk_drift,
    const double* dts, const ssat_t* ssat, double* e, const double* x,
    double* cvs, double* pvs, double dtrp, double dion)
{
    int i, nf = NF(opt);

    for (i = 0;i < nf;i++)
    {
        if ((freq[i] = sat2freq(obs->sat, obs->code[i], nav)) == 0.0) continue;

        /* check SNR mask */
        if (testsnr(base, i, azel[1], obs->SNR[i] * SNR_UNIT, &opt->snrmask))
        {
            continue;
        }
        //        if (iter < 10){
        //            if(ssat[obs[i].sat - 1].ref_y[0] ==0.0) continue;
        //        }
                /* residuals = observable - estimated range */
        if (obs->L[i] != 0.0 && base == 0 && ssat[obs[i].sat - 1].ref_y[i] != 0.0)
        {
            y[i] = ((obs->L[i] - ssat[obs->sat - 1].ref_y[i]) * CLIGHT / freq[i] -
                (
                    -dot(e, x, 3) + rcv_clk_drift
                    + dot(cvs, e, 3) - dot(pvs, ssat[obs->sat - 1].ref_e, 3)
                    - CLIGHT * (dts[0] - ssat[obs->sat - 1].ref_dts)
                    //                            + dtrp - ssat[obs->sat - 1].ref_dtrp[i] - dion *SQR(FREQL1/freq[i]) + ssat[obs->sat - 1].ref_dion[i]*SQR(FREQL1/freq[i])
                    ));
        }
    }
}

static int zdres_td(int base, const obsd_t* obs, int n, const double* rs,
    const double* dts, const double* var, const int* svh,
    const nav_t* nav, const double* rr, const double* prr, const double* x, const prcopt_t* opt,
    double* y, double* e, const double* azel, double* freq, ssat_t* ssat, int iter)
{

    double pos[3], E[9], a[3], e_tdcp[3], vs[3], cosel;
    int i, j, nf = NF(opt);
    double rate = 0;
    double cvs[3], pvs[3];
    double dion = 0.0, vion = 0.0, dtrp = 0.0, vtrp = 0.0, vmeas = 0.0;

    /* init residuals to zero */
    for (i = 0; i < n * nf; i++) y[i] = 0.0;

    if (norm(rr, 3) <= 0.0) return 0; /* no receiver position */

    /* translate rcvr pos from ecef to geodetic */
    ecef2pos(rr, pos);
    xyz2enu(pos, E);

    /* loop through satellites */
    for (i = 0; i < n; i++)
    {
        /* compute geometric-range and azimuth/elevation angle */
        if ((geodist(rs + i * 6, rr, e + i * 3)) <= 0.0) continue;

        /* excluded satellite? */
        if (satexclude(obs[i].sat, var[i], svh[i], opt)) continue;

        if (norm(rs + 3 + i * 6, 3) <= 0.0)
        {
            continue;
        }

        /* line-of-sight vector in ecef */
        cosel = cos(azel[1 + i * 2]);
        a[0] = sin(azel[i * 2]) * cosel;
        a[1] = cos(azel[i * 2]) * cosel;
        a[2] = sin(azel[1 + i * 2]);
        matmul("TN", 3, 1, 3, 1.0, E, a, 0.0, e_tdcp);
        /* satellite velocity relative to receiver in ecef */
        for (j = 0;j < 3;j++)
        {
            cvs[j] = rs[j + i * 6] - prr[j];
            pvs[j] = ssat[obs[i].sat - 1].ref_rs[j] - prr[j];
        }
        /* test SNR mask */
        if (!snrmask(obs + i, azel + i * 2, opt)) continue;

        /* ionospheric correction */
        if (!ionocorr(obs[i].time, nav, obs[i].sat, pos, azel + i * 2, opt->ionoopt, &dion, &vion))
        {
            continue;
        }
        //        dion*=SQR(FREQL1/freq);
        //        vion*=SQR(FREQL1/freq);

                /* tropospheric correction */
        if (!tropcorr(obs[i].time, nav, pos, azel + i * 2, opt->tropopt, &dtrp, &vtrp))
        {
            continue;
        }
        /* calc undifferenced phase/code residual for satellite */
        zdres_sat_td(base, obs + i, nav, azel + i * 2, opt, y + i * nf, freq + i * nf, 0, dts + i * 2, ssat, e + i * 3, x, cvs, pvs, dtrp, dion);
    }

    return 1;

}

static int ddres_td(const prcopt_t* opt, const obsd_t* obs, double* y, double* e,
    const double* azel, double* freq, int ns, double* v, double* H, double* R, int iter)
{
    double threshadj;
    double* Ri, * Rj, freqi, freqj, * Hi = NULL;
    int i, j, k, m, f, nv = 0, nb[NFREQ * 6 * 3 + 2] = { 0 }, b = 0, sysi, sysj, nf = NF(opt);
    int frq;


    Ri = mat(ns * nf + 2, 1); Rj = mat(ns * nf + 2, 1);
    /* step through sat systems: m=0:gps/sbs,1:glo,2:gal,3:bds 4:qzs 5:irn*/
    for (m = 0;m < 6;m++)
    {
        /* step through phases/codes */
        for (f = 0;f < nf;f++)
        {
            frq = f % nf;

            /* find reference satellite with highest elevation, set to i */
            for (i = -1, j = 0;j < ns;j++)
            {
                sysi = satsys(obs[j].sat, NULL);
                if (!test_sys(sysi, m) || sysi == SYS_SBS) continue;
                if (y[f + i * nf] == 0.0 && y[f + j * nf] == 0.0) continue;
                /* skip sat with slip unless no other valid sat */
                if (i < 0 || azel[1 + j * 2] >= azel[1 + i * 2]) i = j;
            }
            if (i < 0) continue;

            /* calculate double differences of residuals (code/phase) for each sat */
            for (j = 0;j < ns;j++)
            {
                if (i == j) continue;  /* skip ref sat */
                sysi = satsys(obs[i].sat, NULL);
                sysj = satsys(obs[j].sat, NULL);
                freqi = freq[frq + i * nf];
                freqj = freq[frq + j * nf];
                if (freqi <= 0.0 || freqj <= 0.0) continue;
                if (!test_sys(sysj, m)) continue;
                if (y[f + i * nf] == 0.0 && y[f + j * nf] == 0.0) continue;
                if (H)
                {
                    Hi = H + nv * 3;
                    for (k = 0;k < 3;k++) Hi[k] = 0.0;
                }

                /* SD residual Doppler*/
                v[nv] = (y[f + i * nf] - y[f + j * nf]);
                /* partial derivatives by rover position, combine unit vectors from two sats */
                if (H)
                {
                    for (k = 0; k < 3; k++)
                    {
                        Hi[k] = -e[k + i * 3] + e[k + j * 3];
                    }
                }

                threshadj = 1;
                //                /* adjust threshold by error stdev ratio unless one of the phase biases was just initialized*/
                //                bool init=(rtk->P[ii+rtk->nx*ii]==SQR(rtk->opt.std[0]))||
                //                          (rtk->P[jj+rtk->nx*jj]==SQR(rtk->opt.std[0]));


                                /* if residual too large, flag as outlier */
                                /* adjust threshold by error stdev ratio unless one of the phase biases was just initialized*/
                //                threshadj=code||(P[ii+rtk->nx*ii]==SQR(rtk->opt.std[0]))||
                //                        (P[jj+rtk->nx*jj]==SQR(rtk->opt.std[0]))?opt->eratio[frq]:1;
                //                if ((opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno*threshadj)
                if ((fabs(v[nv]) > opt->maxinno[2] * threshadj))
                {
                    continue;
                }

                /* single-differenced measurement error variances (m) */
                else
                {
                    Ri[nv] = varerr(satsys(obs[i].sat, NULL), azel[1 + i * 2], obs[i].SNR[frq] * SNR_UNIT, frq, opt, 2);
                    Rj[nv] = varerr(satsys(obs[j].sat, NULL), azel[1 + j * 2], obs[j].SNR[frq] * SNR_UNIT, frq, opt, 2);
                }

                nv++;
                nb[b]++;
            }
            b++;
        }
    }  /* end of system loop */

    /* double-differenced measurement error covariance */
    ddcov(nb, b, Ri, Rj, nv, R);

    free(Ri); free(Rj);
    return nv;
}

/* estimate receiver velocity using doppler------------------------------------------------*/
static void estvel(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const prcopt_t* opt, sol_t* sol,
    const double* azel, const int* vsat)
{
    double x[4] = { 0 }, dx[4], Q[16], * v, * H;
    double err = opt->err[4]; /* Doppler error (Hz) */
    int i, j, nv;

    int stat = 0;
    /*** used for robust estimator **/
    double* P, * H0, * v0;
    P = zeros(n, n);H0 = mat(4, n);v0 = mat(n, 1);

    trace(3, "estvel  : n=%d\n", n);

    v = mat(n, 1); H = mat(4, n);

    for (i = 0;i < MAXITR;i++)
    {

        /* range rate residuals (m/s) */
//        if ((nv=resdop(obs,n,rs,dts,nav,sol->rr,x,azel,vsat,err,v,H))<4) {
//            break;
//        }
        if ((nv = resdop_P(obs, n, rs, dts, nav, sol->rr, x, azel, vsat, err, v, H, P, opt)) < 4)
        {
            sol->dopstat = 0;
            break;
        }

        matcpy(H0, H, 4, n);
        matcpy(v0, v, n, 1);
        /* robust estimator */
        robust_lsq(H0, v0, P, 4, nv, n, H, v, opt, i);

        //        matprintT("H",H,4,nv,15,10);
                /* least square estimation */
        if (lsq(H, v, 4, nv, dx, Q)) break;

        //        matprintT("Q",Q,4,4,15,10);

        for (j = 0;j < 4;j++) x[j] += dx[j];

        if (norm(dx, 4) < 1E-6 || i == 9)
        {
            matcpy(sol->dopvel, x, 4, 1);
            sol->dopqv[0] = (float)Q[0];  /* xx */
            sol->dopqv[1] = (float)Q[5];  /* yy */
            sol->dopqv[2] = (float)Q[10]; /* zz */
            sol->dopqv[3] = (float)Q[1];  /* xy */
            sol->dopqv[4] = (float)Q[6];  /* yz */
            sol->dopqv[5] = (float)Q[2];  /* zx */
            sol->dopstat = 1;
            //            matcpy(sol->rr+3,x,3,1);
            //            sol->qv[0]=(float)Q[0];  /* xx */
            //            sol->qv[1]=(float)Q[5];  /* yy */
            //            sol->qv[2]=(float)Q[10]; /* zz */
            //            sol->qv[3]=(float)Q[1];  /* xy */
            //            sol->qv[4]=(float)Q[6];  /* yz */
            //            sol->qv[5]=(float)Q[2];  /* zx */
            stat = 1;
            break;
        }
    }
    free(v); free(H);
    free(P); free(H0); free(v0);
}

static void estvel_sd(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const prcopt_t* opt, sol_t* sol,
    const double* azel, const int* vsat, const double* vare, const int* svh)
{
    double x[3] = { 0 }, dx[3], Q[9], * v, * H;
    double err = opt->err[4]; /* Doppler error (Hz) */
    int i, j, nv;
    double* e, * freq, * y;
    int stat = 0;
    /*** used for robust estimator **/
    double* R;

    trace(3, "estvel  : n=%d\n", n);

    v = mat(n * NFREQ + 2, 1); R = zeros(n * NFREQ + 2, n * NFREQ + 2); H = zeros(3, n * NFREQ + 2);
    e = zeros(3, n);freq = zeros(NFREQ, n);y = zeros(NFREQ, n);


    for (i = 0;i < MAXITR;i++)
    {

        /* range rate residuals (m/s) */
        zdres_dop(0, obs, n, rs, dts, vare, svh, nav, sol->rr, x, opt, y, e, azel, freq);
        if ((nv = ddres_dop(opt, obs, y, e, azel, freq, n, v, H, R, i)) < 3)
        {
            break;
        }

        /* robust estimator */
//        robust_lsq(H0,v0,P,4,nv,n,H,v,opt,i);
//        matprintT("H",H,3,nv,15,10);
        /* least square estimation */
        if (lsq_R(H, v, R, 3, nv, dx, Q))break;

        //        matprintT("Q",Q,4,4,15,10);

        for (j = 0;j < 3;j++) x[j] += dx[j];

        if (norm(dx, 3) < 1E-6 || i == 9)
        {
            matcpy(sol->dopvel, x, 3, 1);
            sol->dopqv[0] = (float)Q[0];  /* xx */
            sol->dopqv[1] = (float)Q[4];  /* yy */
            sol->dopqv[2] = (float)Q[8]; /* zz */
            sol->dopqv[3] = (float)Q[1];  /* xy */
            sol->dopqv[4] = (float)Q[5];  /* yz */
            sol->dopqv[5] = (float)Q[2];  /* zx */
            sol->dopstat = 1;
            sol->sddopns = nv;
            //            matcpy(sol->rr+3,x,3,1);
            //            sol->qv[0]=(float)Q[0];  /* xx */
            //            sol->qv[1]=(float)Q[5];  /* yy */
            //            sol->qv[2]=(float)Q[10]; /* zz */
            //            sol->qv[3]=(float)Q[1];  /* xy */
            //            sol->qv[4]=(float)Q[6];  /* yz */
            //            sol->qv[5]=(float)Q[2];  /* zx */
            stat = 1;
            break;
        }
    }
    free(v); free(H);
    free(e); free(freq); free(y); free(R);
}
/* detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(ssat_t* ssat, const obsd_t* obs, int i, int rcv, double tt)
{
    uint32_t slip, LLI;
    int f, sat = obs[i].sat;

    trace(4, "detslp_ll: i=%d rcv=%d\n", i, rcv);

    for (f = 0;f < NFREQ;f++)
    {

        if ((obs[i].L[f] == 0.0 && obs[i].LLI[f] == 0) ||
            fabs(timediff(obs[i].time, ssat[sat - 1].pt[rcv - 1][f])) < DTTOL)
        {
            continue;
        }
        /* restore previous LLI */
        if (rcv == 1) LLI = getbitu(&ssat[sat - 1].slip[f], 0, 2); /* rover */
        else        LLI = getbitu(&ssat[sat - 1].slip[f], 2, 2); /* base  */

        /* detect slip by cycle slip flag in LLI */
        if (tt >= 0.0)
        { /* forward */
            if (obs[i].LLI[f] & 1)
            {

            }
            slip = obs[i].LLI[f];
        }
        else
        { /* backward */
            if (LLI & 1)
            {

            }
            slip = LLI;
        }
        /* detect slip by parity unknown flag transition in LLI */
        if (((LLI & 2) && !(obs[i].LLI[f] & 2)) || (!(LLI & 2) && (obs[i].LLI[f] & 2)))
        {
            slip |= 1;
        }
        /* save current LLI */
        if (rcv == 1) setbitu(&ssat[sat - 1].slip[f], 0, 2, obs[i].LLI[f]);
        else        setbitu(&ssat[sat - 1].slip[f], 2, 2, obs[i].LLI[f]);

        /* save slip and half-cycle valid flag */
        ssat[sat - 1].slip[f] |= (uint8_t)slip;
        ssat[sat - 1].half[f] = (obs[i].LLI[f] & 2) ? 0 : 1;
    }
}


/* detect cycle slip by doppler and phase difference -------------------------*/
static void detslp_dop(ssat_t* ssat, const obsd_t* obs, int i, int rcv,
    const nav_t* nav, double err)
{
    const double DTTOL_GNSS = 0.005;            /* tolerance of time difference (s) */
    const double MAXACC = 30.0;     /* max accel for doppler slip detection (m/s^2) */

    int f, sat = obs[i].sat;double tt, dph, dpt, lam, thres;

    trace(4, "detslp_dop: i=%d rcv=%d\n", i, rcv);

    double freq = sat2freq(obs[i].sat, obs[i].code[0], nav);

    for (f = 0;f < NFREQ;f++)
    {
        if (obs[i].L[f] == 0.0 || obs[i].D[f] == 0.0 || ssat[sat - 1].ph[rcv - 1][f] == 0.0)
        {
            continue;
        }
        if (fabs(tt = timediff(obs[i].time, ssat[sat - 1].pt[rcv - 1][f])) < DTTOL_GNSS) continue;
        lam = CLIGHT / freq;
        if (lam <= 0.0) continue;

        /* cycle slip threshold (cycle) */
        thres = MAXACC * tt * tt / 2.0 / lam + err * fabs(tt) * 4.0;

        /* phase difference and doppler x time (cycle) */
        dph = obs[i].L[f] - ssat[sat - 1].ph[rcv - 1][f];
        dpt = -obs[i].D[f] * tt;

        if (fabs(dph - dpt) <= thres) continue;

        ssat[sat - 1].slip[f] |= 1;
    }
}

/* estimate receiver velocity using TDCP------------------------------------------------*/
static void estvel_td(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const prcopt_t* opt, sol_t* sol,
    const double* azel, const int* vsat, ssat_t* ssat, double* vare)
{
    double x[4] = { 0 }, dx[4], Q[16], * v, * H;
    double err = opt->err[4]; /* Doppler error (Hz) */
    int i, j, nv;

    /*** 尝试用多普勒初始化 ***/
    double var;
    var = sol->dopqv[0] + sol->dopqv[1] + sol->dopqv[2];
    if (var < 100)
    {
        for (i = 0; i < 4; i++)
        {
            x[i] = sol->dopvel[i];
        }
    }
    /*** used for robust estimator **/
    double* P, * H0, * v0;
    P = zeros(n, n);H0 = mat(4, n);v0 = mat(n, 1);

    v = mat(n, 1); H = mat(4, n);

    for (i = 0;i < n && i < MAXOBS;i++)
    {
        /* detect cycle slip by LLI */
        for (int k = 0;k < NFREQ;k++) ssat[obs[i].sat - 1].slip[k] &= 0xFC;
        detslp_ll(ssat, obs, i, 1, sol->tt);
        /* detect cycle slip by doppler and phase difference */
//        detslp_dop(rtk,obs,iu[i],1,nav);
    }

    for (i = 0;i < MAXITR;i++)
    {

        /* range rate residuals (m/s) */
        ///TODO:添加一个配置参数判断：1.单频tdcp 2.多频tdcp
//        if ((nv=td_res(obs,n,rs,dts,nav,sol->rr,sol->ref_rr,x,azel,vsat,err,v,H,ssat,i,vare,opt))<4) {
//            break;
//        }
        if ((nv = td_res_P(obs, n, rs, dts, nav, sol->rr, sol->ref_rr, x, azel, vsat, err, v, H, ssat, i, vare, opt, P)) < 4)
        {
            //            sol->tdcpstat = 0;
            //            break;
            //        }
            //        if ((nv=td_res_P_nf(obs,n,rs,dts,nav,sol->rr,sol->ref_rr,x,azel,vsat,err,v,H,ssat,i,vare,opt,P))<4) {
            sol->tdcpvel[0] = 0;
            sol->tdcpvel[1] = 0;
            sol->tdcpvel[2] = 0;
            sol->tdcpstat = 0;
            sol->tdcpns = 0;
            break;
        }
        //        matprintT("P",P,n,n,15,10);
        //        sleepms(1000);
        matcpy(H0, H, 4, n);
        matcpy(v0, v, n, 1);
        /* robust estimator */
        robust_lsq(H0, v0, P, 4, nv, n, H, v, opt, i);

        /* least square estimation */
        if (lsq(H, v, 4, nv, dx, Q)) break;

        for (j = 0;j < 4;j++) x[j] += dx[j];

        //        matprintT("dx",dx,1,4,15,10);
        if (norm(dx, 4) < 1E-6 || i == 9)
        {
            trace(3, "estvel : vx=%.3f vy=%.3f vz=%.3f, n=%d\n", x[0], x[1], x[2], n);
            matcpy(sol->tdcpvel, x, 3, 1);
            sol->tdcpqv[0] = (float)Q[0];  /* xx */
            sol->tdcpqv[1] = (float)Q[5];  /* yy */
            sol->tdcpqv[2] = (float)Q[10]; /* zz */
            sol->tdcpqv[3] = (float)Q[1];  /* xy */
            sol->tdcpqv[4] = (float)Q[6];  /* yz */
            sol->tdcpqv[5] = (float)Q[2];  /* zx */
            sol->tdcpstat = 1;
            sol->tdcpns = nv;
            //            matcpy(sol->rr+3,x,3,1);
            //            sol->qv[0]=(float)Q[0];  /* xx */
            //            sol->qv[1]=(float)Q[5];  /* yy */
            //            sol->qv[2]=(float)Q[10]; /* zz */
            //            sol->qv[3]=(float)Q[1];  /* xy */
            //            sol->qv[4]=(float)Q[6];  /* yz */
            //            sol->qv[5]=(float)Q[2];  /* zx */
            break;
        }
    }

    //    td_res_P_nf(obs,n,rs,dts,nav,sol->rr,sol->ref_rr,x,azel,vsat,err,v,H,ssat,11,vare, opt,P);
    td_res(obs, n, rs, dts, nav, sol->rr, sol->ref_rr, x, azel, vsat, err, v, H, ssat, 11, vare, opt);

    free(v); free(H);
    free(P); free(H0); free(v0);
}
/* estimate receiver velocity ------------------------------------------------*/
static void detect_cycleslip(const obsd_t* obs, int n,
    const nav_t* nav, const prcopt_t* opt, sol_t* sol,
    ssat_t* ssat)
{
    double err = opt->err[4]; /* Doppler error (Hz) */

    for (int i = 0;i < n && i < MAXOBS;i++)
    {
        /* detect cycle slip by LLI */
        for (int k = 0;k < NFREQ;k++) ssat[obs[i].sat - 1].slip[k] &= 0xFC;
        detslp_ll(ssat, obs, i, 1, sol->tt);
        /* detect cycle slip by doppler and phase difference */
//        detslp_dop(ssat,obs,i,1,nav,err);
    }
}


static void estvel_td_sd(const obsd_t* obs, int n, const double* rs, const double* dts,
    const nav_t* nav, const prcopt_t* opt, sol_t* sol,
    const double* azel, const int* vsat, ssat_t* ssat, double* vare, int* svh)
{
    double x[3] = { 0 }, dx[3], Q[9], * v, * H;
    double err = opt->err[4]; /* Doppler error (Hz) */
    int i, j, nv;
    double* e, * freq, * y;
    int stat = 0;
    /*** used for robust estimator **/
    double* R;

    trace(3, "estvel  : n=%d\n", n);
    double var;
    var = sol->dopqv[0] + sol->dopqv[1] + sol->dopqv[2];
    if (var < 100)
    {
        for (i = 0; i < 3; i++)
        {
            x[i] = sol->dopvel[i];
        }
    }

    v = mat(n * NFREQ + 2, 1); R = zeros(n * NFREQ + 2, n * NFREQ + 2); H = zeros(3, n * NFREQ + 2);
    e = zeros(3, n);freq = zeros(NFREQ, n);y = zeros(NFREQ, n);

    for (i = 0;i < n && i < MAXOBS;i++)
    {
        /* detect cycle slip by LLI */
        for (int k = 0;k < NFREQ;k++) ssat[obs[i].sat - 1].slip[k] &= 0xFC;
        detslp_ll(ssat, obs, i, 1, sol->tt);
        /* detect cycle slip by doppler and phase difference */
//        detslp_dop(rtk,obs,iu[i],1,nav);
    }

    for (i = 0;i < MAXITR;i++)
    {

        /* range rate residuals (m/s) */
        zdres_td(0, obs, n, rs, dts, vare, svh, nav, sol->rr, sol->ref_rr, x, opt, y, e, azel, freq, ssat, i);
        if ((nv = ddres_td(opt, obs, y, e, azel, freq, n, v, H, R, i)) < 3)
        {
            sol->sdtdcpstat = 0;
            sol->sdtdcpns = 0;
            break;
        }

        /* robust estimator */
//        robust_lsq(H0,v0,P,4,nv,n,H,v,opt,i);
//        matprintT("H",H,3,nv,15,10);
        /* least square estimation */
        if (lsq_R(H, v, R, 3, nv, dx, Q))break;

        //        matprintT("Q",Q,4,4,15,10);

        for (j = 0;j < 3;j++) x[j] += dx[j];

        if (norm(dx, 3) < 1E-6 || i == 9)
        {
            matcpy(sol->tdcpvel, x, 3, 1);
            sol->tdcpqv[0] = (float)Q[0];  /* xx */
            sol->tdcpqv[1] = (float)Q[4];  /* yy */
            sol->tdcpqv[2] = (float)Q[8]; /* zz */
            sol->tdcpqv[3] = (float)Q[1];  /* xy */
            sol->tdcpqv[4] = (float)Q[5];  /* yz */
            sol->tdcpqv[5] = (float)Q[2];  /* zx */
            sol->tdcpstat = 1;
            sol->tdcpns = nv;
            //            matcpy(sol->rr+3,x,3,1);
            //            sol->qv[0]=(float)Q[0];  /* xx */
            //            sol->qv[1]=(float)Q[5];  /* yy */
            //            sol->qv[2]=(float)Q[10]; /* zz */
            //            sol->qv[3]=(float)Q[1];  /* xy */
            //            sol->qv[4]=(float)Q[6];  /* yz */
            //            sol->qv[5]=(float)Q[2];  /* zx */
            stat = 1;
            break;
        }
    }
    nv = td_res_P_nf(obs, n, rs, dts, nav, sol->rr, sol->ref_rr, x, azel, vsat, err, v, H, ssat, 11, vare, opt, NULL);
    free(v); free(H);
    free(e); free(freq); free(y); free(R);
}

static void velcheck(sol_t* sol)
{
    if (sol->dopstat)
    {
        matcpy(sol->checkvel, sol->dopvel, 3, 1);
        memcpy(sol->checkqv, sol->dopqv, sizeof(float) * 6);
        sol->checkstat = 2;
    }
    if (sol->tdcpstat)    
{
        matcpy(sol->checkvel, sol->tdcpvel, 3, 1);
        memcpy(sol->checkqv, sol->tdcpqv, sizeof(float) * 6);
        sol->checkstat = 1;
    }
}


/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int pntpos(const obsd_t* obs, int n, const nav_t* nav,
    const prcopt_t* opt, sol_t* sol, double* azel, ssat_t* ssat,
    char* msg)
{
    prcopt_t opt_ = *opt;
    double* rs, * dts, * var, * azel_, * resp;
    int i, stat, vsat[MAXOBS] = { 0 }, svh[MAXOBS];

    trace(3, "pntpos  : tobs=%s n=%d\n", time_str(obs[0].time, 3), n);

    sol->stat = SOLQ_NONE;

    Eigen::Vector3d RxPosPre(sol->rr[0], sol->rr[1], sol->rr[2]);

    if (n <= 0)
    {
        strcpy(msg, "no observation data");
        return 0;
    }
    gtime_t time = sol->time; /* previous epoch */
    sol->time = obs[0].time;
    sol->tt = timediff(sol->time, time);
    msg[0] = '\0';
    sol->eventime = obs[0].eventime;

    rs = mat(6, n); dts = mat(2, n); var = mat(1, n); azel_ = zeros(2, n); resp = mat(1, n);

    if (ssat)
    {
        for (i = 0;i < MAXSAT;i++)
        {
            ssat[i].snr_rover[0] = 0;
            ssat[i].snr_base[0] = 0;
        }
        for (i = 0;i < n;i++)
            ssat[obs[i].sat - 1].snr_rover[0] = obs[i].SNR[0];
    }

    if (opt_.mode != PMODE_SINGLE)
    { /* for precise positioning */
        opt_.ionoopt = IONOOPT_BRDC;
        opt_.tropopt = TROPOPT_SAAS;
    }

    /* construct data for WLS with nlosExclusion::GNSS_Raw_Array*/
//    rtklib::GNSS_Raw_Array gnss_data;
//    int current_week = 0;
//    double current_tow = time2gpst(obs[0].time, &current_week);
//    double epoch_time[100];
//    time2epoch(obs[0].time, epoch_time);

    /* satellite positons, velocities and clocks */
    satposs(sol->time, obs, n, nav, opt_.sateph, rs, dts, var, svh);

    for (i = 0;i < n;i++)
    {
        int msat = obs[i].sat;
        ssat[msat - 1].ephvar = var[i];
        ssat[msat - 1].svh = svh[i];
    }

    //    detect_cycleslip(obs,n,nav,opt,sol,ssat);

        /* estimate receiver position with pseudorange */
    stat = estpos(obs, n, rs, dts, var, svh, nav, &opt_, ssat, sol, azel_, vsat, resp, msg);

    /* RAIM FDE */
    if (!stat && n >= 6 && opt->posopt[4])
    {
        stat = raim_fde(obs, n, rs, dts, var, svh, nav, &opt_, ssat, sol, azel_, vsat, resp, msg);
    }
    /* estimate receiver velocity with Doppler */
    /* estimate receiver velocity with TDCP */
    if (stat)
    {
        estvel(obs, n, rs, dts, nav, &opt_, sol, azel_, vsat);
        //        estvel_sd(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat,var,svh);
        //        estvel_td(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat,ssat,var);
        //        estvel_td_sd(obs, n,rs, dts,nav, &opt_, sol,azel_, vsat,ssat, var,svh);
                /***check velocity***/
        velcheck(sol);
        //        printf("%.2lf %lf %lf %lf\n", (sol->time.time+sol->time.sec),sol->rr[3],sol->rr[4],sol->rr[5]);
        matcpy(sol->rr + 3, sol->checkvel, 3, 1);
        //        matcpy(sol->rr+3,sol->tdcpvel,3,1);
        //        memcpy(sol->ref_rr,sol->rr,sizeof(double)*6);
        for (i = 0;i < 6;i++)
        {
            sol->qv[i] = sol->checkqv[i];
        }
        //        fprintf(opt->fp,"%.2lf %lf %lf %lf\n", (sol->time.time+sol->time.sec),sol->rr[3],sol->rr[4],sol->rr[5]);
        //        fflush(opt->fp);
    }


    double pos[3], vel[3];
    ecef2pos(sol->rr, pos);
    ecef2enu(pos, sol->rr + 3, vel);

    if (azel)
    {
        for (i = 0;i < n * 2;i++) azel[i] = azel_[i];
    }
    if (ssat)
    {
        for (i = 0;i < MAXSAT;i++)
        {
            ssat[i].vs = 0;
            ssat[i].azel[0] = ssat[i].azel[1] = 0.0;
            ssat[i].resp[0] = ssat[i].resc[0] = 0.0;
        }
        for (i = 0;i < n;i++)
        {
            ssat[obs[i].sat - 1].azel[0] = azel_[i * 2];
            ssat[obs[i].sat - 1].azel[1] = azel_[1 + i * 2];
            if (!vsat[i]) continue;
            ssat[obs[i].sat - 1].vs = 1;
            ssat[obs[i].sat - 1].resp[0] = resp[i];
            //            printf("resp %.3lf\n",resp[i]);
            for (int j = 0;j < NFREQ;j++)
            {
                double freq = sat2freq(obs[i].sat, obs[i].code[j], nav);
                if (freq != 0)
                    ssat[obs[i].sat - 1].lam[j] = CLIGHT / freq;
                else
                    ssat[obs[i].sat - 1].lam[j] = 0;
            }
            for (int type = 0;type < 3;type++)
                for (int j = 0;j < 3;j++)
                    ssat[obs[i].sat - 1].outlier_obs[type][j] = 0;
            ssat[obs[i].sat - 1].sys = satsys(obs[i].sat, NULL);
        }
        for (i = 0;i < 3;i++)
            sol->vv[i] = sol->rr[3 + i];
    }


    free(rs); free(dts); free(var); free(azel_); free(resp);
    return stat;
}
