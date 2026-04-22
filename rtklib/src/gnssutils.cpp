//
// Created by long on 2023/6/7.
//

#include "rtklib.h"
#define MAX(x,y)    ((x)>=(y)?(x):(y))

extern rtklib::obsdt obs2msg(obsd_t obs)
{
    rtklib::obsdt obsmsg;
    obsmsg.time = (double)obs.time.time + obs.time.sec;
    obsmsg.sat = obs.sat;
    obsmsg.rcv = obs.rcv;
    memcpy(obsmsg.SNR.data(), obs.SNR, sizeof(uint16_t) * 3);
    memcpy(obsmsg.LLI.data(), obs.LLI, sizeof(uint8_t) * 3);
    memcpy(obsmsg.code.data(), obs.code, sizeof(uint8_t) * 3);
    memcpy(obsmsg.L.data(), obs.L, sizeof(double) * 3);
    memcpy(obsmsg.P.data(), obs.P, sizeof(double) * 3);
    memcpy(obsmsg.D.data(), obs.D, sizeof(double) * 3);

    obsmsg.timevalid = obs.timevalid;
    obsmsg.eventime = (double)obs.eventime.time + obs.eventime.sec;
    memcpy(obsmsg.Lstd.data(), obs.qualL, sizeof(uint8_t) * 3);
    memcpy(obsmsg.Pstd.data(), obs.qualP, sizeof(uint8_t) * 3);
    obsmsg.freq = obs.freq;
    return obsmsg;
}

extern rtklib::sat_state ssat2msg(ssat_t ssat)
{
    rtklib::sat_state satState;

    satState.sys = ssat.sys;
    satState.vs = ssat.vs;
    memcpy(satState.azel.data(), ssat.azel, sizeof(double) * 2);
    memcpy(satState.azel_b.data(), ssat.azel_b, sizeof(double) * 2);
    memcpy(satState.resp.data(), ssat.resp, sizeof(double) * 3);
    memcpy(satState.resc.data(), ssat.resc, sizeof(double) * 3);
    memcpy(satState.resd.data(), ssat.resd, sizeof(double) * 3);
    memcpy(satState.icbias.data(), ssat.icbias, sizeof(double) * 3);

    memcpy(satState.vsat.data(), ssat.vsat, sizeof(uint8_t) * 3);
    memcpy(satState.vsatL.data(), ssat.vsatL, sizeof(uint8_t) * 3);
    memcpy(satState.vsatP.data(), ssat.vsatP, sizeof(uint8_t) * 3);
    memcpy(satState.vsatD.data(), ssat.vsatD, sizeof(uint8_t) * 3);

    memcpy(satState.snr_rover.data(), ssat.snr_rover, sizeof(uint16_t) * 3);
    memcpy(satState.snr_base.data(), ssat.snr_base, sizeof(uint16_t) * 3);

    memcpy(satState.fix.data(), ssat.fix, sizeof(uint8_t) * 3);
    memcpy(satState.slip.data(), ssat.slip, sizeof(uint8_t) * 3);
    memcpy(satState.half.data(), ssat.half, sizeof(uint8_t) * 3);
    memcpy(satState.outc.data(), ssat.outc, sizeof(int) * 3);
    memcpy(satState.slipc.data(), ssat.slipc, sizeof(int) * 3);
    memcpy(satState.rejc.data(), ssat.rejc, sizeof(int) * 3);

    memcpy(satState.gf.data(), ssat.gf, sizeof(double) * 2);
    memcpy(satState.mw.data(), ssat.mw, sizeof(int) * 2);
    satState.phw = ssat.phw;
    memcpy(satState.pt.data(), ssat.pt, sizeof(double) * 6);
    memcpy(satState.ph.data(), ssat.ph, sizeof(double) * 6);

    memcpy(satState.P_COR.data(), ssat.P_COR, sizeof(double) * 3);
    memcpy(satState.tgd.data(), ssat.tgd, sizeof(double) * 3);
    memcpy(satState.init.data(), ssat.init, sizeof(int) * 3);

    memcpy(satState.outlier_obs.data(), ssat.outlier_obs, sizeof(int) * 9);
    memcpy(satState.lam.data(), ssat.lam, sizeof(double) * 3);
    memcpy(satState.amb_id.data(), ssat.amb_id, sizeof(int) * 3);
    satState.ephvar = ssat.ephvar;
    satState.svh = ssat.svh;

    return satState;

}