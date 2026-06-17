#include "AP_NavEKF3_core.h"
#include <cstdlib>
#include <cstdio>

namespace {
FILE *ekf3_att_debug_file()
{
    // Opt-in via env var so default flights are clean (heavy logging stalls the
    // real-time loop and distorts the trajectory). Set EK3_ATT_DEBUG=1 to enable.
    static const bool _ek3_att_dbg_enabled = (std::getenv("EK3_ATT_DEBUG") != nullptr);
    if (!_ek3_att_dbg_enabled) { return nullptr; }
    static FILE *fp = nullptr;
    if (fp == nullptr) {
        fp = std::fopen("/home/mbublyk/Documents/ardu/estimator_files/ardupilot_ekf3_att_debug.log", "a");

        if (fp != nullptr) { static char ek3buf[1*1024*1024]; std::setvbuf(fp, ek3buf, _IOFBF, sizeof(ek3buf)); std::atexit([](){ std::fflush(nullptr); }); }
    }
    return fp;
}
}

// reset the body axis gyro bias states to zero and re-initialise the corresponding covariances
// Assume that the calibration is performed to an accuracy of 0.5 deg/sec which will require averaging under static conditions
// WARNING - a non-blocking calibration method must be used
void NavEKF3_core::resetGyroBias(void)
{
    const Vector3F gyro_bias_before = stateStruct.gyro_bias;
    const ftype p_before_10 = P[10][10];
    const ftype p_before_11 = P[11][11];
    const ftype p_before_12 = P[12][12];

    stateStruct.gyro_bias.zero();
    zeroRows(P,10,12);
    zeroCols(P,10,12);

    P[10][10] = sq(radians(0.5f * dtIMUavg));
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];

    if (FILE *fp = ekf3_att_debug_file()) {
        std::fprintf(
            fp,
            "EK3_STATE_DELTA ts_ms=%lu time_us=%llu core=%u imu=%u source=GYRO_BIAS_RESET gyro_bias_before=(%.9g,%.9g,%.9g) gyro_bias_after=(%.9g,%.9g,%.9g) d_gyro_bias=(%.9g,%.9g,%.9g) P_gyro_bias_before=(%.9g,%.9g,%.9g) P_gyro_bias_after=(%.9g,%.9g,%.9g)\n",
            (unsigned long)imuSampleTime_ms,
            (unsigned long long)imuDataDelayed.time_ms,
            (unsigned)core_index,
            (unsigned)imu_index,
            (double)gyro_bias_before.x, (double)gyro_bias_before.y, (double)gyro_bias_before.z,
            (double)stateStruct.gyro_bias.x, (double)stateStruct.gyro_bias.y, (double)stateStruct.gyro_bias.z,
            (double)(stateStruct.gyro_bias.x-gyro_bias_before.x), (double)(stateStruct.gyro_bias.y-gyro_bias_before.y), (double)(stateStruct.gyro_bias.z-gyro_bias_before.z),
            (double)p_before_10, (double)p_before_11, (double)p_before_12,
            (double)P[10][10], (double)P[11][11], (double)P[12][12]);
    }
}

/*
   vehicle specific initial gyro bias uncertainty in deg/sec
 */
ftype NavEKF3_core::InitialGyroBiasUncertainty(void) const
{
    return 2.5f;
}

