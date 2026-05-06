#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "AP_GPS/AP_GPS.h"

namespace Estimator {
namespace Ipc {

class AP_EstimatorIpcServer {
private:
    AP_EstimatorIpcServer();

public:
    ~AP_EstimatorIpcServer();
    AP_EstimatorIpcServer(const AP_EstimatorIpcServer&) = delete;
    AP_EstimatorIpcServer& operator=(const AP_EstimatorIpcServer&) = delete;
    AP_EstimatorIpcServer(AP_EstimatorIpcServer&&) = delete;
    AP_EstimatorIpcServer& operator=(AP_EstimatorIpcServer&&) = delete;
 
    static AP_EstimatorIpcServer& getSingleton();
 
    bool setPeer(const char* peer_path);
 
    void sendGyro(uint64_t ts, uint8_t instance, float x, float y, float z, bool healthy, bool calibrated);
    void sendAccel(uint64_t ts, uint8_t instance, float x, float y, float z, bool healthy, bool calibrated);
    void sendBarometer(uint64_t ts, uint8_t instance, float temperature, float pressure, bool healthy, bool calibrated);
    void sendGPS(uint64_t ts, const AP_GPS::GPS_State& state, bool healthy);

private:
    bool sendBuffer(std::size_t n);
    uint64_t getCurrentTSMicrosec();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace Ipc
} // namespace Estimator
