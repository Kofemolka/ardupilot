#include "AP_EstimatorIpcServer.h"
#include "measurements.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace Estimator {
namespace Ipc {

constexpr const char* c_local_sock_path = "/tmp/ipc_server";
constexpr const char* c_peer_sock_path = "/tmp/ipc_client";
constexpr size_t c_buf_size = 255;

struct AP_EstimatorIpcServer::Impl {
    int sock_fd = -1;
    struct sockaddr_un local_addr = {};
    struct sockaddr_un peer_addr = {};
    uint8_t buffer[c_buf_size] = {};
    bool ready = false;
    bool peer_valid = false;
};

AP_EstimatorIpcServer& AP_EstimatorIpcServer::getSingleton()
{
    static AP_EstimatorIpcServer instance;
    return instance;
}

AP_EstimatorIpcServer::AP_EstimatorIpcServer()
    : m_impl{new Impl{}}
{
    m_impl->sock_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (m_impl->sock_fd == -1) {
        return;
    }

    m_impl->local_addr.sun_family = AF_UNIX;
    const int ret = std::snprintf(m_impl->local_addr.sun_path,
                                  sizeof(m_impl->local_addr.sun_path),
                                  "%s",
                                  c_local_sock_path);

    if (ret < 0 || static_cast<size_t>(ret) >= sizeof(m_impl->local_addr.sun_path)) {
        close(m_impl->sock_fd);
        m_impl->sock_fd = -1;
        return;
    }

    unlink(c_local_sock_path);

    if (bind(m_impl->sock_fd,
             reinterpret_cast<struct sockaddr*>(&m_impl->local_addr),
             sizeof(m_impl->local_addr)) == -1)
    {
        close(m_impl->sock_fd);
        m_impl->sock_fd = -1;
        return;
    }

    m_impl->ready = true;

    setPeer(c_peer_sock_path);
}

AP_EstimatorIpcServer::~AP_EstimatorIpcServer()
{
    if (m_impl->sock_fd >= 0) {
        close(m_impl->sock_fd);
    }
    unlink(c_local_sock_path);
}

uint64_t AP_EstimatorIpcServer::getCurrentTSMicrosec()
{
    auto duration = std::chrono::steady_clock::now().time_since_epoch();
    auto microsec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    return microsec;
}

bool AP_EstimatorIpcServer::setPeer(const char* peer_path)
{
    if (!m_impl->ready || peer_path == nullptr) {
        return false;
    }

    std::memset(&m_impl->peer_addr, 0, sizeof(m_impl->peer_addr));
    m_impl->peer_addr.sun_family = AF_UNIX;

    const int ret = std::snprintf(m_impl->peer_addr.sun_path,
                                  sizeof(m_impl->peer_addr.sun_path),
                                  "%s",
                                  peer_path);

    if (ret < 0 || static_cast<size_t>(ret) >= sizeof(m_impl->peer_addr.sun_path)) {
        m_impl->peer_valid = false;
        return false;
    }

    m_impl->peer_valid = true;
    return true;
}

void AP_EstimatorIpcServer::sendGyro(uint64_t ts, uint8_t instance, float x, float y, float z, bool healthy, bool calibrated)
{
    using namespace Estimator::Types;
    if (!m_impl->ready || !m_impl->peer_valid) {
        return;
    }

    static_assert(sizeof(Gyro) <= c_buf_size, "Gyro too large");

    Gyro msg = {};
    msg.ts = ts;
    msg.type = static_cast<uint8_t>(EType::Gyro);
    msg.instance = instance;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.healthy = healthy;
    msg.calibrated = calibrated;

    std::memcpy(m_impl->buffer, &msg, sizeof(msg));
    sendBuffer(sizeof(msg));
}

void AP_EstimatorIpcServer::sendAccel(uint64_t ts, uint8_t instance, float x, float y, float z, bool healthy, bool calibrated)
{
    using namespace Estimator::Types;
    if (!m_impl->ready || !m_impl->peer_valid) {
        return;
    }

    static_assert(sizeof(Accel) <= c_buf_size, "Accel too large");

    Accel msg = {};
    msg.ts = ts;
    msg.type = static_cast<uint8_t>(EType::Accel);
    msg.instance = instance;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.healthy = healthy;
    msg.calibrated = calibrated;

    std::memcpy(m_impl->buffer, &msg, sizeof(msg));
    sendBuffer(sizeof(msg));
}

void AP_EstimatorIpcServer::sendBarometer(uint64_t ts, uint8_t instance, float temperature, float pressure, bool healthy, bool calibrated)
{
    using namespace Estimator::Types;
    if (!m_impl->ready || !m_impl->peer_valid) {
        return;
    }

    static_assert(sizeof(Barometer) <= c_buf_size, "Barometer too large");

    Barometer msg = {};
    msg.ts = ts;
    msg.type = static_cast<uint8_t>(EType::Barometer);
    msg.instance = instance;
    msg.temperature = temperature;
    msg.pressure = pressure;
    msg.healthy = healthy;
    msg.calibrated = calibrated;

    std::memcpy(m_impl->buffer, &msg, sizeof(msg));
    sendBuffer(sizeof(msg));
}

void AP_EstimatorIpcServer::sendGPS(uint64_t ts, const AP_GPS::GPS_State& state, bool healthy)
{
    using namespace Estimator::Types;
    if (!m_impl->ready || !m_impl->peer_valid) {
        return;
    }

    static_assert(sizeof(GPS) <= c_buf_size, "GPS too large");

    GPS msg = {};
    msg.ts = ts;
    msg.type = static_cast<uint8_t>(EType::GPS);
    msg.instance = state.instance;
    msg.lat = state.location.lat;
    msg.lon = state.location.lng;
    msg.alt = state.location.alt;
    msg.vx = state.velocity.x;
    msg.vy = state.velocity.y;
    msg.vz = state.velocity.z;
    msg.fix_type = state.status;
    msg.num_sats = state.num_sats;
    msg.healthy = healthy;

    std::memcpy(m_impl->buffer, &msg, sizeof(msg));
    sendBuffer(sizeof(msg));
}

bool AP_EstimatorIpcServer::sendBuffer(std::size_t n)
{
    if (!m_impl->ready || !m_impl->peer_valid) {
        return false;
    }

    const ssize_t ret = ::sendto(m_impl->sock_fd,
                                 m_impl->buffer,
                                 n,
                                 0,
                                 reinterpret_cast<struct sockaddr*>(&m_impl->peer_addr),
                                 sizeof(m_impl->peer_addr));

    return ret == static_cast<ssize_t>(n);
}

} // namespace Ipc
} // namespace Estimator
