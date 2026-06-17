#pragma once

#include <cstdint>

#include "time.hpp"

namespace Estimator {
namespace Types {

enum class EType : uint8_t {
    Gyro = 0,
    Accel = 1,
    Barometer = 2,
    GPS = 3,
    BeaconRange = 4,
    Magnetometer = 5,
};

enum class EGPSFixType : uint8_t {
    No_GPS = 0,
    No_Fix = 1,
    Fix_2D = 2,
    Fix_3D = 3,
    Fix_3D_DGPS = 4,
    Fix_3D_RTK_FLOAT = 5,
    Fix_3D_RTK_FIXED = 6,
};

#pragma pack(push, 1)
struct Gyro {
    uint8_t type;
    Time::TimestampMcs ts;
    uint8_t instance;
    float x;
    float y;
    float z;
    uint8_t healthy;
    uint8_t calibrated;
};

struct Accel {
    uint8_t type;
    Time::TimestampMcs ts;
    uint8_t instance;
    float x;
    float y;
    float z;
    uint8_t healthy;
    uint8_t calibrated;
};

struct Barometer {
    uint8_t type;
    Time::TimestampMcs ts;
    uint8_t instance;
    float temperature;
    float pressure;
    uint8_t healthy;
    uint8_t calibrated;
};

struct GPS {
    uint8_t type;
    Time::TimestampMcs ts;
    uint8_t instance;
    uint8_t fix_type;
    uint8_t num_sats;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    float vx;
    float vy;
    float vz;
    uint8_t healthy;
};

struct BeaconRange {
    uint8_t type;
    Time::TimestampMcs ts;
    uint8_t instance;
    int32_t lat;
    int32_t lon;
    int32_t alt;
    float range;
    float variance;
    uint8_t healthy;
};

struct Magnetometer {
    uint8_t type;
    Time::TimestampMcs ts;
    uint8_t instance;
    float x;
    float y;
    float z;
    float ofs_x;
    float ofs_y;
    float ofs_z;
    uint8_t healthy;
    uint8_t calibrated;
};
#pragma pack(pop)

} // namespace Estimator
} // namespace Types
