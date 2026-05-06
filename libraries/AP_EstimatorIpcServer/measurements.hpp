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
    BeaconRange = 4
};

enum class EGPSFixType : uint8_t {
    No_GPS = 0,           // No GPS connected/detected
    No_Fix = 1,           // Receiving valid GPS messages but no lock
    Fix_2D = 2,           // Receiving valid messages and 2D lock
    Fix_3D = 3,           // Receiving valid messages and 3D lock
    Fix_3D_DGPS = 4,      // Receiving valid messages and 3D lock with differential improvements
    Fix_3D_RTK_FLOAT = 5, // Receiving valid messages and 3D RTK Float
    Fix_3D_RTK_FIXED = 6, // Receiving valid messages and 3D RTK Fixed
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
    uint8_t fix_type; // see EGPSFixType
    uint8_t num_sats; // number of satellites
    int32_t lat; // degrees * 1e7
    int32_t lon; // degrees * 1e7
    int32_t alt; // cm
    float vx; // m/s
    float vy; // m/s
    float vz; // m/s
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
#pragma pack(pop)

} // namespace Estimator
} // namespace Types
