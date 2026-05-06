#pragma once

#include <cstdint>
#include <chrono>

namespace Estimator {
namespace Time {

using DurationMcs = int64_t; // microseconds
using TimestampMcs = uint64_t; // microseconds

inline TimestampMcs getCurrentTimestamp() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

inline DurationMcs getDiff(const TimestampMcs goal, const TimestampMcs now) {
    DurationMcs duration = static_cast<DurationMcs>(goal) - static_cast<DurationMcs>(now);
    return duration;
}

} // namespace Estimator
} // namespace Time
