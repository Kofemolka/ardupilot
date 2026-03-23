#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_SINE_ENABLED

class AP_Beacon_Sine : public AP_Beacon_Backend
{
public:
    // constructor
    using AP_Beacon_Backend::AP_Beacon_Backend;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

private:
    uint32_t last_update_ms = 0;
};

#endif // AP_BEACON_SINE_ENABLED