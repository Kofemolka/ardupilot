#include "AP_Beacon_Sine.h"

#if AP_BEACON_SINE_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Sine::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_Sine::update(void)
{

}

#endif // AP_BEACON_SINE_ENABLED