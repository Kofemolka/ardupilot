#include "AP_Beacon_Sine.h"

#if AP_BEACON_SINE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

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
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Beacon_Sine::update");

    set_beacon_position(0, Vector3f(100.0f, 100.0f, 1.0f));
    set_beacon_distance(0, 50.0f);

    set_beacon_position(1, Vector3f(200.0f, 100.0f, 1.0f));
    set_beacon_distance(1, 50.0f);

    last_update_ms = AP_HAL::millis();
}

#endif // AP_BEACON_SINE_ENABLED