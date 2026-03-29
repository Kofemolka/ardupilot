#include "AP_Beacon_Sine.h"

#if AP_BEACON_SINE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
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
    
}

// handle mavlink message
void AP_Beacon_Sine::handle_msg(const mavlink_message_t &msg)
{
    mavlink_file_transfer_protocol_t pkt;
    mavlink_msg_file_transfer_protocol_decode(&msg, &pkt);

    if (pkt.target_component != MAV_COMP_ID_USER66) {
        return;
    }

    // parse custom beacon payload: "<Bfffff" = uint8 beacon_id, float lat, lon, alt, range_m, variance
    uint8_t beacon_id = pkt.payload[0];
    float lat, lon, alt, range_m, variance;
    const uint8_t *p = pkt.payload;
    memcpy(&lat,      p +  1, sizeof(lat));
    memcpy(&lon,      p +  5, sizeof(lon));
    memcpy(&alt,      p +  9, sizeof(alt));
    memcpy(&range_m,  p + 13, sizeof(range_m));
    memcpy(&variance, p + 17, sizeof(variance));

    // convert lat/lon/alt to NED relative to EKF origin
    Location ekf_origin;
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return; // EKF not yet initialised
    }
    Location beacon_loc;
    beacon_loc.lat = (int32_t)(lat * 1.0e7f);
    beacon_loc.lng = (int32_t)(lon * 1.0e7f);
    beacon_loc.alt = (int32_t)(alt * 100.0f); // metres → cm
    const Vector3f pos_ned = ekf_origin.get_distance_NED(beacon_loc);

    gcs().send_text(MAV_SEVERITY_INFO,
        "Beacon %u: lat=%.7f lon=%.7f alt=%.2f range=%.2fm var=%.4f NED=(%.1f,%.1f,%.1f)",
        (unsigned)beacon_id, (double)lat, (double)lon, (double)alt, (double)range_m, (double)variance,
        (double)pos_ned.x, (double)pos_ned.y, (double)pos_ned.z);

    set_beacon_position(beacon_id, pos_ned);
    set_beacon_distance(beacon_id, range_m);

    last_update_ms = AP_HAL::millis();
}

#endif // AP_BEACON_SINE_ENABLED