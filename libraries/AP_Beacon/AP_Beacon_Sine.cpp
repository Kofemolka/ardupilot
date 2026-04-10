#include "AP_Beacon_Sine.h"

#if AP_BEACON_SINE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_Beacon_Sine::AP_Beacon_Sine(AP_Beacon &frontend)
    : AP_Beacon_Backend(frontend)
{
    // Pre-register all beacons so num_beacons is non-zero when the DAL
    // snapshots the count on the first EKF frame.  Real position and distance
    // values are filled in once measurements arrive.
    for (uint8_t i = 0; i < AP_BEACON_MAX_BEACONS; i++) {
        set_beacon_position(i, Vector3f(0.0f, 0.0f, 0.0f));
        set_beacon_distance(i, -1.0f);
    }
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Sine::healthy()
{
    return (AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS;
}

// update the state of the sensor
void AP_Beacon_Sine::update(void) {}

// handle mavlink message
void AP_Beacon_Sine::handle_msg(const mavlink_message_t &msg)
{
    if (msg.msgid != MAVLINK_MSG_ID_TUNNEL) {
        return;
    }

    mavlink_tunnel_t pkt;
    mavlink_msg_tunnel_decode(&msg, &pkt);

    if (pkt.payload_type != 66 || pkt.payload_length < 13) {
        return;
    }

    switch (pkt.payload[0]) {
    case 0:
        if (pkt.payload_length >= 21) {
            handle_range_msg(pkt.payload, pkt.payload_length);
        }
        break;
    case 1:
        handle_pose_msg(pkt.payload, pkt.payload_length);
        break;
    default:
        return;
    }

    last_update_ms = AP_HAL::millis();
}

/*
 * Range message layout (21 bytes):
 *   [0]      msg_type  uint8   = 0
 *   [1]      id        uint8
 *   [2..5]   x_north   float32  metres in beacon NED frame
 *   [6..9]   y_east    float32  metres in beacon NED frame
 *   [10..13] z_down    float32  metres in beacon NED frame
 *   [14..17] range     float32  metres
 *   [18..21] variance  float32  m²
 */
void AP_Beacon_Sine::handle_range_msg(const uint8_t *payload, uint8_t payload_length)
{
    uint8_t beacon_id = payload[1];
    if (beacon_id >= AP_BEACON_MAX_BEACONS) {
        return;
    }

    float x, y, z, range_m, variance;
    memcpy(&x,        payload + 2,  sizeof(x));
    memcpy(&y,        payload + 6,  sizeof(y));
    memcpy(&z,        payload + 10, sizeof(z));
    memcpy(&range_m,  payload + 14, sizeof(range_m));
    memcpy(&variance, payload + 18, sizeof(variance));

    set_beacon_position(beacon_id, Vector3f(x, y, z));
    set_beacon_distance(beacon_id, range_m);
}

/*
 * Pose message layout (13 bytes):
 *   [0]    msg_type  uint8   = 1
 *   [1..4] x_north   float32  metres in beacon NED frame
 *   [5..8] y_east    float32  metres in beacon NED frame
 *   [9..12] pos_error float32  metres
 */
void AP_Beacon_Sine::handle_pose_msg(const uint8_t *payload, uint8_t payload_length)
{
    float x, y, pos_error;
    memcpy(&x,         payload + 1, sizeof(x));
    memcpy(&y,         payload + 5, sizeof(y));
    memcpy(&pos_error, payload + 9, sizeof(pos_error));

    // z=0: altitude is handled by the EKF height source (baro/GPS), not beacons
    set_vehicle_position(Vector3f(x, y, 0.0f), pos_error);
}

#endif // AP_BEACON_SINE_ENABLED
