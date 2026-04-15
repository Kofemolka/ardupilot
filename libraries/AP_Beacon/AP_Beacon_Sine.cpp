#include "AP_Beacon_Sine.h"

#if AP_BEACON_SINE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_PipeDash/AP_PipeDash.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_Beacon_Sine::AP_Beacon_Sine(AP_Beacon &frontend)
    : AP_Beacon_Backend(frontend) {
  // Pre-register all beacons so num_beacons is non-zero when the DAL
  // snapshots the count on the first EKF frame.  Real position and distance
  // values are filled in once measurements arrive.
  for (uint8_t i = 0; i < AP_BEACON_MAX_BEACONS; i++) {
    set_beacon_position(i, Vector3f(0.0f, 0.0f, 0.0f));
    set_beacon_distance(i, -1.0f);
  }
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Sine::healthy() {
  return (AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS;
}

// update the state of the sensor
void AP_Beacon_Sine::update(void) {}

// handle mavlink message
void AP_Beacon_Sine::handle_msg(const mavlink_message_t &msg) {
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
 * Range message layout (22 bytes):
 *   [0]      msg_type  uint8   = 0
 *   [1]      id        uint8
 *   [2..5]   lat       int32   degrees × 1e7
 *   [6..9]   lon       int32   degrees × 1e7
 *   [10..13] alt       float32 metres AMSL
 *   [14..17] range     float32 metres
 *   [18..21] variance  float32 m²
 */
void AP_Beacon_Sine::handle_range_msg(const uint8_t *payload,
                                      uint8_t payload_length) {
  uint8_t beacon_id = payload[1];
  if (beacon_id >= AP_BEACON_MAX_BEACONS) {
    return;
  }

  // Discard until the EKF has an origin to translate against.
  Location ekf_origin;
  if (!AP::ahrs().get_origin(ekf_origin)) {
    return;
  }

  int32_t lat, lon;
  float alt_m, range_m, variance;
  memcpy(&lat, payload + 2, sizeof(lat));
  memcpy(&lon, payload + 6, sizeof(lon));
  memcpy(&alt_m, payload + 10, sizeof(alt_m));
  memcpy(&range_m, payload + 14, sizeof(range_m));
  memcpy(&variance, payload + 18, sizeof(variance));

  const Location beacon_loc(lat, lon, (int32_t)(alt_m * 100.0f),
                            Location::AltFrame::ABSOLUTE);
  const Vector3f ned = ekf_origin.get_distance_NED(beacon_loc);

  set_beacon_position(beacon_id, ned);
  set_beacon_distance(beacon_id, range_m);
}

/*
 * Pose message layout (13 bytes):
 *   [0]    msg_type  uint8   = 1
 *   [1..4] lat       int32   degrees × 1e7
 *   [5..8] lon       int32   degrees × 1e7
 *   [9..12] pos_error float32  metres
 */
void AP_Beacon_Sine::handle_pose_msg(const uint8_t *payload,
                                     uint8_t payload_length) {
  // Discard until the EKF has an origin to translate against.
  Location ekf_origin;
  if (!AP::ahrs().get_origin(ekf_origin)) {
    return;
  }

  int32_t lat, lon;
  float pos_error;
  memcpy(&lat, payload + 1, sizeof(lat));
  memcpy(&lon, payload + 5, sizeof(lon));
  memcpy(&pos_error, payload + 9, sizeof(pos_error));

  // Use AHRS AMSL altitude for the vehicle position; fall back to 0 if
  // unavailable.
  int32_t alt_amsl_cm = 0;
  Location ahrs_loc;
  if (AP::ahrs().get_location(ahrs_loc)) {
    int32_t ahrs_alt;
    if (ahrs_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ahrs_alt)) {
      alt_amsl_cm = ahrs_alt;
    }
  }

  const Location vehicle_loc(lat, lon, alt_amsl_cm,
                             Location::AltFrame::ABSOLUTE);
  const Vector3f ned = ekf_origin.get_distance_NED(vehicle_loc);

  set_vehicle_position(ned, pos_error);
}

#endif // AP_BEACON_SINE_ENABLED
