#include "AP_Beacon_Sine.h"

#if AP_BEACON_SINE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_Beacon_Sine::AP_Beacon_Sine(AP_Beacon &frontend)
    : AP_Beacon_Backend(frontend) {

  // set invalid distance to unblock EKF beacon fusion logic
  for (uint8_t i = 0; i < AP_BEACON_MAX_BEACONS; i++) {
    set_beacon_distance(i, -1.0f);
  }
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_Sine::healthy() {
  // healthy if we have parsed a message within the past 300ms
  const auto ok = ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);

  // gcs().send_text(MAV_SEVERITY_INFO, "[SB] %s", ok ? "OK" : "NO DATA");

  return ok;
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

  if (pkt.payload_type != 66 || pkt.payload_length < 21) {
    return;
  }

  switch (pkt.payload[0]) {
  case 0: // range message
    handle_range_msg(pkt.payload, pkt.payload_length);
    break;
  case 1: // pose message
    handle_pose_msg(pkt.payload, pkt.payload_length);
    break;
  default:
    return; // unknown message type
  }

  // set_vehicle_position(const Vector3f& pos, float accuracy_estimate);

  last_update_ms = AP_HAL::millis();
}

void AP_Beacon_Sine::handle_range_msg(const uint8_t *payload,
                                      uint8_t payload_length) {
  uint8_t beacon_id = payload[1];
  float lat, lon, alt, range_m, variance;
  const uint8_t *p = payload + 1; // skip message type byte
  memcpy(&lat, p + 1, sizeof(lat));
  memcpy(&lon, p + 5, sizeof(lon));
  memcpy(&alt, p + 9, sizeof(alt));
  memcpy(&range_m, p + 13, sizeof(range_m));
  memcpy(&variance, p + 17, sizeof(variance));

  // convert lat/lon/alt to NED relative to EKF origin
  Location ekf_origin;
  if (!AP::ahrs().get_origin(ekf_origin)) {
    return; // EKF not yet initialised
  }

  // TODO: check projection distortion for long distances

  Location beacon_loc;
  beacon_loc.lat = (int32_t)(lat * 1.0e7f);
  beacon_loc.lng = (int32_t)(lon * 1.0e7f);
  beacon_loc.alt = (int32_t)(alt * 100.0f); // metres → cm
  const Vector3f pos_ned = ekf_origin.get_distance_NED(beacon_loc);

  // gcs().send_text(MAV_SEVERITY_INFO,
  //                 "Beacon %u: lat=%.7f lon=%.7f alt=%.2f range=%.2fm
  //                 var=%.4f " "NED=(%.1f,%.1f,%.1f)", (unsigned)beacon_id,
  //                 (double)lat, (double)lon, (double)alt, (double)range_m,
  //                 (double)variance, (double)pos_ned.x, (double)pos_ned.y,
  //                 (double)pos_ned.z);

  set_beacon_position(beacon_id, pos_ned);
  set_beacon_distance(beacon_id, range_m);
}

void AP_Beacon_Sine::handle_pose_msg(const uint8_t *payload,
                                     uint8_t payload_length) {
  if (payload_length < 13) {
    return;
  }

  float lat, lon, pos_error;
  memcpy(&lat, payload + 1, sizeof(lat));
  memcpy(&lon, payload + 5, sizeof(lon));
  memcpy(&pos_error, payload + 9, sizeof(pos_error));

  // get EKF origin for NED conversion
  Location ekf_origin;
  if (!AP::ahrs().get_origin(ekf_origin)) {
    return;
  }

  // altitude not provided by external system — use AHRS estimate
  Location current_loc;
  if (!AP::ahrs().get_location(current_loc)) {
    return;
  }

  Location vehicle_loc;
  vehicle_loc.lat = (int32_t)(lat * 1.0e7f);
  vehicle_loc.lng = (int32_t)(lon * 1.0e7f);
  vehicle_loc.alt = current_loc.alt;

  const Vector3f pos_ned = ekf_origin.get_distance_NED(vehicle_loc);
  // const Vector3f local_ned = ekf_origin.get_distance_NED(current_loc);

  // gcs().send_text(MAV_SEVERITY_INFO,
  //                 "Pose: ext=(%.1f,%.1f) local=(%.1f,%.1f) err=%.2f",
  //                 (double)pos_ned.x, (double)pos_ned.y,
  //                 (double)local_ned.x, (double)local_ned.y,
  //                 (double)pos_error);

  set_vehicle_position(pos_ned, pos_error);
}

#endif // AP_BEACON_SINE_ENABLED