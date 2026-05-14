#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_SINE_ENABLED

class AP_Beacon_Sine : public AP_Beacon_Backend {
public:
  // constructor
  AP_Beacon_Sine(AP_Beacon &frontend);

  // return true if sensor is basically healthy (we are receiving data)
  bool healthy() override;

  // update
  void update() override;

  // handle mavlink message
  void handle_msg(const mavlink_message_t &msg) override;

private:
  void warmup();
  bool handle_range_msg(const mavlink_ranging_beacon_t& bcn_range);

  uint32_t last_update_ms = 0;

  uint32_t warmup_readings = 0;
  bool warmup_complete = false;
};

#endif // AP_BEACON_SINE_ENABLED