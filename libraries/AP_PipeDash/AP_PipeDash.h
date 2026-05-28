#pragma once

#include "AP_PipeDash_config.h"

#include <GCS_MAVLink/GCS.h>

// TODO: if WHAT?
template<typename T, uint32_t interval_ms>
class GCS_DBG {
public:
  GCS_DBG(const char* fmt) {
    strncpy(fmt_, fmt, sizeof(fmt_) - 1);
    fmt_[sizeof(fmt_) - 1] = '\0';
  }

  void update(const T value) {
    if(AP_HAL::millis() - last_update_ < interval_ms)
      return;

    last_update_ = AP_HAL::millis();
    last_value_ = value;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt_, value);
  }

  void on_change(const T value) {
    if(value != last_value_) {
      GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt_, value);

      last_value_ = value;
    }
  }

private:
  uint32_t last_update_ = 0;
  T last_value_{};
  char fmt_[50];
};

#if AP_PIPEDASH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <stdint.h>
#include <stdio.h>

/*
 * AP_PipeDash — SITL-only debug dashboard helper.
 *
 * Writes KEY=VALUE lines to a named FIFO (/tmp/ardupilot_dash) that is read
 * by pipe_dashboard.py.  Writes are non-blocking; the pipe is opened lazily
 * and re-opened automatically after a reader disconnect.
 *
 * A single global instance is constructed at program startup so the singleton
 * is available before any scheduler or vehicle init runs.
 *
 * Usage:
 *   #include <AP_PipeDash/AP_PipeDash.h>
 *   ...
 *   if (auto *dash = AP_PipeDash::get_singleton()) {
 *       dash->set("bcn.0.dist", range_m);
 *   }
 */
class AP_PipeDash {
public:
  static AP_PipeDash *get_singleton();

  // Write key=<float> (4 decimal places)
  void set(const char *key, float value);

  // Write key=<integer>
  void set(const char *key, int32_t value);

  void set(const char *key, bool value);

  // Write key=<string>  (value must not contain newlines)
  void set(const char *key, const char *value);

private:
  AP_PipeDash();
  CLASS_NO_COPY(AP_PipeDash);

  void _ensure_open();
  void _write(const char *key, const char *value_str);

  int _fd{-1};

  static constexpr const char *PIPE_PATH = "/tmp/ardupilot_dash";
};

#define PIPE(key, value)                                                       \
  do {                                                                         \
    if (auto *dash = AP_PipeDash::get_singleton()) {                           \
      dash->set(key, value);                                                   \
    }                                                                          \
  } while (0)

#else

#define PIPE(key, value)

#endif // AP_PIPEDASH_ENABLED
