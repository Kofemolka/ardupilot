#pragma once

#include "AP_PipeDash_config.h"

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
