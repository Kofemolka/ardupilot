#include "AP_PipeDash.h"

#if AP_PIPEDASH_ENABLED

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

AP_PipeDash *AP_PipeDash::get_singleton() {
    static AP_PipeDash inst{};

    return &inst;
}

AP_PipeDash::AP_PipeDash()
{

}

void AP_PipeDash::_ensure_open()
{
    if (_fd >= 0) {
        return;
    }
    // O_WRONLY|O_NONBLOCK on a FIFO succeeds only when a reader is already
    // listening; otherwise open() returns -1 / ENXIO immediately.
    _fd = ::open(PIPE_PATH, O_WRONLY | O_NONBLOCK);
}

void AP_PipeDash::_write(const char *key, const char *value_str)
{
    _ensure_open();
    if (_fd < 0) {
        return;
    }

    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%s=%s\n", key, value_str);
    if (n <= 0 || n >= (int)sizeof(buf)) {
        return;
    }

    if (::write(_fd, buf, n) < 0) {
        // EPIPE  — reader closed the read end
        // EAGAIN — pipe buffer full (shouldn't happen at our rate)
        // Either way: close and attempt reconnect on next call.
        ::close(_fd);
        _fd = -1;
    }
}

void AP_PipeDash::set(const char *key, float value)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.4f", value);
    _write(key, buf);
}

void AP_PipeDash::set(const char *key, int32_t value)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)value);
    _write(key, buf);
}

void AP_PipeDash::set(const char *key, const char *value)
{
    _write(key, value);
}

#endif  // AP_PIPEDASH_ENABLED
