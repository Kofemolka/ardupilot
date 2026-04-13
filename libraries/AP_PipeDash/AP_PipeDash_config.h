#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_PIPEDASH_ENABLED
#define AP_PIPEDASH_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif
