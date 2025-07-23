#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>


// Board-specific directories should be defined in hwdef.h
#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY "/logs"
#endif
#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY "/terrain"
#endif

#include "HAL_ESP32_Namespace.h"
#include "HAL_ESP32_Class.h"


#define HAL_CANIface ESP32::CANIface
