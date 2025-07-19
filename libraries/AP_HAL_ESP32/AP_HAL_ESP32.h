#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>


#define HAL_BOARD_LOG_DIRECTORY "/logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "/terrain"

#include "HAL_ESP32_Namespace.h"
#include "HAL_ESP32_Class.h"


#define HAL_CANIface ESP32::CANIface
