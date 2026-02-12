#pragma once

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>

// ESP32 requires unaligned field handling for MAVLink
// This must be defined before any MAVLink headers are included
#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 0
#endif

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
