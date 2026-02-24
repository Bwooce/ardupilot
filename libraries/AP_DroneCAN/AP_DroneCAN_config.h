#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_WITH_DRONECAN
#define HAL_WITH_DRONECAN 1
#endif

// DroneCAN node status/info MAVLink broadcasting (messages 310, 311)
// Disabled on 1MB boards (fmuv2) to stay within flash limits
#ifndef AP_DRONECAN_MAVLINK_REPORTING_ENABLED
#define AP_DRONECAN_MAVLINK_REPORTING_ENABLED (HAL_ENABLE_DRONECAN_DRIVERS && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024))
#endif

// PARAM_EXT bridge for DroneCAN node parameters (messages 320-324)
// Requires DroneCAN drivers for DSDL types and param get/set support
// Disabled on 1MB boards (fmuv2) to stay within flash limits
#ifndef AP_DRONECAN_PARAM_EXT_ENABLED
#define AP_DRONECAN_PARAM_EXT_ENABLED (HAL_ENABLE_DRONECAN_DRIVERS && (HAL_PROGRAM_SIZE_LIMIT_KB > 1024))
#endif
