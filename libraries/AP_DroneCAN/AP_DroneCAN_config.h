#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// DroneCAN node status/info MAVLink broadcasting (messages 310, 311)
#ifndef AP_DRONECAN_MAVLINK_REPORTING_ENABLED
#define AP_DRONECAN_MAVLINK_REPORTING_ENABLED 0
#endif

// PARAM_EXT bridge for DroneCAN node parameters (messages 320-324)
#ifndef AP_DRONECAN_PARAM_EXT_ENABLED
#define AP_DRONECAN_PARAM_EXT_ENABLED 0
#endif
