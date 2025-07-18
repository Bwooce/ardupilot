#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ESP32/Semaphores.h>

namespace Canard {
    typedef ESP32::Semaphore Semaphore;
    typedef ESP32::BinarySemaphore BinarySemaphore;
};
