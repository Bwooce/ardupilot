#pragma once

// ESP32 needs _GNU_SOURCE for asprintf/vasprintf before any stdio.h includes
// Use same value as WAF ap_config.h to avoid redefinition warning
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

// ESP32 has different struct alignment than other platforms
// Force MAVLink to use safe byte-by-byte packing instead of direct struct access
// This must be set BEFORE any MAVLink headers are included
#ifdef MAVLINK_ALIGNED_FIELDS
#undef MAVLINK_ALIGNED_FIELDS
#endif
#define MAVLINK_ALIGNED_FIELDS 0

// ESP32: Force byte-wise field access for all MAVLink operations
// The MAVLink generator determines packing needs based on standard alignment,
// but ESP32 has different alignment requirements causing corruption in messages
// like HIGH_LATENCY2. Setting MAVLINK_ALIGNED_FIELDS=0 forces safe byte-wise
// access instead of direct struct field access, avoiding alignment issues.
#define ESP32_MAVLINK_OVERRIDE_APPLIED 1

// UHCI (UART-DMA) configuration - conditionally enable based on SOC capabilities
#ifdef SOC_UHCI_SUPPORTED
#if SOC_UHCI_SUPPORTED
#define AP_HAL_ESP32_UHCI_SUPPORTED 1
#else
#define AP_HAL_ESP32_UHCI_SUPPORTED 0
#endif
#else
#define AP_HAL_ESP32_UHCI_SUPPORTED 0
#endif

// Board-specific configuration now handled automatically by hwdef.h
// Generated during build process by esp32_hwdef.py
#include <hwdef.h>

#ifndef HAL_BOARD_STATE_DIRECTORY
#define HAL_BOARD_STATE_DIRECTORY "/SDCARD/APM"
#endif

#ifndef HAL_BOARD_LOG_DIRECTORY
#define HAL_BOARD_LOG_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/LOGS"
#endif

#ifndef HAL_BOARD_TERRAIN_DIRECTORY
#define HAL_BOARD_TERRAIN_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/TERRAIN"
#endif

#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY HAL_BOARD_STATE_DIRECTORY "/STORAGE"
#endif
#define HAL_BOARD_NAME "ESP32"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#ifndef HAL_WITH_DRONECAN
#define HAL_WITH_DRONECAN 0
#endif
#ifndef HAL_WITH_UAVCAN
#define HAL_WITH_UAVCAN 0
#endif
#ifndef HAL_MAX_CAN_PROTOCOL_DRIVERS
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0
#endif
#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0

#define HAL_WITH_IO_MCU 0

#define O_CLOEXEC 0
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE (16384)
#endif

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048
#endif

#ifdef __cplusplus
// allow for static semaphores
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore
#define HAL_BinarySemaphore ESP32::BinarySemaphore
#endif

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif
#ifndef HAL_MEM_CLASS
// Set memory class based on chip variant's internal DRAM (not PSRAM)
// ESP32: 520KB SRAM, ~280-300KB usable DRAM
// ESP32-S3: 512KB SRAM, ~320-370KB usable DRAM
// ESP32-S2: 320KB SRAM, ~200-230KB usable DRAM
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)
#define HAL_MEM_CLASS HAL_MEM_CLASS_300
#else
#define HAL_MEM_CLASS HAL_MEM_CLASS_192
#endif
#endif
// disable uncommon stuff that we'd otherwise get 
#define AP_EXTERNAL_AHRS_ENABLED 0
#ifndef HAL_GENERATOR_ENABLED
#define HAL_GENERATOR_ENABLED 0
#endif

#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

// ArduPilot uses __RAMFUNC__ to place functions in fast instruction RAM
#ifndef __RAMFUNC__
#define __RAMFUNC__ IRAM_ATTR
#endif


// whenever u get ... error: "xxxxxxx" is not defined, evaluates to 0 [-Werror=undef]  just define it below as 0
// Only define if not already defined by ESP-IDF to avoid redefinition warnings
#ifndef CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY
#define CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY 0
#endif
#ifndef XCHAL_ERRATUM_453
#define XCHAL_ERRATUM_453 0
#endif
//#define CONFIG_FREERTOS_CORETIMER_0 0
// Stack overflow checking: Enable in debug mode, disable for performance
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE
#ifdef ESP32_DEBUG_MODE
#if ESP32_DEBUG_MODE
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 0  // Enable stack checking (NONE=0 means checking is ON)
#else
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 1  // Disable for performance
#endif
#else
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 1  // Default: performance mode
#endif
#endif
#ifndef CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL 0
#endif
#ifndef CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP
#define CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP 0
#endif
#ifndef CONFIG_FREERTOS_USE_TICKLESS_IDLE
#define CONFIG_FREERTOS_USE_TICKLESS_IDLE 0
#endif
#ifndef CONFIG_SYSVIEW_ENABLE
#define CONFIG_SYSVIEW_ENABLE 0
#endif
#ifndef CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED
#define CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED 0
#endif
#ifndef CONFIG_SPI_FLASH_ENABLE_COUNTERS
#define CONFIG_SPI_FLASH_ENABLE_COUNTERS 0
#endif
#ifndef CONFIG_LWIP_DHCP_RESTORE_LAST_IP
#define CONFIG_LWIP_DHCP_RESTORE_LAST_IP 0
#endif
#ifndef CONFIG_LWIP_STATS
#define CONFIG_LWIP_STATS 0
#endif
#ifndef CONFIG_LWIP_PPP_SUPPORT
#define CONFIG_LWIP_PPP_SUPPORT 0
#endif
//#define CONFIG_ESP32_WIFI_CSI_ENABLED 0
//#define CONFIG_ESP32_WIFI_NVS_ENABLED 0
#ifndef CONFIG_NEWLIB_NANO_FORMAT
#define CONFIG_NEWLIB_NANO_FORMAT 0
#endif
#ifndef CONFIG_LWIP_IP4_REASSEMBLY
#define CONFIG_LWIP_IP4_REASSEMBLY 0
#endif
#ifndef CONFIG_LWIP_IP6_REASSEMBLY
#define CONFIG_LWIP_IP6_REASSEMBLY 0
#endif
#ifndef LWIP_COMPAT_SOCKET_INET
#define LWIP_COMPAT_SOCKET_INET 0
#endif
#ifndef LWIP_COMPAT_SOCKET_ADDR
#define LWIP_COMPAT_SOCKET_ADDR 0
#endif
//#define CONFIG_ESP32_WIFI_TX_BA_WIN 0
//#define CONFIG_ESP32_WIFI_RX_BA_WIN 0


// turn off all the compasses by default.. 
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif

// we don't need 32, 16 is enough
#define NUM_SERVO_CHANNELS 16

// disble temp cal of gyros by default
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0

//turn off a bunch of advanced plane scheduler table things. see Plane.cpp
#define AP_ADVANCEDFAILSAFE_ENABLED 0
#define AP_ICENGINE_ENABLED 0
#define AP_OPTICALFLOW_ENABLED 0
#define AP_RPM_ENABLED 0
#define AP_AIRSPEED_AUTOCAL_ENABLE 0
#ifndef HAL_MOUNT_ENABLED
#define HAL_MOUNT_ENABLED 0
#endif
#define AP_CAMERA_ENABLED 0
#define HAL_SOARING_ENABLED 0
#ifndef AP_TERRAIN_AVAILABLE
#define AP_TERRAIN_AVAILABLE 0
#endif
#define HAL_ADSB_ENABLED 0
#ifndef HAL_BUTTON_ENABLED
#define HAL_BUTTON_ENABLED 0
#endif
#ifndef AP_GRIPPER_ENABLED
#define AP_GRIPPER_ENABLED 0
#endif
#define AP_LANDINGGEAR_ENABLED 0

// disable avoid-fence-follow by default, these all kinda need each other, so its all or none.
// per-board override via hwdef.dat: define AP_FENCE_ENABLED 1
#ifndef AP_AVOIDANCE_ENABLED
#define AP_AVOIDANCE_ENABLED 0
#endif
#ifndef AP_FENCE_ENABLED
#define AP_FENCE_ENABLED 0
#endif
#ifndef MODE_FOLLOW_ENABLED
#define MODE_FOLLOW_ENABLED 0
#endif
#ifndef AP_OAPATHPLANNER_ENABLED
#define AP_OAPATHPLANNER_ENABLED 0
#endif


// other big things..
#define HAL_QUADPLANE_ENABLED 0
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif
#ifndef FFT_DEFAULT_WINDOW_SIZE
#define FFT_DEFAULT_WINDOW_SIZE 32
#endif

// remove once ESP32 isn't so chronically slow
#define AP_SCHEDULER_OVERTIME_MARGIN_US 50000UL

#ifndef AP_NOTIFY_BUZZER_ENABLED
#define AP_NOTIFY_BUZZER_ENABLED 1
#endif

// ESP32 asprintf/vasprintf support - add at end to avoid include order issues
#ifdef __cplusplus
extern "C" {
#endif
// ESP-IDF provides asprintf/vasprintf but may need feature macros enabled
#include <stdio.h>
#include <stdarg.h>
#ifdef __cplusplus
}
#endif
