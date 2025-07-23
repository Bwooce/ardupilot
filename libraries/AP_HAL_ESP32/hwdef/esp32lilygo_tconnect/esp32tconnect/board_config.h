/*
 * Board configuration for LilyGO T-Connect ESP32-S3
 * ArduPilot Rover Configuration with WiFi disabled
 */

#pragma once

#include "hwdef.h"

// Board identification
#define BOARD_NAME "LilyGO T-Connect"
#define BOARD_ID 1050
#define BOARD_TYPE_DEFAULT 1050

// MCU Configuration
#define MCU_ESP32_S3 1
#define HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_TCONNECT

// Flash and Memory
#define FLASH_SIZE_KB 16384
#define HAL_STORAGE_SIZE 32768
#define HAL_MEM_CLASS HAL_MEM_CLASS_192

// CAN Bus Configuration
#define HAL_WITH_UAVCAN 1
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 4
#define HAL_NUM_CAN_IFACES 4
#define HAL_ENABLE_DRONECAN 1

// CAN Protocol Defaults
#define CAN1_PROTOCOL DRONECAN
#define CAN2_PROTOCOL DRONECAN
#define CAN3_PROTOCOL DRONECAN
#define CAN4_PROTOCOL DRONECAN

// UART Protocols
#define DEFAULT_SERIAL1_PROTOCOL SerialProtocol_GPS
#define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_MAVLink
#define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_GPS
#define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_None

// I2C Configuration
#define HAL_I2C_INTERNAL_MASK 1
#define HAL_I2C_EXTERNAL_MASK 2

// IMU Configuration
#define HAL_DEFAULT_INS_FAST_SAMPLE 1
#define HAL_IMU_TEMP_DEFAULT 55

// Compass Configuration
#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK09915

// Barometer Configuration
#define HAL_BARO_DEFAULT HAL_BARO_DPS310
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

// Rover Frame Configuration
#define HAL_DEFAULT_FRAME_CLASS 2  // Rover frame class
#define HAL_DEFAULT_FRAME_TYPE 1   // Rover frame type

// Feature Enables
#define HAL_ENABLE_LIDAR_DRIVERS 1
#define HAL_MOUNT_ENABLED 1
#define HAL_RALLY_ENABLED 1
#define HAL_BEACON_ENABLED 1
#define HAL_PROXIMITY_ENABLED 1
#define HAL_ADSB_ENABLED 1
#define HAL_TORQEEDO_ENABLED 1
#define HAL_GENERATOR_ENABLED 1
#define HAL_EFI_ENABLED 1
#define HAL_WHEEL_ENCODER_ENABLED 1
#define HAL_SPRAYER_ENABLED 1
#define HAL_GRIPPER_ENABLED 1
#define HAL_LANDING_GEAR_ENABLED 1
#define HAL_BUTTON_ENABLED 1

// Networking - DISABLED for stability and reduced overhead
// #define HAL_ENABLE_NETWORKING 1
// #define HAL_NETWORKING_BACKEND_CHIBIOS 1

// Storage
#define HAL_OS_FATFS_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"

// Radio Input
#define HAL_RCINPUT_WITH_AP_RADIO 1

// CAN Bootloader
#define HAL_ENABLE_CANBOOTLOADER 1
#define HAL_CANBOOTLOADER_TIMEOUT 5000

// Bootloader
#define HAL_BOOTLOADER_TIMEOUT 5000
#define HAL_USE_EMPTY_STORAGE 1
#define FLASH_RESERVE_START_KB 64

// USB
#define HAL_USE_OTG 1

// Debug and Statistics
#define HAL_DEBUG_BUILD 0
#define HAL_ENABLE_THREAD_STATISTICS 1

// Feature optimization
#define HAL_MINIMIZE_FEATURES 0
#define HAL_MINIMIZE_FLASH_SIZE 0

// Pin definitions for easy reference
#define PIN_LED_BOOTLOADER 13
#define PIN_LED_ACTIVITY 14
#define PIN_LED_ERROR 15
#define PIN_SAFETY_SWITCH 0
#define PIN_BUZZER 8

// ADC pins
#define PIN_BATT_VOLTAGE 0
#define PIN_BATT_CURRENT 1
#define PIN_BATT2_VOLTAGE 2
#define PIN_BATT2_CURRENT 3
#define PIN_RSSI 4

// PWM output pins for 4-wheel rover
#define PIN_SERVO1_PWM 0   // Front Left Motor
#define PIN_SERVO2_PWM 1   // Front Right Motor
#define PIN_SERVO3_PWM 15  // Rear Left Motor
#define PIN_SERVO4_PWM 3   // Rear Right Motor
#define PIN_SERVO5_PWM 6   // Front Left Steering
#define PIN_SERVO6_PWM 7   // Front Right Steering
#define PIN_SERVO7_PWM 8   // Rear Left Steering
#define PIN_SERVO8_PWM 9   // Rear Right Steering

// CAN pin definitions
#define PIN_CAN1_RX 11
#define PIN_CAN1_TX 12
#define PIN_CAN2_RX 5
#define PIN_CAN2_TX 6
#define PIN_CAN3_RX 8
#define PIN_CAN3_TX 9
#define PIN_CAN4_RX 0
#define PIN_CAN4_TX 1

// UART pin definitions
#define PIN_UART1_TX 9
#define PIN_UART1_RX 10
#define PIN_UART2_TX 2
#define PIN_UART2_RX 3
#define PIN_UART3_TX 10
#define PIN_UART3_RX 11
#define PIN_UART4_TX 10
#define PIN_UART4_RX 11
#define PIN_UART5_TX 12
#define PIN_UART5_RX 2

// I2C pin definitions
#define PIN_I2C1_SCL 6
#define PIN_I2C1_SDA 7
#define PIN_I2C2_SCL 10
#define PIN_I2C2_SDA 11

// SPI pin definitions
#define PIN_SPI1_SCK 5
#define PIN_SPI1_MISO 6
#define PIN_SPI1_MOSI 7
#define PIN_SPI1_CS 4
#define PIN_SPI2_SCK 13
#define PIN_SPI2_MISO 14
#define PIN_SPI2_MOSI 15
#define PIN_SPI2_CS1 12
#define PIN_SPI2_CS2 7
#define PIN_SPI3_SCK 10
#define PIN_SPI3_MISO 11
#define PIN_SPI3_MOSI 12
#define PIN_SPI3_CS1 3
#define PIN_SPI3_CS2 4

// Sensor I2C addresses
#define I2C_ADDRESS_COMPASS 0x0C
#define I2C_ADDRESS_BARO 0x77

// Default parameter values for rover
#define DEFAULT_FRAME_CLASS 2
#define DEFAULT_FRAME_TYPE 1

// Power monitoring scaling
#define BATT_VOLTAGE_SCALE 10.1f
#define BATT_CURRENT_SCALE 17.0f
#define BATT2_VOLTAGE_SCALE 10.1f
#define BATT2_CURRENT_SCALE 17.0f