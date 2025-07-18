/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_LILYGO_TCONNECT

#define HAL_ESP32_BOARD_NAME "esp32lilygo_tconnect"

#define HAL_ESP32_RCOUT_MAX 8
#define HAL_ESP32_RCOUT {GPIO_NUM_0,GPIO_NUM_1,GPIO_NUM_2,GPIO_NUM_3,GPIO_NUM_4,GPIO_NUM_5,GPIO_NUM_6,GPIO_NUM_7}

#define HAL_ESP32_SPI_BUSES \
    {.host=SPI2_HOST, .dma_ch=SPI_DMA_CH_AUTO, .mosi=GPIO_NUM_7, .miso=GPIO_NUM_6, .sclk=GPIO_NUM_5}, \
    {.host=SPI3_HOST, .dma_ch=SPI_DMA_CH_AUTO, .mosi=GPIO_NUM_15, .miso=GPIO_NUM_14, .sclk=GPIO_NUM_13}

#define HAL_ESP32_SPI_DEVICES \
    { "imu", 0, 0, GPIO_NUM_4, 3, 1000, 20000 }, \
    { "flash", 1, 0, GPIO_NUM_12, 3, 1000, 8000 }

#define HAL_ESP32_I2C_BUSES \
    {.port=I2C_NUM_0, .sda=GPIO_NUM_8, .scl=GPIO_NUM_9, .speed=400000, .internal=true, .soft=false}, \
    {.port=I2C_NUM_1, .sda=GPIO_NUM_3, .scl=GPIO_NUM_4, .speed=400000, .internal=false, .soft=false}

#define HAL_ESP32_UART_DEVICES \
    { .port = UART_NUM_0, .rx = GPIO_NUM_18, .tx = GPIO_NUM_17 }, \
    { .port = UART_NUM_1, .rx = GPIO_NUM_10, .tx = GPIO_NUM_9 }, \
    { .port = UART_NUM_2, .rx = GPIO_NUM_7, .tx = GPIO_NUM_6 }

#define HAL_ESP32_ADC_PINS { \
    {1, 1, 0}, \
    {2, 1, 1}, \
    {3, 1, 2}, \
    {4, 1, 3}, \
    {5, 1, 4}, \
    {6, 1, 5}, \
    {7, 1, 6}, \
    {8, 1, 7}, \
    {9, 2, 0}, \
    {10, 2, 1}, \
    {11, 2, 2}, \
    {12, 2, 3}, \
    {13, 2, 4}, \
    {14, 2, 5}, \
    {15, 2, 6}, \
    {16, 2, 7}, \
    {17, 2, 8}, \
    {18, 2, 9} \
}

#define HAL_CAN1_TX_PIN 4
#define HAL_CAN1_RX_PIN 5

#define HAL_ESP32_RMT_RX_PIN_NUMBER GPIO_NUM_14

#define HAL_WITH_UAVCAN 1
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 1
#define HAL_ENABLE_LIDAR_DRIVERS 1
#define HAL_ENABLE_DRONECAN 1
#define HAL_NUM_CAN_IFACES 1
#define CAN1_PROTOCOL DRONECAN
#define HAL_USE_OTG 1
#define HAL_I2C_INTERNAL_MASK 1
#define HAL_I2C_EXTERNAL_MASK 2
#define HAL_IMU_TEMP_DEFAULT 55
#define HAL_DEFAULT_INS_FAST_SAMPLE 1
#define HAL_COMPASS_DEFAULT HAL_COMPASS_AK09915
#define HAL_BARO_DEFAULT HAL_BARO_DPS310
#define HAL_DEFAULT_FRAME_CLASS 2
#define HAL_DEFAULT_FRAME_TYPE 2
#define HAL_MINIMIZE_FEATURES 0
#define HAL_MINIMIZE_FLASH_SIZE 0
#define HAL_ESP32_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_S3
#define HAL_ENABLE_NETWORKING 0
#define HAL_NETWORKING_BACKEND_CHIBIOS 1
#define HAL_OS_FATFS_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#define HAL_RCINPUT_WITH_AP_RADIO 1
#define HAL_MOUNT_ENABLED 1
#define HAL_RALLY_ENABLED 1
#define HAL_BEACON_ENABLED 1
#define HAL_PROXIMITY_ENABLED 1
#define HAL_ADSB_ENABLED 0
#define HAL_TORQEEDO_ENABLED 1
#define HAL_GENERATOR_ENABLED 1
#define HAL_EFI_ENABLED 1
#define HAL_WHEEL_ENCODER_ENABLED 1
#define HAL_DRONECAN_ESC_ENABLED 1
#define HAL_ENABLE_CANBOOTLOADER 1
#define HAL_CANBOOTLOADER_TIMEOUT 5000
#define HAL_BOOTLOADER_TIMEOUT 5000
#define HAL_USE_EMPTY_STORAGE 1
#define FLASH_RESERVE_START_KB 64
#define HAL_BARO_ALLOW_INIT_NO_BARO 1
#define HAL_SPRAYER_ENABLED 1
#define HAL_GRIPPER_ENABLED 1
#define HAL_LANDING_GEAR_ENABLED 1
#define HAL_BUTTON_ENABLED 1
#define HAL_DEBUG_BUILD 0
#define HAL_ENABLE_THREAD_STATISTICS 1