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

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include "HAL_ESP32_Class.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#endif
#include "ESP32_Debug.h"
#include "ESP32_Params.h"
#include "esp_log_redirect.h"
#include "Scheduler.h"
#include "I2CDevice.h"
#include "SPIDevice.h"
#include "UARTDriver.h"
#include "WiFiDriver.h"
#include "WiFiUdpDriver.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "GPIO.h"
#include "Storage.h"
#include "AnalogIn.h"
#include "Util.h"
#ifndef HAL_NUM_CAN_IFACES
#error HAL_NUM_CAN_IFACES not defined
#endif
#if HAL_NUM_CAN_IFACES == 0
#error HAL_NUM_CAN_IFACES is zero
#endif
#if HAL_NUM_CAN_IFACES > 0
#include "CANIface.h"
#if HAL_NUM_CAN_IFACES > 1
#include "CANIface_MCP2515.h"
#endif
#endif
#if AP_SIM_ENABLED
#include <AP_HAL/SIMState.h>
#endif

static ESP32::UARTDriver cons(0);
#ifdef HAL_ESP32_WIFI
#if HAL_ESP32_WIFI == 1
static ESP32::WiFiDriver serial1Driver; //tcp, client should connect to 192.168.4.1 port 5760
#elif HAL_ESP32_WIFI == 2
static ESP32::WiFiUdpDriver serial1Driver; //udp
#else
static ESP32::UARTDriver serial1Driver(1); // UART1 for SERIAL1
#endif
#else
static ESP32::UARTDriver serial1Driver(1); // UART1 for SERIAL1
#endif
static ESP32::UARTDriver serial2Driver(2); // UART2 for SERIAL2
static Empty::UARTDriver serial3Driver; // No UART3 on ESP32
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;

#if HAL_WITH_DSP
static Empty::DSP dspDriver;
#endif

static ESP32::I2CDeviceManager i2cDeviceManager;
static ESP32::SPIDeviceManager spiDeviceManager;
#ifndef HAL_DISABLE_ADC_DRIVER
static ESP32::AnalogIn analogIn;
#else
static Empty::AnalogIn analogIn;
#endif
#ifdef HAL_USE_EMPTY_STORAGE
static Empty::Storage storageDriver;
#else
static ESP32::Storage storageDriver;
#endif
static ESP32::GPIO gpioDriver;
#if AP_SIM_ENABLED
static Empty::RCOutput rcoutDriver;
#else
static ESP32::RCOutput rcoutDriver;
#endif
static ESP32::RCInput rcinDriver;
static ESP32::Scheduler schedulerInstance;
static ESP32::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

#if HAL_NUM_CAN_IFACES > 0
static AP_HAL::CANIface* canDrivers[HAL_NUM_CAN_IFACES];
#endif

#if AP_SIM_ENABLED
static AP_HAL::SIMState xsimstate;
#endif

extern const AP_HAL::HAL& hal;

HAL_ESP32::HAL_ESP32() :
    AP_HAL::HAL(
        &cons, //Console/mavlink (UART0)
        &serial1Driver, //Telem 1 (UART1 or WiFi)
        &serial2Driver, //Telem 2 (UART2)
        &serial3Driver, //GPS 1 (unused - no UART3)
        &serial4Driver, //GPS 2 (unused)
        &serial5Driver, //Extra 1
        &serial6Driver, //Extra 2
        &serial7Driver, //Extra 3
        &serial8Driver, //Extra 4
        &serial9Driver, //Extra 5
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
#if AP_SIM_ENABLED
        &xsimstate,
#endif
#if HAL_WITH_DSP
        &dspDriver,
#endif
#if HAL_NUM_CAN_IFACES > 0
        (AP_HAL::CANIface**)canDrivers
#else
        nullptr
#endif
    )
{
    // NOTE: Cannot use ESP_LOG here - constructor runs before ESP-IDF initialization!
    // Any logging here will cause system to hang before app_main()
#if HAL_NUM_CAN_IFACES > 0
    // Initialize CAN driver array based on configuration
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        canDrivers[i] = nullptr;
    }
    
    // CAN interface 0: Native TWAI (if available)
    if (HAL_NUM_CAN_IFACES > 0) {
        // Cannot use ESP_LOG here - constructor runs before ESP-IDF initialization!
        canDrivers[0] = new ESP32::CANIface(0);
    }
    
    // MCP2515 interfaces: Auto-detect based on pin definitions
#if HAL_NUM_CAN_IFACES > 1
    // TEMPORARILY DISABLE MCP2515 TO DEBUG BOOT
    // uint8_t mcp_index = 1; // Start after native TWAI
    
    // MCP2515 #1 (traditional pin names for backward compatibility)
// #if defined(MCP2515_CS_PIN)
//     if (mcp_index < HAL_NUM_CAN_IFACES) {
//         canDrivers[mcp_index] = new ESP32::CANIface_MCP2515(mcp_index);
//         mcp_index++;
//     }
// #endif

    // TEMPORARILY DISABLED FOR DEBUGGING
    // MCP2515 #2 (numbered pins)
// #if HAL_NUM_CAN_IFACES > 2 && defined(MCP2515_2_CS_PIN)
//     if (mcp_index < HAL_NUM_CAN_IFACES) {
//         canDrivers[mcp_index] = new ESP32::CANIface_MCP2515(mcp_index);
//         mcp_index++;
//     }
// #endif

    // MCP2515 #3 (numbered pins)
// #if HAL_NUM_CAN_IFACES > 3 && defined(MCP2515_3_CS_PIN)
//     if (mcp_index < HAL_NUM_CAN_IFACES) {
//         canDrivers[mcp_index] = new ESP32::CANIface_MCP2515(mcp_index);
//         mcp_index++;
//     }
// #endif

    // MCP2515 #4 (numbered pins) - extend as needed
// #if HAL_NUM_CAN_IFACES > 4 && defined(MCP2515_4_CS_PIN)
//     if (mcp_index < HAL_NUM_CAN_IFACES) {
//         canDrivers[mcp_index] = new ESP32::CANIface_MCP2515(mcp_index);
//         mcp_index++;
//     }
// #endif
#endif // HAL_NUM_CAN_IFACES > 1
#endif
}

void HAL_ESP32::run(int argc, char * const argv[], Callbacks* callbacks) const
{
    // Initialize ESP32 parameters before any ESP-IDF logging
    ESP32::esp32_params()->init();
    
    // Initialize ESP log redirection to MAVLink if needed
    esp32_log_redirect_init();
    
    // Force a test message to verify redirect is working  
    if (ESP32::esp32_params()->log_to_mavlink.get()) {
        ESP_LOGE("TEST", "ESP32 log redirect ACTIVE");
        // Also send directly as CRITICAL to ensure it gets through
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ESP32: Log redirect enabled=%d", 
                     (int)ESP32::esp32_params()->log_to_mavlink.get());
    } else {
        // Send message that redirect is disabled
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "ESP32: Log redirect DISABLED");
    }
    
    // Debug via MAVLink STATUSTEXT - safe from serial contamination
    ESP32_DEBUG_INFO("HAL run starting with callbacks");
    ((ESP32::Scheduler *)hal.scheduler)->set_callbacks(callbacks);
    ESP32_DEBUG_INFO("Calling scheduler init");
    hal.scheduler->init();
    ESP32_DEBUG_INFO("Scheduler init completed - ESP32 tasks created");
    ESP32_DEBUG_VERBOSE("ESP32 scheduler uses FreeRTOS tasks, main loop in _main_thread");
    
    ESP32_DEBUG_INFO("ArduPilot HAL setup complete - entering infinite loop");
    ESP32_DEBUG_INFO("Main ArduPilot logic now running in FreeRTOS tasks");
    
    // ESP32 HAL: Keep the main ESP-IDF task alive
    // The actual ArduPilot main loop runs in the _main_thread FreeRTOS task
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Main task just sleeps - all real work is in FreeRTOS tasks
    }
}

void AP_HAL::init()
{
}

