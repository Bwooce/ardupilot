
#include "CANIface.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/AP_HAL.h>
#include "driver/gpio.h"
#include "driver/twai.h"

using namespace ESP32;

CANIface::CANIface(uint8_t instance) :
    initialized(false),
    instance(instance)
{
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    if (initialized) {
        return true;
    }

    gpio_num_t tx_pin, rx_pin;

    if (instance == 0) {
#if HAL_NUM_CAN_IFACES > 0
        tx_pin = (gpio_num_t)HAL_CAN1_TX_PIN;
        rx_pin = (gpio_num_t)HAL_CAN1_RX_PIN;
        printf("CAN: Initializing CAN interface %d with TX pin %d, RX pin %d, bitrate %u\n", 
               instance, (int)tx_pin, (int)rx_pin, (unsigned)bitrate);
#else
        printf("CAN: HAL_NUM_CAN_IFACES is 0, CAN disabled\n");
        return false;
#endif
    } else {
        // Only support CAN interface 0 with native TWAI
        // CAN interface 1+ should use MCP2515 or other external controllers
        printf("CAN: Instance %d not supported (only instance 0 supported)\n", instance);
        return false;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;
    switch (bitrate) {
    case 1000000:
        t_config = TWAI_TIMING_CONFIG_1MBITS();
        break;
    case 500000:
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 250000:
        t_config = TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 125000:
        t_config = TWAI_TIMING_CONFIG_125KBITS();
        break;
    default:
        return false;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    printf("CAN: Installing TWAI driver...\n");
    esp_err_t install_result = twai_driver_install(&g_config, &t_config, &f_config);
    if (install_result != ESP_OK) {
        printf("CAN: TWAI driver install failed with error %d\n", install_result);
        return false;
    }
    printf("CAN: TWAI driver installed successfully\n");

    printf("CAN: Starting TWAI...\n");
    esp_err_t start_result = twai_start();
    if (start_result != ESP_OK) {
        printf("CAN: TWAI start failed with error %d\n", start_result);
        return false;
    }
    printf("CAN: TWAI started successfully\n");

    printf("CAN: Creating RX queue...\n");
    rx_queue = xQueueCreate(128, sizeof(CanRxItem));
    if (rx_queue == NULL) {
        printf("CAN: Failed to create RX queue\n");
        return false;
    }

    printf("CAN: Creating TX queue...\n");
    tx_queue = xQueueCreate(128, sizeof(CanTxItem));
    if (tx_queue == NULL) {
        printf("CAN: Failed to create TX queue\n");
        return false;
    }

    printf("CAN: Creating RX and TX tasks...\n");
    xTaskCreate(rx_task, "can_rx", 4096, this, 5, NULL);
    xTaskCreate(tx_task, "can_tx", 4096, this, 5, NULL);

    initialized = true;
    printf("CAN: Interface %d initialization complete!\n", instance);

    return true;
}

int16_t CANIface::send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags)
{
    if (!initialized) {
        return -1;
    }

    CanTxItem tx_item;
    tx_item.frame = frame;
    tx_item.deadline_us = tx_deadline;
    tx_item.flags = flags;

    if (xQueueSendToBack(tx_queue, &tx_item, 0) != pdPASS) {
        return 0;
    }

    return 1;
}

int16_t CANIface::receive(AP_HAL::CANFrame &frame, uint64_t &timestamp_us, AP_HAL::CANIface::CanIOFlags &flags)
{
    if (!initialized) {
        return -1;
    }

    CanRxItem rx_item;
    if (xQueueReceive(rx_queue, &rx_item, 0) != pdPASS) {
        return 0;
    }

    frame = rx_item.frame;
    timestamp_us = rx_item.timestamp_us;
    flags = rx_item.flags;

    return 1;
}

bool CANIface::add_to_rx_queue(const CanRxItem &rx_item)
{
    return xQueueSendToBack(rx_queue, &rx_item, 0) == pdPASS;
}

void CANIface::rx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;

    while (true) {
        twai_message_t message;
        if (twai_receive(&message, portMAX_DELAY) != ESP_OK) {
            continue;
        }

#if CAN_LOGLEVEL >= 4
        printf("CAN RX: ID=0x%08X DLC=%d DATA=[", (unsigned)message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            printf("%02X", message.data[i]);
            if (i < message.data_length_code - 1) printf(" ");
        }
        printf("]\n");
#endif

        CanRxItem rx_item;
        rx_item.timestamp_us = AP_HAL::micros64();
        rx_item.frame.id = message.identifier;
        rx_item.frame.dlc = message.data_length_code;
        memcpy(rx_item.frame.data, message.data, message.data_length_code);

        iface->add_to_rx_queue(rx_item);
    }
}

void CANIface::tx_task(void *arg)
{
    CANIface *iface = (CANIface *)arg;
    CanTxItem tx_item;

    while (true) {
        // Continuously pull frames from queue until we get a non-expired one
        uint32_t expired_count = 0;
        do {
            if (xQueueReceive(iface->tx_queue, &tx_item, portMAX_DELAY) != pdPASS) {
                continue;
            }

            uint64_t now_us = AP_HAL::micros64();
            
            // Check if this frame has expired
            if (tx_item.deadline_us != 0 && now_us > tx_item.deadline_us) {
                expired_count++;
#if CAN_LOGLEVEL >= 3
                // Only log first few expired frames to avoid spam
                if (expired_count <= 5 || expired_count % 10 == 0) {
                    printf("CAN TX: Dropped expired frame ID=0x%08X (deadline=%llu, now=%llu) [count=%lu]\n", 
                           (unsigned)tx_item.frame.id, tx_item.deadline_us, now_us, (unsigned long)expired_count);
                }
#endif
                continue; // Get next frame
            }
            
            // Frame is not expired, break out of loop to transmit it
            break;
            
        } while (true);

#if CAN_LOGLEVEL >= 4
        printf("CAN TX: ID=0x%08X DLC=%d DATA=[", (unsigned)tx_item.frame.id, tx_item.frame.dlc);
        for (int i = 0; i < tx_item.frame.dlc; i++) {
            printf("%02X", tx_item.frame.data[i]);
            if (i < tx_item.frame.dlc - 1) printf(" ");
        }
        printf("]\n");
#endif

        twai_message_t message;
        message.identifier = tx_item.frame.id;
        message.data_length_code = tx_item.frame.dlc;
        message.extd = 1;  // Enable 29-bit extended CAN frames (required for DroneCAN)
        message.rtr = 0;   // Data frame, not remote transmission request
        memcpy(message.data, tx_item.frame.data, tx_item.frame.dlc);

        // Single transmission attempt for DroneCAN compliance
        // DroneCAN handles reliability at protocol layer, HAL retries violate timing assumptions
        const TickType_t timeout_ms = 1; // Minimal timeout for real-time performance
        
        esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(timeout_ms));
        
        if (result != ESP_OK) {
#if CAN_LOGLEVEL >= 3
            printf("CAN TX: Failed to transmit message ID=0x%08X, error=%d\n", 
                   (unsigned)tx_item.frame.id, result);
#endif
        }
    }
}

#endif // HAL_NUM_CAN_IFACES > 0
