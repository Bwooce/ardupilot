
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
    tx_queue = xQueueCreate(128, sizeof(AP_HAL::CANFrame));
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

    if (xQueueSendToBack(tx_queue, &frame, 0) != pdPASS) {
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
    AP_HAL::CANFrame frame;

    while (true) {
        if (xQueueReceive(iface->tx_queue, &frame, portMAX_DELAY) != pdPASS) {
            continue;
        }

#if CAN_LOGLEVEL >= 4
        printf("CAN TX: ID=0x%08X DLC=%d DATA=[", (unsigned)frame.id, frame.dlc);
        for (int i = 0; i < frame.dlc; i++) {
            printf("%02X", frame.data[i]);
            if (i < frame.dlc - 1) printf(" ");
        }
        printf("]\n");
#endif

        twai_message_t message;
        message.identifier = frame.id;
        message.data_length_code = frame.dlc;
        memcpy(message.data, frame.data, frame.dlc);

        esp_err_t result = twai_transmit(&message, portMAX_DELAY);
        if (result != ESP_OK) {
#if CAN_LOGLEVEL >= 2
            printf("CAN TX ERROR: Failed to transmit message, error=%d\n", result);
#endif
        }
    }
}

#endif // HAL_NUM_CAN_IFACES > 0
