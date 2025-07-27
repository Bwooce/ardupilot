
#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/CANIface.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class ESP32::CANIface : public AP_HAL::CANIface {
public:
    CANIface(uint8_t instance);

    bool init(const uint32_t bitrate, const OperatingMode mode) override;
    int16_t send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags) override;
    int16_t receive(AP_HAL::CANFrame &frame, uint64_t &timestamp_us, AP_HAL::CANIface::CanIOFlags &flags) override;
    bool is_initialized() const override { return initialized; }

protected:
    int8_t get_iface_num() const override { return instance; };
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

private:
    struct CanTxItem {
        AP_HAL::CANFrame frame;
        uint64_t deadline_us;
        AP_HAL::CANIface::CanIOFlags flags;
    };

    static void rx_task(void *arg);
    static void tx_task(void *arg);

    bool initialized;
    uint8_t instance;

    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
};

#endif // HAL_NUM_CAN_IFACES > 0
