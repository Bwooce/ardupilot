
#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES > 0

#include "ESP32_CANBase.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace ESP32 {

class CANIface : public ESP32_CANBase {
public:
    CANIface(uint8_t instance);

    bool init(const uint32_t bitrate, const OperatingMode mode) override;
    int16_t send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags) override;
    int16_t receive(AP_HAL::CANFrame &frame, uint64_t &timestamp_us, AP_HAL::CANIface::CanIOFlags &flags) override;
    bool is_initialized() const override { return initialized; }

    // Periodic status reporting (call from main loop)
    void update_status();
    
    // Enhanced bus health monitoring
    void check_bus_health();

protected:
    int8_t get_iface_num() const override { return instance; };
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

    // ESP32_CANBase pure virtual implementations
    bool configure_hw_filters(const CanFilterConfig* filter_configs, uint16_t num_configs) override;
    void collect_hw_stats() override;
    const char* get_controller_name() const override { return "TWAI"; }

private:
    struct CanTxItem {
        AP_HAL::CANFrame frame;
        uint64_t deadline_us;
        AP_HAL::CANIface::CanIOFlags flags;
    };

    static void rx_task(void *arg);
    static void tx_task(void *arg);

    bool initialized;
    uint32_t current_bitrate;
    // instance moved to ESP32_CANBase

    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
};

} // namespace ESP32

#endif // HAL_NUM_CAN_IFACES > 0
