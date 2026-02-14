
#pragma once

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES > 0

#include "ESP32_CANBase.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

namespace ESP32 {

class CANIface : public ESP32_CANBase {
public:
    CANIface(uint8_t instance);

    bool init(const uint32_t bitrate) override;
    int16_t send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags) override;
    int16_t receive(AP_HAL::CANFrame &frame, uint64_t &timestamp_us, AP_HAL::CANIface::CanIOFlags &flags) override;
    bool is_initialized() const override { return initialized; }

    // Periodic status reporting (call from main loop)
    void update_status();

    // Enhanced bus health monitoring
    void check_bus_health();

    // Recovery methods for severe bus errors
    void attempt_driver_restart();

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

    // ISR-safe struct for passing received frames from callback to rx_task
    struct IsrRxFrame {
        uint32_t id;
        uint8_t data[TWAI_FRAME_MAX_LEN];
        uint8_t dlc;
        bool is_extended;
        bool is_rtr;
        uint64_t timestamp_us;
    };

    static void rx_task(void *arg);
    static void tx_task(void *arg);

    // ISR callbacks for new TWAI driver
    static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                      const twai_rx_done_event_data_t *edata,
                                      void *user_ctx);
    static bool IRAM_ATTR on_tx_done(twai_node_handle_t handle,
                                      const twai_tx_done_event_data_t *edata,
                                      void *user_ctx);
    static bool IRAM_ATTR on_state_change(twai_node_handle_t handle,
                                           const twai_state_change_event_data_t *edata,
                                           void *user_ctx);
    static bool IRAM_ATTR on_error(twai_node_handle_t handle,
                                    const twai_error_event_data_t *edata,
                                    void *user_ctx);

    bool initialized;
    uint32_t current_bitrate;

    twai_node_handle_t _node = nullptr;

    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
    QueueHandle_t isr_rx_queue;  // ISR callback → rx_task bridge

    // Task handles for cleanup on re-init
    TaskHandle_t rx_task_handle = NULL;
    TaskHandle_t tx_task_handle = NULL;

    // Track transmitted frames to filter self-reception
    struct TxTracker {
        uint32_t can_id;
        uint64_t timestamp_us;
        bool loopback_requested;
    };
    static constexpr uint8_t TX_TRACKER_SIZE = 32;
    TxTracker tx_tracker[TX_TRACKER_SIZE];
    uint8_t tx_tracker_index = 0;

    // Flag to indicate driver restart in progress
    volatile bool restart_in_progress = false;

    // Current error state from ISR callback (updated by on_state_change)
    volatile twai_error_state_t hw_error_state = TWAI_ERROR_ACTIVE;

    // Bus state tracking for rate-limited error logging
    // Maps directly to TWAI error states (no STOPPED/RECOVERING -- those
    // don't exist in the new esp_twai API)
    enum class BusState {
        GOOD,       // TWAI_ERROR_ACTIVE -- healthy
        WARNING,    // TWAI_ERROR_WARNING or TWAI_ERROR_PASSIVE -- degraded
        BUS_OFF     // TWAI_ERROR_BUS_OFF -- needs recovery
    };
    BusState last_bus_state = BusState::GOOD;
    uint32_t bus_state_change_ms = 0;
    uint32_t last_bus_error_log_ms = 0;

    // Rate-limited bus error logging (private helper)
    void log_bus_state(BusState new_state);

    // Helper to map twai_error_state_t to BusState
    BusState error_state_to_bus_state(twai_error_state_t state) const;
};

} // namespace ESP32

#endif // HAL_NUM_CAN_IFACES > 0
