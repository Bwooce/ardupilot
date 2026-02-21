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

#include "AP_HAL_ESP32.h"

#if HAL_NUM_CAN_IFACES > 1

#include <AP_HAL/CANIface.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

// MCP2515 Register Addresses
#define MCP2515_REG_RXF0SIDH    0x00
#define MCP2515_REG_RXF0SIDL    0x01
#define MCP2515_REG_RXF0EID8    0x02
#define MCP2515_REG_RXF0EID0    0x03
#define MCP2515_REG_RXM0SIDH    0x20
#define MCP2515_REG_RXM0SIDL    0x21
#define MCP2515_REG_CNF3        0x28
#define MCP2515_REG_CNF2        0x29
#define MCP2515_REG_CNF1        0x2A
#define MCP2515_REG_CANINTE     0x2B
#define MCP2515_REG_CANINTF     0x2C
#define MCP2515_REG_EFLG        0x2D
#define MCP2515_REG_TXB0CTRL    0x30
#define MCP2515_REG_TXB0SIDH    0x31
#define MCP2515_REG_TXB0SIDL    0x32
#define MCP2515_REG_TXB0EID8    0x33
#define MCP2515_REG_TXB0EID0    0x34
#define MCP2515_REG_TXB0DLC     0x35
#define MCP2515_REG_TXB0DATA    0x36
#define MCP2515_REG_RXB0CTRL    0x60
#define MCP2515_REG_RXB0SIDH    0x61
#define MCP2515_REG_RXB0SIDL    0x62
#define MCP2515_REG_RXB0EID8    0x63
#define MCP2515_REG_RXB0EID0    0x64
#define MCP2515_REG_RXB0DLC     0x65
#define MCP2515_REG_RXB0DATA    0x66
#define MCP2515_REG_CANCTRL     0x0F
#define MCP2515_REG_CANSTAT     0x0E

// MCP2515 Commands
#define MCP2515_CMD_WRITE       0x02
#define MCP2515_CMD_READ        0x03
#define MCP2515_CMD_BITMOD      0x05
#define MCP2515_CMD_LOAD_TX0    0x40
#define MCP2515_CMD_LOAD_TX1    0x42
#define MCP2515_CMD_LOAD_TX2    0x44
#define MCP2515_CMD_RTS_TX0     0x81
#define MCP2515_CMD_RTS_TX1     0x82
#define MCP2515_CMD_RTS_TX2     0x84
#define MCP2515_CMD_RTS_ALL     0x87
#define MCP2515_CMD_READ_RX0    0x90
#define MCP2515_CMD_READ_RX1    0x94
#define MCP2515_CMD_READ_STATUS 0xA0
#define MCP2515_CMD_RX_STATUS   0xB0
#define MCP2515_CMD_RESET       0xC0

// MCP2515 Mode bits
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_SLEEP      0x20
#define MCP2515_MODE_LOOPBACK   0x40
#define MCP2515_MODE_LISTENONLY 0x60
#define MCP2515_MODE_CONFIG     0x80

namespace ESP32 {

class CANIface_MCP2515 : public AP_HAL::CANIface {
public:
    CANIface_MCP2515(uint8_t instance);
    ~CANIface_MCP2515();

    bool init(const uint32_t bitrate) override;
    int16_t send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags) override;
    int16_t receive(AP_HAL::CANFrame &frame, uint64_t &timestamp_us, AP_HAL::CANIface::CanIOFlags &flags) override;
    bool is_initialized() const override { return initialized; }

protected:
    int8_t get_iface_num() const override { return instance; };
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

private:
    // MCP2515 specific methods
    bool mcp2515_init(uint32_t bitrate);
    bool mcp2515_reset();
    bool mcp2515_set_mode(uint8_t mode);
    bool mcp2515_configure_bitrate(uint32_t bitrate);
    uint8_t mcp2515_read_register(uint8_t address);
    bool mcp2515_write_register(uint8_t address, uint8_t data);
    bool mcp2515_modify_register(uint8_t address, uint8_t mask, uint8_t data);
    bool mcp2515_read_rx_buffer(uint8_t buffer, AP_HAL::CANFrame &frame);
    bool mcp2515_load_tx_buffer(uint8_t buffer, const AP_HAL::CANFrame &frame);
    bool mcp2515_send_message(uint8_t buffer);
    uint8_t mcp2515_read_status();
    
    // SPI communication
    bool spi_transfer(uint8_t *tx_data, uint8_t *rx_data, size_t len);
    
    // Task functions
    static void rx_task(void *arg);
    static void tx_task(void *arg);

    bool initialized;
    uint8_t instance;
    
    // SPI configuration
    spi_device_handle_t spi_device;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
    
    // FreeRTOS queues and tasks
    QueueHandle_t rx_queue;
    QueueHandle_t tx_queue;
    TaskHandle_t rx_task_handle;
    TaskHandle_t tx_task_handle;
};

} // namespace ESP32

#endif // HAL_NUM_CAN_IFACES > 1