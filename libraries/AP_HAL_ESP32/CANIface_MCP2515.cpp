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

#include "CANIface_MCP2515.h"

#if HAL_NUM_CAN_IFACES > 1

#include <AP_HAL/AP_HAL.h>
#include "driver/gpio.h"
#include "ESP32_Debug.h"

using namespace ESP32;

CANIface_MCP2515::CANIface_MCP2515(uint8_t instance) :
    initialized(false),
    instance(instance),
    spi_device(nullptr),
    cs_pin(GPIO_NUM_NC),
    rst_pin(GPIO_NUM_NC),
    rx_queue(nullptr),
    tx_queue(nullptr),
    rx_task_handle(nullptr),
    tx_task_handle(nullptr)
{
}

CANIface_MCP2515::~CANIface_MCP2515()
{
    if (rx_task_handle) {
        vTaskDelete(rx_task_handle);
    }
    if (tx_task_handle) {
        vTaskDelete(tx_task_handle);
    }
    if (rx_queue) {
        vQueueDelete(rx_queue);
    }
    if (tx_queue) {
        vQueueDelete(tx_queue);
    }
}

bool CANIface_MCP2515::init(const uint32_t bitrate, const OperatingMode mode)
{
    if (initialized) {
        return true;
    }

    // Get pin assignments from board config based on instance
    if (instance == 1) {
        // First MCP2515 uses traditional pin names
#ifdef MCP2515_CS_PIN
        cs_pin = (gpio_num_t)MCP2515_CS_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_CS_PIN not defined for instance %d", instance);
        return false;
#endif
#ifdef MCP2515_RST_PIN
        rst_pin = (gpio_num_t)MCP2515_RST_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_RST_PIN not defined for instance %d", instance);
        return false;
#endif
    } else if (instance == 2) {
        // Second MCP2515 uses numbered pins
#ifdef MCP2515_2_CS_PIN
        cs_pin = (gpio_num_t)MCP2515_2_CS_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_2_CS_PIN not defined for instance %d", instance);
        return false;
#endif
#ifdef MCP2515_2_RST_PIN
        rst_pin = (gpio_num_t)MCP2515_2_RST_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_2_RST_PIN not defined for instance %d", instance);
        return false;
#endif
    } else if (instance == 3) {
        // Third MCP2515 uses numbered pins
#ifdef MCP2515_3_CS_PIN
        cs_pin = (gpio_num_t)MCP2515_3_CS_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_3_CS_PIN not defined for instance %d", instance);
        return false;
#endif
#ifdef MCP2515_3_RST_PIN
        rst_pin = (gpio_num_t)MCP2515_3_RST_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_3_RST_PIN not defined for instance %d", instance);
        return false;
#endif
    } else {
        ESP32_DEBUG_ERROR("Unsupported MCP2515 instance %d", instance);
        return false;
    }

    // Get SPI pin assignments based on instance
    gpio_num_t mosi_pin, miso_pin, sclk_pin;
    if (instance == 1) {
        mosi_pin = (gpio_num_t)MCP2515_MOSI_PIN;
        miso_pin = (gpio_num_t)MCP2515_MISO_PIN;
        sclk_pin = (gpio_num_t)MCP2515_SCLK_PIN;
    } else if (instance == 2) {
#ifdef MCP2515_2_MOSI_PIN
        mosi_pin = (gpio_num_t)MCP2515_2_MOSI_PIN;
        miso_pin = (gpio_num_t)MCP2515_2_MISO_PIN;
        sclk_pin = (gpio_num_t)MCP2515_2_SCLK_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_2 SPI pins not defined");
        return false;
#endif
    } else if (instance == 3) {
#ifdef MCP2515_3_MOSI_PIN
        mosi_pin = (gpio_num_t)MCP2515_3_MOSI_PIN;
        miso_pin = (gpio_num_t)MCP2515_3_MISO_PIN;
        sclk_pin = (gpio_num_t)MCP2515_3_SCLK_PIN;
#else
        ESP32_DEBUG_ERROR("MCP2515_3 SPI pins not defined");
        return false;
#endif
    } else {
        ESP32_DEBUG_ERROR("Unsupported MCP2515 instance %d for SPI", instance);
        return false;
    }

    // Initialize SPI
    spi_bus_config_t bus_config = {
        .mosi_io_num = mosi_pin,
        .miso_io_num = miso_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                      // SPI mode 0
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 10000000,     // 10 MHz
        .input_delay_ns = 0,
        .spics_io_num = cs_pin,
        .flags = 0,
        .queue_size = 10,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    // Initialize SPI bus (only if not already initialized)
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP32_DEBUG_ERROR("Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return false;
    }

    // Add device to SPI bus
    ret = spi_bus_add_device(SPI2_HOST, &dev_config, &spi_device);
    if (ret != ESP_OK) {
        ESP32_DEBUG_ERROR("Failed to add SPI device: %s", esp_err_to_name(ret));
        return false;
    }

    // Initialize reset pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Initialize MCP2515
    if (!mcp2515_init(bitrate)) {
        ESP32_DEBUG_ERROR("Failed to initialize MCP2515");
        return false;
    }

    // Create queues
    rx_queue = xQueueCreate(64, sizeof(CanRxItem));
    tx_queue = xQueueCreate(64, sizeof(CanTxItem));
    if (!rx_queue || !tx_queue) {
        ESP32_DEBUG_ERROR("Failed to create queues");
        return false;
    }

    // Create tasks pinned to SLOWCPU (Core 1)
    // Priority 10 = same as RCOUT, above I2C/IO, below WiFi
    #define SLOWCPU 1
    if (xTaskCreatePinnedToCore(rx_task, "mcp2515_rx", 4096, this, 10, &rx_task_handle, SLOWCPU) != pdPASS) {
        ESP32_DEBUG_ERROR("Failed to create RX task");
        return false;
    }

    if (xTaskCreatePinnedToCore(tx_task, "mcp2515_tx", 4096, this, 10, &tx_task_handle, SLOWCPU) != pdPASS) {
        ESP32_DEBUG_ERROR("Failed to create TX task");
        return false;
    }

    initialized = true;
    ESP32_DEBUG_INFO("MCP2515 CAN interface %d initialized at %lu bps", instance, bitrate);
    return true;
}

bool CANIface_MCP2515::mcp2515_init(uint32_t bitrate)
{
    // Reset MCP2515
    if (!mcp2515_reset()) {
        return false;
    }

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set configuration mode
    if (!mcp2515_set_mode(MCP2515_MODE_CONFIG)) {
        return false;
    }

    // Configure bitrate
    if (!mcp2515_configure_bitrate(bitrate)) {
        return false;
    }

    // Configure RX masks and filters (accept all)
    mcp2515_write_register(MCP2515_REG_RXM0SIDH, 0x00);
    mcp2515_write_register(MCP2515_REG_RXM0SIDL, 0x00);

    // Enable interrupts
    mcp2515_write_register(MCP2515_REG_CANINTE, 0x01); // RX0IE

    // Set normal mode
    if (!mcp2515_set_mode(MCP2515_MODE_NORMAL)) {
        return false;
    }

    return true;
}

bool CANIface_MCP2515::mcp2515_reset()
{
    // Hardware reset
    gpio_set_level(rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Software reset
    uint8_t cmd = MCP2515_CMD_RESET;
    return spi_transfer(&cmd, nullptr, 1);
}

bool CANIface_MCP2515::mcp2515_set_mode(uint8_t mode)
{
    return mcp2515_modify_register(MCP2515_REG_CANCTRL, 0xE0, mode);
}

bool CANIface_MCP2515::mcp2515_configure_bitrate(uint32_t bitrate)
{
    uint8_t cnf1, cnf2, cnf3;
    
    // Configuration for 8MHz crystal
    switch (bitrate) {
        case 1000000: // 1Mbps
            cnf1 = 0x00; cnf2 = 0x80; cnf3 = 0x80;
            break;
        case 500000:  // 500kbps
            cnf1 = 0x00; cnf2 = 0x90; cnf3 = 0x82;
            break;
        case 250000:  // 250kbps
            cnf1 = 0x00; cnf2 = 0xB1; cnf3 = 0x85;
            break;
        case 125000:  // 125kbps
            cnf1 = 0x01; cnf2 = 0xB1; cnf3 = 0x85;
            break;
        default:
            return false;
    }

    mcp2515_write_register(MCP2515_REG_CNF1, cnf1);
    mcp2515_write_register(MCP2515_REG_CNF2, cnf2);
    mcp2515_write_register(MCP2515_REG_CNF3, cnf3);

    return true;
}

uint8_t CANIface_MCP2515::mcp2515_read_register(uint8_t address)
{
    uint8_t tx_data[3] = {MCP2515_CMD_READ, address, 0x00};
    uint8_t rx_data[3];
    
    if (spi_transfer(tx_data, rx_data, 3)) {
        return rx_data[2];
    }
    return 0;
}

bool CANIface_MCP2515::mcp2515_write_register(uint8_t address, uint8_t data)
{
    uint8_t tx_data[3] = {MCP2515_CMD_WRITE, address, data};
    return spi_transfer(tx_data, nullptr, 3);
}

bool CANIface_MCP2515::mcp2515_modify_register(uint8_t address, uint8_t mask, uint8_t data)
{
    uint8_t tx_data[4] = {MCP2515_CMD_BITMOD, address, mask, data};
    return spi_transfer(tx_data, nullptr, 4);
}

bool CANIface_MCP2515::spi_transfer(uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };

    esp_err_t ret = spi_device_transmit(spi_device, &trans);
    return (ret == ESP_OK);
}

int16_t CANIface_MCP2515::send(const AP_HAL::CANFrame &frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags)
{
    if (!initialized) {
        return -1;
    }

    CanTxItem tx_item;
    tx_item.frame = frame;
    tx_item.deadline = tx_deadline;
    tx_item.loopback = (flags & AP_HAL::CANIface::Loopback) != 0;
    tx_item.abort_on_error = (flags & AP_HAL::CANIface::AbortOnError) != 0;

    if (xQueueSend(tx_queue, &tx_item, 0) == pdTRUE) {
        return 1;
    }
    return 0;
}

int16_t CANIface_MCP2515::receive(AP_HAL::CANFrame &frame, uint64_t &timestamp_us, AP_HAL::CANIface::CanIOFlags &flags)
{
    if (!initialized) {
        return -1;
    }

    CanRxItem rx_item;
    if (xQueueReceive(rx_queue, &rx_item, 0) == pdTRUE) {
        frame = rx_item.frame;
        timestamp_us = rx_item.timestamp_us;
        flags = rx_item.flags;
        return 1;
    }
    return 0;
}

bool CANIface_MCP2515::add_to_rx_queue(const CanRxItem &rx_item)
{
    return xQueueSend(rx_queue, &rx_item, 0) == pdTRUE;
}

void CANIface_MCP2515::rx_task(void *arg)
{
    CANIface_MCP2515 *iface = static_cast<CANIface_MCP2515*>(arg);
    
    while (true) {
        // Check for received messages
        uint8_t status = iface->mcp2515_read_status();
        
        if (status & 0x01) { // RX0IF - message in RX buffer 0
            AP_HAL::CANFrame frame;
            if (iface->mcp2515_read_rx_buffer(0, frame)) {
                CanRxItem rx_item;
                rx_item.frame = frame;
                rx_item.timestamp_us = AP_HAL::micros64();
                rx_item.flags = 0;
                iface->add_to_rx_queue(rx_item);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms polling interval
    }
}

void CANIface_MCP2515::tx_task(void *arg)
{
    CANIface_MCP2515 *iface = static_cast<CANIface_MCP2515*>(arg);
    
    while (true) {
        CanTxItem tx_item;
        if (xQueueReceive(iface->tx_queue, &tx_item, portMAX_DELAY) == pdTRUE) {
            // Load message into TX buffer 0 and send
            if (iface->mcp2515_load_tx_buffer(0, tx_item.frame)) {
                iface->mcp2515_send_message(0);
            }
        }
    }
}

bool CANIface_MCP2515::mcp2515_read_rx_buffer(uint8_t buffer, AP_HAL::CANFrame &frame)
{
    uint8_t cmd = MCP2515_CMD_READ_RX0;
    uint8_t tx_data[14] = {cmd};
    uint8_t rx_data[14];
    
    if (!spi_transfer(tx_data, rx_data, 14)) {
        return false;
    }
    
    // Extract CAN frame data
    uint32_t id = ((uint32_t)rx_data[1] << 3) | (rx_data[2] >> 5);
    uint8_t dlc = rx_data[5] & 0x0F;
    
    frame.id = id;
    frame.dlc = dlc;
    // Note: CANFrame doesn't have flags field - flags are handled in CanRxItem
    
    for (int i = 0; i < dlc && i < 8; i++) {
        frame.data[i] = rx_data[6 + i];
    }
    
    return true;
}

bool CANIface_MCP2515::mcp2515_load_tx_buffer(uint8_t buffer, const AP_HAL::CANFrame &frame)
{
    uint8_t tx_data[14];
    tx_data[0] = MCP2515_CMD_LOAD_TX0;
    
    // Set ID (standard frame)
    tx_data[1] = (frame.id >> 3) & 0xFF;
    tx_data[2] = (frame.id << 5) & 0xE0;
    tx_data[3] = 0x00; // EID8
    tx_data[4] = 0x00; // EID0
    tx_data[5] = frame.dlc & 0x0F; // DLC
    
    // Set data
    for (int i = 0; i < frame.dlc && i < 8; i++) {
        tx_data[6 + i] = frame.data[i];
    }
    
    return spi_transfer(tx_data, nullptr, 6 + frame.dlc);
}

bool CANIface_MCP2515::mcp2515_send_message(uint8_t buffer)
{
    uint8_t cmd = MCP2515_CMD_RTS_TX0;
    return spi_transfer(&cmd, nullptr, 1);
}

uint8_t CANIface_MCP2515::mcp2515_read_status()
{
    uint8_t tx_data[2] = {MCP2515_CMD_READ_STATUS, 0x00};
    uint8_t rx_data[2];
    
    if (spi_transfer(tx_data, rx_data, 2)) {
        return rx_data[1];
    }
    return 0;
}

#endif // HAL_NUM_CAN_IFACES > 1