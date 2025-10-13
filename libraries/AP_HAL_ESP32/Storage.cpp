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
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "Storage.h"
#include "esp_log.h"
#include "esp_partition.h"
#ifdef CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"
#endif
#include <string.h>
#include <stdio.h>

//#define STORAGEDEBUG 1

using namespace ESP32;

extern const AP_HAL::HAL& hal;

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }
#ifdef STORAGEDEBUG
    printf("%s:%d _storage_open \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // Opening storage partition
    
    _dirty_mask.clearall();
    p = esp_partition_find_first((esp_partition_type_t)0x45, ESP_PARTITION_SUBTYPE_ANY, nullptr);
    
    if (p == nullptr) {
        ESP_LOGE("STORAGE", "ERROR - No storage partition found (type 0x45)!");
        hal.console->printf("Storage: ERROR - No storage partition found (type 0x45)!\n");
    } else {
#ifdef STORAGEDEBUG
        ESP_LOGI("STORAGE", "Found partition '%s' at 0x%08lx, size %lu bytes", 
                 p->label, (unsigned long)p->address, (unsigned long)p->size);
#endif
    }
    
    // load from storage backend
    _flash_load();
    _initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
#ifdef STORAGEDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint16_t end = loc + length;
    for (uint16_t line=loc>>STORAGE_LINE_SHIFT;
         line <= end>>STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
#ifdef STORAGEDEBUG
        printf("%s:%d read_block failed \n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    // Delay storage init until first use to avoid early boot issues
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
#ifdef STORAGEDEBUG
        printf("%s:%d write_block failed \n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

    // save to storage backend
    _flash_write(i);
}

/*
  load all data from flash
 */
void Storage::_flash_load(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!_flash.init()) {
        AP_HAL::panic("unable to init flash storage");
    }
}

/*
  write one storage line. This also updates _dirty_mask.
*/
void Storage::_flash_write(uint16_t line)
{
#ifdef STORAGEDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    bool write_result = _flash.write(line*STORAGE_LINE_SIZE, STORAGE_LINE_SIZE);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Debug critical storage writes (line 0 contains magic number)
    if (line == 0 || !write_result) {
        ESP_LOGI("STORAGE", "_flash_write line %d: %s", line, write_result ? "SUCCESS" : "FAILED");
        if (!write_result) {
            ESP_LOGE("STORAGE", "AP_FlashStorage write failed for line %d - data NOT persisted!", line);
        }
    }
#endif

    if (write_result) {
        // mark the line clean
        _dirty_mask.clear(line);
    } else {
        // Write failed - keep line dirty for retry
        ESP_LOGE("STORAGE", "Flash write failed for line %d, keeping dirty bit set", line);
    }
}

/*
  callback to write data to flash
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    size_t address = sector * STORAGE_SECTOR_SIZE + offset;
    esp_err_t err = esp_partition_write(p, address, data, length);
    bool ret = (err == ESP_OK);

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Debug flash writes, especially failures
    if (!ret) {
        ESP_LOGE("STORAGE", "esp_partition_write FAILED: sector=%d offset=%lu length=%d addr=0x%lx err=%s (%d)",
                 sector, (unsigned long)offset, length, (unsigned long)address,
                 esp_err_to_name(err), err);
    } else if (address < 32) {
        // Log writes to the critical first bytes (magic number area)
        ESP_LOGI("STORAGE", "esp_partition_write SUCCESS: sector=%d offset=%lu length=%d addr=0x%lx",
                 sector, (unsigned long)offset, length, (unsigned long)address);
    }
#endif

    if (!ret && _flash_erase_ok()) {
        // we are getting flash write errors while disarmed. Try
        // re-writing all of flash
        uint32_t now = AP_HAL::millis();
        if (now - _last_re_init_ms > 5000) {
            _last_re_init_ms = now;
            ESP_LOGW("STORAGE", "Flash write failed, attempting re-initialization...");
            bool ok = _flash.re_initialise();
            DEV_PRINTF("Storage: failed at %u:%u for %u - re-init %u\n",
                                (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
#ifdef STORAGEDEBUG
            printf("Storage: failed at %u:%u for %u - re-init %u\n",
                   (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
#endif
            ESP_LOGI("STORAGE", "Re-initialization %s", ok ? "SUCCEEDED" : "FAILED");
        }
    }
    return ret;
}

/*
  callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    size_t address = sector * STORAGE_SECTOR_SIZE + offset;
#ifdef STORAGEDEBUG
    printf("%s:%d  -> sec:%u off:%d len:%d addr:%d\n", __PRETTY_FUNCTION__, __LINE__,sector,offset,length,address);
#endif
    
    // Debug: Check if we're reading FRAME_CLASS parameter area
    // FRAME_CLASS is typically in the first few sectors
    if (sector == 0 && offset < 100) {
        ESP_LOGI("STORAGE", "Reading sector %u offset %u len %u (possible param area)", 
                 sector, offset, length);
    }
    
    esp_err_t err = esp_partition_read(p, address, data, length);
    if (err != ESP_OK) {
        ESP_LOGE("STORAGE", "Read failed at addr 0x%x: %s", address, esp_err_to_name(err));
        return false;
    }
    
    // Debug: Log first few bytes if in param area
    if (sector == 0 && offset < 100 && length > 0) {
        ESP_LOGI("STORAGE", "Read data[0-3]: 0x%02X 0x%02X 0x%02X 0x%02X",
                 data[0], data[1], data[2], data[3]);
    }
    
    return true;
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    size_t address = sector * STORAGE_SECTOR_SIZE;
    return esp_partition_erase_range(p, address, STORAGE_SECTOR_SIZE) == ESP_OK;
}

/*
  callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // only allow erase while disarmed
    return !hal.util->get_soft_armed();
}

/*
  consider storage healthy if we have nothing to write sometime in the
  last 2 seconds
 */
bool Storage::healthy(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return _initialised && AP_HAL::millis() - _last_empty_ms < 2000;
}

/*
  get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (!_initialised) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}
