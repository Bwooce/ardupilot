/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/// @file	GCS_MAVLink.cpp

/*
This provides some support code and variables for MAVLink enabled sketches

*/

#include "GCS_config.h"

#if HAL_MAVLINK_BINDINGS_ENABLED

#include "GCS.h"
#include "GCS_MAVLink.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#include "esp_debug_helpers.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#endif

extern const AP_HAL::HAL& hal;

#ifdef MAVLINK_SEPARATE_HELPERS
// Shut up warnings about missing declarations; TODO: should be fixed on
// mavlink/pymavlink project for when MAVLINK_SEPARATE_HELPERS is defined
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#include "include/mavlink/v2.0/mavlink_helpers.h"
#pragma GCC diagnostic pop
#endif

mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
#if HAL_GCS_ENABLED
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return nullptr;
    }
    return link->channel_buffer();
#else
    return nullptr;
#endif
}

mavlink_status_t* mavlink_get_channel_status(uint8_t chan) {
#if HAL_GCS_ENABLED
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return nullptr;
    }
    return link->channel_status();
#else
    return nullptr;
#endif
}

#endif // HAL_MAVLINK_BINDINGS_ENABLED

#if HAL_GCS_ENABLED

AP_HAL::UARTDriver	*mavlink_comm_port[MAVLINK_COMM_NUM_BUFFERS];
bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

// per-channel lock
static HAL_Semaphore chan_locks[MAVLINK_COMM_NUM_BUFFERS];
static bool chan_discard[MAVLINK_COMM_NUM_BUFFERS];

mavlink_system_t mavlink_system = {7,1};

// routing table
MAVLink_routing GCS_MAVLINK::routing;

GCS_MAVLINK *GCS_MAVLINK::find_by_mavtype_and_compid(uint8_t mav_type, uint8_t compid, uint8_t &sysid) {
    mavlink_channel_t channel;
    if (!routing.find_by_mavtype_and_compid(mav_type, compid, sysid, channel)) {
        return nullptr;
    }
    return gcs().chan(channel);
}

// set a channel as private. Private channels get sent heartbeats, but
// don't get broadcast packets or forwarded packets
void GCS_MAVLINK::set_channel_private(mavlink_channel_t _chan)
{
    const uint8_t mask = (1U<<(unsigned)_chan);
    mavlink_private |= mask;
}

// return a MAVLink parameter type given a AP_Param type
MAV_PARAM_TYPE GCS_MAVLINK::mav_param_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAV_PARAM_TYPE_INT8;
    }
    if (t == AP_PARAM_INT16) {
	    return MAV_PARAM_TYPE_INT16;
    }
    if (t == AP_PARAM_INT32) {
	    return MAV_PARAM_TYPE_INT32;
    }
    // treat any others as float
    return MAV_PARAM_TYPE_REAL32;
}


/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan)
{
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link == nullptr) {
        return 0;
    }
    return link->txspace();
}

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    if (!valid_channel(chan) || mavlink_comm_port[chan] == nullptr || chan_discard[chan]) {
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
#endif
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // ESP32: Debug first messages and track the mysterious len=10 message
    static uint32_t msg_count = 0;
    msg_count++;
    if (msg_count <= 15) {  // Track more messages to catch the mystery one
        hal.console->printf("ESP32 TX: #%lu len=%d chan=%d - Raw: ", (unsigned long)msg_count, len, chan);
        
        // ALWAYS print ALL bytes regardless of message type
        for (int i = 0; i < len; i++) {
            hal.console->printf("%02x ", buf[i]);
        }
        
        // Add message type analysis
        if (len >= 10 && buf[0] == 0xfd) {
            uint32_t msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16);
            hal.console->printf("(MAVLink v2, msgid=%lu)", (unsigned long)msgid);
        }
        else if (len >= 6 && buf[0] == 0xfe) {
            hal.console->printf("(MAVLink v1, msgid=%d)", buf[5]);
        }
        else if (len == 10) {
            hal.console->printf("*** MYSTERY 10-BYTE MESSAGE ***");
        }
        else {
            hal.console->printf("(Unknown protocol)");
        }
        
        hal.console->printf("\n");
    }
    
    // ESP32: Check heap integrity before processing MAVLink messages
    // Check heap integrity to catch memory corruption early
    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE("MAVLINK", "HEAP CORRUPTION DETECTED before MAVLink send!");
        hal.console->printf("ESP32: HEAP CORRUPTION DETECTED before MAVLink send!\n");
        esp_backtrace_print(10);
    }
    
    // ESP32: Validate MAVLink message structure before sending
    if (len >= 10 && buf[0] == 0xfd) { // MAVLink v2 message
        uint32_t msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16);
        
        // Only log first few messages for debugging, skip normal heartbeats
        if (msg_count <= 5 && msgid != 0) {
            ESP_LOGE("MAVLINK", "Checking message: len=%d, msgid=%lu", len, (unsigned long)msgid);
        }
        
        // Check for obviously invalid message IDs that indicate corruption  
        if (msgid > 50000) { // Most valid MAVLink message IDs are < 50000
            static bool first_corruption = true;
            static uint32_t last_corrupt_time = 0;
            static uint8_t corruption_count = 0;
            uint32_t now = AP_HAL::millis();
            
            corruption_count++;
            
            // Use both ESP-IDF logging AND hal.console to ensure visibility
            ESP_LOGE("MAVLINK", "=== CORRUPTION #%d (dt=%lums) ===", 
                     corruption_count, (unsigned long)(now - last_corrupt_time));
            ESP_LOGE("MAVLINK", "Channel: %d, Length: %d, Message ID: %lu", chan, len, (unsigned long)msgid);
            ESP_LOGE("MAVLINK", "Raw header: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
            
            // Check for specific corruption patterns
            if (buf[0] == 0xFD) {
                // Valid MAVLink v2 start
                uint32_t extracted_msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16);
                ESP_LOGE("MAVLINK", "Extracted msgid from buf[7-9]: %lu (0x%06lx)", 
                         (unsigned long)extracted_msgid, (unsigned long)extracted_msgid);
                
                // Check if buffer might contain overlapping messages
                for (int i = 1; i < len && i < 30; i++) {
                    if (buf[i] == 0xFD) {
                        ESP_LOGE("MAVLINK", "WARNING: Found 0xFD at offset %d - possible buffer overlap!", i);
                        // Print surrounding bytes for context
                        if (i > 2 && i < len - 2) {
                            ESP_LOGE("MAVLINK", "Context: [%d]=%02x %02x [%02x] %02x %02x", 
                                     i, buf[i-2], buf[i-1], buf[i], buf[i+1], buf[i+2]);
                        }
                    }
                }
                
                // Check if this looks like ASCII text corruption
                bool looks_like_text = true;
                for (int i = 7; i < 10 && looks_like_text; i++) {
                    if (buf[i] < 0x20 || buf[i] > 0x7E) {
                        looks_like_text = false;
                    }
                }
                if (looks_like_text) {
                    ESP_LOGE("MAVLINK", "Msgid bytes look like ASCII: '%c%c%c' (0x%02x%02x%02x)",
                             buf[7], buf[8], buf[9], buf[7], buf[8], buf[9]);
                }
                
                // Check for partial overwrite pattern
                if ((msgid & 0xFF) == 0xFD) {
                    ESP_LOGE("MAVLINK", "Pattern: msgid LSB is 0xFD - likely buffer overwrite");
                }
            }
            
            // Check UART buffer status when corruption detected
            if (mavlink_comm_port[chan]) {
                uint32_t txspace = mavlink_comm_port[chan]->txspace();
                ESP_LOGE("MAVLINK", "Ch%d TX buffer space: %lu bytes", chan, (unsigned long)txspace);
            }
            
            last_corrupt_time = now;
            
            hal.console->printf("\n=== ESP32: CORRUPTED MAVLink MESSAGE DETECTED ===\n");
            hal.console->printf("Channel: %d, Length: %d, Message ID: %lu\n", chan, len, (unsigned long)msgid);
            hal.console->printf("Raw header bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
            
            // Check heap integrity when corruption is detected
            if (!heap_caps_check_integrity_all(true)) {
                ESP_LOGE("MAVLINK", "HEAP CORRUPTION also detected!");
                hal.console->printf("ESP32: HEAP CORRUPTION also detected!\n");
            }
            
            if (first_corruption) {
                ESP_LOGE("MAVLINK", "Call stack trace (first occurrence only):");
                hal.console->printf("Call stack trace (first occurrence only):\n");
                first_corruption = false;
                
                // Print stack trace without crashing
                esp_backtrace_print(20); // Print up to 20 stack frames
            } else {
                ESP_LOGE("MAVLINK", "(Subsequent corruption - not printing stack trace again)");
                hal.console->printf("(Subsequent corruption - not printing stack trace again)\n");
            }
            
            // Don't send corrupted messages
            return;
        }
    }
#endif
#if HAL_HIGH_LATENCY2_ENABLED
    // if it's a disabled high latency channel, don't send
    GCS_MAVLINK *link = gcs().chan(chan);
    if (link->is_high_latency_link && !gcs().get_high_latency_status()) {
        return;
    }
#endif
    if (gcs_alternative_active[chan]) {
        // an alternative protocol is active
        return;
    }
    
    // Use atomic packet writing to prevent interleaving with debug output
    size_t written = 0;
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    // Track timing between messages
    static uint32_t last_send_time[MAVLINK_COMM_NUM_BUFFERS] = {};
    static uint32_t rapid_send_count[MAVLINK_COMM_NUM_BUFFERS] = {};
    uint32_t now = AP_HAL::micros();
    uint32_t dt = now - last_send_time[chan];
    
    if (dt < 100 && last_send_time[chan] != 0) { // Less than 100us between sends
        rapid_send_count[chan]++;
        if (rapid_send_count[chan] % 100 == 0) {
            ESP_LOGE("MAVLINK", "Ch%d: %lu rapid sends (dt<%luus)", 
                     chan, (unsigned long)rapid_send_count[chan], (unsigned long)dt);
        }
    }
    last_send_time[chan] = now;
#endif
    
    // Use standard write() - all platforms handle atomicity correctly
    written = mavlink_comm_port[chan]->write(buf, len);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (written < len && !mavlink_comm_port[chan]->is_write_locked()) {
        AP_HAL::panic("Short write on UART: %lu < %u", (unsigned long)written, len);
    }
#else
    (void)written;
#endif
}

/*
  lock a channel for send
  if there is insufficient space to send size bytes then all bytes
  written to the channel by the mavlink library will be discarded
  while the lock is held.
 */
void comm_send_lock(mavlink_channel_t chan_m, uint16_t size)
{
    const uint8_t chan = uint8_t(chan_m);
    chan_locks[chan].take_blocking();
    if (mavlink_comm_port[chan]->txspace() < size) {
        chan_discard[chan] = true;
        gcs_out_of_space_to_send(chan_m);
    }
}

/*
  unlock a channel
 */
void comm_send_unlock(mavlink_channel_t chan_m)
{
    const uint8_t chan = uint8_t(chan_m);
    chan_discard[chan] = false;
    chan_locks[chan].give();
}

/*
  return reference to GCS channel lock, allowing for
  HAVE_PAYLOAD_SPACE() to be run with a locked channel
 */
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan)
{
    return chan_locks[uint8_t(chan)];
}

#endif  // HAL_GCS_ENABLED
