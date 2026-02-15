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


// Permanent static asserts for all platforms to catch packing issues
// These check for obviously wrong sizes that indicate struct packing problems
// Rather than exact sizes, we check for reasonable ranges

// Message structs should never be larger than their wire format + reasonable padding
static_assert(sizeof(mavlink_heartbeat_t) <= 12, "mavlink_heartbeat_t too large - packing issue!");
static_assert(sizeof(mavlink_attitude_t) <= 32, "mavlink_attitude_t too large - packing issue!");
static_assert(sizeof(mavlink_sys_status_t) <= 36, "mavlink_sys_status_t too large - packing issue!");

// Message structs should never be smaller than their minimum wire size
static_assert(sizeof(mavlink_heartbeat_t) >= 9, "mavlink_heartbeat_t too small - missing fields!");
static_assert(sizeof(mavlink_attitude_t) >= 28, "mavlink_attitude_t too small - missing fields!");

// The main message container should be within a reasonable range
// On most platforms it's 280 bytes, but ESP32 may add padding
// The critical thing is that the payload and header offsets are correct
static_assert(sizeof(mavlink_message_t) >= 280 && sizeof(mavlink_message_t) <= 300,
              "mavlink_message_t size out of expected range - possible struct definition error!");

// More important than total size is that the payload array is at the right offset
static_assert(offsetof(mavlink_message_t, payload64) <= 32,
              "mavlink_message_t payload offset wrong - will corrupt messages!");

// Verify critical field offsets - these MUST be exact for wire format compatibility
// ATTITUDE message (ID 30) - the one we know is getting corrupted
static_assert(offsetof(mavlink_attitude_t, time_boot_ms) == 0, "attitude time_boot_ms offset wrong!");
static_assert(offsetof(mavlink_attitude_t, roll) == 4, "attitude roll offset wrong!");
static_assert(offsetof(mavlink_attitude_t, pitch) == 8, "attitude pitch offset wrong!");
static_assert(offsetof(mavlink_attitude_t, yaw) == 12, "attitude yaw offset wrong!");
static_assert(offsetof(mavlink_attitude_t, rollspeed) == 16, "attitude rollspeed offset wrong!");
static_assert(offsetof(mavlink_attitude_t, pitchspeed) == 20, "attitude pitchspeed offset wrong!");
static_assert(offsetof(mavlink_attitude_t, yawspeed) == 24, "attitude yawspeed offset wrong!");

// HEARTBEAT message (ID 0) - most frequently sent
static_assert(offsetof(mavlink_heartbeat_t, custom_mode) == 0, "heartbeat custom_mode offset wrong!");
static_assert(offsetof(mavlink_heartbeat_t, type) == 4, "heartbeat type offset wrong!");
static_assert(offsetof(mavlink_heartbeat_t, autopilot) == 5, "heartbeat autopilot offset wrong!");
static_assert(offsetof(mavlink_heartbeat_t, base_mode) == 6, "heartbeat base_mode offset wrong!");
static_assert(offsetof(mavlink_heartbeat_t, system_status) == 7, "heartbeat system_status offset wrong!");
static_assert(offsetof(mavlink_heartbeat_t, mavlink_version) == 8, "heartbeat mavlink_version offset wrong!");

// IMPORTANT: The mavlink_message_t structure is NOT in wire format order!
// The structure has checksum FIRST for efficiency, not wire format
// Actual structure order (from mavlink_types.h):
//   uint16_t checksum;      // offset 0
//   uint8_t magic;          // offset 2
//   uint8_t len;            // offset 3
//   uint8_t incompat_flags; // offset 4
//   uint8_t compat_flags;   // offset 5
//   uint8_t seq;            // offset 6
//   uint8_t sysid;          // offset 7
//   uint8_t compid;         // offset 8
//   uint32_t msgid:24;      // offset 9 (bitfield)
//   uint64_t payload64[...];// offset 12
// Wire format is different - magic comes first!

static_assert(offsetof(mavlink_message_t, checksum) == 0, "checksum should be at offset 0");
static_assert(offsetof(mavlink_message_t, magic) == 2, "magic should be at offset 2");
static_assert(offsetof(mavlink_message_t, sysid) == 7, "sysid should be at offset 7");
static_assert(offsetof(mavlink_message_t, compid) == 8, "compid should be at offset 8");
static_assert(offsetof(mavlink_message_t, payload64) >= 12 && offsetof(mavlink_message_t, payload64) <= 16,
              "payload should be around offset 12-16");
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
    // Periodic MAVLink throughput logging for debugging telemetry issues
    static uint32_t send_count[MAVLINK_COMM_NUM_BUFFERS] = {};
    static uint32_t send_bytes[MAVLINK_COMM_NUM_BUFFERS] = {};
    static uint32_t last_stats_log_ms[MAVLINK_COMM_NUM_BUFFERS] = {};

    send_count[chan]++;
    send_bytes[chan] += len;

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_stats_log_ms[chan] >= 10000) {  // Log every 10 seconds
        mavlink_status_t *status = mavlink_get_channel_status(chan);
        if (status) {
            float duration_sec = (now_ms - last_stats_log_ms[chan]) / 1000.0f;
            if (duration_sec > 0) {
                float msgs_per_sec = send_count[chan] / duration_sec;
                float bytes_per_sec = send_bytes[chan] / duration_sec;

                ESP_LOGI("MAVLINK", "Ch%d: TX=%lu msgs (%.1f msg/s, %.0f B/s), RX_OK=%u, RX_DROP=%u",
                         chan,
                         (unsigned long)send_count[chan], msgs_per_sec, bytes_per_sec,
                         status->packet_rx_success_count, status->packet_rx_drop_count);
            }
        }

        // Reset counters
        send_count[chan] = 0;
        send_bytes[chan] = 0;
        last_stats_log_ms[chan] = now_ms;
    }

    // ESP32: Check heap integrity periodically if CONFIG_HEAP_POISONING is enabled
#if defined(CONFIG_HEAP_POISONING_COMPREHENSIVE) || defined(CONFIG_HEAP_POISONING_LIGHT)
    {
        static __thread bool in_heap_check = false;
        static uint32_t heap_check_count = 0;
#ifdef DEBUG_BUILD
        const uint32_t check_interval = 100;
#else
        const uint32_t check_interval = 1000;
#endif
        if (!in_heap_check && (++heap_check_count % check_interval) == 0) {
            in_heap_check = true;
            bool heap_ok = heap_caps_check_integrity_all(false);
            in_heap_check = false;
            if (!heap_ok) {
                hal.console->printf("ESP32: HEAP CORRUPTION DETECTED at msg %lu\n", (unsigned long)heap_check_count);
            }
        }
    }
#endif
    
    // ESP32: Validate MAVLink message structure before sending
    static uint32_t msg_count = 0;
    msg_count++;
    if (len >= 10 && buf[0] == 0xfd) { // MAVLink v2 message
        uint32_t msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16);
        
        // Only log first few messages for debugging in verbose mode
        #ifdef DEBUG_BUILD
        if (msg_count <= 5 && msgid != 0) {
            ESP_LOGD("MAVLINK", "Checking message: len=%d, msgid=%lu", len, (unsigned long)msgid);
        }
        #endif
        
        // Check for obviously invalid message IDs that indicate corruption  
        if (msgid > 50000) { // Most valid MAVLink message IDs are < 50000
            static bool first_corruption = true;
            static uint32_t last_corrupt_time = 0;
            static uint8_t corruption_count = 0;
            uint32_t now = AP_HAL::millis();
            
            corruption_count++;
            
            // Use WARNING level to avoid QGC error popups, but still show in logs
            ESP_LOGW("MAVLINK", "=== CORRUPTION #%d (dt=%lums) ===",
                     corruption_count, (unsigned long)(now - last_corrupt_time));
            ESP_LOGW("MAVLINK", "Channel: %d, Length: %d, Message ID: %lu", chan, len, (unsigned long)msgid);
            ESP_LOGW("MAVLINK", "Raw header: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
            
            // Check for specific corruption patterns
            if (buf[0] == 0xFD) {
                // Valid MAVLink v2 start
                uint32_t extracted_msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16);
                ESP_LOGE("MAVLINK", "Extracted msgid from buf[7-9]: %lu (0x%06lx)", 
                         (unsigned long)extracted_msgid, (unsigned long)extracted_msgid);
                
                // Check for specific corruption pattern: ends with 0xC0
                if ((extracted_msgid & 0xFF) == 0xC0) {
                    ESP_LOGE("MAVLINK", "PATTERN: msgid ends with 0xC0 - consistent corruption pattern detected!");
                }
                
                // Check if msgid looks like ESP32 memory address (possible overflow)
                if ((extracted_msgid & 0xFF0000) == 0x420000 || 
                    (extracted_msgid & 0xFF0000) == 0x3F0000 ||
                    (extracted_msgid & 0xFF0000) == 0x400000) {
                    ESP_LOGE("MAVLINK", "WARNING: msgid looks like ESP32 address! Possible memory corruption");
                    
                    // Dump surrounding memory for analysis
                    ESP_LOGE("MAVLINK", "Buffer context at %p:", buf);
                    for (int i = -16; i < 32 && i < (int)len; i++) {
                        if (i >= 0) {
                            printf("%02X ", buf[i]);
                        }
                    }
                    printf("\n");
                }
                
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
            hal.console->printf("Channel: %d, Length: %d, Message ID: %lu (0x%06lX)\n",
                               chan, len, (unsigned long)msgid, (unsigned long)msgid);

            // Decode what message this was supposed to be
            const char* expected_msg = "UNKNOWN";
            if (msgid == 0x03E2B5) {
                // This pattern appears to be ATTITUDE (30) with corruption
                expected_msg = "Likely ATTITUDE (30/0x1E) corrupted to 0x03E2B5";
            } else if (msgid == 0x9B76B6) {
                // Another corruption pattern
                expected_msg = "Likely ATTITUDE (30/0x1E) corrupted to 0x9B76B6";
            }
            hal.console->printf("Expected: %s\n", expected_msg);

            hal.console->printf("Raw header bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);

            // Show where message ID bytes should be in MAVLink v2
            // Bytes 7-9 are the message ID in MAVLink v2
            hal.console->printf("Message ID bytes [7-9]: %02x %02x %02x", buf[7], buf[8], buf[9]);

            // Decode what the message ID should have been based on length
            if (len == 28) {
                hal.console->printf(" (28 bytes = likely ATTITUDE or GLOBAL_POSITION_INT)\n");
                if (buf[7] == 0xC0) {
                    hal.console->printf("ERROR: 0xC0 byte contamination detected at msgid position!\n");
                    hal.console->printf("0xC0 is SLIP protocol END marker - possible serial framing issue\n");
                }
            } else {
                hal.console->printf(" (expected 1E 00 00 for ATTITUDE)\n");
            }

            // Check what's at the offsets where msgid MIGHT be if packing is wrong
            hal.console->printf("Alternate locations - [10-12]: %02x %02x %02x, [13-15]: %02x %02x %02x\n",
                               buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]);

            // Check if this looks like memory from elsewhere (pointer values?)
            // B5 E2 03 looks suspiciously like part of an ESP32 address (0x403E2B5)
            // B6 76 9B could be part of 0x409B76B6
            if ((msgid & 0xFF0000) == 0x030000 || (msgid & 0xFF0000) == 0x9B0000) {
                hal.console->printf("WARNING: Message ID looks like ESP32 memory address fragment!\n");
                hal.console->printf("Could be buffer overrun or uninitialized memory\n");

                // Try to decode what we're actually seeing
                uint32_t seen_at_7 = buf[7] | (buf[8] << 8) | (buf[9] << 16);
                uint32_t seen_at_10 = buf[10] | (buf[11] << 8) | (buf[12] << 16);
                hal.console->printf("Value at offset 7: 0x%06lX, at offset 10: 0x%06lX\n",
                                   (unsigned long)seen_at_7, (unsigned long)seen_at_10);
            }

            // Log more of the message to understand the corruption pattern
            if (len > 10) {
                hal.console->printf("Next 20 bytes: ");
                for (int i = 10; i < 30 && i < len; i++) {
                    hal.console->printf("%02x ", buf[i]);
                }
                hal.console->printf("\n");

                // Check if the rest is zeros, 0xFF, or has a pattern
                bool all_zeros = true;
                bool all_ff = true;
                for (int i = 10; i < len && i < 50; i++) {
                    if (buf[i] != 0) all_zeros = false;
                    if (buf[i] != 0xFF) all_ff = false;
                }

                if (all_zeros) {
                    hal.console->printf("Rest of message appears to be all zeros\n");
                } else if (all_ff) {
                    hal.console->printf("Rest of message appears to be all 0xFF\n");
                } else {
                    hal.console->printf("Message contains mixed data\n");
                }
            }
            
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
    
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 && defined(DEBUG_BUILD)
    // Track timing between messages (only in debug builds)
    static uint32_t last_send_time[MAVLINK_COMM_NUM_BUFFERS] = {};
    static uint32_t rapid_send_count[MAVLINK_COMM_NUM_BUFFERS] = {};
    uint32_t now = AP_HAL::micros();
    uint32_t dt = now - last_send_time[chan];
    
    if (dt < 100 && last_send_time[chan] != 0) { // Less than 100us between sends
        rapid_send_count[chan]++;
        // Only log if this becomes a persistent problem (>10000 rapid sends)
        if (rapid_send_count[chan] == 10000) {
            ESP_LOGW("MAVLINK", "Ch%d: Excessive rapid sends detected (%lu)", 
                     chan, (unsigned long)rapid_send_count[chan]);
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
