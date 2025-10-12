/*
   GCS MAVLink functions related to parameter handling

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

#include "GCS_config.h"

#if HAL_GCS_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "GCS.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#if HAL_ENABLE_DRONECAN_DRIVERS
#include <AP_DroneCAN/AP_DroneCAN.h>
#endif

extern const AP_HAL::HAL& hal;

// queue of pending parameter requests and replies
ObjectBuffer<GCS_MAVLINK::pending_param_request> GCS_MAVLINK::param_requests(20);
ObjectBuffer<GCS_MAVLINK::pending_param_reply> GCS_MAVLINK::param_replies(5);

#if HAL_ENABLE_DRONECAN_DRIVERS
// queue of pending PARAM_EXT requests for DroneCAN nodes
ObjectBuffer<GCS_MAVLINK::pending_param_ext_request> GCS_MAVLINK::param_ext_requests(5);

// DroneCAN parameter get/set callbacks
// These are called when a DroneCAN node responds to parameter requests
static bool dronecan_param_get_set_float_cb(AP_DroneCAN* ap_dronecan, const uint8_t node_id,
                                             const char* name, float &value)
{
    // Find matching pending request
    GCS_MAVLINK::pending_param_ext_request req;
    bool found = false;

    // Search through all pending requests
    for (uint8_t i = 0; i < GCS_MAVLINK::param_ext_requests.available(); i++) {
        if (GCS_MAVLINK::param_ext_requests.peek(req, i)) {
            if (req.node_id == node_id &&
                strncmp(req.param_name, name, sizeof(req.param_name)) == 0) {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        return false;  // No matching request found
    }

    // Format value as string
    char value_str[128];
    snprintf(value_str, sizeof(value_str), "%.6f", value);

    // Send response to GCS
    GCS_MAVLINK *gcs_chan = gcs().chan(req.chan);
    if (gcs_chan != nullptr) {
        if (req.is_set) {
            // This was a set operation - send PARAM_EXT_ACK
            gcs_chan->send_param_ext_ack(req.param_name, value_str,
                                         MAV_PARAM_EXT_TYPE_REAL32, PARAM_ACK_ACCEPTED, node_id);
        } else {
            // This was a get operation - send PARAM_EXT_VALUE
            gcs_chan->send_param_ext_value(req.param_name, value_str,
                                           MAV_PARAM_EXT_TYPE_REAL32, node_id);
        }
    }

    // Remove request from queue (TODO: implement proper removal)
    // For now we rely on timeout to clear it

    return true;
}

static bool dronecan_param_get_set_int_cb(AP_DroneCAN* ap_dronecan, const uint8_t node_id,
                                           const char* name, int32_t &value)
{
    // Find matching pending request
    GCS_MAVLINK::pending_param_ext_request req;
    bool found = false;

    // Search through all pending requests
    for (uint8_t i = 0; i < GCS_MAVLINK::param_ext_requests.available(); i++) {
        if (GCS_MAVLINK::param_ext_requests.peek(req, i)) {
            if (req.node_id == node_id &&
                strncmp(req.param_name, name, sizeof(req.param_name)) == 0) {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        return false;  // No matching request found
    }

    // Format value as string
    char value_str[128];
    snprintf(value_str, sizeof(value_str), "%ld", (long)value);

    // Send response to GCS
    GCS_MAVLINK *gcs_chan = gcs().chan(req.chan);
    if (gcs_chan != nullptr) {
        if (req.is_set) {
            // This was a set operation - send PARAM_EXT_ACK
            gcs_chan->send_param_ext_ack(req.param_name, value_str,
                                         MAV_PARAM_EXT_TYPE_INT32, PARAM_ACK_ACCEPTED, node_id);
        } else {
            // This was a get operation - send PARAM_EXT_VALUE
            gcs_chan->send_param_ext_value(req.param_name, value_str,
                                           MAV_PARAM_EXT_TYPE_INT32, node_id);
        }
    }

    // Remove request from queue (TODO: implement proper removal)
    // For now we rely on timeout to clear it

    return true;
}

static bool dronecan_param_get_set_string_cb(AP_DroneCAN* ap_dronecan, const uint8_t node_id,
                                              const char* name, AP_DroneCAN::string &value)
{
    // Find matching pending request
    GCS_MAVLINK::pending_param_ext_request req;
    bool found = false;

    // Search through all pending requests
    for (uint8_t i = 0; i < GCS_MAVLINK::param_ext_requests.available(); i++) {
        if (GCS_MAVLINK::param_ext_requests.peek(req, i)) {
            if (req.node_id == node_id &&
                strncmp(req.param_name, name, sizeof(req.param_name)) == 0) {
                found = true;
                break;
            }
        }
    }

    if (!found) {
        return false;  // No matching request found
    }

    // Copy string value (ensure null termination)
    char value_str[128];
    size_t copy_len = MIN(value.len, sizeof(value_str) - 1);
    memcpy(value_str, value.data, copy_len);
    value_str[copy_len] = '\0';

    // Send response to GCS
    GCS_MAVLINK *gcs_chan = gcs().chan(req.chan);
    if (gcs_chan != nullptr) {
        if (req.is_set) {
            // This was a set operation - send PARAM_EXT_ACK
            gcs_chan->send_param_ext_ack(req.param_name, value_str,
                                         MAV_PARAM_EXT_TYPE_CUSTOM, PARAM_ACK_ACCEPTED, node_id);
        } else {
            // This was a get operation - send PARAM_EXT_VALUE
            gcs_chan->send_param_ext_value(req.param_name, value_str,
                                           MAV_PARAM_EXT_TYPE_CUSTOM, node_id);
        }
    }

    // Remove request from queue (TODO: implement proper removal)
    // For now we rely on timeout to clear it

    return true;
}
#endif

bool GCS_MAVLINK::param_timer_registered;

/**
 * @brief Send the next pending parameter, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_param_send()
{
    // send parameter async replies
    uint8_t async_replies_sent_count = send_parameter_async_replies();

    // now send the streaming parameters (from PARAM_REQUEST_LIST)
    if (_queued_parameter == nullptr) {
        // .... or not....
        return;
    }

    const uint32_t tnow = AP_HAL::millis();
    const uint32_t tstart = AP_HAL::micros();

    // use at most 30% of bandwidth on parameters
    const uint32_t link_bw = _port->bw_in_bytes_per_second();

    uint32_t bytes_allowed = link_bw * (tnow - _queued_parameter_send_time_ms) / 3333;
    const uint16_t size_for_one_param_value_msg = MAVLINK_MSG_ID_PARAM_VALUE_LEN + packet_overhead();
    if (bytes_allowed < size_for_one_param_value_msg) {
        bytes_allowed = size_for_one_param_value_msg;
    }
    if (bytes_allowed > txspace()) {
        bytes_allowed = txspace();
    }
    uint32_t count = bytes_allowed / size_for_one_param_value_msg;

    // when we don't have flow control we really need to keep the
    // param download very slow, or it tends to stall
    if (!have_flow_control() && count > 5) {
        count = 5;
    }
    if (async_replies_sent_count >= count) {
        return;
    }
    count -= async_replies_sent_count;

    while (count && _queued_parameter != nullptr && last_txbuf_is_greater(33)) {
        char param_name[AP_MAX_NAME_SIZE];
        _queued_parameter->copy_name_token(_queued_parameter_token, param_name, sizeof(param_name), true);

        mavlink_msg_param_value_send(
            chan,
            param_name,
            _queued_parameter->cast_to_float(_queued_parameter_type),
            mav_param_type(_queued_parameter_type),
            _queued_parameter_count,
            _queued_parameter_index);

        _queued_parameter = AP_Param::next_scalar(&_queued_parameter_token, &_queued_parameter_type);
        _queued_parameter_index++;

        if (AP_HAL::micros() - tstart > 1000) {
            // don't use more than 1ms sending blocks of parameters
            break;
        }
        count--;
    }
    _queued_parameter_send_time_ms = tnow;
}

/*
  return true if a channel has flow control
 */
bool GCS_MAVLINK::have_flow_control(void)
{
    if (_port == nullptr) {
        return false;
    }

    if (_port->flow_control_enabled()) {
        return true;
    }

    if (chan == MAVLINK_COMM_0) {
        // assume USB console has flow control
        return hal.gpio->usb_connected();
    }

    return false;
}


/*
  handle a request to change stream rate. Note that copter passes in
  save==false so we don't want the save to happen when the user connects the
  ground station.
 */
void GCS_MAVLINK::handle_request_data_stream(const mavlink_message_t &msg)
{
    mavlink_request_data_stream_t packet;
    mavlink_msg_request_data_stream_decode(&msg, &packet);

    int16_t freq = 0;     // packet frequency

    if (packet.start_stop == 0)
        freq = 0;                     // stop sending
    else if (packet.start_stop == 1)
        freq = packet.req_message_rate;                     // start sending
    else
        return;

    // if stream_id is still NUM_STREAMS at the end of this switch
    // block then either we set stream rates for all streams, or we
    // were asked to set the streamrate for an unrecognised stream
    streams stream_id = NUM_STREAMS;
    switch (packet.req_stream_id) {
    case MAV_DATA_STREAM_ALL:
        for (uint8_t i=0; i<NUM_STREAMS; i++) {
            if (i == STREAM_PARAMS) {
                // don't touch parameter streaming rate; it is
                // considered "internal".
                continue;
            }
            if (persist_streamrates()) {
                streamRates[i].set_and_save_ifchanged(freq);
            } else {
                streamRates[i].set(freq);
            }
            initialise_message_intervals_for_stream((streams)i);
        }
        break;
    case MAV_DATA_STREAM_RAW_SENSORS:
        stream_id = STREAM_RAW_SENSORS;
        break;
    case MAV_DATA_STREAM_EXTENDED_STATUS:
        stream_id = STREAM_EXTENDED_STATUS;
        break;
    case MAV_DATA_STREAM_RC_CHANNELS:
        stream_id = STREAM_RC_CHANNELS;
        break;
    case MAV_DATA_STREAM_RAW_CONTROLLER:
        stream_id = STREAM_RAW_CONTROLLER;
        break;
    case MAV_DATA_STREAM_POSITION:
        stream_id = STREAM_POSITION;
        break;
    case MAV_DATA_STREAM_EXTRA1:
        stream_id = STREAM_EXTRA1;
        break;
    case MAV_DATA_STREAM_EXTRA2:
        stream_id = STREAM_EXTRA2;
        break;
    case MAV_DATA_STREAM_EXTRA3:
        stream_id = STREAM_EXTRA3;
        break;
    }

    if (stream_id == NUM_STREAMS) {
        // asked to set rate on unknown stream (or all were set already)
        return;
    }

    AP_Int16 *rate = &streamRates[stream_id];

    if (rate != nullptr) {
        if (persist_streamrates()) {
            rate->set_and_save_ifchanged(freq);
        } else {
            rate->set(freq);
        }
        initialise_message_intervals_for_stream(stream_id);
    }
}

void GCS_MAVLINK::handle_param_request_list(const mavlink_message_t &msg)
{
    if (!params_ready()) {
        return;
    }

    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(&msg, &packet);

    // requesting parameters is a convenient way to get extra information
    send_banner();

    // Start sending parameters - next call to ::update will kick the first one out
    _queued_parameter = AP_Param::first(&_queued_parameter_token, &_queued_parameter_type);
    _queued_parameter_index = 0;
    _queued_parameter_count = AP_Param::count_parameters();
    _queued_parameter_send_time_ms = AP_HAL::millis(); // avoid initial flooding
}

void GCS_MAVLINK::handle_param_request_read(const mavlink_message_t &msg)
{
    if (param_requests.space() == 0) {
        // we can't process this right now, drop it
        return;
    }
    
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(&msg, &packet);

    /*
      we reserve some space for sending parameters if the client ever
      fails to get a parameter due to lack of space
     */
    uint32_t saved_reserve_param_space_start_ms = reserve_param_space_start_ms;
    reserve_param_space_start_ms = 0; // bypass packet_overhead_chan reservation checking
    if (!HAVE_PAYLOAD_SPACE(chan, PARAM_VALUE)) {
        reserve_param_space_start_ms = AP_HAL::millis();
    } else {
        reserve_param_space_start_ms = saved_reserve_param_space_start_ms;
    }

    struct pending_param_request req;
    req.chan = chan;
    req.param_index = packet.param_index;
    memcpy(req.param_name, packet.param_id, MIN(sizeof(packet.param_id), sizeof(req.param_name)));
    req.param_name[AP_MAX_NAME_SIZE] = 0;

    // queue it for processing by io timer
    param_requests.push(req);

    // speaking of which, we'd best make sure it is running:
    if (!param_timer_registered) {
        param_timer_registered = true;
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&GCS_MAVLINK::param_io_timer, void));
    }
}

void GCS_MAVLINK::handle_param_set(const mavlink_message_t &msg)
{
    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(&msg, &packet);
    enum ap_var_type var_type;

    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    // find existing param so we can get the old value
    uint16_t parameter_flags = 0;
    vp = AP_Param::find(key, &var_type, &parameter_flags);
    if (vp == nullptr || isnan(packet.param_value) || isinf(packet.param_value)) {
        return;
    }

    float old_value = vp->cast_to_float(var_type);

    if (!vp->allow_set_via_mavlink(parameter_flags)) {
        // don't warn the user about this failure if we are dropping
        // messages here.  This is on the assumption that scripting is
        // currently responsible for setting parameters and may set
        // the value instead of us.
        if (gcs().get_allow_param_set()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Param write denied (%s)", key);
        }
        // send the readonly value
        send_parameter_value(key, var_type, old_value);
        return;
    }

    // set the value
    vp->set_float(packet.param_value, var_type);

    /*
      we force the save if the value is not equal to the old
      value. This copes with the use of override values in
      constructors, such as PID elements. Otherwise a set to the
      default value which differs from the constructor value doesn't
      save the change
     */
    bool force_save = !is_equal(packet.param_value, old_value);

    // save the change
    vp->save(force_save);

    if (force_save && (parameter_flags & AP_PARAM_FLAG_ENABLE)) {
        AP_Param::invalidate_count();
    }

#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Parameter(key, vp->cast_to_float(var_type));
    }
#endif
}

void GCS_MAVLINK::send_parameter_value(const char *param_name, ap_var_type param_type, float param_value)
{
    if (!HAVE_PAYLOAD_SPACE(chan, PARAM_VALUE)) {
        return;
    }
    mavlink_msg_param_value_send(
        chan,
        param_name,
        param_value,
        mav_param_type(param_type),
        AP_Param::count_parameters(),
        -1);
}

/*
  send a parameter value message to all active MAVLink connections
 */
void GCS::send_parameter_value(const char *param_name, ap_var_type param_type, float param_value)
{
    mavlink_param_value_t packet{};
    const uint8_t to_copy = MIN(ARRAY_SIZE(packet.param_id), strlen(param_name));
    memcpy(packet.param_id, param_name, to_copy);
    packet.param_value = param_value;
    packet.param_type = GCS_MAVLINK::mav_param_type(param_type);
    packet.param_count = AP_Param::count_parameters();
    packet.param_index = -1;

    gcs().send_to_active_channels(MAVLINK_MSG_ID_PARAM_VALUE,
                                  (const char *)&packet);

#if HAL_LOGGING_ENABLED
    // also log to AP_Logger
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger != nullptr) {
        logger->Write_Parameter(param_name, param_value);
    }
#endif
}


/*
  timer callback for async parameter requests
 */
void GCS_MAVLINK::param_io_timer(void)
{
    struct pending_param_request req;

    // this is mostly a no-op, but doing this here means we won't
    // block the main thread counting parameters (~30ms on PH)
    AP_Param::count_parameters();

    if (param_replies.space() == 0) {
        // no room
        return;
    }
    
    if (!param_requests.pop(req)) {
        // nothing to do
        return;
    }

    struct pending_param_reply reply;
    AP_Param *vp;

    if (req.param_index != -1) {
        AP_Param::ParamToken token {};
        vp = AP_Param::find_by_index(req.param_index, &reply.p_type, &token);
        if (vp == nullptr) {
            return;
        }
        vp->copy_name_token(token, reply.param_name, AP_MAX_NAME_SIZE, true);
    } else {
        strncpy(reply.param_name, req.param_name, AP_MAX_NAME_SIZE+1);
        vp = AP_Param::find(req.param_name, &reply.p_type);
        if (vp == nullptr) {
            return;
        }
    }

    reply.chan = req.chan;
    reply.param_name[AP_MAX_NAME_SIZE] = 0;
    reply.value = vp->cast_to_float(reply.p_type);
    reply.param_index = req.param_index;
    reply.count = AP_Param::count_parameters();

    // queue for transmission
    param_replies.push(reply);
}

/*
  send replies to PARAM_REQUEST_READ
 */
uint8_t GCS_MAVLINK::send_parameter_async_replies()
{
    uint8_t async_replies_sent_count = 0;

    while (async_replies_sent_count < 5) {
        struct pending_param_reply reply;
        if (!param_replies.peek(reply)) {
            return async_replies_sent_count;
        }

        /*
          we reserve some space for sending parameters if the client ever
          fails to get a parameter due to lack of space
        */
        uint32_t saved_reserve_param_space_start_ms = reserve_param_space_start_ms;
        reserve_param_space_start_ms = 0; // bypass packet_overhead_chan reservation checking
        if (!HAVE_PAYLOAD_SPACE(reply.chan, PARAM_VALUE)) {
            reserve_param_space_start_ms = AP_HAL::millis();
            return async_replies_sent_count;
        }
        reserve_param_space_start_ms = saved_reserve_param_space_start_ms;

        mavlink_msg_param_value_send(
            reply.chan,
            reply.param_name,
            reply.value,
            mav_param_type(reply.p_type),
            reply.count,
            reply.param_index);

        _queued_parameter_send_time_ms = AP_HAL::millis();
        async_replies_sent_count++;

        if (!param_replies.pop()) {
            // internal error...
            return async_replies_sent_count;
        }
    }
    return async_replies_sent_count;
}

void GCS_MAVLINK::handle_common_param_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        handle_param_request_list(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
        handle_param_set(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_param_request_read(msg);
        break;
    }
}

#if HAL_ENABLE_DRONECAN_DRIVERS

void GCS_MAVLINK::handle_common_param_ext_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
        handle_param_ext_request_list(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
        handle_param_ext_request_read(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_EXT_SET:
        handle_param_ext_set(msg);
        break;
    }
}

void GCS_MAVLINK::handle_param_ext_request_list(const mavlink_message_t &msg)
{
    // PARAM_EXT_REQUEST_LIST is not supported for DroneCAN nodes
    // DroneCAN protocol doesn't provide a way to list all parameters
    // Send a PARAM_EXT_ACK with result PARAM_ACK_FAILED to indicate not supported
    mavlink_param_ext_request_list_t packet;
    mavlink_msg_param_ext_request_list_decode(&msg, &packet);

    // Note: We could send an empty list, but it's clearer to just not respond
    // GCS should use PARAM_EXT_REQUEST_READ for specific parameters
}

void GCS_MAVLINK::handle_param_ext_request_read(const mavlink_message_t &msg)
{
    mavlink_param_ext_request_read_t packet;
    mavlink_msg_param_ext_request_read_decode(&msg, &packet);

    // Check if component ID is in USER range for DroneCAN nodes
    if (packet.target_component < 25 || packet.target_component > 99) {
        // Not a DroneCAN node request
        return;
    }

    // Extract node ID from component ID
    uint8_t node_id = packet.target_component - 24;  // Component 25 → node 1

    // Find which CAN interface has this node
    AP_DroneCAN *ap_dronecan = nullptr;
    uint8_t can_driver_index = 0;

    for (uint8_t i = 0; i < AP_DRONECAN_MAX_INSTANCES; i++) {
        AP_DroneCAN *candidate = AP_DroneCAN::get_dronecan(i);
        if (candidate != nullptr &&
            candidate->get_dna_server().node_seen.get(node_id)) {
            ap_dronecan = candidate;
            can_driver_index = i;
            break;  // Use first interface that has this node
        }
    }

    if (ap_dronecan == nullptr) {
        // Node not found on any interface
        send_param_ext_ack(packet.param_id, "",
                          MAV_PARAM_EXT_TYPE_REAL32, PARAM_ACK_FAILED, node_id);
        return;
    }

    // Extract parameter name
    char param_name[17];
    memcpy(param_name, packet.param_id, 16);
    param_name[16] = '\0';

    // Create pending request
    pending_param_ext_request req;
    req.chan = chan;
    req.can_driver_index = can_driver_index;
    req.node_id = node_id;
    strncpy(req.param_name, param_name, sizeof(req.param_name) - 1);
    req.param_name[sizeof(req.param_name) - 1] = '\0';
    req.is_set = false;
    req.request_time_ms = AP_HAL::millis();

    // Try to initiate DroneCAN parameter get (try float first, most common)
    static AP_DroneCAN::ParamGetSetFloatCb float_cb(dronecan_param_get_set_float_cb);
    if (ap_dronecan->get_parameter_on_node(node_id, param_name, &float_cb)) {
        param_ext_requests.push(req);
        return;
    }

    // Try integer
    static AP_DroneCAN::ParamGetSetIntCb int_cb(dronecan_param_get_set_int_cb);
    if (ap_dronecan->get_parameter_on_node(node_id, param_name, &int_cb)) {
        param_ext_requests.push(req);
        return;
    }

    // Try string
    static AP_DroneCAN::ParamGetSetStringCb string_cb(dronecan_param_get_set_string_cb);
    if (ap_dronecan->get_parameter_on_node(node_id, param_name, &string_cb)) {
        param_ext_requests.push(req);
        return;
    }

    // All attempts failed - probably busy
    send_param_ext_ack(packet.param_id, "",
                      MAV_PARAM_EXT_TYPE_REAL32, PARAM_ACK_FAILED, node_id);
}

void GCS_MAVLINK::handle_param_ext_set(const mavlink_message_t &msg)
{
    mavlink_param_ext_set_t packet;
    mavlink_msg_param_ext_set_decode(&msg, &packet);

    // Check if component ID is in USER range for DroneCAN nodes
    if (packet.target_component < 25 || packet.target_component > 99) {
        // Not a DroneCAN node request
        return;
    }

    // Extract node ID from component ID
    uint8_t node_id = packet.target_component - 24;  // Component 25 → node 1

    // Find which CAN interface has this node
    AP_DroneCAN *ap_dronecan = nullptr;
    uint8_t can_driver_index = 0;

    for (uint8_t i = 0; i < AP_DRONECAN_MAX_INSTANCES; i++) {
        AP_DroneCAN *candidate = AP_DroneCAN::get_dronecan(i);
        if (candidate != nullptr &&
            candidate->get_dna_server().node_seen.get(node_id)) {
            ap_dronecan = candidate;
            can_driver_index = i;
            break;  // Use first interface that has this node
        }
    }

    // Extract parameter name
    char param_name[17];
    memcpy(param_name, packet.param_id, 16);
    param_name[16] = '\0';

    // Extract parameter value (ensure null termination)
    char param_value[129];
    memcpy(param_value, packet.param_value, 128);
    param_value[128] = '\0';

    if (ap_dronecan == nullptr) {
        // Node not found on any interface
        send_param_ext_ack(packet.param_id, param_value, packet.param_type, PARAM_ACK_FAILED, node_id);
        return;
    }

    // Create pending request
    pending_param_ext_request req;
    req.chan = chan;
    req.can_driver_index = can_driver_index;
    req.node_id = node_id;
    strncpy(req.param_name, param_name, sizeof(req.param_name) - 1);
    req.param_name[sizeof(req.param_name) - 1] = '\0';
    strncpy(req.param_value, param_value, sizeof(req.param_value) - 1);
    req.param_value[sizeof(req.param_value) - 1] = '\0';
    req.param_type = packet.param_type;
    req.is_set = true;
    req.request_time_ms = AP_HAL::millis();

    // Parse value and call appropriate DroneCAN set function
    bool success = false;

    switch (packet.param_type) {
    case MAV_PARAM_EXT_TYPE_REAL32: {
        float value = atof(param_value);
        static AP_DroneCAN::ParamGetSetFloatCb float_cb(dronecan_param_get_set_float_cb);
        success = ap_dronecan->set_parameter_on_node(node_id, param_name, value, &float_cb);
        break;
    }
    case MAV_PARAM_EXT_TYPE_INT32: {
        int32_t value = atol(param_value);
        static AP_DroneCAN::ParamGetSetIntCb int_cb(dronecan_param_get_set_int_cb);
        success = ap_dronecan->set_parameter_on_node(node_id, param_name, value, &int_cb);
        break;
    }
    case MAV_PARAM_EXT_TYPE_CUSTOM: {
        AP_DroneCAN::string value;
        value.len = strlen(param_value);
        if (value.len > sizeof(value.data)) {
            value.len = sizeof(value.data);
        }
        memcpy(value.data, param_value, value.len);
        static AP_DroneCAN::ParamGetSetStringCb string_cb(dronecan_param_get_set_string_cb);
        success = ap_dronecan->set_parameter_on_node(node_id, param_name, value, &string_cb);
        break;
    }
    default:
        // Unsupported parameter type
        send_param_ext_ack(packet.param_id, param_value, packet.param_type, PARAM_ACK_VALUE_UNSUPPORTED, node_id);
        return;
    }

    if (success) {
        // Queue the request
        if (!param_ext_requests.push(req)) {
            // Queue full
            send_param_ext_ack(packet.param_id, param_value, packet.param_type, PARAM_ACK_FAILED, node_id);
        }
    } else {
        // DroneCAN call failed - probably busy
        send_param_ext_ack(packet.param_id, param_value, packet.param_type, PARAM_ACK_FAILED, node_id);
    }
}

void GCS_MAVLINK::send_param_ext_value(const char *param_name, const char *param_value,
                                        uint8_t param_type, uint8_t node_id)
{
    mavlink_message_t msg;
    mavlink_msg_param_ext_value_pack(
        mavlink_system.sysid,
        25 + (node_id - 1),  // Source component = node's component ID
        &msg,
        param_name,
        param_value,
        param_type,
        0,  // param_count - not used for DroneCAN parameters
        0   // param_index - not used for DroneCAN parameters
    );

    MAVLINK_ALIGNED_BUF(buf, MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
    comm_send_buffer(chan, (uint8_t*)buf, len);
}

void GCS_MAVLINK::send_param_ext_ack(const char *param_name, const char *param_value,
                                      uint8_t param_type, uint8_t param_result, uint8_t node_id)
{
    mavlink_message_t msg;
    mavlink_msg_param_ext_ack_pack(
        mavlink_system.sysid,
        25 + (node_id - 1),  // Source component = node's component ID
        &msg,
        param_name,
        param_value,
        param_type,
        param_result
    );

    MAVLINK_ALIGNED_BUF(buf, MAVLINK_MAX_PACKET_LEN);
    uint16_t len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);
    comm_send_buffer(chan, (uint8_t*)buf, len);
}

#endif // HAL_ENABLE_DRONECAN_DRIVERS

#endif  // HAL_GCS_ENABLED
