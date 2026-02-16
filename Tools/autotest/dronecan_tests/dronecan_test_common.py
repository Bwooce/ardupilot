#!/usr/bin/env python3
"""
Shared utilities for standalone DroneCAN tests.

These tests can run against SITL or real ESP32 hardware via any MAVLink
connection string (UDP, serial, TCP).

Requires: MAVLINK20=1 environment variable for MAVLink2 message support.
"""

import os
import sys
import time

# Ensure MAVLink2 is enabled
os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil


def connect(connection_string="udp:127.0.0.1:14550", source_system=255, timeout=30):
    """Create a MAVLink connection and wait for heartbeat."""
    print("Connecting to %s ..." % connection_string)
    mav = mavutil.mavlink_connection(
        connection_string,
        source_system=source_system,
        source_component=0,
    )
    mav.wait_heartbeat(timeout=timeout)
    print("Connected (sysid=%d compid=%d)" % (mav.target_system, mav.target_component))
    return mav


def wait_dronecan_node(mav, timeout=60):
    """Wait for a UAVCAN_NODE_STATUS message and return the node_id (1-127)."""
    print("Waiting for UAVCAN_NODE_STATUS ...")
    tstart = time.time()
    while time.time() - tstart < timeout:
        m = mav.recv_match(type='UAVCAN_NODE_STATUS', blocking=True, timeout=1)
        if m is not None:
            node_id = m.get_srcComponent()
            if 1 <= node_id <= 127:
                print("Found DroneCAN node %d (health=%d mode=%d uptime=%d)" %
                      (node_id, m.health, m.mode, m.uptime_sec))
                return node_id
    raise TimeoutError("No UAVCAN_NODE_STATUS received after %ds" % timeout)


def pad_param_id(name):
    """Pad a parameter name to 16 bytes (MAVLink param_id field)."""
    if isinstance(name, str):
        name = name.encode('utf-8')
    return name.ljust(16, b'\x00')


def pad_param_value(value):
    """Pad a parameter value string to 128 bytes (MAVLink param_value field)."""
    if isinstance(value, str):
        value = value.encode('utf-8')
    return value.ljust(128, b'\x00')


def param_ext_read(mav, node_id, name, timeout=30):
    """Read a single parameter from a DroneCAN node via PARAM_EXT.

    Returns (name_str, value_str, param_type) or raises TimeoutError.
    """
    mav.mav.param_ext_request_read_send(
        mav.target_system,
        node_id,
        pad_param_id(name),
        -1  # use param_id, not index
    )

    tstart = time.time()
    while time.time() - tstart < timeout:
        m = mav.recv_match(type='PARAM_EXT_VALUE', blocking=True, timeout=1)
        if m is not None:
            got_name = m.param_id.rstrip('\x00')
            got_value = m.param_value.rstrip('\x00')
            return (got_name, got_value, m.param_type)

    raise TimeoutError("No PARAM_EXT_VALUE for '%s' after %ds" % (name, timeout))


def param_ext_set(mav, node_id, name, value, param_type, timeout=30):
    """Set a parameter on a DroneCAN node via PARAM_EXT.

    Returns True if ACK received with ACCEPTED, False otherwise.
    """
    mav.mav.param_ext_set_send(
        mav.target_system,
        node_id,
        pad_param_id(name),
        pad_param_value(str(value)),
        param_type
    )

    tstart = time.time()
    while time.time() - tstart < timeout:
        m = mav.recv_match(type='PARAM_EXT_ACK', blocking=True, timeout=1)
        if m is not None:
            # PARAM_ACK_ACCEPTED = 0
            return m.param_result == 0

    raise TimeoutError("No PARAM_EXT_ACK for '%s' after %ds" % (name, timeout))


def param_ext_enumerate(mav, node_id, timeout=30, silence_timeout=5):
    """Enumerate all parameters from a DroneCAN node via PARAM_EXT.

    Returns dict of {index: (name, value, type)}.
    """
    mav.mav.param_ext_request_list_send(
        mav.target_system,
        node_id
    )

    params = {}
    last_receive = time.time()
    deadline = time.time() + timeout

    while time.time() < deadline:
        m = mav.recv_match(type='PARAM_EXT_VALUE', blocking=True, timeout=1)
        if m is not None:
            name = m.param_id.rstrip('\x00')
            value = m.param_value.rstrip('\x00')
            params[m.param_index] = (name, value, m.param_type)
            last_receive = time.time()
        elif time.time() - last_receive > silence_timeout:
            break

    return params
