#!/usr/bin/env python3
"""
Test PARAM_EXT protocol for DroneCAN node parameter access.

Tests parameter read, set, and enumerate operations via the MAVLink
PARAM_EXT bridge.

Usage:
    MAVLINK20=1 python3 test_dronecan_param_ext.py [connection] [node_id]

    connection: MAVLink connection string (default: udp:127.0.0.1:14550)
    node_id:    DroneCAN node ID (default: auto-discover via NODE_STATUS)

Exit codes:
    0 = all tests pass
    1 = any test fails
"""

import os
import sys
import time

os.environ.setdefault('MAVLINK20', '1')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dronecan_test_common import (
    connect, wait_dronecan_node,
    param_ext_read, param_ext_set, param_ext_enumerate,
)


def test_param_read(mav, node_id):
    """Test reading a known parameter."""
    print("\n--- Test: PARAM_EXT_REQUEST_READ ---")

    # GPS1_TYPE should be 1 on SITL periph, may differ on ESP32
    param_name = "GPS1_TYPE"
    print("Reading '%s' from node %d ..." % (param_name, node_id))

    try:
        name, value, ptype = param_ext_read(mav, node_id, param_name, timeout=30)
    except TimeoutError as e:
        print("FAIL: %s" % e)
        return False

    print("  Got: %s = '%s' (type=%d)" % (name, value, ptype))

    if name != param_name:
        print("FAIL: Expected param_id '%s', got '%s'" % (param_name, name))
        return False

    try:
        float(value)
    except ValueError:
        print("FAIL: Could not parse value '%s' as number" % value)
        return False

    print("PASS: param_ext_read")
    return True


def test_param_set(mav, node_id):
    """Test setting and restoring a parameter."""
    print("\n--- Test: PARAM_EXT_SET ---")

    param_name = "BARO_ENABLE"
    print("Reading current '%s' from node %d ..." % (param_name, node_id))

    try:
        name, original, ptype = param_ext_read(mav, node_id, param_name, timeout=30)
    except TimeoutError as e:
        print("FAIL: %s" % e)
        return False

    print("  Current: %s = '%s' (type=%d)" % (name, original, ptype))

    # Toggle: if 1 set to 0, if 0 set to 1
    try:
        orig_float = float(original)
    except ValueError:
        print("FAIL: Could not parse original value '%s'" % original)
        return False

    new_val = "0" if orig_float > 0.5 else "1"
    print("Setting %s to %s ..." % (param_name, new_val))

    try:
        ok = param_ext_set(mav, node_id, param_name, new_val, ptype, timeout=30)
    except TimeoutError as e:
        print("FAIL: %s" % e)
        return False

    if not ok:
        print("FAIL: PARAM_EXT_SET was not ACCEPTED")
        return False

    # Read back
    time.sleep(0.5)
    try:
        name, readback, _ = param_ext_read(mav, node_id, param_name, timeout=30)
    except TimeoutError as e:
        print("FAIL: readback: %s" % e)
        return False

    print("  Readback: %s = '%s'" % (name, readback))
    try:
        if abs(float(readback) - float(new_val)) > 0.01:
            print("FAIL: Readback '%s' != expected '%s'" % (readback, new_val))
            return False
    except ValueError:
        print("FAIL: Could not parse readback '%s'" % readback)
        return False

    # Restore original
    print("Restoring %s to %s ..." % (param_name, original))
    try:
        ok = param_ext_set(mav, node_id, param_name, original, ptype, timeout=30)
    except TimeoutError as e:
        print("WARNING: restore failed: %s" % e)
    else:
        if not ok:
            print("WARNING: restore was not ACCEPTED")

    print("PASS: param_ext_set")
    return True


def test_param_enumerate(mav, node_id):
    """Test parameter enumeration."""
    print("\n--- Test: PARAM_EXT_REQUEST_LIST ---")

    print("Enumerating parameters from node %d ..." % node_id)
    params = param_ext_enumerate(mav, node_id, timeout=60, silence_timeout=5)

    print("  Received %d parameters" % len(params))

    if len(params) < 5:
        print("FAIL: Expected at least 5 params, got %d" % len(params))
        return False

    # Check sequential indices
    indices = sorted(params.keys())
    for i, idx in enumerate(indices):
        if idx != i:
            print("FAIL: Non-sequential index: expected %d, got %d" % (i, idx))
            return False

    # Print first few
    for idx in indices[:10]:
        name, value, ptype = params[idx]
        print("  [%d] %s = '%s' (type=%d)" % (idx, name, value, ptype))
    if len(indices) > 10:
        print("  ... (%d more)" % (len(indices) - 10))

    print("PASS: param_ext_enumerate (%d params)" % len(params))
    return True


def main():
    connection = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
    node_id_arg = int(sys.argv[2]) if len(sys.argv) > 2 else None

    try:
        mav = connect(connection, timeout=30)
    except Exception as e:
        print("FAIL: Connection failed: %s" % e)
        return 1

    # Discover or use specified node_id
    if node_id_arg is not None:
        node_id = node_id_arg
        print("Using specified node_id=%d" % node_id)
    else:
        try:
            node_id = wait_dronecan_node(mav, timeout=60)
        except TimeoutError as e:
            print("FAIL: %s" % e)
            return 1

    # Allow node time to fully initialize
    time.sleep(2)

    results = []
    results.append(("read", test_param_read(mav, node_id)))
    results.append(("set", test_param_set(mav, node_id)))
    results.append(("enumerate", test_param_enumerate(mav, node_id)))

    print("\n=== Results ===")
    all_pass = True
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print("  %s: %s" % (name, status))
        if not passed:
            all_pass = False

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
