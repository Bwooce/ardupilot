#!/usr/bin/env python3
"""
Test UAVCAN_NODE_STATUS MAVLink broadcasting from DroneCAN nodes.

Usage:
    MAVLINK20=1 python3 test_dronecan_node_status.py [connection] [timeout]

    connection: MAVLink connection string (default: udp:127.0.0.1:14550)
    timeout:    seconds to wait for messages (default: 60)

Exit codes:
    0 = pass
    1 = fail
"""

import os
import sys
import time

os.environ.setdefault('MAVLINK20', '1')

# Add parent path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dronecan_test_common import connect, wait_dronecan_node


def main():
    connection = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
    timeout = int(sys.argv[2]) if len(sys.argv) > 2 else 60

    try:
        mav = connect(connection, timeout=timeout)
    except Exception as e:
        print("FAIL: Connection failed: %s" % e)
        return 1

    # Wait for a DroneCAN node to appear
    try:
        node_id = wait_dronecan_node(mav, timeout=timeout)
    except TimeoutError as e:
        print("FAIL: %s" % e)
        return 1

    # Collect and validate NODE_STATUS messages
    print("Collecting UAVCAN_NODE_STATUS messages from node %d ..." % node_id)
    status_count = 0
    tstart = time.time()

    while time.time() - tstart < 10:
        m = mav.recv_match(type='UAVCAN_NODE_STATUS', blocking=True, timeout=1)
        if m is None:
            continue

        src = m.get_srcComponent()
        if src != node_id:
            continue

        status_count += 1
        print("  [%d] health=%d mode=%d sub_mode=%d uptime=%d vendor_status=%d" %
              (status_count, m.health, m.mode, m.sub_mode, m.uptime_sec,
               m.vendor_specific_status_code))

        # Validate fields
        if m.health > 3:
            print("FAIL: Invalid health value %d (expected 0-3)" % m.health)
            return 1
        if m.mode > 7:
            print("FAIL: Invalid mode value %d (expected 0-7)" % m.mode)
            return 1

        if status_count >= 3:
            break

    if status_count == 0:
        print("FAIL: No NODE_STATUS messages received from node %d" % node_id)
        return 1

    # Verify uptime is progressing
    if m.uptime_sec == 0:
        print("WARNING: uptime_sec is 0 (node may have just started)")

    print("PASS: Received %d UAVCAN_NODE_STATUS messages from node %d" %
          (status_count, node_id))
    return 0


if __name__ == "__main__":
    sys.exit(main())
