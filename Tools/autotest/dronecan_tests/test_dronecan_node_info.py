#!/usr/bin/env python3
"""
Test UAVCAN_NODE_INFO MAVLink broadcasting from DroneCAN nodes.

Usage:
    MAVLINK20=1 python3 test_dronecan_node_info.py [connection] [timeout]

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

    # Collect NODE_INFO
    print("Waiting for UAVCAN_NODE_INFO from node %d ..." % node_id)
    tstart = time.time()
    while time.time() - tstart < timeout:
        m = mav.recv_match(type='UAVCAN_NODE_INFO', blocking=True, timeout=1)
        if m is None:
            continue

        src = m.get_srcComponent()
        if src != node_id:
            continue

        name = m.name.rstrip('\x00')
        print("  name='%s'" % name)
        print("  hw_version=%d.%d" % (m.hw_version_major, m.hw_version_minor))
        print("  sw_version=%d.%d" % (m.sw_version_major, m.sw_version_minor))
        print("  sw_vcs_commit=%d" % m.sw_vcs_commit)
        print("  uptime_sec=%d" % m.uptime_sec)

        uid = m.hw_unique_id
        uid_hex = ':'.join('%02x' % b for b in uid)
        print("  hw_unique_id=%s" % uid_hex)

        # Validate
        errors = []
        if len(name) == 0:
            errors.append("node name is empty")
        if all(b == 0 for b in uid):
            errors.append("hw_unique_id is all zeros")

        if errors:
            for e in errors:
                print("FAIL: %s" % e)
            return 1

        print("PASS: UAVCAN_NODE_INFO validated for node %d ('%s')" % (node_id, name))
        return 0

    print("FAIL: No UAVCAN_NODE_INFO received from node %d after %ds" % (node_id, timeout))
    return 1


if __name__ == "__main__":
    sys.exit(main())
