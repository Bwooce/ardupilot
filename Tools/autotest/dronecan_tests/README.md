# DroneCAN Test Suite

Tests for DroneCAN MAVLink broadcasting (UAVCAN_NODE_STATUS/INFO) and PARAM_EXT
parameter bridge protocol.

## Architecture

There are two test harnesses:

### SITL Autotest (`test.DroneCAN`)

Integrated into the ArduPilot autotest framework. Uses Copter with a SITL
periph instance connected via multicast CAN. Test methods live in
`vehicle_test_suite.py`, registered via `AutoTestDroneCAN` in `arducopter.py`.

### Standalone Scripts

Pure pymavlink scripts that connect to any MAVLink endpoint. No dependency on
the autotest framework -- works with SITL, Pixhawk, ESP32, or anything else
that speaks MAVLink2.

## Running SITL Tests

Build periph and run the test group:

```
python3 Tools/autotest/autotest.py build.SITLPeriphUniversal test.DroneCAN
```

Run a specific test:

```
python3 Tools/autotest/autotest.py test.DroneCAN.DroneCAN_NodeStatus
```

## Running Standalone Scripts

All standalone scripts require MAVLink2 (`MAVLINK20=1`).

### Against SITL

Start SITL with CAN enabled, then connect:

```
# Terminal 1: start sim_vehicle with CAN
sim_vehicle.py -v Copter --map --console -A "--uartF=sim:periph_universal" \
    --add-param-file=Tools/autotest/default_params/periph.parm

# Terminal 2: run tests
cd Tools/autotest/dronecan_tests
MAVLINK20=1 python3 test_dronecan_node_status.py udp:127.0.0.1:14550
MAVLINK20=1 python3 test_dronecan_node_info.py udp:127.0.0.1:14550
MAVLINK20=1 python3 test_dronecan_param_ext.py udp:127.0.0.1:14550
```

### Against Hardware (ESP32, Pixhawk, etc.)

```
# WiFi UDP
MAVLINK20=1 python3 test_dronecan_param_ext.py udp:192.168.4.1:14550

# Serial
MAVLINK20=1 python3 test_dronecan_param_ext.py /dev/ttyUSB0,115200

# TCP
MAVLINK20=1 python3 test_dronecan_param_ext.py tcp:192.168.4.1:5760
```

Specify a known node ID to skip auto-discovery:

```
MAVLINK20=1 python3 test_dronecan_param_ext.py udp:192.168.4.1:14550 42
```

## Test Descriptions

| Test | SITL Method | Standalone Script | What it tests |
|------|------------|-------------------|---------------|
| Node Status | `DroneCAN_NodeStatus` | `test_dronecan_node_status.py` | NODE_STATUS broadcasting, health/mode fields, maintenance mode transition |
| Node Info | `DroneCAN_NodeInfo` | `test_dronecan_node_info.py` | NODE_INFO broadcasting, name, version, unique_id |
| Param Read | `DroneCAN_ParamExtRead` | `test_dronecan_param_ext.py` | PARAM_EXT_REQUEST_READ for known parameters |
| Param Set | `DroneCAN_ParamExtSet` | `test_dronecan_param_ext.py` | PARAM_EXT_SET with ACK, readback verification |
| Param List | `DroneCAN_ParamExtList` | `test_dronecan_param_ext.py` | PARAM_EXT_REQUEST_LIST enumeration, sequential indices |

## Adding a New SITL Test

1. Add a method to `vehicle_test_suite.py` following this pattern:

```python
def DroneCAN_MyNewTest(self):
    '''description of what this tests'''
    self.context_push()
    self.set_parameters({
        "CAN_P1_DRIVER": 1,
        "GPS1_TYPE": 9,
        "SIM_GPS1_ENABLE": 0,
    })
    self.reboot_sitl()

    # Wait for periph node
    node_id = self.wait_dronecan_node(timeout=60)

    # ... test logic ...
    # Use self.assert_receive_message('MSG_TYPE', timeout=30) to wait for messages
    # Use self.mav.mav.<msg>_send() to send messages
    # Raise NotAchievedException("reason") on failure

    self.context_pop()
    self.reboot_sitl()
```

2. Register in `arducopter.py` `AutoTestDroneCAN.tests()`:

```python
class AutoTestDroneCAN(AutoTestCopter):
    def tests(self):
        return [
            # ... existing tests ...
            self.DroneCAN_MyNewTest,
        ]
```

## Adding a New Standalone Script

1. Create `test_dronecan_mytest.py` in this directory
2. Import helpers from `dronecan_test_common.py`
3. Follow the exit code convention: 0 = pass, 1 = fail
4. Accept connection string as first argument, default to `udp:127.0.0.1:14550`
5. Set `MAVLINK20=1` at the top of the script

## Key Patterns

### Node Discovery

SITL:
```python
node_id = self.wait_dronecan_node(timeout=60)
```

Standalone:
```python
from dronecan_test_common import connect, wait_dronecan_node
mav = connect("udp:127.0.0.1:14550")
node_id = wait_dronecan_node(mav, timeout=60)
```

### PARAM_EXT Message Format

- `param_id`: 16-byte char array, null-padded
- `param_value`: 128-byte char array, null-padded, contains string representation of value
- `target_component`: DroneCAN node ID (1-127, per MAVLink UAVCAN spec)
- `param_type`: `MAV_PARAM_EXT_TYPE_REAL32` (9) for floats, etc.

### Maintenance Mode

In SITL, toggle periph maintenance mode to test health/mode changes:
```python
self.stop_sup_program(instance=0)
self.start_sup_program(instance=0, args="-M")  # maintenance mode
```

## Troubleshooting

- **No UAVCAN_NODE_STATUS**: Ensure `MAVLINK20=1` is set. These are MAVLink2-only
  messages (id 310-311, 320-324 are all > 255).
- **Node discovery timeout**: The periph needs time to boot and complete DNA
  allocation. Increase timeout or add delay.
- **PARAM_EXT_ACK with FAILED**: The node may not have been discovered yet on
  the CAN bus. Wait for NODE_STATUS first.
- **Enumeration terminates early**: The silence timeout (default 5s) may be
  too short if the node has many parameters. Increase `silence_timeout`.
