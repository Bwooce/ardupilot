# CAN Bus BUS_OFF State Diagnosis

## Problem Summary
The ESP32 TWAI controller is immediately entering BUS_OFF state with `tx_err=128` (maximum error count). This indicates the CAN transmitter cannot get ACK bits from other nodes on the bus.

## Observed Behavior
```
TX ESP_ERR_INVALID_STATE: state=BUS_OFF, tx_err=128, rx_err=0
```

The cycle repeats:
1. **BUS_OFF** (tx_err=128) → Initiates recovery
2. **RECOVERING** → Brief recovery attempt
3. **STOPPED** → Falls back to stopped state
4. Attempt to start → Immediately returns to **BUS_OFF**

## Root Cause
The `tx_err=128` with `rx_err=0` pattern indicates the transmitter is not receiving hardware ACKs. This is NOT a software issue but a CAN bus physical/electrical problem.

## Common Causes (Check These)

### 1. **Bitrate Mismatch** (MOST LIKELY)
- ESP32 defaults to 1Mbps (1000000)
- Check if your other node uses a different rate (125k, 250k, 500k are common)
- **Fix**: Set `CAN_P1_BITRATE` parameter to match your node's rate
- Example for 500kbps: `param set CAN_P1_BITRATE 500000`

### 2. **Missing Termination Resistors**
- CAN bus requires 120Ω resistors at each end of the bus
- Without termination, signals reflect causing errors
- **Fix**: Add 120Ω resistors between CAN_H and CAN_L at both ends

### 3. **Wiring Issues**
- CAN_H and CAN_L might be swapped
- Poor connections or broken wires
- **Fix**: Verify wiring, ensure CAN_H→CAN_H and CAN_L→CAN_L

### 4. **Transceiver Power**
- CAN transceiver might not have power
- ESP32 GPIO outputs CAN signals but needs transceiver for bus
- **Fix**: Check transceiver has proper 3.3V/5V power

### 5. **Ground Reference**
- All CAN nodes must share a common ground
- **Fix**: Connect GND between all nodes

## Diagnostic Steps

1. **Check other node's bitrate** - This is likely the issue
2. **Measure bus voltage** - Should see ~2.5V on both CAN_H and CAN_L when idle
3. **Check termination** - Measure ~60Ω between CAN_H and CAN_L (two 120Ω in parallel)
4. **Swap CAN_H/CAN_L** - Quick test if wiring is reversed

## Test Commands
```bash
# Try different bitrates (restart required after each change)
param set CAN_P1_BITRATE 500000
param set CAN_P1_BITRATE 250000
param set CAN_P1_BITRATE 125000

# Monitor CAN stats
dronecan status
```

## Note
The fact that you see `CAN_RX: MISMATCH` messages suggests SOME data is getting through, which points to bitrate mismatch as the most likely cause.