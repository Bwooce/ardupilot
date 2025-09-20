# Automatic Serial Port Coordination

## Overview
The serial port coordination system automatically manages port access between the monitor and upload agents without any manual intervention required.

## How It Works

### Lock Files
- Uses standard UNIX lock files at `/var/lock/LCK..ttyACM0`
- Each lock file contains:
  - Line 1: Process ID (PID)
  - Line 2: Tool name (e.g., "esp32_monitor_agent", "esp32_upload")

### Automatic Coordination

1. **Monitor Agent Running**:
   - Holds lock on serial port
   - Monitors for errors and crashes
   - Listens for SIGUSR1 signal

2. **Upload Agent Needs Port**:
   - Checks if port is locked
   - Sends SIGUSR1 to monitor agent's PID
   - Waits up to 30 seconds for port to be released

3. **Monitor Agent Receives Signal**:
   - Gracefully disconnects from serial port
   - Releases lock file
   - Exits cleanly (can be restarted after upload)

4. **Upload Completes**:
   - Uploads firmware
   - Releases lock
   - Monitor can be restarted

## Agent Commands

### Start Serial Monitor
```bash
python3 /home/bruce/ardupilot/esp32_monitor_agent.py --port /dev/ttyACM0
```

### Upload Firmware
```bash
python3 /home/bruce/ardupilot/esp32_upload_with_lock.py
```
The upload will automatically signal the monitor to disconnect if needed.

## Signal Flow

```
Monitor Agent                    Upload Agent
     |                                |
     |<-- SIGUSR1 (release request) --|
     |                                |
     |-- Graceful disconnect -------->|
     |-- Release lock --------------->|
     |                                |
     |                          Acquire lock
     |                          Upload firmware
     |                          Release lock
     |                                |
```

## No Manual Intervention Required

The system handles all coordination automatically:
- No need to manually stop the monitor
- No need to check if port is busy
- No need to wait and retry
- Agents coordinate seamlessly

## Troubleshooting

### Check Lock Status
```bash
ls -la /var/lock/LCK..tty*
cat /var/lock/LCK..ttyACM0
```

### Emergency Lock Release
```bash
rm /var/lock/LCK..ttyACM0
```

### View Running Agents
```bash
ps aux | grep esp32_monitor
ps aux | grep esp32_upload
```