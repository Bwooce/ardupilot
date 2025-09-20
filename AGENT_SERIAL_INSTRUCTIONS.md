# Serial Port Coordination for ArduPilot Agents

## IMPORTANT: Always use the lock-enabled scripts!

### For Serial Monitoring (ardupilot-serial-monitor agent):
```bash
# Start monitoring with automatic lock management:
python3 /home/bruce/ardupilot/esp32_monitor_agent.py --port /dev/ttyACM0

# The script will:
# - Acquire lock at /var/lock/LCK..ttyACM0
# - Wait if port is busy
# - Show who has the lock
# - Release on exit (Ctrl+C)
```

### For Firmware Upload (ardupilot-rover-builder agent):
```bash
# Upload with automatic lock management:
python3 /home/bruce/ardupilot/esp32_upload_with_lock.py

# Or specify port explicitly:
python3 /home/bruce/ardupilot/esp32_upload_with_lock.py --port /dev/ttyACM0

# The script will:
# - Wait for serial monitor to release port
# - Upload firmware using waf
# - Release lock when done
```

### Check Lock Status:
```bash
# See if port is locked:
ls -la /var/lock/LCK..tty*

# Check who has lock:
cat /var/lock/LCK..ttyACM0
# First line: PID
# Second line: Tool name
```

### Emergency Release:
```bash
# If a process dies without releasing lock:
rm /var/lock/LCK..ttyACM0
```

## DO NOT USE:
- Raw `serial.Serial()` connections
- Direct `./waf rover --upload` commands
- `esptool.py` directly
- Any other serial monitoring scripts

## Lock File Format:
```
/var/lock/LCK..ttyACM0
Line 1: Process ID
Line 2: Tool name (e.g., "esp32_monitor_agent", "esp32_upload")
```

The lock system prevents:
- Upload failures due to busy port
- Monitor crashes during upload
- Data corruption from simultaneous access
- Claude TUI lockups from serial conflicts