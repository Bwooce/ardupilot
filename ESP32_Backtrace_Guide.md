# ESP32 Stack Protection Backtrace Guide

## What You Get When Stack Protection Triggers

### 1. Console Output (Serial/USB at 115200 baud)
When stack overflow is detected, you'll see:
```
Debug exception reason: Stack canary watchpoint triggered (task_name)

Core  0 register dump:
PC      : 0x400e14ed  PS      : 0x00060033  A0      : 0x800d0802  A1      : 0x3ffb5030  
A2      : 0x3ffb5180  A3      : 0x3ffb5050  A4      : 0x00000001  A5      : 0x00000001  
...

Backtrace: 0x400e14ed:0x3ffb5030 0x400d0802:0x3ffb5050 0x400d0856:0x3ffb5070
```

Or for compiler stack protection:
```
***ERROR*** A stack overflow in task main_task has been detected.
abort() was called at PC 0x40083805 on core 0
Stack smashing protect failure!

Backtrace: 0x40083805:0x3ffb5030 0x400e14ed:0x3ffb5050 ...
```

### 2. Over MAVLink
You'll see a STATUSTEXT message after reboot:
```
ESP32 PANIC: Stack overflow or exception (3)
```

### 3. Capturing the Data

#### Option A: Direct Serial Connection
```bash
# Connect to ESP32 serial port
screen /dev/cu.usbmodem* 115200

# Or with minicom
minicom -D /dev/cu.usbmodem* -b 115200

# Copy the entire panic output when it occurs
```

#### Option B: Via MAVProxy Console
```bash
# The panic output appears in MAVProxy console
# Copy the Backtrace line
```

## Decoding the Backtrace

### Using the Provided Script
```bash
# Method 1: Pass backtrace directly
./decode_esp32_backtrace.sh "0x400e14ed:0x3ffb5030 0x400d0802:0x3ffb5050"

# Method 2: Paste full panic output
./decode_esp32_backtrace.sh
# Then paste the panic output and press Ctrl+D

# Output shows function names and line numbers:
0x400e14ed: send_vibration() at GCS_Common.cpp:3201
0x400d0802: GCS_MAVLINK::try_send_message() at GCS_Common.cpp:1523
```

### Manual Decoding
```bash
# Set the path to your ELF file
ELF=build/esp32lilygo_tconnect/esp-idf_build/ardupilot.elf

# Use addr2line
xtensa-esp32s3-elf-addr2line -pfiaC -e $ELF 0x400e14ed 0x400d0802

# Or use the full path
/Users/bruce/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32s3-elf-addr2line \
  -pfiaC -e build/esp32lilygo_tconnect/esp-idf_build/ardupilot.elf \
  0x400e14ed 0x400d0802
```

## Understanding the Results

### Example Decoded Backtrace
```
0x400e14ed: send_vibration() at libraries/GCS_MAVLink/GCS_Common.cpp:3201
0x400d0802: try_send_message(msgid=241) at libraries/GCS_MAVLink/GCS_Common.cpp:1523  
0x400d0856: update() at libraries/GCS_MAVLink/GCS_Common.cpp:1600
0x400d1234: main_loop() at Rover/Rover.cpp:456
```

This tells you:
1. **Stack overflow happened in `send_vibration()`** at line 3201
2. It was called by `try_send_message()` sending message ID 241 (VIBRATION)
3. The call chain shows how we got there

### Common Causes to Check

1. **Large Local Arrays**
   ```cpp
   void send_vibration() {
       char buffer[1024];  // <-- Too big for stack!
   ```

2. **Recursive Functions**
   ```cpp
   void process_data(int depth) {
       process_data(depth + 1);  // <-- No depth limit!
   ```

3. **Stack-Allocated Messages**
   ```cpp
   mavlink_message_t msg;  // <-- Consider using static or heap
   ```

4. **String Operations Without Bounds**
   ```cpp
   char buf[32];
   sprintf(buf, "%s %s %s", long_str1, long_str2, long_str3);  // <-- Overflow!
   ```

## Tips for Finding the Problem

1. **The last address before `__stack_chk_fail`** is usually the function with the overflow

2. **Check the task name** - it tells you which thread had the problem:
   - `main_task` - Main ArduPilot loop
   - `io_thread` - I/O operations
   - `wifi` - WiFi operations
   - `can` - CAN bus operations

3. **Look at local variable sizes** in the flagged function

4. **Add debug prints** before suspected functions:
   ```cpp
   printf("Stack free: %d\n", uxTaskGetStackHighWaterMark(NULL));
   ```

5. **Increase stack size** temporarily to confirm it's a stack issue:
   - Edit task creation in Scheduler.cpp
   - Increase by 2x to test

## Preventing Stack Overflows

1. **Move large buffers to heap**:
   ```cpp
   // Instead of: char buffer[1024];
   char *buffer = (char*)malloc(1024);
   // ... use buffer ...
   free(buffer);
   ```

2. **Use static for large temporary buffers**:
   ```cpp
   static char buffer[1024];  // Not on stack
   ```

3. **Add recursion limits**:
   ```cpp
   if (depth > MAX_DEPTH) return;
   ```

4. **Check stack usage**:
   ```cpp
   size_t free_stack = uxTaskGetStackHighWaterMark(NULL);
   if (free_stack < 512) {
       // Warning: Low stack!
   }
   ```

## Remote Debugging

Since the full backtrace only appears on console, for remote debugging:

1. **Enable logging to SD card** if possible
2. **Capture serial output** during testing
3. **The MAVLink message** tells you a panic occurred
4. **Physical access** is needed for full backtrace

The backtrace is your best tool for finding exactly where the overflow occurs!