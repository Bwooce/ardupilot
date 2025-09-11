# ESP32 Stack Protection in Debug Builds

When building for ESP32 with `./waf --debug`, additional stack protection and memory debugging features are now automatically enabled to help catch corruption issues.

## Enabled Protections

### 1. Compiler Stack Protection (`CONFIG_COMPILER_STACK_CHECK_MODE_STRONG`)
- Enables GCC's `-fstack-protector-strong` flag
- Adds canary values to detect stack buffer overflows
- When triggered: `Stack smashing protect failure!`

### 2. FreeRTOS Stack Overflow Detection (`CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY`)
- Places canary bytes at the end of each task's stack
- Checked on every context switch
- When triggered: `Debug exception reason: Stack canary watchpoint triggered (task_name)`

### 3. Hardware Watchpoint (`CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK`)
- Sets hardware watchpoint on last 32 bytes of stack
- Triggers before canary corruption
- More reliable than software-only detection

### 4. Heap Protection (`CONFIG_HEAP_POISONING_COMPREHENSIVE`)
- Fills freed memory with pattern to detect use-after-free
- Adds guard bytes around allocations to detect overflows
- Performance impact but catches many memory corruption bugs

### 5. Hardware Stack Guard (`CONFIG_ESP_SYSTEM_HW_STACK_GUARD`)
- ESP32-S3 specific hardware protection
- Triggers exception on stack overflow

## How to Use

1. Build with debug flag:
   ```bash
   ./waf configure --board esp32lilygo_tconnect --debug
   ./waf rover
   ```

2. Flash and monitor:
   ```bash
   ./waf flash
   ```

3. If stack corruption occurs, you'll see detailed panic output with:
   - Task name where corruption occurred
   - Full backtrace
   - Memory dump around corruption point

## Debugging Stack Issues

When you get a stack overflow/corruption error:

1. **Check the backtrace** - Shows exactly where overflow occurred
2. **Increase stack size** - If legitimate usage, increase task stack
3. **Check for large local arrays** - Move to heap if needed
4. **Look for recursive functions** - Add depth limits
5. **Check for buffer overruns** - Especially with string operations

## Performance Impact

These protections do have performance overhead:
- ~5-10% CPU overhead from stack checking
- ~20-30% memory overhead from heap poisoning
- Slightly larger binary size

Only use `--debug` builds for testing and debugging, not for production flights.

## Intermittent Corruption

The VIBRATION message corruption (ID 241 → 0x2F7C00) is likely:
- **Stack corruption**: Local buffer being overwritten
- **Race condition**: Multiple tasks accessing same memory
- **DMA issue**: Hardware writing to wrong memory

With these protections enabled, if it's stack corruption, you should get a clear panic message pointing to the exact location.