#!/bin/bash
# ESP32 Backtrace Decoder for ArduPilot
# 
# Usage: ./decode_esp32_backtrace.sh "0x400e14ed:0x3ffb5030 0x400d0802:0x3ffb5050 ..."
# Or paste the full panic output and it will extract the backtrace

# Configuration
ELF_FILE="${ELF_FILE:-build/esp32lilygo_tconnect/esp-idf_build/ardupilot.elf}"
ADDR2LINE="${ADDR2LINE:-/Users/bruce/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32s3-elf-addr2line}"

# Check if ELF file exists
if [ ! -f "$ELF_FILE" ]; then
    echo "Error: ELF file not found at $ELF_FILE"
    echo "Please set ELF_FILE environment variable to the correct path"
    exit 1
fi

# Check if addr2line exists
if [ ! -x "$ADDR2LINE" ]; then
    echo "Error: addr2line not found at $ADDR2LINE"
    echo "Please set ADDR2LINE environment variable to the correct path"
    exit 1
fi

echo "ESP32 Backtrace Decoder"
echo "========================"
echo "ELF file: $ELF_FILE"
echo "addr2line: $ADDR2LINE"
echo ""

# Extract addresses from input
# Handles both formats: "0x400e14ed:0x3ffb5030" and just "0x400e14ed"
if [ -z "$1" ]; then
    echo "Reading from stdin (paste panic output and press Ctrl+D when done):"
    INPUT=$(cat)
else
    INPUT="$@"
fi

# Extract backtrace line if present
if echo "$INPUT" | grep -q "Backtrace:"; then
    BACKTRACE=$(echo "$INPUT" | grep "Backtrace:" | sed 's/.*Backtrace://')
else
    BACKTRACE="$INPUT"
fi

# Extract addresses (format: 0x400e14ed:0x3ffb5030 or just 0x400e14ed)
ADDRESSES=$(echo "$BACKTRACE" | grep -oE '0x[0-9a-fA-F]+:0x[0-9a-fA-F]+|0x[0-9a-fA-F]+' | cut -d: -f1)

if [ -z "$ADDRESSES" ]; then
    echo "No addresses found in input"
    echo "Expected format: Backtrace: 0x400e14ed:0x3ffb5030 0x400d0802:0x3ffb5050"
    exit 1
fi

echo "Decoded backtrace:"
echo "------------------"

for addr in $ADDRESSES; do
    # Use addr2line to get function name and source location
    result=$($ADDR2LINE -pfiaC -e "$ELF_FILE" "$addr")
    echo "$addr: $result"
done

echo ""
echo "Stack Protection Errors:"
echo "------------------------"
echo "If you see 'Stack canary watchpoint triggered', the overflow happened in that task"
echo "If you see 'Stack smashing protect failure', check the last function before __stack_chk_fail"
echo ""
echo "Tips:"
echo "- The last good address before corruption is usually the culprit"
echo "- Look for functions with large local arrays or recursive calls"
echo "- Check for buffer overruns in string operations"