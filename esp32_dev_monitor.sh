#!/bin/bash

# ESP32 Development Monitor - Auto-upload and monitor on ELF changes
# Usage: ./esp32_dev_monitor.sh [board] [port] [--no-initial-upload]

BOARD=${1:-esp32lilygo_tconnect}
PORT=${2:-/dev/ttyACM0}
ELF_FILE="build/${BOARD}/bin/ardurover.elf"
BAUDRATE=115200

# Check for --no-initial-upload flag
SKIP_INITIAL=false
for arg in "$@"; do
    if [ "$arg" = "--no-initial-upload" ] || [ "$arg" = "-n" ]; then
        SKIP_INITIAL=true
    fi
done

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}ESP32 Development Monitor${NC}"
echo -e "Board: ${YELLOW}${BOARD}${NC}"
echo -e "Port: ${YELLOW}${PORT}${NC}"
echo -e "ELF: ${YELLOW}${ELF_FILE}${NC}"
echo ""

# Function to upload and monitor
upload_and_monitor() {
    echo -e "${YELLOW}[$(date '+%H:%M:%S')] Uploading to ESP32...${NC}"

    # Source ESP-IDF environment
    source modules/esp_idf/export.sh 2>/dev/null

    # Kill any existing monitor sessions
    pkill -f "serial.tools.miniterm.*${PORT}" 2>/dev/null || true
    pkill -f "esp_idf_monitor.*${PORT}" 2>/dev/null || true

    # Run upload
    if ./waf rover --upload 2>&1 | tee /tmp/esp32_upload.log | grep -E "(Writing|Wrote|Hash|Leaving|Hard resetting)"; then
        echo -e "${GREEN}[$(date '+%H:%M:%S')] Upload successful!${NC}"

        # Brief delay to let ESP32 start up
        sleep 0.5

        echo -e "${YELLOW}[$(date '+%H:%M:%S')] Starting IDF serial monitor...${NC}"
        echo -e "${YELLOW}Press Ctrl+C to stop current monitor session${NC}"
        echo -e "${YELLOW}Press Ctrl+C twice quickly to exit completely${NC}"
        echo ""

        # Run monitor in foreground with TTY attached
        python3 -m esp_idf_monitor \
            --port ${PORT} \
            --baud ${BAUDRATE} \
            --timestamps \
            ${ELF_FILE}

        echo -e "${YELLOW}[$(date '+%H:%M:%S')] Monitor stopped.${NC}"
    else
        echo -e "${RED}[$(date '+%H:%M:%S')] Upload failed! Check /tmp/esp32_upload.log${NC}"
        sleep 5
    fi
}

# Initial action - upload or just monitor
if [ "$SKIP_INITIAL" = true ]; then
    echo -e "${YELLOW}[$(date '+%H:%M:%S')] Skipping initial upload, starting monitor only...${NC}"
    echo -e "${YELLOW}Press Ctrl+C to stop current monitor session${NC}"
    echo ""

    # Just start monitoring without upload
    python3 -m esp_idf_monitor \
        --port ${PORT} \
        --baud ${BAUDRATE} \
        --timestamps \
        ${ELF_FILE}

    echo -e "${YELLOW}[$(date '+%H:%M:%S')] Monitor stopped.${NC}"
else
    # Do initial upload and monitor
    upload_and_monitor
fi

# Check if inotifywait is installed
if ! command -v inotifywait &> /dev/null; then
    echo -e "${RED}inotifywait not found! Install with: sudo apt-get install inotify-tools${NC}"
    echo -e "${YELLOW}Falling back to manual mode (press Ctrl+] to re-upload)${NC}"
    INOTIFY_AVAILABLE=false
else
    INOTIFY_AVAILABLE=true
fi

# PID tracking for monitor process
MONITOR_PID=""

# Function to start monitor in background
start_monitor_bg() {
    echo -e "${YELLOW}[$(date '+%H:%M:%S')] Starting IDF serial monitor...${NC}"
    echo -e "${YELLOW}Press Ctrl+] to stop monitor and trigger re-upload${NC}"
    echo -e "${YELLOW}Press Ctrl+C to exit completely${NC}"
    echo ""

    # Run monitor in background
    python3 -m esp_idf_monitor \
        --port ${PORT} \
        --baud ${BAUDRATE} \
        --timestamps \
        ${ELF_FILE} &

    MONITOR_PID=$!
}

# Set up trap to handle Ctrl+C gracefully - kill monitor and exit
cleanup() {
    echo -e "\n${YELLOW}Exiting ESP32 monitor...${NC}"
    if [ -n "$MONITOR_PID" ]; then
        kill -TERM $MONITOR_PID 2>/dev/null || true
    fi
    pkill -f "esp_idf_monitor.*${PORT}" 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# Watch for changes and re-upload
if [ "$INOTIFY_AVAILABLE" = true ]; then
    echo -e "${GREEN}[$(date '+%H:%M:%S')] Watching for changes to ${ELF_FILE}...${NC}"

    # Main loop with inotify watching
    while true; do
        # Check if monitor is still running or wait for file changes
        if [ -n "$MONITOR_PID" ] && kill -0 $MONITOR_PID 2>/dev/null; then
            # Monitor is running, wait for file change with timeout
            if inotifywait -t 2 -e modify,close_write "$ELF_FILE" 2>/dev/null; then
                echo -e "${GREEN}[$(date '+%H:%M:%S')] File changed! Triggering upload...${NC}"

                # Kill existing monitor
                if [ -n "$MONITOR_PID" ]; then
                    kill -TERM $MONITOR_PID 2>/dev/null || true
                    wait $MONITOR_PID 2>/dev/null || true
                    MONITOR_PID=""
                fi

                # Small delay to ensure file write is complete
                sleep 0.5

                # Upload and restart monitor
                upload_and_monitor
            fi
        else
            # Monitor has exited (user pressed Ctrl+])
            echo -e "${YELLOW}[$(date '+%H:%M:%S')] Monitor exited, re-uploading...${NC}"
            upload_and_monitor
        fi
    done
else
    # Fallback mode without inotify
    while true; do
        upload_and_monitor
        echo -e "${GREEN}[$(date '+%H:%M:%S')] Press Ctrl+] to re-upload or Ctrl+C to exit...${NC}"
        sleep 2
    done
fi