#!/bin/bash

# Quick ESP32 monitor-only script (no upload)
# This is a shortcut for: ./esp32_dev_monitor.sh [board] [port] --no-initial-upload

BOARD=${1:-esp32lilygo_tconnect}
PORT=${2:-/dev/ttyACM0}

exec ./esp32_dev_monitor.sh "$BOARD" "$PORT" --no-initial-upload