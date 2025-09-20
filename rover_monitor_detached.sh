#!/bin/bash
# Fully detached rover monitor that won't freeze Claude TUI

PORT=${1:-/dev/ttyACM1}
LOG_FILE="/tmp/rover_monitor.log"
PID_FILE="/tmp/rover_monitor.pid"

# Kill any existing monitor
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    kill $OLD_PID 2>/dev/null
    rm "$PID_FILE"
fi

# Start monitor completely detached from terminal
nohup python3 /home/bruce/ardupilot/esp32_monitor_agent.py \
    --port "$PORT" \
    --interval 30 \
    > "$LOG_FILE" 2>&1 &

MONITOR_PID=$!
echo $MONITOR_PID > "$PID_FILE"

echo "✓ Monitor started in fully detached mode (PID $MONITOR_PID)"
echo "  View output: tail -f $LOG_FILE"
echo "  Stop monitor: kill $MONITOR_PID"
echo ""
echo "This will NOT freeze your terminal - it's completely detached."