#!/bin/bash

BAUD=${1:-38400}
LOG_FILE="usart_$(date +%Y%m%d_%H%M%S).log"

# 1. Clear stuck screen sessions
killall screen 2>/dev/null

# 2. Locate PL2303 / usbserial port
SERIAL_PORT=$(ls /dev/cu.usbserial-* /dev/cu.PL2303-* 2>/dev/null | head -n 1)

if [ -z "$SERIAL_PORT" ]; then
    echo "Error: No USB-to-Serial adapter found in /dev/cu.*"
    exit 1
fi

echo "Found port: $SERIAL_PORT"
echo "Logging output to: $LOG_FILE"
echo "Opening terminal at $BAUD baud..."
sleep 1

# Remove any old screenlog.0 file to avoid appending to previous sessions
rm -f screenlog.0

# 3. Launch screen with macOS-compatible logging (-L)
screen -L "$SERIAL_PORT" "$BAUD"

# 4. Save and rename log file upon exit
if [ -f screenlog.0 ]; then
    mv screenlog.0 "$LOG_FILE"
    echo ""
    echo "=== Saved log to $LOG_FILE ==="
fi