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

# Disable hardware flow control (RTS/CTS). This is a real, well-known cause of
# "port opens fine, nothing ever shows up": with a bare 3-wire TX/RX/GND hookup
# (no CTS/RTS wired), some drivers still honor flow control by default and
# simply hold everything back with no error. -f is macOS stty; adjust to -F on
# Linux if this script is ever run there.
stty -f "$SERIAL_PORT" "$BAUD" cs8 -parenb -cstopb -crtscts 2>/dev/null

sleep 1

# Remove any old screenlog.0 file to avoid appending to previous sessions
rm -f screenlog.0

# The start-up banner (reset cause, supply voltage, gyro offsets, which run is
# armed) prints ONCE at boot. Connecting a few seconds after a flash means it
# has already gone by; the telemetry CSV lines that follow every 100 ms are
# still visible, so a completely blank screen means no bytes are arriving at
# all -- check TX->RX crossover and the shared GND. To see the banner, reset
# the board with this terminal already open.
echo "The boot banner prints once. To see it, reset the board NOW while this"
echo "is open. CSV lines every 100 ms follow; a totally blank screen means no"
echo "bytes are arriving -- check TX->RX crossover and the shared GND."

# 3. Launch screen with macOS-compatible logging (-L)
screen -L "$SERIAL_PORT" "$BAUD"

# 4. Save and rename log file upon exit
if [ -f screenlog.0 ]; then
    mv screenlog.0 "$LOG_FILE"
    echo ""
    echo "=== Saved log to $LOG_FILE ==="
fi