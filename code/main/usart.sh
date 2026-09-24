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

# MOST BUILD MODES PRINT ONLY ONCE AT BOOT (the banner) and go quiet after --
# there is no periodic heartbeat unless the firmware is actively driving and
# emitting telemetry. Connecting a few seconds after a flash means that
# one-shot text is already gone: the screen will look identical to a dead
# USART. If nothing appears below, power-cycle or reset the board NOW, with
# this terminal already attached, or trigger a state change (press a button)
# rather than assuming the link is broken.
echo "If this looks blank: reset/power-cycle the board NOW while this is open,"
echo "or trigger something (e.g. press a button). Many modes print once at"
echo "boot and then stay silent until something happens."

# 3. Launch screen with macOS-compatible logging (-L)
screen -L "$SERIAL_PORT" "$BAUD"

# 4. Save and rename log file upon exit
if [ -f screenlog.0 ]; then
    mv screenlog.0 "$LOG_FILE"
    echo ""
    echo "=== Saved log to $LOG_FILE ==="
fi