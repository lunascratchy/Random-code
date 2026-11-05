#!/bin/bash
# Flash a compiled .hex onto the Auracle Nano with avrdude.
#   ./flash_nano.sh firmware.hex            (uses /dev/arduino, new bootloader 115200)
#   ./flash_nano.sh firmware.hex 57600      (old-bootloader Nano clones)
# Stop ROS first: the serial port can only have one user.
set -e
HEX="$1"
BAUD="${2:-115200}"
PORT="${PORT:-/dev/arduino}"

if [ -z "$HEX" ] || [ ! -f "$HEX" ]; then
  echo "Usage: $0 <firmware.hex> [baud]"; exit 1
fi
if fuser "$PORT" >/dev/null 2>&1; then
  echo "$PORT is in use (is ROS still running?). Stop it and retry."; exit 1
fi

avrdude -p atmega328p -c arduino -P "$PORT" -b "$BAUD" -D -U "flash:w:$HEX:i"
