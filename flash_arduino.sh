#!/bin/bash
file=$1
if [ -z "$file" ]; then
  echo "Usage: ./flash_arduino <hexFile>"
  exit 1
fi
avrdude -patmega2560 -cwiring -P/dev/ttyUSB0 -b115200 -D "-Uflash:w:${file}:i"
