#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera"
$CMD 0x08 0x00 0x00 0x00 0x00 0x40 0x00 0x00 0x02
echo "Set UCID preview"
$CMD 0x06 0x00 0x01 0x00 0x10 0x03 0x00
echo "Set resolution"
$CMD 0x11 0x00 0x02 0x2C 0x00 0x40 0x00 0x00 0x03 0x00 0x80 0x07 0x00 0x00 0x38 0x04 0x00 0x00
echo "Stream on B1"
$CMD 0x0A 0x00 0x03 0x02 0x00 0x40 0x00 0x00 0x11 0x00 0x00
echo "Stream off B1"
$CMD 0x0A 0x00 0x04 0x02 0x00 0x40 0x00 0x00 0x10 0x00 0x00
echo "Close camera"
$CMD 0x08 0x00 0x05 0x00 0x00 0x40 0x00 0x00 0x00
