#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera"
$CMD 0x08 0x00 0x00 0x00 0x00 0x02 0x00 0x00 0x02
echo "Set UCID preview"
$CMD 0x06 0x00 0x01 0x00 0x10 0x03 0x00
echo "Set resolution"
$CMD 0x11 0x00 0x02 0x2C 0x00 0x02 0x00 0x00 0x03 0x00 0x80 0x07 0x00 0x00 0x38 0x04 0x00 0x00
echo "Set sensitivity"
$CMD 0x0D 0x00 0x03 0x30 0x00 0x02 0x00 0x00 0x03 0x00 0x01 0x10 0x00 0x00
echo "Set exposure time"
$CMD 0x11 0x00 0x04 0x32 0x00 0x02 0x00 0x00 0x03 0x00 0x40 0x42 0x0f 0x00 0x00 0x00 0x00 0x00
echo "Set fps"
$CMD 0x0B 0x00 0x05 0x50 0x00 0x02 0x00 0x00 0x03 0x00 0x1E 0x00
echo "Stream on A1"
$CMD 0x0A 0x00 0x06 0x02 0x00 0x02 0x00 0x00 0x11 0x00 0x00
echo "Stream off A1"
$CMD 0x0A 0x00 0x07 0x02 0x00 0x02 0x00 0x00 0x10 0x00 0x00
echo "Close camera"
$CMD 0x08 0x00 0x08 0x00 0x00 0x02 0x00 0x00 0x00
