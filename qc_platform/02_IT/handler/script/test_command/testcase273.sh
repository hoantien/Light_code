#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera"
$CMD 0x08 0x00 0x00 0x00 0x00 0x00 0x80 0x00 0x02
echo "Set UCID preview"
$CMD 0x06 0x00 0x01 0x00 0x10 0x03 0x00
echo "Set sensitivity MAX"
$CMD 0x0D 0x00 0x02 0x30 0x00 0x00 0x80 0x00 0x03 0x00 0xFF 0xFF 0xFF 0xFF
echo "Close camera"
$CMD 0x08 0x00 0x03 0x00 0x00 0x00 0x80 0x00 0x00
