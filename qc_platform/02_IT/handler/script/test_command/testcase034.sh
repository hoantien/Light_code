#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera"
$CMD 0x08 0x00 0x00 0x00 0x00 0x02 0x00 0x00 0x02
echo "Set mirror position AVERAGE"
$CMD 0x09 0x00 0x01 0x44 0x00 0x02 0x00 0x00 0xFF 0x7F
echo "Close camera"
$CMD 0x08 0x00 0x03 0x00 0x00 0x02 0x00 0x00 0x00
