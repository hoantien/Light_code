#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera A1, A5, B2"
$CMD 0x08 0x00 0x00 0x00 0x00 0xA2 0x00 0x00 0x02
echo "Set UCID preview"
$CMD 0x06 0x00 0x01 0x00 0x10 0x03 0x00
echo "Get  fps A1, A5"
$CMD 0x09 0x00 0x02 0x50 0x00 0x22 0x00 0x00 0x03 0x00
$CMD 0x04 0x50 0x00
$CMD cat
echo "Get  fps A1, B2"
$CMD 0x09 0x00 0x03 0x50 0x00 0x82 0x00 0x00 0x03 0x00
$CMD 0x04 0x50 0x00
$CMD cat
echo "Close camera A1, A5, B2"
$CMD 0x08 0x00 0x04 0x00 0x00 0xA2 0x00 0x00 0x00
