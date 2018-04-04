#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera A1, A5, B2"
$CMD 0x08 0x00 0x00 0x00 0x00 0xA2 0x00 0x00 0x02
echo "Get mirror position A1, A5"
$CMD 0x07 0x00 0x01 0x44 0x00 0x22 0x00 0x00
$CMD 0x04 0x44 0x00 
$CMD cat
echo "Get mirror position A1, B2"
$CMD 0x07 0x00 0x02 0x44 0x00 0x82 0x00 0x00
$CMD 0x04 0x44 0x00 
$CMD cat
echo "Close camera A1, A5, B2"
$CMD 0x08 0x00 0x03 0x00 0x00 0xA2 0x00 0x00 0x00
