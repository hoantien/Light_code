#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open all camera"
$CMD 0x08 0x00 0x00 0x00 0x00 0x01 0x00 0x00 0x02
echo "Get module type of all cam"
$CMD 0x07 0x00 0x01 0x4C 0x00 0x01 0x00 0x00 
$CMD 0x01 0x4C 0x00 
$CMD cat
echo "Close all camera"
$CMD 0x08 0x00 0x03 0x00 0x00 0x01 0x00 0x00 0x00
