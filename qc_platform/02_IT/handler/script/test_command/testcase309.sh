#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open all camera in HW_Standby mode"
$CMD 0x08 0x00 0x00 0x00 0x00 0x01 0x00 0x00 0x01
echo "Close all camera"
$CMD 0x08 0x00 0x01 0x00 0x00 0x01 0x00 0x00 0x00
