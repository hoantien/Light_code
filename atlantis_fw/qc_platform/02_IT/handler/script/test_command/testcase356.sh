#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera A1, A5 in HW_Standby mode"
$CMD 0x08 0x00 0x00 0x00 0x00 0x22 0x00 0x00 0x01
echo "Open camera A1, B2 in HW_Standby mode"
$CMD 0x08 0x00 0x01 0x00 0x00 0x82 0x00 0x00 0x01
echo "Close camera A1, A5, B2"
$CMD 0x08 0x00 0x02 0x00 0x00 0xA2 0x00 0x00 0x00
