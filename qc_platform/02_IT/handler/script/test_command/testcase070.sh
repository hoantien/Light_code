#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Set power status IDLE/Standby to CCB"
$CMD 0x06 0x00 0x01 0x7A 0x02 0x03 0x00
