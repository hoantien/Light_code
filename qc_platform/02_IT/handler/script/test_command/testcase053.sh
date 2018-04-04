#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Set log INFO enable"
$CMD 0x05 0x00 0x01 0x19 0x02 0x88
