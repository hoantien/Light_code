#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Get Major and Minor version of protocol ASIC "
$CMD 0x04 0x00 0x01 0x03 0x02
$CMD 0x02 0x03 0x02
$CMD cat
