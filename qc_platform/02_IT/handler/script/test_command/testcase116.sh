#!/bin/bash
CMD="sudo ../../../../../FTDI/FTDI4222/Linux/x86_64/FT4222_FLASH cmd 0x08"
echo "Open camera"
$CMD 0x08 0x00 0x00 0x00 0x00 0x20 0x00 0x00 0x02
echo "Write EEPROM offset MAX"
$CMD 0x0C 0x00 0x01 0x72 0x02 0x20 0x00 0x00 0x02 0x00 0xFF 0x00 0x04
sleep 1
$CMD 0x08 0x00 0x02 0x72 0x02 0x20 0x00 0x00 0x03
sleep 1
$CMD 0x03 0x72 0x02
sleep 1
$CMD cat
sleep 1
$CMD 0x0C 0x00 0x03 0x72 0x02 0x20 0x00 0x00 0x05 0xFF 0xFF 0xFF 0xFF
echo "Close camera"
$CMD 0x08 0x00 0x04 0x00 0x00 0x20 0x00 0x00 0x00
