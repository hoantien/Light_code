#
# Test functionality: CMD_PENDING
#
# Send open sw global, then send command status command immediatly, no delay
echo 6 0x0001 0x00 0x00 0x01 0x00 0x00 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0000 0x24 0x00 0x01 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

#
# Test functionality: CMD_SUCCESS
#
# Send stream open command for cam a2, delay a little bit, then send command status command
echo 4 0x0002 0x00 0x10 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0000 0x24 0x00 0x02 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

#
# Test functionality: CMD_INVALID_ARG
#
# Send stream open command for cam a1 with only 1 byte in data field in stead of 3 bytes, delay a little bit,
# then send command status command
echo 6 0x0003 0x02 0x00 0x02 0x00 0x00 0x11 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 2
echo 4 0x0000 0x24 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

#
# Test functionality: CMD_UNKNOWN
#
# Send a fake command with command ID 0x0004 and 5 bytes following, delay a little bit, then send command
# status command
echo 7 0x0004 0x04 0x00 0x80 0x00 0x00 0x3C 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 2
echo 4 0x0000 0x24 0x00 0x04 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

#
# Test functionality: CMD_ERROR
#
echo 6 0x0005 0x00 0x00 0x02 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0006 0x02 0x00 0x02 0x00 0x00 0x11 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0000 0x24 0x00 0x06 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

#
# Test functionality: ERROR_INVALID_MBITMASK
#
# Send stream open command with bits in reserved field in bitmask field is set, delay a little bit,then send
# command status command
echo 6 0x0007 0x02 0x00 0x00 0x00 0xA6 0x11 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 2
echo 4 0x0000 0x24 0x00 0x07 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

#
# Test functionality: ERROR_ASIC_UNAVAILABLE
#

#
# Test functionality: ERROR_MODULE_FAULT
#

