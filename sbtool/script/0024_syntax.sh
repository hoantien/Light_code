# Send open hw for a1 camera, then send command status command immediatly, no delay
echo 6 0x0000 0x00 0x00 0x01 0x00 0x00 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0001 0x24 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

# Send command status with 1 byte in data field instead of 2 byte, expected result is "command not found"
# or similarity
echo 6 0x0000 0x00 0x00 0x02 0x00 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 3 0x0001 0x24 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send command status with 3 bytes in data field instead of 2 byte, expected result is "command not found"
# or similarity
echo 6 0x0000 0x00 0x00 0x02 0x00 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0001 0x24 0x00 0x00 0x00 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send command status with wrong transaction ID in data field, 0x0005 in stead of 0x0000 in particular, 
# expected result is "0x00 0x00 0x00 0x00" which means "CMD_UNKNOWN"
echo 6 0x0000 0x00 0x00 0x02 0x00 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0001 0x24 0x00 0x05 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send 3 commands for setting fps for cam a1, a2, a3 (60 fps) consecutively, the last setting fps command has 
# 3 bytes in data field. Checking that the command status command is return the status of the lattest command
echo 11 0x0000 0x50 0x00 0x02 0x00 0x00 0x03 0x00 0x3C 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 2
echo 11 0x0000 0x50 0x00 0x04 0x00 0x00 0x03 0x00 0x3C 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 2
echo 11 0x0000 0x50 0x00 0x08 0x00 0x00 0x03 0x00 0x3C 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 2
echo 4 0x0001 0x24 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 4 0x0024 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output


