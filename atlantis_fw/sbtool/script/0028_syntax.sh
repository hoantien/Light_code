# Check syntax for module status command for cam a1
echo 5 0x0000 0x28 0x00 0x02 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

# Check syntax for module status command for cam a2
echo 5 0x0000 0x28 0x00 0x04 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam a3
echo 5 0x0000 0x28 0x00 0x08 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam a4
echo 5 0x0000 0x28 0x00 0x10 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam a5
echo 5 0x0000 0x28 0x00 0x20 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam b1
echo 5 0x0000 0x28 0x00 0x40 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam b2
echo 5 0x0000 0x28 0x00 0x80 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam b3
echo 5 0x0000 0x28 0x00 0x00 0x01 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam b4
echo 5 0x0000 0x28 0x00 0x00 0x02 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam b5
echo 5 0x0000 0x28 0x00 0x00 0x04 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam c1
echo 5 0x0000 0x28 0x00 0x00 0x08 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam c2
echo 5 0x0000 0x28 0x00 0x00 0x10 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam c3
echo 5 0x0000 0x28 0x00 0x00 0x20 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam c4
echo 5 0x0000 0x28 0x00 0x00 0x40 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam c5
echo 5 0x0000 0x28 0x00 0x00 0x80 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam c6
echo 5 0x0000 0x28 0x00 0x00 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command with global bit is set
echo 5 0x0000 0x28 0x00 0x01 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 32 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam a1, b1
echo 5 0x0000 0x28 0x00 0x42 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Check syntax for module status command for cam b3, c6
echo 5 0x0000 0x28 0x00 0x00 0x01 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module command status with 2 bytes in bitmask field instead of 3 bytes, expected result is "command not 
# found" or similarity
echo 4 0x0000 0x28 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module command status with 4 bytes in bitmask field instead of 3 bytes, expected result is "command not 
# found" or similarity
echo 6 0x0000 0x28 0x00 0x00 0x00 0x01 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module command status with bits in reserved field in bitmask field is set
echo 5 0x0000 0x28 0x00 0x00 0x00 0xDE > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output









