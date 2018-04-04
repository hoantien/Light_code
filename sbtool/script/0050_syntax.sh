# FPS values in hexadecimal
# 15 = 000F
# 24 = 0018
# 30 = 001E
# 60 = 003C
# Set fps 30 for cam a1, then read back to check value
echo 9 0x0000 0x50 0x00 0x02 0x00 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0001 0x50 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

# Set fps 30 for cam a2, then read back to check value
echo 9 0x0002 0x50 0x00 0x04 0x00 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0003 0x50 0x00 0x04 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam a3, then read back to check value
echo 9 0x0004 0x50 0x00 0x08 0x00 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0005 0x50 0x00 0x08 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam a4, then read back to check value
echo 9 0x0006 0x50 0x00 0x10 0x00 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0007 0x50 0x00 0x10 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam a5, then read back to check value
echo 9 0x0008 0x50 0x00 0x20 0x00 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0009 0x50 0x00 0x20 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam b1, then read back to check value
echo 9 0x000A 0x50 0x00 0x40 0x00 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x000B 0x50 0x00 0x40 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam b2, then read back to check value
echo 9 0x000C 0x50 0x00 0x80 0x00 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x000D 0x50 0x00 0x80 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam b3, then read back to check value
echo 9 0x000E 0x50 0x00 0x00 0x01 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x000F 0x50 0x00 0x00 0x01 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam b4, then read back to check value
echo 9 0x0010 0x50 0x00 0x00 0x02 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0020 0x50 0x00 0x00 0x02 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam b5, then read back to check value
echo 9 0x0030 0x50 0x00 0x00 0x04 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0040 0x50 0x00 0x00 0x04 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam c1, then read back to check value
echo 9 0x0050 0x50 0x00 0x00 0x08 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0060 0x50 0x00 0x00 0x08 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam c2, then read back to check value
echo 9 0x0070 0x50 0x00 0x00 0x10 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0080 0x50 0x00 0x00 0x10 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam c3, then read back to check value
echo 9 0x0090 0x50 0x00 0x00 0x20 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x00A0 0x50 0x00 0x00 0x20 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam c4, then read back to check value
echo 9 0x00B0 0x50 0x00 0x00 0x40 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x00C0 0x50 0x00 0x00 0x40 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam c5, then read back to check value
echo 9 0x00D0 0x50 0x00 0x00 0x80 0x00 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x00E0 0x50 0x00 0x00 0x80 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam c6, then read back to check value
echo 9 0x00F0 0x50 0x00 0x00 0x00 0x01 0x03 0x00 0x1E 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0100 0x50 0x00 0x00 0x00 0x01 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 15 for cam a1, a2, a3, then read back to check value
echo 13 0x0200 0x50 0x00 0x0E 0x00 0x00 0x03 0x00 0x0F 0x00 0x0F 0x00 0x0F 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0300 0x50 0x00 0x00 0x00 0x01 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 6 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 60 with global bit is set, then read back to check value
echo 9 0x0400 0x50 0x00 0x01 0x00 0x00 0x03 0x00 0x3C 0X00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0100 0x50 0x00 0xFE 0xFF 0x01 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 32 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Read fps with global bit is set, the expected result is "FF FF ..." because read mode doesn't support
# global bit
echo 7 0x0300 0x50 0x00 0x01 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 32 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam a1 with 1 byte in data field instead of 2, the expected value is "command not found" 
# or similarity
echo 8 0x0000 0x50 0x00 0x02 0x00 0x00 0x03 0x00 0x1E  > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0001 0x50 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set fps 30 for cam a1 with 3 byte in data field instead of 2, the expected value is "command not found"
# or similarity
echo 10 0x0000 0x50 0x00 0x02 0x00 0x00 0x03 0x00 0x1E 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0001 0x50 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set an unsupported fps for cam a1, the expected result is the value which was set before (60 in particular)
echo 9 0x0A00 0x50 0x00 0x02 0x00 0x00 0x03 0x00 0x11 0x11 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0C01 0x50 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 2 0x0050 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output
