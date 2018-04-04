# Exposure values in hexadecimal 
# SECOND * 4 	 = 4000000000ns: 00000000EE6B2800
# SECOND * 2 	 = 2000000000ns: 0000000077359400
# SECOND     	 = 1000000000ns: 000000003B9ACA00
# SECOND / 2 	 = 500000000ns : 000000001DCD6500
# SECOND / 4 	 = 250000000ns : 000000000EE6B280
# SECOND / 8 	 = 125000000ns : 0000000007735940 
# SECOND / 15  	 = 66666667ns  : 0000000003F940AB
# SECOND / 30  	 = 33333333ns  : 0000000001FCA055
# SECOND / 60  	 = 16666667ns  : 0000000000FE502B
# SECOND / 125 	 = 8000000ns   : 00000000007A1200
# SECOND / 250 	 = 4000000ns   : 00000000003D0900
# SECOND / 500   = 2000000ns   : 00000000001E8480
# SECOND / 1000  = 1000000ns   : 00000000000F4240
# SECOND / 2000  = 500000ns    : 000000000007A120
# SECOND / 4000  = 250000ns    : 000000000003D090
# SECOND / 8000  = 125000ns    : 000000000001E848
# SECOND / 16000 = 62500ns	   : 000000000000F424
# SECOND / 32000 = 31250ns	   : 0000000000007A12

# Set exposure time to 1/1000 for cam a1, then read back to check written value
echo 15 0x0000 0x32 0x00 0x02 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0001 0x32 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

# Set exposure time to 1/1000 for cam a2, then read back to check written value
echo 15 0x0002 0x32 0x00 0x04 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0003 0x32 0x00 0x04 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam a3, then read back to check written value
echo 15 0x0004 0x32 0x00 0x08 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0005 0x32 0x00 0x08 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam a4, then read back to check written value
echo 15 0x0006 0x32 0x00 0x10 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0007 0x32 0x00 0x10 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam a5, then read back to check written value
echo 15 0x0008 0x32 0x00 0x20 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0009 0x32 0x00 0x20 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam b1, then read back to check written value
echo 15 0x000A 0x32 0x00 0x40 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x000B 0x32 0x00 0x40 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam b2, then read back to check written value
echo 15 0x000C 0x32 0x00 0x80 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x000D 0x32 0x00 0x80 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam b3, then read back to check written value
echo 15 0x000E 0x32 0x00 0x00 0x01 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x000F 0x32 0x00 0x00 0x01 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam b4, then read back to check written value
echo 15 0x0010 0x32 0x00 0x00 0x02 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0020 0x32 0x00 0x00 0x02 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam b5, then read back to check written value
echo 15 0x0030 0x32 0x00 0x00 0x04 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0040 0x32 0x00 0x00 0x04 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam c1, then read back to check written value
echo 15 0x0050 0x32 0x00 0x00 0x08 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0060 0x32 0x00 0x00 0x08 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam c2, then read back to check written value
echo 15 0x0070 0x32 0x00 0x00 0x10 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0080 0x32 0x00 0x00 0x10 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam c3, then read back to check written value
echo 15 0x0090 0x32 0x00 0x00 0x20 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x00A0 0x32 0x00 0x00 0x20 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam c4, then read back to check written value
echo 15 0x00B0 0x32 0x00 0x00 0x40 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x00C0 0x32 0x00 0x00 0x40 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam c5, then read back to check written value
echo 15 0x00D0 0x32 0x00 0x00 0x80 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x00E0 0x32 0x00 0x00 0x80 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/1000 for cam c6, then read back to check written value
echo 15 0x00F0 0x32 0x00 0x00 0x00 0x01 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0100 0x32 0x00 0x00 0x00 0x01 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 4 for cam a1, a2, a3, then read back to check written value
echo 31 0x0200 0x32 0x00 0x0E 0x00 0x00 0x03 0x00 0x00 0x28 0x6B 0xEE 0x00 0x00 0x00 0x00 0x00 0x28 0x6B 0xEE 0x00 0x00 0x00 0x00 0x00 0x28 0x6B 0xEE 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0300 0x32 0x00 0x0E 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 24 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time to 1/2 with global bit is set, then read back to check written value
echo 15 0x0400 0x32 0x00 0x01 0x00 0x00 0x03 0x00 0x00 0x65 0xCD 0x1D 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
#echo 7 0x0500 0x32 0x00 0xFE 0xFF 0x01 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
#sleep 1
#echo 128 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
#cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set an unsupported exposure time to cam a1, then read back to check written value, the expected result is
# the value which was set before (1/2 in particular)
echo 15 0x6000 0x32 0x00 0x02 0x00 0x00 0x03 0x00 0x01 0x01 0x01 0x01 0x01 0x01 0x01 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0700 0x32 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time for cam a1 with 4 bytes in data field instead of 8 bytes, the expected value in "command not 
# found" or similar
echo 11 0x8000 0x32 0x00 0x02 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0900 0x32 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Set exposure time for cam a1 with 10 bytes in data field instead of 8 bytes, the expected value in "command not 
# found" or similar
echo 17 0x0A00 0x32 0x00 0x02 0x00 0x00 0x03 0x00 0x40 0x42 0x0F 0x00 0x00 0x00 0x00 0x00 0x01 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 7 0x0B00 0x32 0x00 0x02 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
sleep 1
echo 8 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Get exposure time with global bit is set, the expected result is "FF FF FF ...." because the command doesn't 
# support global bit for reading
#echo 7 0x0500 0x32 0x00 0x01 0x00 0x00 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
#echo 128 0x0032 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
#cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output
