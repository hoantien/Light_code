#
# Test functionality: MODULE_POWER_ON, MODULE_HW_STANDBY, MODULE_SW_STANDBY
#
# Open sw for cam a1, a2, b1, b2, c1, c2, open hw for cam a3, b3, c3, close the others
echo 11 0x0000 0x00 0x00 0xC6 0x18 0x00 0x02 0x02 0x02 0x02 0x02 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0000 0x00 0x00 0x08 0x21 0x00 0x01 0x01 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 12 0x0000 0x00 0x00 0x30 0xC6 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w

# Send module status command for cam a1
echo 5 0x0000 0x28 0x00 0x02 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

# Send module status command for cam a5
echo 5 0x0000 0x28 0x00 0x20 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command for cam a2, b2
echo 5 0x0000 0x28 0x00 0x84 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command for cam c1, c2
echo 5 0x0000 0x28 0x00 0x00 0x18 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command for cam b3, b4, c6
echo 5 0x0000 0x28 0x00 0x00 0x03 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 12 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command with global bit is set
echo 5 0x0000 0x28 0x00 0x01 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 32 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

#
# Test functionality: MODULE_CLOCK_ON
#

#
# Test functionality: MODULE_STREAM_ON
#
# Stream on cam a1, a2, a3, b1, b2, b3, c1, c2, c3, stream off others
echo 32 0x0000 0x02 0x00 0xCE 0x39 0x00 0x11 0x00 0x00 0x11 0x00 0x00 0x11 0x00 0x00 \
									 	0x11 0x00 0x00 0x11 0x00 0x00 0x11 0x00 0x00 \
									 	0x11 0x00 0x00 0x11 0x00 0x00 0x11 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 26 0x0000 0x02 0x00 0x30 0xC6 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 \
										0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 \
										0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
# Send module status command for cam a1
echo 5 0x0000 0x28 0x00 0x02 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command for cam a5
echo 5 0x0000 0x28 0x00 0x20 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command for cam a2, b2
echo 5 0x0000 0x28 0x00 0x84 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command for cam c1, c2
echo 5 0x0000 0x28 0x00 0x00 0x18 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 8 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command for cam b3, b4, c6
echo 5 0x0000 0x28 0x00 0x00 0x03 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 12 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

# Send module status command with global bit is set
echo 5 0x0000 0x28 0x00 0x01 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 32 0x0028 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

#
# Test functionality: ERROR_SENSOR_I2C_DETECT_FAILURE
#

#
# Test functionality: ERROR_I2C_READ_FAILURE
#

#
# Test functionality: ERROR_I2C_WRITE_FAILURE
#

#
# Test functionality: MODULE_HALL_I2C_DETECT_FAILURE
#

#
# Test functionality: ERROR_MIRROR_DETECT
#

#
# Test functionality: ERROR_LENS_DETECT
#

#
# Test functionality: MOVE_LENS_FAILURE
#

#
# Test functionality: MOVE_MIRROR_FAILURE
#

#
# Test functionality: UCID_FAILURE
#

