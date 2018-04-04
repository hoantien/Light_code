#!/bin/bash
 while true
 do
	python open_camera_asb.py -on g
	echo "python open_camera_asb.py -on g"
	adb shell "echo 4 0x0000 0x00 0x10 0x03 0x00  > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w"
	echo "adb shell \"echo 4 0x0000 0x00 0x10 0x03 0x00  > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w\""
	python camera_script_asb.py -c c5 -s on -csi 0
	echo "	python camera_script_asb.py -c c5 -s on -csi 0"
	adb shell "echo 6 0x0000 0x00 0x00 0x01 0x00 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w"
	echo "adb shell \"echo 6 0x0000 0x00 0x00 0x01 0x00 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w\""
done	