#!/bin/sh
FWFILE="fw.bin"
if [ "$#" -ge 1 ]
then
	FWFILE=$1
fi
sudo adb wait-for-devices
adb root
sleep 3
adb shell "rm /data/fw.bin"
adb push ${FWFILE} /data/fw.bin
adb shell "chmod +x /data/isp_app_p1_5_notrst"
adb shell "cd /data; ./isp_app_p1_5_notrst -m program -i spi -f fw.bin,fw.bin,fw.bin"
