#!/bin/sh
sudo adb wait-for-devices
adb root
sleep 3
adb shell "rm /data/fw.bin"
adb push bin/ /data/
adb shell "cd /data; ./isp_app_p2 -m program -i spi -f ASIC1.bin,ASIC2.bin,ASIC3.bin"