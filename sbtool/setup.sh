#!/bin/sh
sudo adb wait-for-devices
adb root
sleep 3
adb push isp_app_p1_5_notrst /data
adb push prog_app.config /data