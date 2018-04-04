#!/bin/bash
# CHANGE THIS IF YOU WANT
s1cername=light_cert_0630
s1cerpass=12345678
s2cername=light_cert_0630
s2cerpass=12345678

# THESE LINES SHOULD BE CHANGED EXCEPT YOU REALLY WANT TO
if [ "$#" -lt 1 ]; then
	echo "$(tput bold)$(tput setaf 1)Syntax: $0 <atlantis-fw path>/Output $(tput sgr0)"
	exit
fi
rm -rf tmp > /dev/null
mkdir -p tmp

ts=`date +%F-%I-%M-%S`

for var in 1 2 3
do
	echo "Signing $(tput bold)$(tput setaf 4) $1 $(tput sgr0)"
	./sbtool phase0bimghdr $1/AsicFwS1.bin tmp/AsicFwS1.bin.imghdr relative 0x154 relative 0x154
	./sbtool phase0bsyshdr ${s1cername} ${s1cerpass} 0x1234 tmp/S1_sys.sysimg tmp/AsicFwS1.bin.imghdr
	./sbtool phase0bimghdr $1/AsicFwS2_ASIC${var}.bin tmp/AsicFwS2.bin.imghdr relative 0x154 relative 0x154
	./sbtool phase0bsyshdr ${s2cername} ${s2cerpass} 0x1234 tmp/S2_sys.sysimg tmp/AsicFwS2.bin.imghdr
	echo "Creating $(tput bold) ASIC${var}_${ts}.bin $(tput sgr0)"
	cat tmp/S1_sys.sysimg $1/AsicFwS1.bin tmp/S2_sys.sysimg $1/AsicFwS2_ASIC${var}.bin > tmp/ASIC${var}_${ts}.bin
done


echo "Flashing $(tput bold) Firmware $(tput sgr0) to three ASICs"
sudo adb wait-for-devices
adb root
sleep 3

NOFILE="No such file"
CHECKFILE=`adb shell "cd /data; ls prog_app_p2"`
if [[ "$CHECKFILE" == *$NOFILE* ]]; then
	echo "Copying the ISP app to mainboard"
	adb push prog_app_p2 /data
	adb push prog_app.config /data
	adb shell "chmod +x /data/prog_app_p2"
fi

for var in 1 2 3
do
	adb shell "rm /data/ASIC${var}*.bin"
	adb push tmp/ASIC${var}_${ts}.bin /data/ASIC${var}_${ts}.bin
done
# adb shell "prog_app_v02 -m program -i spi -f /data/ASIC1_${ts}.bin,/data/ASIC2_${ts}.bin,/data/ASIC3_${ts}.bin"
adb shell "cd data;./prog_app_p2 -m program -i spi -f /data/ASIC1_${ts}.bin,/data/ASIC2_${ts}.bin,/data/ASIC3_${ts}.bin"
echo "Flashing done"

#echo "Flashing $(tput bold) ${FWFILE} $(tput sgr0)"
#sudo isp_app_p1_5 -m program -f ${FWFILE} -i usart
