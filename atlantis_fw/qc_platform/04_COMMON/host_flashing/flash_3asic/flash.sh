#!/bin/sh

# CHANGE THIS IF YOU WANT
s1cername=light_cert_0630
s1cerpass=12345678
s2cername=light_cert_0630
s2cerpass=12345678









































# THESE LINES SHOULD BE CHANGED EXCEPT YOU REALLY WANT TO
if [ "$#" -lt 2 ]; then
	echo "$(tput bold)$(tput setaf 1)Syntax: $0 FWS1.bin FWS2.bin $(tput sgr0)"
	exit
fi
rm -rf tmp > /dev/null
mkdir -p tmp

ts=`date +%F-%I-%M-%S`
FWFILE=tmp/fw_${ts}.bin

echo "Signing $(tput bold)$(tput setaf 4) $1 $(tput sgr0) \n"
./sbtool phase0bimghdr $1 tmp/AsicFwS1.bin.imghdr relative 0x154 relative 0x154
./sbtool phase0bsyshdr ${s1cername} ${s1cerpass} 0x1234 tmp/S1_sys.sysimg tmp/AsicFwS1.bin.imghdr
echo "Signing $(tput bold)$(tput setaf 4) $2 $(tput sgr0) \n"
./sbtool phase0bimghdr $2 tmp/AsicFwS2.bin.imghdr relative 0x154 relative 0x154
./sbtool phase0bsyshdr ${s2cername} ${s2cerpass} 0x1234 tmp/S2_sys.sysimg tmp/AsicFwS2.bin.imghdr
echo "Creating $(tput bold) ${FWFILE} $(tput sgr0) \n"
cat tmp/S1_sys.sysimg $1 tmp/S2_sys.sysimg $2 > ${FWFILE}

adb shell "rm /data/fw.bin"
adb push ${FWFILE} /data/fw.bin
adb shell "cd /data; ./isp_app_p1_5_notrst -m program -i spi -f fw.bin,fw.bin,fw.bin"
#echo "Flashing $(tput bold) ${FWFILE} $(tput sgr0) \n"
#sudo ./isp_app_p1_5 -m program -f ${FWFILE} -i usart
