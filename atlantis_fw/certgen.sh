#!/bin/bash
# Certificate Generator

OEM_PUB_DST=${1}

CERT_NAME=${2}
CERT_PASS=${3}
CERT_VERSION=${4}
CERT_SIGN=${5}
mkdir -p ${CERT_NAME}

cd ${CERT_NAME}

SB_DIR=${PWD}

sbtool certgen ${CERT_NAME} ${CERT_PASS} rsa1024 ${CERT_SIGN}

sbtool export ${CERT_NAME} ${CERT_NAME}.pubkey

PUBKEY=${CERT_NAME}.pubkey
PUBKEY_SRC=${SB_DIR}/oem_pub.c
echo '
#!/user/bin/python
import sys
import struct
import array

oem_pub_file=sys.argv[1]
pubkey=sys.argv[2]

with open(pubkey, mode="rb") as file:
	fileContent = file.read()
	l = len(fileContent)
	i = 0
	oem_pub = "const unsigned char oem_public_key[] = {-r-n-t"
	j = 0
	while i < l:
		if j < 8 :
			j += 1
		else:
			oem_pub += "-r-n-t"
			j = 0
		ch = ord(fileContent[i])
		byte = " 0x%02X," % ch
		oem_pub += byte
		i += 1
	oem_pub += "};-r-n"
	oem_pub += "const unsigned int oem_public_key_len = %d;-r-n" % l
	oem_pub = oem_pub.replace(",}", "}")
	with open(oem_pub_file, "w") as text_file:
		text_file.write("%s" % oem_pub)
' > ${PWD}/sb.py

cd ..
python ${SB_DIR}/sb.py ${PUBKEY_SRC} ${SB_DIR}/${PUBKEY}

sed -i -- 's/-r/\r/g' ${PUBKEY_SRC}
sed -i -- 's/-n/\n/g' ${PUBKEY_SRC}
sed -i -- 's/-t/\t/g' ${PUBKEY_SRC}

cp ${PUBKEY_SRC} ${OEM_PUB_DST}

echo '
#!/user/bin/bash
OEM_IMAGE=${1}
CERT_NAME="--CERT_NAME"
CERT_PASS="--CERT_PASS"
CERT_SIGN="--CERT_SIGN"
CERT_VERSION="--CERT_VERSION"
sbtool phase0bimghdr ${OEM_IMAGE} ${OEM_IMAGE}.img relative 0x154 relative 0x154
sbtool phase0bsyshdr ${CERT_NAME} ${CERT_PASS} ${CERT_VERSION} ${OEM_IMAGE}.sys ${OEM_IMAGE}.img
' > ${SB_DIR}/sb.sh;
chmod +x ${SB_DIR}/sb.sh

sed -i -- "s#--CERT_NAME#${CERT_NAME}#g" ${SB_DIR}/sb.sh
sed -i -- "s#--CERT_PASS#${CERT_PASS}#g" ${SB_DIR}/sb.sh
sed -i -- "s#--CERT_VERSION#${CERT_VERSION}#g" ${SB_DIR}/sb.sh
sed -i -- "s#--CERT_SIGN#${CERT_SIGN}#g" ${SB_DIR}/sb.sh

