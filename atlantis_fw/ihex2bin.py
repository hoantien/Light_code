#!/usr/bin/python

import sys
import struct
import array
# check input argument
if len(sys.argv) < 3:
	print "\r\n\t Usage: [command] [input hex file] [output binary file]\r\n"
	sys.exit(0)

# check input hex file path
is_input_ihex = sys.argv[1].find(".hex")
if is_input_ihex >= 0:
	ihex_input_path = sys.argv[1]
else:
	print ">>>>>>>>>>> Input hex file is not formatted! <<<<<<<<<<<"
	sys.exit(0)

# check input hex file path
is_output_bin = sys.argv[2].find(".bin")
if is_output_bin >= 0:
	bin_output_path = sys.argv[2]
else:
	print ">>>>>>>>>>> Output bin file is not formatted! <<<<<<<<<<<"
	sys.exit(0)

# Create new binary file
try:
    binfo = open(bin_output_path,'w+r+')   # Trying to create a new file or open one
    binfo.close()
except:
    print 'Something went wrong! Can\'t tell what?'
    sys.exit(0)


# create header list
LIST_ADDR 	  = ["0000", "", "", "", "", "", "", ""]
LIST_SIZE     = [0, 0, 0, 0, 0, 0, 0, 0]
LIST_DATA     = ["", "", "", "", "", "", "", ""]
LIST_IDX  	  = 0
START_EXT_ADDR 	  = 0


def ihexparser( strline ):
	global START_EXT_ADDR
	global LIST_ADDR
	global LIST_IDX
	global LIST_SIZE
	global LIST_DATA

	IHEX_RECORD_TYPE_DATA	= "00" # Data record
	IHEX_RECORD_TYPE_EOF	= "01" # End Of File
	IHEX_RECORD_TYPE_ESA	= "02" # Extended Segment Address
	IHEX_RECORD_TYPE_SSA	= "03" # Start Segment Address
	IHEX_RECORD_TYPE_ELA	= "04" # Extended Linear Address
	IHEX_RECORD_TYPE_SLA	= "05" # Start Linear Address

	IHEX_START_CODE			= ':'
	# data length value
	IHEX_LEN_CODE_IDX		= 1
	# offset code value
	IHEX_OFFSET_CODE_IDX	= 3
	# IHex record type
	IHEX_RECORD_TYPE_IDX	= 7
	# Ihex Data index and length
	IHEX_DATA_IDX			= 9

	# Check first line to get extended linear address of application
	RECORD_TYPE = strline[IHEX_RECORD_TYPE_IDX:IHEX_RECORD_TYPE_IDX + 2]

	if RECORD_TYPE == IHEX_RECORD_TYPE_DATA:
		DATA_LEN = strline[IHEX_LEN_CODE_IDX : IHEX_LEN_CODE_IDX + 2]
		LIST_SIZE[LIST_IDX] = LIST_SIZE[LIST_IDX] + int(DATA_LEN,16)
		LIST_DATA[LIST_IDX] = LIST_DATA[LIST_IDX] + strline[IHEX_DATA_IDX : IHEX_DATA_IDX + (int(DATA_LEN,16) * 2)]
		if START_EXT_ADDR == 0:
			START_EXT_ADDR      = 1
			OFFSET_ADDR         = strline[IHEX_OFFSET_CODE_IDX : IHEX_OFFSET_CODE_IDX + 4]
			LIST_ADDR[LIST_IDX] = LIST_ADDR[LIST_IDX] + OFFSET_ADDR

	if RECORD_TYPE == IHEX_RECORD_TYPE_ELA:
		EXT_ADDR            = strline[IHEX_DATA_IDX : IHEX_DATA_IDX + 4]
		if START_EXT_ADDR != 0:
			LIST_IDX = LIST_IDX + 1
		LIST_ADDR[LIST_IDX] = EXT_ADDR
		START_EXT_ADDR      = 0

	return [0]

def hex_dump(buff):
	l = len(buff)
	i = 0
	m = l / 16
	while i < m:
		line = ""
		count = 0
		while count < 16:
			data = ord(buff[(i * 16) + count])
			line += "0x" + '%02X' % data + ", "
			count = count + 1
		i = i + 1
		print line
	return buff

# Open hex file
with open(ihex_input_path, 'r') as f:
	for line in f:
		ihexparser(line)
		# Check EOF
		if ':00000001FF' in line:
			count = 0
			DATA_WRITE = ""
			while (count <= LIST_IDX):
				SIZE = '%08X' % LIST_SIZE[count]
				ADDR = LIST_ADDR[count]
				DATA = LIST_DATA[count]
				DATA_WRITE = DATA_WRITE + ADDR + SIZE + DATA
				print "\r\nSection " + '%d' % count + " addr 0x" + LIST_ADDR[count] + " size " + '%d' % LIST_SIZE[count]
				count = count + 1
			# Write data to file
			binfo = open(bin_output_path, "w+")
			binfo.write(DATA_WRITE.decode("hex"))
			binfo.close()
			break
