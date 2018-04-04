#!/bin/python
#*******************************************************************************
#    @File's name    : light_cli_lcc.py
#    @Company        : Light Co.
#    @Author         : Fitz.Nguyen - Light's QC team
#    @Revision       : 1.0.1
#    @Date           : 15-Aug-2016
#*******************************************************************************
#                            REVISION HISTORY
#*******************************************************************************
# * 1.0.0   14-Aug-2016 Porting camera script from P1 & P1.1, ASB utilities
# * 1.0.1   15-Aug-2016 Migrate to use CLI tool 
#*******************************************************************************
import argparse
import string
import subprocess
import sys
import time
import datetime
import struct
#*******************************************************************************
print "\
\t\t--------------------------------------------\n\
\n\n\n\
\t\t $$                                         \n\
\t\t $$                    $$            $     \n\
\t\t $$                    $$           $$     \n\
\t\t $$                    $$           $$     \n\
\t\t $$            $$$     $$           $$$$$  \n\
\t\t $$    $$     $ $$$    $$  $$$      $$$$$  \n\
\t\t $$    $$    $   $$$   $$ $ $$$     $$     \n\
\t\t $$    $$   $$    $$   $$$   $$$    $$     \n\
\t\t $$    $$   $$    $$   $$     $$    $$     \n\
\t\t $$    $$    $$  $$    $$     $$    $$$ $  \n\
\t\t $$    $$     $$$$     $$     $$     $$$   \n\
\t\t                                            \n\
\t\t                  $$$                      \n\
\t\t                $$   $$                     \n\
\t\t                $$   $$                     \n\
\t\t                  $$$                       \n\
\n\
\t\t-------------------------------------------\n\
\t\t-------------------V2.2.0------------------\n\
\t\t-----------------2016.08.15----------------\n"

atypecam = [ 'a1', 'a2', 'a3', 'a4', 'a5' ]

camera_group = [ 'AB', 'ab', 'BC', 'bc' ]
ft4222 = "ft4222"
ft4222_cmd = "sudo %s cmd 0x08 %s 0x00 0x00 %s"
useadb = 1

# CLI API
cli_read  = "lcc_cli_tool r "
cli_write = "lcc_cli_tool w "

def translate_to_ft4222(cmd):
	if useadb == 1:
		return cmd
	if cmd.find("manual_control") != -1:
		return cmd
	if cmd.find(i2cpath) == -1:
		return cmd
	cmd = cmd.replace('adb shell ','')
	cmd = cmd.replace('"','')
	cmd = cmd.replace("'","")
	cmd = cmd.replace(i2cpath, '')
	cmd = cmd.replace(' > ', '')
	cmd = cmd.replace(' >', '')
	cmd = cmd.replace('> ', '')
	parts = cmd.split()
	numbyte = int(parts[1]) + 2
	dbytes = ""
	for i in range(3, len(parts)):
		dbytes = dbytes + parts[i] + " "
	newcmd = ft4222_cmd % (ft4222, hex(int(numbyte)), dbytes)
	return newcmd

def execute(cmd):
	cmd = translate_to_ft4222(cmd)
	print(cmd);
	proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True)
	return proc.communicate()[0]

def split_bytes(arg):
	# assume the user always inputs 4 digits (i.e the leading nibbles
	# in each byte are nonzero, so we get 4 digits)
	int_input = len(arg)
	#print int_input
	if (arg[:2] == '0x'):
		if (int_input < 6):
			fstring = '0x' + arg[-2:] + ' 0x0' + arg[2:3]
		else:
			fstring = '0x' + arg[-2:] + ' ' +  arg[0:4]
		return fstring
	else:
	   print "Not support this value"

def one_byte_hex(arg):
	# assume that the input is a string from 0 to 255.
	# the output will be one byte in hex.
	int_input = int(arg)
	if (int_input < 16):
		return '0x0' + hex(int_input)[2:]
	else:
		return '0x' + hex(int_input)[2:]

def convert_to_hex(arg):
	# outputs two bytes in hex, delimited by '0x's
	fstring = arg
	rs = 0
	int_rs = 0;
	if arg.find("0x") != -1:
		rs = arg
	else:
		int_rs =  int(arg)
		rs = hex(int_rs)
	if (int_rs <= 0xf): # one nibble
		fstring = '0x0' + rs[-1:] + ' 0x00'
	elif (int_rs <= 255): # two nibbles
		fstring = rs + ' 0x00'
	elif (int_rs <= 4095): # three nibbles
		fstring = '0x' + rs[-2:] + ' 0x0' + rs[-3:-2]
	else: # four nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2]
	return fstring

def convert_to_hex_4byte(arg):
	# outputs two bytes in hex, delimited by '0x's
	rs = 0
	int_rs = 0;
	if arg.find("0x") != -1:
		rs = arg
		int_rs = arg
	else:
		int_rs =  int(arg)
		rs = hex(int_rs)
	if (int_rs <= 0xF): # one nibble
		fstring = '0x0' + rs[-1:] + ' 0x00' + ' 0x00' + ' 0x00'
	elif (int_rs <= 0xFF): # two nibbles
		fstring = rs + ' 0x00' + ' 0x00'+ ' 0x00'
	elif (int_rs <= 0xFFF): # three nibbles
		fstring = '0x' + rs[-2:] + ' 0x0' + rs[-3:-2] + ' 0x00' + ' 0x00'
	elif (int_rs <= 0xFFFF): # four nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x00' + ' 0x00'
	elif (int_rs <= 0xFFFFF): # five nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x0' + rs[-5:-4]  + ' 0x00'
	elif (int_rs <= 0xFFFFFF): # six nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x' + rs[-6:-4] + ' 0x00'
	elif (int_rs <= 0xFFFFFFF): # seven nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x' + rs[-6:-4] + ' 0x0' + rs[-7:-6]
	else : # eight nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x' + rs[-6:-4] + ' 0x' + rs[-8:-6]
	return fstring

def resolution_convert_to_hex(arg):
	def res_x_y(x,y):
		ret = convert_to_hex_4byte(x) + " "
		ret += convert_to_hex_4byte(y)
		return ret
	return {
		'3M' : res_x_y('2104','1560'),
		'13M' : res_x_y('4208','3120'),
		'720P': res_x_y('1280','720'),
		'1080P': res_x_y('1920','1080'),
		'4K_UHD' : res_x_y('3840','2160'),
		'4K_CINEMA': res_x_y('4096','2160')
	}[arg]

def print_resolution(arg):
	return {
		'3M' : "2104 x 1560",
		'13M' : "4208 x 3120",
		'720P': "1280 x 720",
		'1080P': "1920 x 1080",
		'4K_UHD' : "3840 x 2160",
		'4K_CINEMA': "4096 x 2160"
	}[arg]

def convert_to_hex_8_bytes(arg):
	# outputs eight bytes in hex, delimited by '0x's
	fstring = arg
	rs = 0
	int_rs = 0;
	if arg.find("0x") != -1:
		print "Unsupported hexa number"
	else:
		int_rs =  int(arg)
		rs = hex(int_rs)
	if (int_rs <= 0xf): # one nibble
		fstring = '0x0' + rs[-1:] + ' 0x00 0x00 0x00 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xff): # two nibbles
		fstring = rs + ' 0x00 0x00 0x00 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xfff): # three nibbles
		fstring = '0x' + rs[-2:] + ' 0x0' + rs[-3:-2] + ' 0x00 0x00 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xffff): # four nibbles
		fstring = '0x' + rs[-2:] + ' 0x'+ rs[-4:-2] + ' 0x00 0x00 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xfffff): # five nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x0'+ rs[-5:-4] + ' 0x00 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xffffff): # six nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x00 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xfffffff): # seven nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x0' + rs[-7:-6] + ' 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xffffffff): # eight nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x00 0x00 0x00 0x00'
	elif (int_rs <= 0xfffffffff): # nine nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] +' 0x0' + rs[-9:-8] + ' 0x00 0x00 0x00'
	elif (int_rs <= 0xffffffffff): # ten nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x' + rs[-10:-8] + ' 0x00 0x00 0x00'
	elif (int_rs <= 0xfffffffffff): # eleven nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x' + rs[-10:-8] + ' 0x0' + rs[-11:-10] + ' 0x00 0x00'
	elif (int_rs <= 0xffffffffffff): # twelve nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x' + rs[-10:-8] + ' 0x' + rs[-12:-10] + ' 0x00 0x00'
	elif (int_rs <= 0xfffffffffffff): # thirteen nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x' + rs[-10:-8] + ' 0x' + rs[-12:-10] + ' 0x0' + rs[-13:-12] + ' 0x00'
	elif (int_rs <= 0xffffffffffffff): # fourteen nibbles
		fstring = '0x' + rs[-2:] + ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x' + rs[-10:-8] + ' 0x' + rs[-12:-10] + ' 0x' + rs[-14:-12] + ' 0x00'
	elif (int_rs <= 0xfffffffffffffff): # fifteen nibbles
		fstring = '0x' + rs[-2:]+ ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x' + rs[-10:-8] + ' 0x' + rs[-12:-10] + ' 0x' + rs[-14:-12] + ' 0x0' + rs[-15:-14]
	else: # sixteen nibbles
		fstring = '0x' + rs[-2:]+ ' 0x' + rs[-4:-2] + ' 0x'+ rs[-6:-4] + ' 0x' + rs[-8:-6] + ' 0x' + rs[-10:-8] + ' 0x' + rs[-12:-10] + ' 0x' + rs[-14:-12] + ' 0x' + rs[-16:-14]
	return fstring

def handle_endianness(in_string):
	# in_string should have an '0x' prepended
	# all cameras on (without global): 0x1FFFE
	in_string = in_string[2:]
	in_int = int(in_string, 16)
	#print "in_int = " + hex(in_int)
	if (in_int <= 0xf): # one nibble
		output_string = '0x0' + in_string + ' 0x00 0x00'
	elif (in_int <= 255): # two nibbles
		output_string = '0x' + in_string[0] + in_string[1] + ' 0x00 0x00'
	elif (in_int <= 4095): # three nibbles
		output_string = '0x' + in_string[1] + in_string[2] + ' 0x0' + in_string[0] + ' 0x00'
	elif (in_int <= 65535): # four nibbles
		output_string = '0x' + in_string[2] + in_string[3] + ' 0x' + in_string[0] + in_string[1] + ' 0x00'
	else: # five nibbles
		output_string = '0x' + in_string[3] + in_string[4] + ' 0x' + in_string[1] + in_string[2] + ' 0x0' + in_string[0]
	#print "output_string = "  + output_string
	return output_string

def dutycycle_lookup(x):
	return {
		'light' : "0x00 0x80", # 12.8%
		'Light' : "0x00 0x80",
		'medium': "0x00 0xC8", # 20%
		'Medium': "0x00 0xC8",
		'heavy' : "0x01 0x18", # 28%
		'Heavy' : "0x01 0x18"
	}[x]

def convert_dutycycle(arg):
	# converts a percentage argument into the correct 0xXX format
	# min: 0&
	# max: 28.0&
	output_string = ""

	# easy presets
	if arg == "light"  or arg == "Light" \
	or arg == "medium" or arg == "Medium"\
	or arg == "heavy"  or arg == "Heavy":
		return dutycycle_lookup(arg)

	# allow the user to enter '%' at the end if they wish
	if arg[-1:] == '%':
		arg = arg[:-1]
	arg_float = string.atof(arg)
	arg_dec = int(arg_float * 10)
	hex_string = hex(arg_dec)
	if len(hex_string) == 3:
		output_string = "0x00 " + hex_string[:2] + "0" + hex_string[2:]
	elif len(hex_string) == 4:
		# prepend an 0x00
		output_string = "0x00 " + hex_string
	elif len(hex_string) == 5:
		# prepend an 0x0, take the third character of the input, and append 0xXX
		# hacky...but it works
		output_string = "0x0" + hex_string[2] + " 0x" + hex_string[-2:]
	else:
		print "hex_string = " + hex_string
		print "Should never get here. Check your duty cycle input."
		exit()
	return output_string

def list_little_endian(number):
	# takes an input and splits it into a list of bytes, LSB to MSB
	output = []
	number = int(number) # sanitize input
	# clamp for two bytes
	if number >= 65535:
		number = 65535
	#print "number = " + hex(number)
	while (number > 0):
		LSB = number & 255;
		#print "LSB = " + hex(LSB)
		output.append(LSB)
		number = number >> 8;
	# need to pad for 4 bytes
	while (len(output) < 4):
		output.append(0)
	return output

def two_byte_little_endian(input_string):
	# input format:
	# 0xXXXX
	# 0xXXX
	# 0xXX
	# 0xX
	if input_string.find("0x") != -1:
		input_string = input_string[2:]
	if len(input_string) == 1:   # one nibble
		output_string = '0x0' + input_string + ' 0x00'
	elif len(input_string) == 2: # two nibbles
		output_string = '0x' + input_string[0] + input_string[1]  + ' 0x00'
	elif len(input_string) == 3: # three nibbles
		output_string = '0x' + input_string[1] + input_string[2]  + ' 0x0' + input_string[0]
	elif len(input_string) == 4: # four nibbles
		output_string = '0x' + input_string[2] + input_string[3] + ' 0x' + input_string[0] + input_string[1]
	return output_string;

def module_bitmask(x):
	return {
		'g' : "0x01 0x00 0x00",
		'a1': "0x02 0x00 0x00",
		'a2': "0x04 0x00 0x00",
		'a3': "0x08 0x00 0x00",
		'a4': "0x10 0x00 0x00",
		'a5': "0x20 0x00 0x00",
		'b1': "0x40 0x00 0x00",
		'b2': "0x80 0x00 0x00",
		'b3': "0x00 0x01 0x00",
		'b4': "0x00 0x02 0x00",
		'b5': "0x00 0x04 0x00",
		'c1': "0x00 0x08 0x00",
		'c2': "0x00 0x10 0x00",
		'c3': "0x00 0x20 0x00",
		'c4': "0x00 0x40 0x00",
		'c5': "0x00 0x80 0x00",
		'c6': "0x00 0x00 0x01",
		'a' : "0x3E 0x00 0x00",
		'b' : "0xC0 0x07 0x00",
		'c' : "0x00 0xF8 0x01",
		'ab': "0xFE 0x07 0x00",
		'bc': "0xC0 0xFF 0x00"
	}[x]

def module_channelid(x):
	return {
		'a1': "0x01",
		'a2': "0x02",
		'a3': "0x03",
		'a4': "0x04",
		'a5': "0x05",
		'b1': "0x06",
		'b2': "0x07",
		'b3': "0x08",
		'b4': "0x09",
		'b5': "0x0a",
		'c1': "0x0b",
		'c2': "0x0c",
		'c3': "0x0d",
		'c4': "0x0e",
		'c5': "0x0f",
		'c6': "0x10"
}[x]

def module_one_hot(x):
	return {
		'g' : 1,
		'a1': 2,
		'a2': 4,
		'a3': 8,
		'a4': 16,
		'a5': 32,
		'b1': 64,
		'b2': 128,
		'b3': 256,
		'b4': 512,
		'b5': 1024,
		'c1': 2048,
		'c2': 4096,
		'c3': 8192,
		'c4': 16384,
		'c5': 32768,
		'c6': 65536
}[x]

def ucid_to_hex(x):
	return {
		'preview'	: "0x03 0x00",
		'hires'		: "0x05 0x00",
		'focal'		: "0x06 0x00",
		'hdr'		: "0x07 0x00",
		'video'		: "0x04 0x00"
	}[x]

def change_iso_value(x):
	return {
		'100' : split_bytes("0x2010"),
		'200' : split_bytes("0x2020"),
		'400' : split_bytes("0x2030"),
		'800' : split_bytes("0x213F"),
		'1600' : split_bytes("0x423F"),
		'2400' : split_bytes("0x633F"),
		'3200' : split_bytes("0x843F"),
		'4200' : split_bytes("0xAE3F"),
		'6100' : split_bytes("0xFC3F"),
	}[x]

def open_byte(x):
	return {
		'hw' : "0x01",
		'sw' : "0x02",
		'cl': "0x00"
	}[x]

def data_type(x):
	return {
		'RAW_10'	: " 0x2b ",
		'LIGHT_RAW'	: " 0x30 "
	}[x]

def float_to_hex(f):
	return hex(struct.unpack('<I', struct.pack('<f', f))[0])

parser = argparse.ArgumentParser()
parser.add_argument("-ft", "--ft4222", help="Use FT4222 instead of ADB for sending commands",
					action="store")
parser.add_argument("-c", "--camera", nargs='+',
					help="select camera: from a1->a5, b1->b5 or c1->c6", action="store")
parser.add_argument("-o", "--open", nargs='+', help="open or close camera",
					action="store", choices=['hw', 'sw', 'cl'])
parser.add_argument("-s", "--stream", nargs='+', help="stream on or off",
					action="store", choices=['on', 'off', 'sync'])
parser.add_argument("-u", "--ucid", help="ucid for setting",
					action="store", choices=['preview', 'hires', 'focal', 'video', 'hdr'])
parser.add_argument("-f", "--focus", help="select focus distance for 35mm cameras",\
					action ="store" )
parser.add_argument("-read", "--read", help="read value",
					action="store_true")
parser.add_argument("-l_m", "--lens_manual", help="nudge lens",
					action="store_true")
parser.add_argument("-m_m", "--mirror_manual", help="nudge mirror",
					action="store_true")
parser.add_argument("-l", "--lens", help="lens",
					action="store_true")
parser.add_argument("-m", "--mirror", help="mirror",
					action="store_true")
parser.add_argument("-dir", "--direction", help="direction to nudge",
					action="store")
parser.add_argument("-dc", "--dutycycle", help="duty cycle to nudge",
					action="store")
parser.add_argument("-dur", "--duration", help="duration to nudge",
					action="store")
parser.add_argument("-r", "--resolution", help="select resolution value",
					action="store", choices=['3M', '13M', '720P', '1080P', '4K_UHD', '4K_CINEMA'])
parser.add_argument("-e", "--exposure", help="select exposure value",
					action="store")
parser.add_argument("-g", "--gain", help="select gain value",type=float,
					action="store")
parser.add_argument("-fps", "--fps", help="select fps value",
					action="store")
parser.add_argument("-rhsv", "--read_hall_sensor_value", help="Read the hall sensor for a given 70mm or 150mm camera",
					action="store_true")
parser.add_argument("-ghsv", "--go_to_hall_sensor_value", nargs='+', help="Command a given 70mm or 150mm camera to move to a given hall sensor position",
					action="store")
parser.add_argument("-fn", "--fine_nudge", help="Nudge in fine increments",
					action="store_true")
parser.add_argument("-mul", "--multiplier", help="Multiplier for the fine nudge command",
					action="store")
parser.add_argument("-tx", "--tx_channel", nargs='+', help="TX channel",
					action="store", choices=['0', '1'])
parser.add_argument("-vc", "--virtual_channel", nargs='+', help="Virtual channel",
					action="store")
parser.add_argument("-dt", "--data_type", nargs='+', help="Virtual channel",
					action="store", choices=['RAW_10','LIGHT_RAW'])
parser.add_argument("-tol", "--tolerance", help="tolerance value",
					action="store")

args = parser.parse_args()

if args.ft4222:
	ft4222 = args.ft4222
	useadb = 0

if args.camera:
	# OR the one-hot encoded cameras together into one bitmask
	print "Camera(s) are " + str(args.camera)
	m_bitmask_int = 0
	for camera in args.camera:
		m_bitmask_int |= module_one_hot(camera)
	m_bitmask_str = hex(m_bitmask_int)
	print "m_bitmask_str = " + m_bitmask_str
	m_bitmask = handle_endianness(m_bitmask_str)
	print "m_bitmask = " + m_bitmask
	print ""
	print "\t\t********* Start to send commands *********\n"
	camera_string = ""
	stream_string = ""
	open_string = ""
	ucid_string = ""
	if args.open:
		print "Open %s camera" % (args.open)
		fstring = ""
		byte_count = 5
		idx = 0
		for camera in args.camera:
			fstring += open_byte(args.open[idx])
			fstring += " "
			byte_count += 1
			idx += 1
		open_string = "adb shell am '" + cli_write + "\"0x00 0x00 " + m_bitmask + " " + fstring + "\"'"
		#print open_string
		execute(open_string)
		print ""
		if args.ucid:
			ucid_string = "adb shell am '" + cli_write + "\"0x00 0x10 " + ucid_to_hex(args.ucid) + "\"'"
			print "Sending LIGHT_ACTIVE_UCID %s" % (args.ucid)
			#print ucid_string
			execute(ucid_string)
			print ""
	if args.stream:
		idx = 0
		byte_count = 5
		for camera in args.camera:
			if (args.tx_channel or args.virtual_channel):
				if (len(args.camera) == len(args.tx_channel)) and (len(args.camera) == len(args.virtual_channel)):
					if (args.tx_channel[idx] == "0"):
						stream_string += "0x1"
					else:
						stream_string += "0x2"
					if (args.stream[idx] == 'on'):
						stream_string += "1 "
					else:
						stream_string += "0 "
					if (int(args.virtual_channel[idx]) >= 0):
						stream_string += "0x0"
						stream_string += args.virtual_channel[idx]
					if (args.data_type and (len(args.camera) == len(args.data_type))):
						stream_string += data_type(args.data_type[idx])
					else:
						stream_string += " 0x00 "
				else:
					if (args.stream == 'on'):
						stream_string += "0x11 "
					else:
						stream_string += "0x10 "
					stream_string += " 0x00 0x00 "
			else:
				if (args.stream == 'on'):
					stream_string += "0x11"
				else:
					stream_string += "0x10"
				stream_string += " 0x00 0x00 "
			idx += 1
			byte_count += 3
		camera_string = "adb shell am '"  + cli_write + "\"0x02 0x00 " \
		                + m_bitmask + " " + stream_string + "\"'"
		#print camera_string
		execute(camera_string)
		print ""
else:

	if not args.ucid:
		parser.print_help()

if args.ucid:
	if not args.camera:
		# Light active ucid
		ucid_string = "adb shell am '" + cli_write + "\"0x00 0x10 " \
		              + ucid_to_hex(args.ucid) + "\"'"
		print "Sending LIGHT_ACTIVE_UCID %s \n" % (args.ucid)
		#print ucid_string
		execute(ucid_string)
		exit()
if args.focus:
	fstring = ""
	byte_count = 5
	# check if we have non-35mm camera
	if args.read:
		focus_string = "adb shell am '" + cli_write + "\"0x48 0x00 " \
		               + m_bitmask + "\"'"
		#print focus_string
		execute(focus_string)
		byte_count = 0
		for camera in args.camera:
			if camera == 'g':
				byte_count = 4*16
				break
			else:
				byte_count +=4
		read_string = "adb shell am '" + cli_read + "\"0x48 0x00\"" \
		              + str(byte_count) + "'"
		#print read_string
		execute(read_string)
		print ""
	else:
		for camera in args.camera:
			# add the argument n number of times for n cameras
			fstring += convert_to_hex(args.focus)
			fstring += " 0x00 0x00"
			byte_count += 4
			fstring += " "
		print "Focus is " + args.focus
		if args.ucid:
			ucid = ucid_to_hex(args.ucid)
			focus_string = "adb shell am '" + cli_write + "\"0x48 0x00 " \
			               + m_bitmask + " " + ucid + " " + fstring + "\"'"
		else:
			focus_string = "adb shell am '" + cli_write + "\"0x48 0x00 " \
			               + m_bitmask + " " + fstring + "\"'"
		#print focus_string
		execute(focus_string)
		print ""

if args.lens_manual:
	if (args.camera in atypecam):
		# 35mm
		print "Use the -f focus argument for 35mm cameras."
		exit()
	else:
		# 70mm, 150mm
		#   adb shell am  "lcc_cli_tool w 0x51 0x00 < 3 byte module bitmask >
		#                                      < 1 byte direction >
		#                                      < 2 byte duty cycle >
		#                                      < 1 byte duration >"
		if args.direction and args.dutycycle and args.duration:
			direction_string = one_byte_hex(args.direction)
			dutycycle_string = convert_dutycycle(args.dutycycle)
			duration_string = one_byte_hex(args.duration)
			lens_string = "adb shell am '" + cli_write + "\"0x51 0x00 " \
			               + m_bitmask + " " + direction_string + " " \
			               + dutycycle_string + " " + duration_string + "\"'"
	print "Lens:"
	#print lens_string
	execute(lens_string)
	print ""

if args.mirror_manual:
	if (args.camera in atypecam):
		# 35mm
		print "no mirrors for 35mm, exiting"
		exit()
	else:
		# 70mm, 150mm
		#   adb shell am  "lcc_cli_tool w 0x47 0x00 < 3 byte module bitmask >
		#                                           < 1 byte direction >
		#                                           < 2 byte duty cycle>
		#                                           < 1 byte duration >"
		if args.direction and args.dutycycle and args.duration:
			direction_string = one_byte_hex(args.direction)
			dutycycle_string = convert_dutycycle(args.dutycycle)
			duration_string = one_byte_hex(args.duration)
			mirror_string = "adb shell am '" + cli_write + "\"0x47 0x00 " \
			                + m_bitmask + " " + direction_string + " " \
			                + dutycycle_string + " " + duration_string + "\"'"
	#print mirror_string
	execute(mirror_string)
	print ""

if args.exposure:
	if args.camera:
		if args.read:
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				exposure_string = "adb shell am '" + cli_write + "\"0x32 0x00 " \
				                  + m_bitmask + " " + ucid + "\"'"
				#print exposure_string
				execute(exposure_string)
				byte_count = 0
				for camera in args.camera:
					if camera == 'g':
						byte_count = 8*16
						break
					else:
						byte_count +=8
				read_string = "adb shell am '" + cli_read + "\"0x32 0x00\"" \
				              + str(byte_count) + "'"
				#print read_string
				execute(read_string)
				print ""
			else:
				print "Missing UCID"
				exit()
		else:
			fstring = ""
			byte_count = 5
			for camera in args.camera:
				# add the argument n number of times for n cameras
				fstring += convert_to_hex_8_bytes(args.exposure)
				fstring += " "
				byte_count += 8
			print "Exposure is " + args.exposure + " ns"
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				exposure_string = "adb shell am '" + cli_write + "\"0x32 0x00 "\
				                + m_bitmask + " " + ucid + " " + fstring + "\"'"
			else:
				exposure_string = "adb shell am '" + cli_write + "\"0x32 0x00 "\
				                  + m_bitmask + " " + fstring + "\"'"
			#print exposure_string
			execute(exposure_string)
			print ""
	else:
		print "Please provide a camera argument."
if args.gain:
	if args.camera:
		if args.read:
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				gain_string = "adb shell am '" + cli_write + "\"0x30 0x00 " \
				              + m_bitmask + " " + ucid + "\"'"
				#print gain_string
				execute(gain_string)
				byte_count = 0
				for camera in args.camera:
					if camera == 'g':
						byte_count = 4*16
						break
					else:
						byte_count +=4
				read_string = "adb shell am '" + cli_read + "\"0x30 0x00\"" \
				              + str(byte_count) + "'"
				#print read_string
				execute(read_string)
				print ""
			else:
				print "Missing UCID"
				exit()
		else:
			fstring = ""
			byte_count = 5
			for camera in args.camera:
				# add the argument n number of times for n cameras
				fstring += convert_to_hex_4byte(float_to_hex(args.gain))
				fstring += " "
				byte_count += 4
			print "Total gain is " + str(args.gain)
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				gain_string = "adb shell am '" + cli_write + "\"0x30 0x00 " \
				               + m_bitmask + " " + ucid + " " + fstring + "\"'"
			else:
				gain_string = "adb shell am '" + cli_write + "\"0x30 0x00 " \
				              + m_bitmask + " " + fstring + "\"'"
			#print gain_string
			execute(gain_string)
			print ""
	else:
		print "Please provide a camera argument."

if args.fps:
	if args.camera:
		if args.read:
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				fps_string = "adb shell am '" + cli_write + "\"0x50 0x00 " \
				             + m_bitmask + " " + ucid + "\"'"
				#print fps_string
				execute(fps_string)
				byte_count = 0
				for camera in args.camera:
					if camera == 'g':
						byte_count = 2*16
						break
					else:
						byte_count +=2
				read_string = "adb shell am '" + cli_read + "\"0x50 0x00\"" \
				              + str(byte_count) + "'"
				#print read_string
				execute(read_string)
				print ""
			else:
				print "Missing UCID"
				exit()
		else:
			fstring = ""
			byte_count = 5
			for camera in args.camera:
				# add the argument n number of times for n cameras
				fstring += convert_to_hex(args.fps)
				fstring += " "
				byte_count += 2
			print "FPS is " + args.fps
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				fps_string = "adb shell am '" + cli_write + "\"0x50 0x00 " \
				             + m_bitmask + " " + ucid + " " + fstring + "\"'"
			else:
				fps_string = "adb shell am '" + cli_write + "\"0x50 0x00 " \
				             + m_bitmask + " " + fstring + "\"'"
			#print fps_string
			execute(fps_string)
			print ""
	else:
		print "Please provide a camera argument."

if args.resolution:
	if args.camera:
		if args.read:
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				resolution_string = "adb shell am '" + cli_write \
				                    + "\"0x2c 0x00 " + m_bitmask \
				                    + " " + ucid + "\"'"
				print resolution_string
				execute(resolution_string)
				byte_count = 0
				for camera in args.camera:
					if camera == 'g':
						byte_count = 8*16
						break
					else:
						byte_count +=8
				read_string = "adb shell am '" + cli_read + "\"0x2c 0x00\"" \
				              + str(byte_count) + "'"
				#print read_string
				execute(read_string)
				print ""
			else:
				print "Missing UCID"
				exit()
		else:
			fstring = ""
			byte_count = 5
			for camera in args.camera:
				# add the argument n number of times for n cameras
				fstring += resolution_convert_to_hex(args.resolution)
				fstring += " "
				byte_count += 8
			print "Resolution is " + print_resolution(args.resolution)
			if args.ucid:
				ucid = ucid_to_hex(args.ucid)
				resolution_string = "adb shell am '" + cli_write \
				                    + "\"0x2C 0x00 " + m_bitmask + \
				                    " " + ucid + " " + fstring + "\"'"
			else:
				resolution_string = "adb shell am '" + cli_write \
				                    + "\"0x2C 0x00 " + m_bitmask \
				                    + " " + fstring + "\"'"
			#print resolution_string
			execute(resolution_string)
			print ""
	else:
		print "Please provide a camera argument."

if args.read_hall_sensor_value:
	if not args.camera:
		print "You must provide a camera to read a hall sensor."
		exit()
	byte_count=0
	for camera in args.camera:
		if camera == 'g':
			byte_count = 2*16
			break
		else:
			byte_count +=2
	if args.lens:
		print "Reading lens hall sensor..."
		output_string = "adb shell am '" + cli_write + "\"0x40 0x00 " \
		                + m_bitmask + "\"'"
		#print output_string
		execute(output_string)
		read_string = "adb shell am '" + cli_read + "\"0x40 0x00\"" + str(byte_count) + "'"
		#print read_string
		execute(read_string)
	if args.mirror:
		print "Reading mirror hall sensor..."
		output_string = "adb shell am '" + cli_write + "\"0x44 0x00 " \
		                + m_bitmask + "\"'"
		#print output_string
		execute(output_string)
		read_string = "adb shell am '" + cli_read + "\"0x44 0x00\"" + str(byte_count) + "'"
		#print read_string
		execute(read_string)

if args.go_to_hall_sensor_value:
	if not args.camera:
		print "You must provide a camera to command to move."
		exit()
	if args.lens:
		destination_string = ""
		if len(args.camera) != len(args.go_to_hall_sensor_value):
			print "Not enough arguments. For each camera selected, provide a destination."
			exit()
		for i in xrange( 0, len(args.camera)):
			destination_string = destination_string + " " + two_byte_little_endian(args.go_to_hall_sensor_value[i])
			print "Moving lens "  + str(args.camera[i]) + " to "  + args.go_to_hall_sensor_value[i] + " ..."
		args_len =  5 + (len(args.camera) * 2)
		output_string = "adb shell am '" + cli_write + "\"0x40 0x00 " \
		                + m_bitmask + " " + destination_string + "\"'"
		#print output_string
		execute(output_string)
	if args.mirror:
		print "args.go_to_hall_sensor_value: " + args.go_to_hall_sensor_value[0]
		destination_string = two_byte_little_endian(args.go_to_hall_sensor_value[0])
		print "Destination is " + args.go_to_hall_sensor_value[0]
		print "Moving mirror "  + str(args.camera[0]) + " to "  + args.go_to_hall_sensor_value[0] + " ..."
		output_string = "adb shell am '" + cli_write + "\"0x44 0x00 " \
		                + m_bitmask + " " + destination_string + "\"'"
		#print output_string
		execute(output_string)

if args.fine_nudge:
	output_string = ""
	multiplier = 1
	direction = 1
	if args.camera:
		if args.multiplier:
			multiplier = args.multiplier
		else:
			print "No multiplier given. Using the default value of 1. "

		if args.direction:
			direction = args.direction
		else:
			print "No direction given. Using the default value of 1. "

		if args.lens:
			output_string = "adb shell am '" + cli_write + "\"0x51 0x00 "      \
			              + m_bitmask + " " + one_byte_hex(direction) + " "    \
			              + two_byte_little_endian(hex(int(multiplier))) + "\"'"
		if args.mirror:
			output_string = "adb shell am '" + cli_write + "\"0x52 0x00 "      \
			              + m_bitmask + " " + one_byte_hex(direction) + " "    \
			              + two_byte_little_endian(hex(int(multiplier))) + "\"'"
		#print output_string
		execute(output_string)
	else:
		print "Please provide a camera argument. "
		exit()
