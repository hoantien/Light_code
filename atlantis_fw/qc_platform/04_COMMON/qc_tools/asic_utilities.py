#!/bin/python
#*******************************************************************************
#	@File's name	: asic_utilities.py
#	@Company		: Light Co.
#	@Author			: Fitz.Nguyen - Light's QC team
#	@Revision		: 1.0.1
#	@Date			: 15-Apr-2016
#*******************************************************************************

#*******************************************************************************
#							    REVISION HISTORY
#*******************************************************************************
#	* 1.0.0		17-Mar-2016	Initial revision
#   * 1.0.1		15-Apr-2016 Add support single flashing FPGA
#							Add support flashing dual STM firmwares
#
#*******************************************************************************
import os
import sys
import subprocess
import getpass
#*******************************************************************************
SPACE = " "
#*******************************************************************************
#                             INPUTS PROCESSING
#*******************************************************************************
# Get operation
operation   = str(sys.argv[1])
option      = str(sys.argv[2])
# Get FTDI base directory
path  = str(sys.argv[3])
# Get OS
HOST_OS     = str(os.popen("uname -o").read()).strip()
if "Linux" in HOST_OS:
	HOST_OS = "Linux"
# Get OS revision
HOST_OSR    = str(os.popen("uname -m").read()).strip()
#*******************************************************************************
#                             MAIN PROCESSING
#*******************************************************************************
if (operation == "--flash"):
	# ASIC Firmware
	TOOL = ""
	TOOL_FLAGS = ""
	if len(sys.argv) == 5:
		image = str(sys.argv[4])
	if len(sys.argv) == 6:
		image = str(sys.argv[4]) + SPACE + str(sys.argv[5])
	PW = getpass.getpass()
	if (option == "--firmware"):
		TOOL = os.path.join(path, "FTDI2232", HOST_OS, HOST_OSR, "FT2232_FLASH")
		if len(sys.argv) == 6:
			TOOL_FLAGS += "flash"
	# FPGA
	if (option == "--fpga"):
		TOOL = os.path.join(path, "FTDI4222", HOST_OS, HOST_OSR, "FT4222_FLASH")
		if (len(sys.argv) == 6):
			TOOL_FLAGS += ""
		else:
			TOOL_FLAGS += "flash"
	# Change mode for tool to executable
	os.system("chmod 777" + SPACE + TOOL)
	# Invoke tool
	os.system("echo " + PW + " |sudo" + SPACE + TOOL + SPACE + TOOL_FLAGS + SPACE + image)
	
if (operation == "--gpio"):
	# Reset
	TOOL = ""
	TOOL_FLAGS = ""
	PW = getpass.getpass()
	if (option == "--sys_reset"):
		TOOL = os.path.join(path, "GPIO", HOST_OS, HOST_OSR, "FT2232_GPIO")
		TOOL_FLAGS = "reset"
	if (option == "--uart_boot"):
		TOOL = os.path.join(path, "GPIO", HOST_OS, HOST_OSR, "FT2232_GPIO")
		TOOL_FLAGS = "uart"		
	# Enable execute mode 
	os.system("chmod 777" + SPACE + TOOL)
	print TOOL + SPACE + TOOL_FLAGS
	os.system("echo " + PW + " |sudo " + TOOL + SPACE + TOOL_FLAGS)
		
if (operation == "--i2c_cmd"):
	# Reset
	TOOL = ""
	TOOL_FLAGS = ""
	PW = getpass.getpass()
	if (option == "--enable_spi_upstream"):
		TOOL = os.path.join(path, "FTDI4222", HOST_OS, HOST_OSR, "FT4222_FLASH_I2C")
		TOOL_FLAGS = "spics U775 0"
		# Enable execute mode 
		os.system("chmod 777" + SPACE + TOOL)
		print TOOL + SPACE + TOOL_FLAGS
		os.system("echo " + PW + " |sudo " + TOOL + SPACE + TOOL_FLAGS)	 
#*******************************************************************************
#                                 END OF FILE
#*******************************************************************************
