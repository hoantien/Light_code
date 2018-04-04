#!/bin/python
#*******************************************************************************
#   @File's name    : exe_it.py
#   @Company        : Light Co.
#   @Author         : Fitz.Nguyen - Light's QC team
#   @Revision       : 1.0.0
#   @Date           : May-19-2016
#*******************************************************************************

#*******************************************************************************
#                               REVISION HISTORY
#*******************************************************************************
#   * 1.0.0     17-Mar-2016 Initial revision
#*******************************************************************************

#*******************************************************************************
#                                IMPORT
#*******************************************************************************
import serial
#*******************************************************************************

#*******************************************************************************
#                                     INPUTS
#*******************************************************************************

#*******************************************************************************

#*******************************************************************************
#                                 PUBLIC FUNCTIONS
#*******************************************************************************
def xSerialInit(port="", baud=""):
	'''
	@summary    : Initializing serial port 
	@param      : port	: device port of serial inteface
	@param      : baud	: baud speed mode
	@return     : Initialized serial port object
	@attention  : Make sure serial port is loadded into kernel
	'''
	#! Create serial object 
	serial_obj = serial.Serial()
	serial_obj.baudrate = baud
	serial_obj.port     = port
	# Open serial port 
	serial_obj.open()
	print "Initialize serial port with info as below"
	print serial_obj
	return serial_obj
#*******************************************************************************
def xSerialWriteString(serial_obj, xStr=""):
	'''
	@summary    : Write a string to serial bus 
	@param      : serial_obj	: Initialize serial object
	@param      : xStr			: Sent string
	@return     : NA
	@attention  : Make sure serial object was initialized
	'''
	data = xStr.encode('ascii')
	serial_obj.write(data)
	return
#*******************************************************************************
def xSerialDeinit(serial_obj):
	'''
	@summary    : De-initializing serial port 
	@param      : serial_obj	: Initialize serial object
	@return     : NA
	@attention  : Make sure serial object was initialized before
	'''
	serial_obj.close()
	return
#*******************************************************************************
def xSerialStreamTask(serial_obj, result, event, evt_end):
	'''
	@summary    : Task to streaming serial data received from serial device 
	@param      : serial_obj	: Initialize serial object
	@param      : result		: Result output file path
	@param      : event			: Terminator event to terminate the streaming
	@return     : NA
	@attention  : Make sure serial object was initialized before
	'''
	#! Verifing 
	if (serial_obj.is_open == False):
		print "Serial port have not been opened yet. Please open serial port."
		return
	print "********************************************************************"
	print "[Thread] Start streaming task. Data is being streamed."
	print "********************************************************************"
	# Open file buffer 
	f = open(result, "w+")
	# Start streaming 
	while event.is_set() == False:
		s = serial_obj.readline()
		if (s != ""):
			f.write(s)
			# Check termination task
			if ("Scenario Done" in s):
				# Set terminating event
				evt_end.set()
	# Stope
	f.close()
	return
#*******************************************************************************
#                                 END OF FILE
#*******************************************************************************