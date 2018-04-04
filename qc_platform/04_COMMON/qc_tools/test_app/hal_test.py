#!/bin/python
#*******************************************************************************
#    @File's name  : hal_test.py
#    @Company      : Light Co.
#    @Author       : Fitz.Nguyen - Light's QC team
#    @Revision     : 1.0.1
#    @Date         : 21-Apr-2016
#*******************************************************************************

#*******************************************************************************
#                                REVISION HISTORY
#*******************************************************************************
#   * 1.0.0        21-Mar-2016    Initial revision
#
#*******************************************************************************

#*******************************************************************************
# Copyright
# Light Co.'s copyright 2016
#*******************************************************************************

#*******************************************************************************
# @Brief:
#        This script is use to execute whole HAL validation for Light platform.
#*******************************************************************************

#*******************************************************************************
#                         IMPORT STANDARD MODULES
#*******************************************************************************
import os
import sys
import multiprocessing
import getpass
import pexpect
#*******************************************************************************
#                           ADD INCLUDE PATHS
#*******************************************************************************
sys.path.append(os.path.join(os.getcwd(), "../common"))
sys.path.append(os.path.join(os.getcwd(), "../hal"))
#*******************************************************************************
#                        IMPORT DEVELOMENT MODULES
#*******************************************************************************
import serial_port
import spec_proc
#*******************************************************************************
#                         NONE-VOLATILE VARIABLES
#*******************************************************************************
modules_list = {
				'COM'    : '1'   ,
				'I2C'    : '2'   ,
				'PWM'    : '3'   ,
				'SCU'    : '4'   ,
				'SYNCIO' : '5'   ,
				'SPI'    : '6'   ,
				'WDT'    : '7'   ,
				'TIMER'  : '8'   ,
				'GPIO'   : '9'   ,
				'MIPI'   : '11'  ,
				'DDR'    : '12'
			}
current_dir = os.getcwd()
pw = getpass.getpass()
#*******************************************************************************
#                          INPUTS PROCESSING
#*******************************************************************************
# Get serial port
port = "/dev/ttyUSB1"
#*******************************************************************************
#                            LOCAL FUNCTION
#*******************************************************************************
def modules_test(testlist={}, mode=""):
	'''
	@summary    : HAL modules test
	@param      : testlist	: module list as a hash table
	@return     : NA
	@attention  : Required test specifications as *.xls of all module
	'''
	print "********************************************************************"
	print "EXECUTE HAL AUTOMATION TEST"
	print "********************************************************************"
	# Create logs folder
	os.system("mkdir logs")
	for module in testlist.keys():
		module_test(testlist, module)
		# Rename report file
		if (mode != ""):
			os.system("mv -f test_report.xls Test_Reports_Module_" + module + "_" + mode + "_" + ".xls")
		else:
			os.system("mv -f test_report.xls Test_Reports_Module_" + module + ".xls")
		# Create log folder
		os.system("mkdir logs/" + module)
		# Move all log file into log folder
		os.system("mv -f " + module + "*.txt logs/" + module)
	# End of run_test()
#*******************************************************************************
def module_test(tests={}, module="", mode=""):
	'''
	@summary    : HAL module test
	@param      : module	: module name
	@return     : NA
	@attention  : Required test specifications as *.xls
	'''
	print "*******************************************************************"
	print "Welcome to Light HAL verification"
	print "*******************************************************************"
	test_ids   = []
	scenarios  = []
	handlers   = []
	approaches = []
	cflags     = []
	logs       = []
	results    = []
	# Daisy chain I2C or SPI
	if module == "I2C" or module == "SPI":
		if mode == "MASTER" or mode == "SLAVE":
			spec_path = current_dir + "/../../../02_IT/platform/asic/hal/spec/asic_" + \
			        	module.lower() + "_" + mode.lower() + "_verification_specification.xls"
		else:
			spec_path = current_dir + "/../../../02_IT/platform/asic/hal/spec/asic_" + \
			       		module.lower() + "_verification_specification.xls"
	else:
		# STEP 1: Read test specfication
		spec_path = current_dir + "/../../../02_IT/platform/asic/hal/spec/asic_" + \
			        module.lower() + "_verification_specification.xls"
	# Read specification
	spec_proc.read_spec(spec_path, test_ids, scenarios, \
	                       handlers, approaches, cflags)
	# Configure serial port  
	serial_obj = serial_port.xSerialInit(port, "115200")
	# STEP 2: Run all test
	index = 0
	while index < len(test_ids):
		print '%4s' % index,             " | ", '%6s' % test_ids[index], " | ",\
		      '%25s' % scenarios[index], " | ", '%8s' % handlers[index], " | ",\
		      '%10s' % approaches[index]," | ", '%16s' % cflags[index]
		# Create one event
		xRecordEndEvent = multiprocessing.Event()
		xRecordDoneEvent = multiprocessing.Event()
		# Indicate result file
		xResult = "./result.txt"
		# Remove garbage file
		if os.path.exists(xResult) == True:
			os.system("rm -f " + xResult)
		# Create streaming task
		xStreamingTask = multiprocessing.Process(name='xStreamingTask',
		                          target=serial_port.xSerialStreamTask,
		                          args=(serial_obj, xResult, \
		                          xRecordEndEvent,xRecordDoneEvent,))
		# Start streaming task
		xStreamingTask.start()
		# Go to workspace
		os.chdir(current_dir + "/../../../05_WORKSPACE")
		# Do reset Asic board
		child = pexpect.spawn('make reset')
		# Create auto passed parameters
		child.expect_exact('Password:')
		# Send password
		child.sendline(pw)
		# Get back to current directory
		os.chdir(current_dir)
		# Sleep 2s
		os.system("sleep 3")
		# Send module selection
		serial_port.xSerialWriteString(serial_obj, tests[module] + "\r\n")
		# Sleep 3s
		os.system("sleep 3")
		# Send test case handler
		serial_port.xSerialWriteString(serial_obj, str(handlers[index]) + "\r\n")
		# Sleep 2s
		os.system("sleep 1")
		# Send scenario
		serial_port.xSerialWriteString(serial_obj, scenarios[index] + "\r\n")
		# Waiting for finish 7s
		while xRecordDoneEvent.is_set() == False:
			''
		# Set terminate event
		xRecordEndEvent.set()
		# Convert result file
		os.system("mv ./result.txt " + test_ids[index] + ".txt")
		# Move to next test case
		index += 1
	# Denitialize serial port 
	serial_port.xSerialDeinit(serial_obj)
	# STEP 3: Do check results
	index = 0
	while index < len(test_ids):
		if (module.lower() in handlers[index]):
			# Read test result file
			f = open(test_ids[index] + ".txt", "r")
			record_start = False
			data = ""
			for line in f.readlines():
				if (record_start == False):
					if ("Select test application:" in line):
						record_start = True;
						data += line
				else:
					data += line
			# Save log
			logs.append(data)
			# Save result
			if ("PASSED" in logs[index]):
				results.append("PASSED")
			elif ("FAILED" in logs[index]):
				results.append("FAILED")
			else:
				results.append("")
		else:
			# Save log
			logs.append("NA")
			# Save result
			results.append("NA")
		# Move to next instance 
		index += 1
	# Step 4: Write to report
	spec_proc.report(spec_path, test_ids, logs, results)
	# Step 5: Print all result to screen
	index = 0
	while index < len(test_ids):
		print '%4s'  % index,            " | ", '%6s' % test_ids[index], " | ",\
		      '%25s' % scenarios[index], " | ", '%8s' % handlers[index], " | ",\
		      '%10s' % approaches[index]," | ", '%16s' % cflags[index],  " | ",\
		      '%8s'  % results[index],   " | " 
		# Move to next instance
		index += 1
	# End of module_test()
#*******************************************************************************
if __name__ == '__main__':
	'''
	@summary    : HAL test start entry
	@param      : NA
	@return     : NA
	@attention  : NA
	'''
	if (len(sys.argv) != 4):
		# Call regression test 
		modules_test(modules_list)
	else:
		# Module test
		port        = str(sys.argv[1])
		module_name = str(sys.argv[2]).upper()
		module_mode = str(sys.argv[3]).upper()
		# Execute module test
		module_test(modules_list, module_name, module_mode)
		# Rename report file 
		if module_name == "I2C" or module_name == "SPI":
			if module_mode == "SLAVE" or module_mode == "MASTER":
				os.system("mv -f test_report.xls Test_Reports_Module_" + module_name + "_" + module_mode + ".xls")
			else:
				os.system("mv -f test_report.xls Test_Reports_Module_" + module_name + ".xls")
		else:	
			os.system("mv -f test_report.xls Test_Reports_Module_" + module_name + ".xls")
#*******************************************************************************
#                            END OF FILE
#*******************************************************************************
