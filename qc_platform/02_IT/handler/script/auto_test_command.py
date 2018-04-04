#!/bin/python
#*******************************************************************************
#    @File's name  : auto_test_command.py
#    @Company      : Light Co.
#    @Author       : Hung.Bui - Light's QC team
#    @Revision     : 1.0.0
#    @Date         : 16-August-2016
#*******************************************************************************

#*******************************************************************************
#                                REVISION HISTORY
#*******************************************************************************
#   * 1.0.0        16-August-2016    Initial revision
#
#*******************************************************************************

#*******************************************************************************
# Copyright
# Light Co.'s copyright 2016
#*******************************************************************************

#*******************************************************************************
# @Brief:
#        This script is use to execute whole LCC COMMAND for Light.
#*******************************************************************************

#*******************************************************************************
#                         IMPORT STANDARD MODULES
#*******************************************************************************
import os
import sys
import multiprocessing
import getpass
import subprocess
import pexpect
#*******************************************************************************
#                           ADD INCLUDE PATHS
#*******************************************************************************
sys.path.append(os.path.join(os.getcwd(), "../../../04_COMMON/qc_tools/common"))
#sys.path.append(os.path.join(os.getcwd(), "../hal"))
#*******************************************************************************
#                        IMPORT DEVELOMENT MODULES
#*******************************************************************************
import serial_port
import spec_proc
#*******************************************************************************
#                         NONE-VOLATILE VARIABLES
#*******************************************************************************
modules_list = {
				'UCID'   : '1'   ,
			}
current_dir = os.getcwd()
pw = getpass.getpass()
#*******************************************************************************
#                          INPUTS PROCESSING
#*******************************************************************************

#*******************************************************************************
#                            LOCAL FUNCTION
#*******************************************************************************
def modules_test(testlist={}):
	print "********************************************************************"
	print "EXECUTE LCC COMMAND AUTOMATION TEST"
	print "********************************************************************"
	# Create logs folder
	os.system("mkdir logs")
	for module in testlist.keys():
		module_test(testlist, module)
		# Rename report file
		os.system("mv -f test_report.xls Test_Reports_" + module_name + ".xls")
		# Create log folder
		os.system("mkdir logs/" + module)
		# Move all log file into log folder
		os.system("mv -f " + module + "*.txt logs/" + module)
	# End of run_test()
#*******************************************************************************
def module_test(tests={}, module=""):
	print "*******************************************************************"
	print "Welcome to Light LCC COMMAND verification"
	print "*******************************************************************"
	test_ids   = []
	scenarios  = []
	handlers   = []
	approaches = []
	cflags     = []
	logs       = []
	results    = []
	# STEP 1: Read test specfication
	spec_path = current_dir + "/../spec/test_" + "command_" + module.lower() + ".xls"
	# Read specification
	spec_proc.read_spec(spec_path, test_ids, scenarios, \
	                       handlers, approaches, cflags)
	# Configure serial port  
	serial_obj = serial_port.xSerialInit("/dev/ttyUSB1", "115200")
	# STEP 2: Run all test
	index = 0
	while index < len(test_ids):
		print '%4s' % index,             " | ", '%6s' % test_ids[index], " | ",\
		      '%8s' % handlers[index]
		# Create one event
		xTerminateEvent = multiprocessing.Event()
		# Indicate result file
		xResult = "./result.txt"
		# Remove garbage file
		if os.path.exists(xResult) == True:
			os.system("rm -f " + xResult)
		# Create streaming task
		xStreamingTask = multiprocessing.Process(name='xStreamingTask',
		                          target=serial_port.xSerialStreamTask,
		                          args=(serial_obj, xResult, xTerminateEvent,))
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
		# Go to test module
		os.chdir(current_dir + "/test_ucid")
		# Sleep 2s
		os.system("sleep 3")
		# Excute test cases
		child = pexpect.spawn("./" + handlers[index])
		# Create auto passed parameters
		# Get back directory
		os.chdir(current_dir)
		# Waiting for finish 18s
		os.system("sleep 20")
		# Set terminate event
		xTerminateEvent.set()
		# Convert result file
		os.system("mv ./result.txt " + test_ids[index] + ".txt")
		# Move to next test case
		index += 1
	# Denitialize serial port 
	serial_port.xSerialDeinit(serial_obj)
	# Step 3: Write to report
	#spec_proc.report(spec_path, test_ids, logs, results)
	# Step 4: Print all result to screen
	index = 0
	while index < len(test_ids):
		# Read test result file
		f = open(test_ids[index] + ".txt", "r")
		record_start = False
		data = ""
		for line in f.readlines():
			if (record_start == False):
				if ("Start task_cam_ctrl: c5" in line):
					record_start = True;
					data += line
			else:
				data += line
		# Save log
		logs.append(data)
		# Save result
		results.append("NA")
		# Move to next instance
		index += 1
	# End of module_test()
	spec_proc.report(spec_path, test_ids, logs, results)
#*******************************************************************************
if __name__ == '__main__':
	module_name = str(sys.argv[1]).upper()
	# Execute module test
	module_test(modules_list, module_name)
	# Rename report file 
	os.system("mv -f test_report.xls Test_Reports_Module_" + module_name + ".xls")
#*******************************************************************************
#                            END OF FILE
#*******************************************************************************
