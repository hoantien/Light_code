#!/bin/python
#*******************************************************************************
#	@File's name	: spec_proc.py
#	@Company		: Light Co.
#	@Author			: Fitz.Nguyen - Light's QC team
#	@Revision		: 1.0.0
#	@Date			: May-19-2016
#*******************************************************************************

#*******************************************************************************
#							    REVISION HISTORY
#*******************************************************************************
#	* 1.0.0		17-Mar-2016	Initial revision
#*******************************************************************************

#*******************************************************************************
#                                IMPORT
#*******************************************************************************
import re
import xlrd
import xlwt
import itertools
from xlutils.copy import copy
#*******************************************************************************

#*******************************************************************************
#                                     INPUTS
#*******************************************************************************

#*******************************************************************************

#*******************************************************************************
#                                 LOCAL FUNCTIONS
#*******************************************************************************
def get_sheet_by_name(wb=None, name=[]):
	'''
	@summary    : Get worksheet from workbook by name to write data
	@param[in]    wb          : Writable orginal specification
	@param[out]   name        : Name of sheet that would like to get
	@return:                  : File written worksheet
	@attention:   The wb is copied from original specification using 
	              xlutils.copy() of xlutils package 
	'''
	sheet = None
	try:
		for index in itertools.count():
			sheet = wb.get_sheet(index)
			if (sheet.name == name):
				return sheet
	except IndexError:
		print "Invalid sheet name."
		return None
#*******************************************************************************
#                                 PUBLIC FUNCTIONS
#*******************************************************************************
def read_spec(spec, test_ids = [], scenarios = [], \
              handlers = [], approaches = [], cflags = []):
	'''
	@summary    : Read test specification from xls file
	@param[in]    spec        : Test specification file path
	@param[out]   test_ids    : Test case ID that are marked in test spec
	@param[out]   scenarios   : Test scenarios which are defined in test spec
	@param[out]   handlers    : Test handlers which are indicated in test spec
	@param[out]   approaches  : Applied test approach that is indicated in spec
	@param[out]   cflags      : Additional build flag of all test cases  
	@return:                  : File object of test specification
	@attention:   The specification that was defined for HAL
	'''
	# Get workbook
	f = xlrd.open_workbook(spec)
	# Get sheet 
	s = f.sheet_by_name("Verification_specification")
	# Mark columns info
	col_id       = 0
	col_scenario = 2
	col_handler  = 3
	col_approach = 4
	col_cflags   = 8
	tests_num    = range(1, s.nrows)
	# Read document 
	for row in tests_num:
		# Read row
		test_id  = str(s.cell_value(row, col_id))
		scenario = str(s.cell_value(row, col_scenario))
		handler  = str(s.cell_value(row, col_handler))
		approach = str(s.cell_value(row, col_approach))
		cflag    = str(s.cell_value(row, col_cflags))
		# Remove railling and trailling spaces
		test_id  = test_id.strip()
		scenario = scenario.strip()
		handler  = handler.strip()
		approach = approach.strip()
		cflag    = cflag.strip()
		# Customize handlers
		handler  = re.sub("it_hal_", "", handler)
		# Write to lists
		test_ids.append(test_id)
		scenarios.append(scenario)
		handlers.append(handler)
		approaches.append(approach)
		cflags.append(cflag)
	return f
#*******************************************************************************
def open_spec(spec_path=""):
	'''
	@summary    : Open specification as file object
	@param[in]  : spec_path : path to test specification
	@return     : file object of test specification
	@attention: : The specification that was defined for HAL (xls format)
	'''
	f = xlrd.open_workbook(spec_path, formatting_info=True, on_demand=True)
	return f
#*******************************************************************************
def copy_spec(spec_obj=None):
	'''
	@summary    : Copy object from input specification
	@param[in]  : spec_obj : Test specification file object
	@return     : copied object at new address
	@attention: : The specification that was defined for HAL
	'''
	mirror = copy(spec_obj)
	return mirror
#*******************************************************************************
def report(spec_path="", test_ids=[], logs=[], results=[]):
	'''
	@summary    : Copy object from input specification
	@param[in]    spec_obj : Test specification file object
	@return:      copied object at new address
	@attention:   The specification that was defined for HAL
	'''
	col_test_ids = 0
	col_logs     = 9
	col_results  = 10
	rb           = open_spec(spec_path)
	wb           = copy_spec(rb)
	ws           = get_sheet_by_name(wb, "Verification_specification")
	rs           = rb.sheet_by_name("Verification_specification")
	n_style      = "align: wrap on, vert top, horiz left;" + \
	               "borders: top thin, left thin, bottom thin, right thin;"
	p_style      = "align: wrap on, vert top, horiz center;" + \
	               "borders: top thin, left thin, bottom thin, right thin;"
	# Use API: xlwt.Style.easyxf()
	# Start writing test results 
	index = 0
	while (index < len(test_ids)):
		for row in range(1, rs.nrows):
			if (test_ids[index] == rs.cell_value(row, col_test_ids)):
				# Write test output 
				ws.write(row, col_logs, logs[index], xlwt.easyxf(n_style))
				# Write test result
				if (results[index] == "PASSED"):
					color = "pattern: pattern solid, fore_colour green;"
				elif (results[index] == "FAILED"):
					color = "pattern: pattern solid, fore_colour red;"
				else:
					color = "pattern: pattern solid, fore_colour gray25;"
				r_style = p_style + color
				ws.write(row, col_results, results[index], xlwt.easyxf(r_style))
				break
		# Move to next test case 
		index += 1
	wb.save("test_report.xls")
	return
#*******************************************************************************