#!/bin/python
#*******************************************************************************
#    @File's name    : cli_lcc.py
#    @Company        : Light Co.
#    @Author         : Fitz.Nguyen - Light's QC team
#    @Revision       : 1.0.0
#    @Date           : 15-Aug-2016
#*******************************************************************************

#*******************************************************************************
#                                REVISION HISTORY
#*******************************************************************************
#    * 1.0.0        15-Aug-2016    Initial revision
#*******************************************************************************

#*******************************************************************************
#                         SYSTEM'S MODULES IMPORT 
#*******************************************************************************
import os
import sys
import getpass
#*******************************************************************************
#                     DEVELOPMENT'S MODULES IMPORT 
#*******************************************************************************

#*******************************************************************************
#                                INCLUDE
#*******************************************************************************
sys.path.append(os.path.join(os.getcwd(), "test_apps"))
current_dir = os.getcwd()
cli_tool_path = os.path.join(os.getcwd(), "../../../04_COMMON/cli/lcc_cli_tool")
#*******************************************************************************
#                           DEVELOPMENT IMPORT 
#*******************************************************************************
import syntax_0000
import syntax_1000
import interrupt_0000
import interrupt_1000
import functional_0000
import functional_1000

from syntax_0000     import syntax_0000_tests
from syntax_1000     import syntax_1000_tests
from interrupt_0000  import irq_0000_tests
from interrupt_1000  import irq_1000_tests
from functional_0000 import func_0000_tests
from functional_1000 import func_1000_tests
#*******************************************************************************
#                                FUNCTIONS
#*******************************************************************************
def host_init(passwd=""):
    '''
    @summary    : Initializing adb port 
    @param      : passwd    : root password
    @return     : NA
    @attention  : Make sure host is plugged into PC via USB debug port
    '''
    os.system("echo " + passwd + "| sudo -s adb kill-server")
    os.system("echo " + passwd + "| sudo -s adb-root")
    os.system("adb push " + cli_tool_path + " /data")

def main():
    '''
    @summary    : Application entry of testing app (FTM) 
    @param      : NA
    @return     : NA
    @attention  : Applicatipn entry of testing app (FTM)
    '''
    # Get current directory
    current_dir = os.getcwd()
    # Get root password
    passwd = getpass.getpass()
    # Initializing adb port
    host_init(passwd)
    # Testing
    test_package = [func_0000_tests, irq_0000_tests, syntax_0000_tests, \
                    func_1000_tests, irq_1000_tests, syntax_1000_tests]
    for module in test_package:
        for test in module:
            for command in test:
                print "Send command ID: " + command['id']
                print "adb shell 'cd /data;" + command['api'] + " "\
                          + command['cmd'] + " " + command['param'] + "'"
                # Execute command
                os.system("adb shell 'cd /data;" + command['api'] + " "\
                          + command['cmd'] + " " + command['param'] + "'")
                os.system("sleep 1")
#*******************************************************************************
#                             INPUTS PROCESSING
#*******************************************************************************

#*******************************************************************************
#                             MAIN PROCESSING
#*******************************************************************************
main()
#*******************************************************************************
#                                 END OF FILE
#*******************************************************************************