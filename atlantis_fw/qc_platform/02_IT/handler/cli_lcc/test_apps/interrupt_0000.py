#!/bin/python
#*******************************************************************************
#    @File's name    : interrupt_0000.py
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

#*******************************************************************************
#                                INCLUDE
#*******************************************************************************

#*******************************************************************************
#                           DEVELOPMENT IMPORT 
#*******************************************************************************

#*******************************************************************************
#                                FUNCTIONS
#*******************************************************************************

#*******************************************************************************
#                             INPUTS PROCESSING
#*******************************************************************************
# Get CLI tool path
CLI_TOOL = "lcc_cli_tool"
CLI_READ = CLI_TOOL + " r "
CLI_WRITE = CLI_TOOL + " w "
# Sub-test cases
irq_0000_tests = \
            [
                [
                    {'id':"irq_001_01",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x02 0x00 0x00 0x00\""},
                    {'id':"irq_001_02",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x02 0x00 0x00 0x01\""},
                    {'id':"irq_001_03",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x02 0x00 0x00 0x02\""},
                    {'id':"irq_001_04",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x06 0x00 0x00 0x01 0x02\""},
                    {'id':"irq_001_05",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x01 0x00 0x00 0x00\""},
                    {'id':"irq_001_06",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x01 0x00 0x00 0x02 0x02\""},
                    {'id':"irq_001_07",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x06 0x00 0x00 0x02\""},
                    {'id':"irq_001_08",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x06 0x00 0x00 0x03\""},
                    {'id':"irq_001_09",'api':CLI_WRITE,'cmd':"\"0x00 0x80", 'param':"0x06 0x00 0x00 0x03\""}
                ],
                [
                    {'id':"irq_002_01",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_02",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_03",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_04",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_05",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_06",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_07",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_08",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"},
                    {'id':"irq_002_09",'api':CLI_READ,'cmd':"\"0x7C 0x02\"", 'param':"4"}
                ]
            ]
#*******************************************************************************
#                             MAIN PROCESSING
#*******************************************************************************

#*******************************************************************************
#                                 END OF FILE
#*******************************************************************************