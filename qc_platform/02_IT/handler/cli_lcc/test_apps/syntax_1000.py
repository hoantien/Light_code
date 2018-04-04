#!/bin/python
#*******************************************************************************
#    @File's name    : syntax_1000.py
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
syntax_1000_tests = \
            [
                [
                    {'id':"syntax_001_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x00 0x00\""},
                    {'id':"syntax_001_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_001_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_002_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x01 0x00\""},
                    {'id':"syntax_002_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_002_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_003_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x02 0x00\""},
                    {'id':"syntax_003_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_003_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_004_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x03 0x00\""},
                    {'id':"syntax_004_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_004_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_005_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x04 0x00\""},
                    {'id':"syntax_005_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_005_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_006_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x05 0x00\""},
                    {'id':"syntax_006_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_006_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_007_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x06 0x00\""},
                    {'id':"syntax_007_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_007_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_008_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x07 0x00\""},
                    {'id':"syntax_008_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_008_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_009_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x08 0x00\""},
                    {'id':"syntax_009_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_009_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_010_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x09 0x00\""},
                    {'id':"syntax_010_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_010_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_011_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x0A 0x00\""},
                    {'id':"syntax_011_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_011_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_012_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x03 0x00\""},
                    {'id':"syntax_012_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_012_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_013_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x03 0x00 0x00\""},
                    {'id':"syntax_013_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_013_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_014_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x03\""},
                    {'id':"syntax_014_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_014_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ],
                [
                    {'id':"syntax_015_01",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"0x0B 0x00\""},
                    {'id':"syntax_015_02",'api':CLI_WRITE,'cmd':"\"0x00 0x10", 'param':"\""},
                    {'id':"syntax_015_03",'api':CLI_READ ,'cmd':"\"0x00 0x10\"", 'param':"2"},
                ]
            ]
#*******************************************************************************
#                             MAIN PROCESSING
#*******************************************************************************

#*******************************************************************************
#                                 END OF FILE
#*******************************************************************************