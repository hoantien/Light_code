#*******************************************************************************
#	@File's name	: asic_utilities.py
#	@Company		: Light Co.
#	@Author			: Fitz.Nguyen - Light's QC team
#	@Revision		: 1.0.1
#	@Date			: 15-Apr-2016
#*******************************************************************************
#                             REVISION HISTORY
# * 1.0.0	28-Feb-2016	Initial revision. Infrastructure
# * 1.0.1	17-Apr-2016	Release with test Asic firmware 1.0.0
#*******************************************************************************
#
# Bootflow State
#
#******************************************************************************
FIRMWARE	=	QC_ASIC1
include $(DEV_BASE)/qc_platform/05_WORKSPACE/qc_platform.mak

APPLI_START_ADDRESS=0x022a0000
READ_COMMAND=0x12
TARGET_DESC=${BOOT_S1_DIR}
TARGET_ENTRY_POINT=start
export COMPILER=${OBJ_BUILD}/${TARGET_DESC}

#******************************************************************************
#
# The compiler to be used.
#
#******************************************************************************
DIR_APP=${PWD}/src

VPATH+=:${DIR_APP}

################################### STARTUP CODE ##############################
VPATH+=:${STARTUP_DIR}/${TARGET_DESC}
LINKER_FILE=${STARTUP_DIR}/${TARGET_DESC}/linker.ld
OBJ_LIBS=${DIR_PLATFORM}/common/sblib/libsb.a \
		${DIR_PLATFORM}/common/sblib/libellipsys_p0.a

CFLAGS+=-DSBOOT=${SBOOT_BUILD}
BUILD_TARGET=S1_TARGET
#******************************************************************************
#
# Make/clean all targets
#
#******************************************************************************
OBJ_HAL=${COMPILER}/hal_com.o 	\
        ${COMPILER}/hal_gpio.o 	\
        ${COMPILER}/hal_timer.o \
        ${COMPILER}/hal_wdt.o 	\
        ${COMPILER}/hal_vic.o   \
        ${COMPILER}/hal_qspi.o  \

OBJ_DRIVER=${COMPILER}/log.o    \
		   ${COMPILER}/flash.o  \
		   ${COMPILER}/timestamp.o  \
		   ${COMPILER}/assert.o\

OBJ_ASIC_START=${COMPILER}/armv7_start.o \
			   ${COMPILER}/armv7_cache.o \
			   ${COMPILER}/jump.o \
			   ${COMPILER}/int_vectors.o \
			   ${COMPILER}/reset_handler.o\
			   ${COMPILER}/startup.o \

OBJ_APP=${COMPILER}/oem_pub.o


all: ${COMPILER}                    \
	${COMPILER}/${TARGET_DESC}.elf \
	showsize

showsize: ${COMPILER}
		@$(OBJSIZE) -B ${COMPILER}/${TARGET_DESC}.elf

-include ${PROJECT_ROOT}/common.mk

#
# The rule to create the target directory
#

${COMPILER}/${TARGET_DESC}.elf: ${OBJ_ASIC_START} ${OBJ_HAL} ${OBJ_DRIVER} ${OBJ_APP} ${OBJ_BOOT} ${OBJ_LIBS}

#
#
# Include the automatically generated dependency files.
#
-include ${wildcard ${COMPILER}/*.d} __dummy__
