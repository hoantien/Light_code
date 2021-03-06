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
#******************************************************************************
#
# Bootflow State
#
#******************************************************************************
FIRMWARE	=	QC_ASIC2
include $(DEV_BASE)/qc_platform/05_WORKSPACE/qc_platform.mak

APPLI_START_ADDRESS=0x022a0000
READ_COMMAND=0x12
TARGET_DESC=${BOOT_S2_DIR}_${ASIC_NUM}
TARGET_ASIC_STARTUP=${BOOT_S2_DIR}

################################### ASIC Entry Point ##########################

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
VPATH+=:${STARTUP_DIR}/${TARGET_ASIC_STARTUP}
LINKER_FILE=${STARTUP_DIR}/${TARGET_ASIC_STARTUP}/linker.ld

#******************************************************************************
#
# Make/clean all targets
#
#******************************************************************************
OBJ_APP=

OBJ_OS=	${COMPILER}/croutine.o				\
		${COMPILER}/list.o					\
		${COMPILER}/queue.o					\
		${COMPILER}/tasks.o					\
		${COMPILER}/timers.o				\
		${COMPILER}/port.o					\
		${COMPILER}/os_port.o				\
		${COMPILER}/portASM.o				\
		${COMPILER}/heap_4.o				\
		${COMPILER}/FreeRTOSTickConfig.o

OBJ_DRIVER=	${COMPILER}/i2c_slave.o			\
			${COMPILER}/spi_slave.o			\
			${COMPILER}/timer.o				\
			${COMPILER}/ar1335.o			\
			${COMPILER}/lcc_cmd_log.o		\
			${COMPILER}/lcc_system.o		\
			${COMPILER}/init_platform.o		\
			${COMPILER}/trace_timer.o		\
			${COMPILER}/rtc.o				\
			${COMPILER}/vcm.o				\
			${COMPILER}/temp_sensor.o		\
			${COMPILER}/camera.o			\
			${COMPILER}/ina231.o			\
			${COMPILER}/i2cm.o				\
			${COMPILER}/hall_sensor.o		\

OBJ_HAL=${COMPILER}/hal_com.o		\
		${COMPILER}/hal_gpio.o		\
		${COMPILER}/hal_timer.o		\
		${COMPILER}/hal_wdt.o		\
		${COMPILER}/hal_i2c.o		\
		${COMPILER}/hal_spi.o		\
		${COMPILER}/hal_syncio.o	\
		${COMPILER}/hal_pwm.o		\
		${COMPILER}/hal_vic.o		\
		${COMPILER}/hal_dma.o		\
		${COMPILER}/hal_mipi2axi.o	\
		${COMPILER}/hal_axi2mipi.o	\
		${COMPILER}/hal_ddr.o		\

OBJ_ASIC_START=	${COMPILER}/armv7_start.o	\
				${COMPILER}/armv7_cache.o	\
				${COMPILER}/int_vectors.o	\
				${COMPILER}/reset_handler.o	\
				${COMPILER}/startup.o		\

#******************************************************************************
#
# The compiler to be used.
#
#******************************************************************************
CFLAGS+=-mthumb -mlong-calls -fno-common -fno-builtin
CFLAGS+=-D ASIC_NUM=${ASIC_NUM} -DOS -DASIC_FWS2
CFLAGS+=-D CAMERA_TBL=${LOOKUP_TBL_${ASIC_NUM}}
AFLAGS+=-D ASIC_NUM=${ASIC_NUM} -DOS
LDFLAGS+=-Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
BUILD_TARGET=S2_TARGET
all: ${COMPILER}					\
	 ${COMPILER}/${TARGET_DESC}.elf \
	 showsize

showsize: ${COMPILER}
	  	  @$(OBJSIZE) -B ${COMPILER}/${TARGET_DESC}.elf

-include ${PROJECT_ROOT}/common.mk
-include ${PROJECT_ROOT}/lookup_tbl.mk

#
# The rule to create the target directory
#

${COMPILER}/${TARGET_DESC}.elf: ${OBJ_ASIC_START} ${OBJ_HAL} ${OBJ_DRIVER} ${OBJ_OS} ${OBJ_APP} ${QC_LIBS_OBJ} ${OBJ_QC}

#
#
# Include the automatically generated dependency files.
#
-include ${wildcard ${COMPILER}/*.d} __dummy__
