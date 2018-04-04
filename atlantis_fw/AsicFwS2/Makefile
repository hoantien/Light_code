#******************************************************************************
#
# Bootflow State
#
#******************************************************************************
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
OBJ_APP=${COMPILER}/main.o								\
		${COMPILER}/line2.o								\
		${COMPILER}/calibclasses.o						\
		${COMPILER}/disparityfocus.o					\
		${COMPILER}/basic_image.o						\
		${COMPILER}/roitransfer.o						\
		${COMPILER}/mirrorsystems.o						\
		${COMPILER}/lcc_cmd.o							\
		${COMPILER}/lcc_cmd_base_0000.o					\
		${COMPILER}/lcc_cmd_base_0200.o					\
		${COMPILER}/lcc_cmd_base_1000.o					\
		${COMPILER}/lcc_cmd_log.o						\
		${COMPILER}/task_cam_ctrl.o						\
		${COMPILER}/task_ccb_ctrl.o						\
		${COMPILER}/vcm.o								\
		${COMPILER}/actuator.o							\
		${COMPILER}/optical.o							\
		${COMPILER}/img_sensor.o						\
		${COMPILER}/usecase.o							\
		${COMPILER}/light_system.o						\
		${COMPILER}/usecase_hires.o						\
		${COMPILER}/timestamp.o							\
		${COMPILER}/camera_id.pb-c.o					\
		${COMPILER}/camera_module.pb-c.o				\
		${COMPILER}/color_calibration.pb-c.o			\
		${COMPILER}/dead_pixel_map.pb-c.o				\
		${COMPILER}/device_temp.pb-c.o					\
		${COMPILER}/distortion.pb-c.o					\
		${COMPILER}/flash_calibration.pb-c.o			\
		${COMPILER}/geometric_calibration.pb-c.o		\
		${COMPILER}/hot_pixel_map.pb-c.o				\
		${COMPILER}/hw_info.pb-c.o						\
		${COMPILER}/light_header.o						\
		${COMPILER}/lightheader.pb-c.o					\
		${COMPILER}/matrix3x3f.pb-c.o					\
		${COMPILER}/matrix4x4f.pb-c.o					\
		${COMPILER}/mirror_system.pb-c.o				\
		${COMPILER}/point2f.pb-c.o						\
		${COMPILER}/point2i.pb-c.o						\
		${COMPILER}/point3f.pb-c.o						\
		${COMPILER}/proximity_sensors.pb-c.o			\
		${COMPILER}/range2f.pb-c.o						\
		${COMPILER}/sensor_characterization.pb-c.o		\
		${COMPILER}/sensor_type.pb-c.o					\
		${COMPILER}/time_stamp.pb-c.o					\
		${COMPILER}/tof_calibration.pb-c.o				\
		${COMPILER}/view_preferences.pb-c.o				\
		${COMPILER}/vignetting_characterization.pb-c.o	\
		${COMPILER}/imu_data.pb-c.o						\
		${COMPILER}/protobuf-c.o						\
		${COMPILER}/af_helper.o							\
		${COMPILER}/test_ddr.o							\
		${COMPILER}/contrast_processor.o				\
		${COMPILER}/contrast_refine_step.o				\
		${COMPILER}/contrast_sweep.o	 				\
		${COMPILER}/contrast_sweep_manager.o	 		\
		${COMPILER}/uniform_contrast_pass.o	 			\
		${COMPILER}/max_search_contrast_pass.o 			\
		${COMPILER}/binary_refine_contrast_pass.o 		\
		${COMPILER}/rectangle.o							\
		${COMPILER}/lens_position_control.o				\
		${COMPILER}/image_raw10.o						\
		${COMPILER}/image_pixel_iter.o					\
		${COMPILER}/task_af_ctrl.o						\
		${COMPILER}/operator_new.o						\
		${COMPILER}/gen_heap.o							\
		${COMPILER}/ddr_heap.o							\
		${COMPILER}/af_ae.o								\
		${COMPILER}/af_lens_calib.o						\
		${COMPILER}/af_roi_transfer.o					\
		${COMPILER}/calib_data_manager.o				\
		${COMPILER}/opticalzoom.o						\
		${COMPILER}/optical_zoom_manager.o				\
		
OBJ_OS=	${COMPILER}/croutine.o				\
		${COMPILER}/list.o					\
		${COMPILER}/queue.o					\
		${COMPILER}/tasks.o					\
		${COMPILER}/event_groups.o			\
		${COMPILER}/timers.o				\
		${COMPILER}/port.o					\
		${COMPILER}/os_port.o				\
		${COMPILER}/portASM.o				\

OBJ_DRIVER=	${COMPILER}/log.o				\
			${COMPILER}/assert.o			\
			${COMPILER}/i2cm.o				\
			${COMPILER}/i2cs.o				\
			${COMPILER}/spis.o				\
			${COMPILER}/timer.o				\
			${COMPILER}/rtc.o				\
			${COMPILER}/ina231.o			\
			${COMPILER}/tof.o				\
			${COMPILER}/gyro.o				\
			${COMPILER}/ar1335.o			\
			${COMPILER}/eeprom.o			\
			${COMPILER}/hall_a1457.o		\
			${COMPILER}/hall_as5510.o		\
			${COMPILER}/temperature.o		\
			${COMPILER}/task_list.o			\
			${COMPILER}/flash.o				\

OBJ_HAL=${COMPILER}/hal_com.o				\
		${COMPILER}/hal_gpio.o				\
		${COMPILER}/hal_timer.o				\
		${COMPILER}/hal_wdt.o				\
		${COMPILER}/hal_i2c.o				\
		${COMPILER}/hal_spi.o				\
		${COMPILER}/hal_syncio.o			\
		${COMPILER}/hal_pwm.o				\
		${COMPILER}/hal_vic.o				\
		${COMPILER}/hal_dma.o				\
		${COMPILER}/hal_mipi2axi.o			\
		${COMPILER}/hal_axi2mipi.o			\
		${COMPILER}/hal_qspi.o				\
		${COMPILER}/hal_cache.o				\
		
OBJ_ASIC_START=	${COMPILER}/armv7_start.o	\
				${COMPILER}/int_vectors.o	\
				${COMPILER}/reset_handler.o	\
				${COMPILER}/startup.o		\

#******************************************************************************
#
# The compiler to be used.
#
#******************************************************************************
COMMON_CFLAGS=-mthumb -mlong-calls -fno-common
COMMON_CFLAGS+=-D ASIC_NUM=${ASIC_NUM} -DOS -DASIC_FWS2
COMMON_CFLAGS+=-D CAMERA_TBL=${LOOKUP_TBL_${ASIC_NUM}}
COMMON_CFLAGS+=-D BUILD_ID=${BUILD_ID}
ifdef BUILD_EVT2
COMMON_CFLAGS+=-D BUILD_EVT2
else
ifdef EVT3_REWORK
COMMON_CFLAGS+=-D EVT3_REWORK -D EVT3_NEW_MIRR_MODULE
else
#COMMON_CFLAGS+=-D EVT3_NEW_MIRR_MODULE
endif
endif
COMMON_CFLAGS+=-D LOG_VERBOSE_DEFAULT=${LOG_VERBOSE_DEFAULT}
COMMON_CFLAGS+=-DMIPI_SPEED_1500MHZ -DMICRON_LPDDR3 -DP2 -DPREVIEW_TEST -DMIRROR_RESTRICT_MOVE
COMMON_CFLAGS+=-DUSING_TOF_FUNC
COMMON_CFLAGS+=-DUSING_GYRO_FUNC
#COMMON_CFLAGS+=-DALWAYS_CALIB_MIRROR
COMMON_CFLAGS+=-DMIRROR_HARD_CODE_SENSITIVITY
CFLAGS+=${COMMON_CFLAGS}
CCFLAGS+=${COMMON_CFLAGS}
CCFLAGS+=-fno-exceptions
AFLAGS+=-D ASIC_NUM=${ASIC_NUM} -DOS -DP2 -DMIPI_SPEED_1500MHZ -DPREVIEW_TEST -DMIRROR_RESTRICT_MOVE
#LDFLAGS+= --specs=nano.specs --enable-newlib-nano-formatted-io
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

${COMPILER}/${TARGET_DESC}.elf: ${OBJ_ASIC_START} ${OBJ_HAL} ${OBJ_DRIVER} ${OBJ_OS} ${OBJ_APP}

#
#
# Include the automatically generated dependency files.
#
-include ${wildcard ${COMPILER}/*.d} __dummy__
