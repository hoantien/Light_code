#*******************************************************************************
# @File's name	: qc_platform.mak
# @Company		: Light Co.
# @Author 		: Fitz-Light's QC team
# @Revision		: 1.0.1
# @Date			: 26-Mar-2016
#*******************************************************************************
#							REVISION HISTORY
#*******************************************************************************
#	1.0.0	30-Dec-2015	: Initial revision. 
#	1.0.1	16-Mar-2015	: Update according to change Asic FW design
#						* Remove it_hal_spi_flash
#						* Re-structure according to development source structure
#
#*******************************************************************************
# QC libraries base directory
QC_LIBS_PATH  = $(DEV_BASE)/qc_platform/04_COMMON/qc_libs
VPATH        += $(QC_LIBS_PATH)/source
CFLAGS       += -I $(QC_LIBS_PATH)/include
#*******************************************************************************
#                               HAL LAYER
#*******************************************************************************
ifeq ($(LAYER), HAL)
# QC application for IT phase
QC_APPS_DIR   = $(DEV_BASE)/qc_platform/02_IT/platform/asic/hal/app
# Add source searching paths
VPATH        += $(QC_APPS_DIR)/source
# Add compiling include paths
CFLAGS       += -I $(QC_APPS_DIR)/include
# Define global precompile macro for ASIC stage 1
ifeq ($(FIRMWARE), QC_ASIC1)
CFLAGS       += -DQC_ASIC1
endif

ifeq ($(FIRMWARE), QC_ASIC2)
# HAL verification objects
OBJ_QC       += $(COMPILER)/it_hal_com.o       \
                $(COMPILER)/it_hal_pwm.o       \
                $(COMPILER)/it_hal_scu.o       \
                $(COMPILER)/it_hal_i2c.o       \
                $(COMPILER)/it_hal_syncio.o    \
                $(COMPILER)/it_hal_wdt.o       \
                $(COMPILER)/it_hal_spi.o       \
                $(COMPILER)/it_hal_timer.o     \
                $(COMPILER)/it_hal_gpio.o      \
                $(COMPILER)/it_hal_ddr.o       \
                $(COMPILER)/it_hal_mipi.o      \
                $(COMPILER)/it_hal_app.o

# QC common objects
QC_LIBS_OBJ  += $(COMPILER)/qc_assert.o        \
                $(COMPILER)/qc_common.o        \
                $(COMPILER)/it_log_swapper.o   \
		$(COMPILER)/qc_camera.o        \
# Enable assert
CFLAGS       += -D_USE_FULL_ASSERT
CFLAGS       += -DQC_ASIC2
endif
# End of ($(FIRMWARE), QC_ASIC2)
endif
# End of ($(LAYER), HAL)

#*******************************************************************************
#                               DRIVERS LAYER
#*******************************************************************************
ifeq ($(LAYER), DRIVER)
# QC application for IT phase
QC_HAL_DIR   = $(DEV_BASE)/qc_platform/02_IT/platform/asic/hal/app
# Add source searching paths
VPATH        += $(QC_HAL_DIR)/source
# Add compiling include paths
CFLAGS       += -I $(QC_HAL_DIR)/include
# Add drivers paths
QC_APPS_DIR   = $(DEV_BASE)/qc_platform/02_IT/platform/drivers/app
# Add source searching paths
VPATH        += $(QC_APPS_DIR)/source         \
                $(QC_APPS_DIR)/syscfg/source  \
                $(QC_APPS_DIR)/sysctrl/source
# Add compiling include paths
CFLAGS       += -I $(QC_APPS_DIR)/include            \
                -I $(QC_APPS_DIR)/syscfg/include     \
                -I $(QC_APPS_DIR)/sysctrl/include    \
                -I $(DEV_BASE)/platform/driver/src
# Add application objects
OBJ_QC       += $(COMPILER)/it_drv_app.o \
				$(COMPILER)/it_drv_temp_sensor.o \
				$(COMPILER)/it_drv_i2cm.o \
				$(COMPILER)/it_drv_ina231.o      \
				$(COMPILER)/it_drv_hall_sensor.o \
                $(COMPILER)/bash.o       \
                $(COMPILER)/com_srv.o    \
                $(COMPILER)/i2c_cmd.o    \
                $(COMPILER)/spi_cmd.o    \
                $(COMPILER)/storage.o    \
                $(COMPILER)/sys_call.o   \
                $(COMPILER)/sys_ctrl.o   \
                $(COMPILER)/sys_tasks.o  \
                $(COMPILER)/sys_cfg.o    \
                $(COMPILER)/mailbox.o    \
                $(COMPILER)/message.o    \
                $(COMPILER)/mspu.o       \
                $(COMPILER)/time.o
# Enable RTOS
CFLAGS       += -DENABLE_RTOS
# Build option for stage 1
ifeq ($(FIRMWARE), QC_ASIC1)
CFLAGS       += -DQC_ASIC1
# End of ($(FIRMWARE), QC_ASIC1)
endif

# Build option for stage 2
ifeq ($(FIRMWARE), QC_ASIC2)
# Driver verification objects
#OBJ_QC       += $(COMPILER)/it_drv_ar1335.o     \
                $(COMPILER)/it_drv_flash.o       \
                $(COMPILER)/it_drv_i2cm.o        \
                $(COMPILER)/it_drv_ina231.o      \
                $(COMPILER)/it_drv_spi_slave.o   \
                $(COMPILER)/it_drv_timer.o       \
                $(COMPILER)/it_drv_trace_timer.o \
                $(COMPILER)/it_drv_camera.o      \
                $(COMPILER)/it_drv_hall_sensor.o \
                $(COMPILER)/it_drv_i2c_slave.o   \
                $(COMPILER)/it_drv_rtc.o         \
                $(COMPILER)/it_drv_temp_sensor.o \
                $(COMPILER)/it_drv_timestamp.o   \
                $(COMPILER)/it_drv_vcm.o
# QC common objects
QC_LIBS_OBJ  += $(COMPILER)/qc_assert.o        \
				$(COMPILER)/it_log_swapper.o   \
				$(COMPILER)/qc_common.o        \
               
# Force enable assert
CFLAGS       += -D_USE_FULL_ASSERT
CFLAGS       += -DQC_ASIC2
endif
# End of ($(FIRMWARE), QC_ASIC2)
endif
# End of ($(LAYER), DRIVER)
#*******************************************************************************
#                                BOOTLOADER
#*******************************************************************************
ifeq ($(FIRMWARE), QC_ASIC1) 
QC_BOOT_LOADER= $(DEV_BASE)/qc_platform/02_IT/platform/bootloader
VPATH        += $(QC_BOOT_LOADER)/source
CFLAGS       += -I $(QC_BOOT_LOADER)/include
OBJ_BOOT     += $(COMPILER)/bootloader.o
endif
#*******************************************************************************
# 								END OF FILE
#*******************************************************************************
