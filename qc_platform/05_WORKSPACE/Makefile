#*******************************************************************************
# @File's name	: Makefile
# @Company		: Light Co.
# @Author 		: Fitz Nguyen -- Light's QC team
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
.PHONY:	clean S1S2_ASIC1.bin S1S2_ASIC2.bin S1S2_ASIC3.bin

# Get information
QC_WORKSPACE   = ${shell pwd}
DEV_BASE       = $(QC_WORKSPACE)/../..
QC_TOOLS_PATH = $(DEV_BASE)/qc_platform/04_COMMON/qc_tools
HOST_OS       = $(shell uname -m)
DEBUG_TOOL    = /opt/t32/bin/pc_linux64/t32marm
DEBUG_FLAGS  += -s
DEBUG_SCRIPT  = $(QC_WORKSPACE)/../04_COMMON/qc_tools/*.cmm
UTILITIES     = $(QC_WORKSPACE)/../04_COMMON/qc_tools/asic_utilities.py
FTDI_BASE     = $(DEV_BASE)/FTDI
QC_LINKER     = $(DEV_BASE)/qc_platform/02_IT/platform/asic/hal/app/linker
DEV_LINKER    = $(DEV_BASE)/platform/asic/cortex-r4/startup

SB_CREATE_FW  = $(QC_WORKSPACE)/../04_COMMON/qc_tools/create_FW.sh
SB_TOOLS      = $(QC_WORKSPACE)/../04_COMMON/sb_tools
SB_MERGE_IMG  = $(QC_WORKSPACE)/../04_COMMON/qc_tools/mergebin.py
# Optional binary utilities
SB_BIN_UNITY  = $(QC_WORKSPACE)/../04_COMMON/qc_tools/boot_utilities.bin
ISP_APP_PATH  = $(QC_WORKSPACE)/../04_COMMON/isp_app
ISP_APP       = prg_app_linux
AUTO_SCR_PATH = $(QC_WORKSPACE)/../04_COMMON/qc_tools/test_app
AUTO_SCR      = ""
HOST_FLASH_PATH = $(QC_WORKSPACE)/../04_COMMON/host_flashing
HOST_FLASH      = isp_app_p1_5_notrst
HOST_FLASH_CFG  = prog_app.config
ifeq ($(FPGA), )
FPGA          = $(DEV_BASE)/FPGA/*fpga*.bit
endif
#Generate make file from development
all: build

build: bin

# Build HAL Integration test
ifeq ($(LAYER), HAL)
bin:
	@if [ -d bin ];  \
	then             \
		make clean;  \
	fi
	@mkdir bin
	@mv     $(DEV_BASE)/platform/asic/cortex-r4/startup/AsicFwS2/linker.ld ./swap_ld/hal/AsicFwS2.ldbk
	@cp -f  ./swap_ld/hal/AsicFwS2.ld $(DEV_BASE)/platform/asic/cortex-r4/startup/AsicFwS2/linker.ld
	@mv     $(DEV_BASE)/AsicFwS1/Makefile ./swap_mak/hal/AsicFwS1.makbk
	@mv     $(DEV_BASE)/AsicFwS2/Makefile ./swap_mak/hal/AsicFwS2.makbk
	@cp -f  ./swap_mak/hal/AsicFwS1.mak $(DEV_BASE)/AsicFwS1/Makefile
	@cp -f  ./swap_mak/hal/AsicFwS2.mak $(DEV_BASE)/AsicFwS2/Makefile
	@cd     $(DEV_BASE) ; make DEV_BASE=$(DEV_BASE)
	@cp     $(DEV_BASE)/Output/*.* bin
	@cp -f  ./swap_mak/hal/AsicFwS1.makbk $(DEV_BASE)/AsicFwS1/Makefile
	@cp -f  ./swap_mak/hal/AsicFwS2.makbk $(DEV_BASE)/AsicFwS2/Makefile
	@mv     ./swap_ld/hal/AsicFwS2.ldbk   $(DEV_BASE)/platform/asic/cortex-r4/startup/AsicFwS2/linker.ld 
endif

# Build drivers Integration test
ifeq ($(LAYER), DRIVER)
bin:
	@if [ -d bin ];  \
	then             \
		make clean;  \
	fi
	@mkdir bin
	@mv    $(DEV_BASE)/AsicFwS1/Makefile ./swap_mak/drivers/AsicFwS1.makbk
	@mv    $(DEV_BASE)/AsicFwS2/Makefile ./swap_mak/drivers/AsicFwS2.makbk
	@cp -f ./swap_mak/drivers/AsicFwS1.mak $(DEV_BASE)/AsicFwS1/Makefile
	@cp -f ./swap_mak/drivers/AsicFwS2.mak $(DEV_BASE)/AsicFwS2/Makefile
	@cd    $(DEV_BASE) ; make DEV_BASE=$(DEV_BASE)
	@cp    $(DEV_BASE)/Output/*.* bin
	@cp -f ./swap_mak/drivers/AsicFwS1.makbk $(DEV_BASE)/AsicFwS1/Makefile
	@cp -f ./swap_mak/drivers/AsicFwS2.makbk $(DEV_BASE)/AsicFwS2/Makefile
endif
#*******************************************************************************
#                       FLASHING NONE SECURE BOOT
#*******************************************************************************
ifeq ($(SBOOT_ENABLE), )
# Target to flashing stage 1 firmware
FLASH_S1_ASIC:  bin             \
                $(UTILITIES)
	@echo "Flashing stage 1 ASIC firmware..."
	@python $(UTILITIES) --flash --firmware $(FTDI_BASE) $(QC_WORKSPACE)/bin/AsicFwS1.bin
# Target to flashing stage 2 firmware ASIC 1
FLASH_S2_ASIC1:    bin          \
                $(UTILITIES)
	@echo "Flashing stage 2 ASIC firmware 1..."
	@python $(UTILITIES) --flash --firmware $(FTDI_BASE) $(QC_WORKSPACE)/bin/AsicFwS1.bin $(QC_WORKSPACE)/bin/AsicFwS2_ASIC1.bin
# Target to flashing stage 2 firmware ASIC 2
FLASH_S2_ASIC2:    bin          \
                $(UTILITIES)
	@echo "Flashing stage 2 ASIC firmware 2..."
	@python $(UTILITIES) --flash --firmware $(FTDI_BASE) $(QC_WORKSPACE)/bin/AsicFwS1.bin $(QC_WORKSPACE)/bin/AsicFwS2_ASIC2.bin
# Target to flashing firmware ASIC 3
FLASH_S2_ASIC3:    bin          \
                $(UTILITIES)
	@echo "Flashing stage 2 ASIC firmware 3..."
	@python $(UTILITIES) --flash --firmware $(FTDI_BASE) $(QC_WORKSPACE)/bin/AsicFwS1.bin $(QC_WORKSPACE)/bin/AsicFwS2_ASIC3.bin
# Target to flashing FPGA
FLASH_FPGA:     $(FPGA)         \
                $(UTILITIES)
	@echo "Flashing FPGA..."
	@python $(UTILITIES) --flash --fpga $(FTDI_BASE) $(FLASH_MODE) $(FPGA)
endif
#*******************************************************************************
#                           FLASHING SECURE BOOT
#*******************************************************************************
ifeq ($(SBOOT_ENABLE),TRUE)
ifeq ($(ISP_BOOT), ANDROID)
ASIC123: S1S2_ASIC1.bin S1S2_ASIC2.bin S1S2_ASIC3.bin
	sudo adb wait-for-devices
	adb root
	sleep 3
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH) /data
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH_CFG) /data
	adb push ./bin/S1S2_ASIC1.bin /data
	adb push ./bin/S1S2_ASIC2.bin /data
	adb push ./bin/S1S2_ASIC3.bin /data
	adb shell "gpio_set -n 130 -s 0"
	sleep 1
	adb shell "gpio_set -n 130 -s 1"
	sleep 1
	adb shell "gpio_set -n 128 -s 0"
	sleep 1
	adb shell "gpio_set -n 128 -s 1"
	sleep 1
	adb shell "cd /data; chmod +x isp_app_p1_5_notrst; ./isp_app_p2 -i spi -m program -f S1S2_ASIC1.bin,S1S2_ASIC2.bin,S1S2_ASIC3.bin"
endif
ifeq ($(ISP_BOOT), UART)
FLASH_S2_ASIC1: S1S2_ASIC1.bin $(ISP_APP_PATH)/$(ISP_APP) uart_boot
	@chmod 777 $(ISP_APP_PATH)/$(ISP_APP)
	cd $(ISP_APP_PATH) && sudo ./$(ISP_APP) -i usart -m program -f $(QC_WORKSPACE)/bin/S1S2_ASIC1.bin
endif
ifeq ($(ISP_BOOT), ANDROID)
FLASH_S2_ASIC1: S1S2_ASIC1.bin $(HOST_FLASH_PATH)/$(HOST_FLASH) $(HOST_FLASH_PATH)/$(HOST_FLASH_CFG)
	adb kill-server
	sudo adb root
	adb wait-for-devices
	adb shell "gpio_set –n 128 –s 0"
	adb shell "gpio_set –n 128 –s 1"
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH) /data
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH_CFG) /data
	adb push $(QC_WORKSPACE)/bin/S1S2_ASIC1.bin /data
	adb shell "cd /data; chmod +x isp_app_p1_5_notrst; ./$(HOST_FLASH) -i spi -m program -f S1S2_ASIC1.bin"
endif
ifeq ($(ISP_BOOT), NONE)
FLASH_S2_ASIC1: S1S2_ASIC1.bin $(SB_BIN_UNITY)
	@chmod 777 $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH
	sudo $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH $(QC_WORKSPACE)/bin/S1S2_ASIC1.bin  70000
	sudo $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH $(SB_BIN_UNITY)                     60000
endif

ifeq ($(ISP_BOOT), UART)
FLASH_S2_ASIC2: S1S2_ASIC2.bin $(ISP_APP_PATH)/$(ISP_APP) uart_boot
	@chmod 777 $(ISP_APP_PATH)/$(ISP_APP)
	cd $(ISP_APP_PATH) && sudo ./$(ISP_APP) -i usart -m program -f $(QC_WORKSPACE)/bin/S1S2_ASIC2.bin
endif
ifeq ($(ISP_BOOT), ANDROID)
FLASH_S2_ASIC2: S1S2_ASIC2.bin $(HOST_FLASH_PATH)/$(HOST_FLASH) $(HOST_FLASH_PATH)/$(HOST_FLASH_CFG)
	adb kill-server
	sudo adb root
	adb shell "gpio_set –n 128 –s 0"
	adb shell "gpio_set –n 128 –s 1"
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH) /data
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH_CFG) /data
	adb push $(QC_WORKSPACE)/bin/S1S2_ASIC2.bin /data
	adb shell "cd /data; chmod +x isp_app_p1_5_notrst; ./$(HOST_FLASH) -i spi -m program -f S1S2_ASIC2.bin"
endif
ifeq ($(ISP_BOOT), NONE)
FLASH_S2_ASIC2: S1S2_ASIC2.bin $(SB_BIN_UNITY)
	@chmod 777 $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH
	sudo $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH $(QC_WORKSPACE)/bin/S1S2_ASIC2.bin  70000
	sudo $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH $(SB_BIN_UNITY)                     60000
endif

ifeq ($(ISP_BOOT), USED)
FLASH_S2_ASIC3: S1S2_ASIC3.bin $(ISP_APP_PATH)/$(ISP_APP) uart_boot
	@chmod 777 $(ISP_APP_PATH)/$(ISP_APP)
	cd $(ISP_APP_PATH) && sudo ./$(ISP_APP) -i usart -m program -f $(QC_WORKSPACE)/bin/S1S2_ASIC3.bin
endif
ifeq ($(ISP_BOOT), ANDROID)
FLASH_S2_ASIC3: S1S2_ASIC3.bin $(HOST_FLASH_PATH)/$(HOST_FLASH) $(HOST_FLASH_PATH)/$(HOST_FLASH_CFG)
	adb kill-server
	sudo adb root
	adb shell "gpio_set –n 128 –s 0"
	adb shell "gpio_set –n 128 –s 1"
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH) /data
	adb push $(HOST_FLASH_PATH)/$(HOST_FLASH_CFG) /data
	adb push $(QC_WORKSPACE)/bin/S1S2_ASIC3.bin /data
	adb shell "cd /data; chmod +x isp_app_p1_5_notrst; ./$(HOST_FLASH) -i spi -m program -f S1S2_ASIC3.bin"
endif
ifeq ($(ISP_BOOT), NONE)
FLASH_S2_ASIC3: S1S2_ASIC3.bin $(SB_BIN_UNITY)
	@chmod 777 $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH
	sudo $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH $(QC_WORKSPACE)/bin/S1S2_ASIC3.bin  70000
	sudo $(FTDI_BASE)/FTDI2232/Linux/x86_64/FT2232_FLASH $(SB_BIN_UNITY)                     60000
endif

S1S2_ASIC1.bin: bin $(SB_TOOLS) $(SB_CREATE_FW) $(SB_MERGE_IMG) $(SB_BIN_UNITY)
	@chmod 777 $(SB_CREATE_FW)
	@$(SB_CREATE_FW) $(QC_WORKSPACE)/bin/AsicFwS1.bin $(QC_WORKSPACE)/bin/AsicFwS2_ASIC1.bin $(SB_TOOLS) $(SB_MERGE_IMG) $(QC_WORKSPACE)/bin S1S2_ASIC1.bin

S1S2_ASIC2.bin: bin $(SB_TOOLS) $(SB_CREATE_FW) $(SB_MERGE_IMG) $(SB_BIN_UNITY)
	@chmod 777 $(SB_CREATE_FW)
	@$(SB_CREATE_FW) $(QC_WORKSPACE)/bin/AsicFwS1.bin $(QC_WORKSPACE)/bin/AsicFwS2_ASIC2.bin $(SB_TOOLS) $(SB_MERGE_IMG) $(QC_WORKSPACE)/bin S1S2_ASIC2.bin

S1S2_ASIC3.bin: bin $(SB_TOOLS) $(SB_CREATE_FW) $(SB_MERGE_IMG) $(SB_BIN_UNITY)
	@chmod 777 $(SB_CREATE_FW)
	@$(SB_CREATE_FW) $(QC_WORKSPACE)/bin/AsicFwS1.bin $(QC_WORKSPACE)/bin/AsicFwS2_ASIC3.bin $(SB_TOOLS) $(SB_MERGE_IMG) $(QC_WORKSPACE)/bin S1S2_ASIC3.bin
endif
#*******************************************************************************
# Target to reset ASIC at system level
reset: $(UTILITIES)
	@echo "Reset FPGA..."
	@python $(UTILITIES) --gpio --sys_reset $(FTDI_BASE)
# Target to execute test
ifeq ($(LAYER), HAL)
execute:
	@cd $(AUTO_SCR_PATH); sudo python hal_test.py
endif
ifeq ($(LAYER), DRIVER)
execute:
	
endif
# Target for report
report: execute

#Target to activate SPI upstream CS
ENABLE_SPI_UPSTREAM: $(UTILITIES)
	@echo "Invoke I2C command"
	@python $(UTILITIES) --i2c_cmd --enable_spi_upstream $(FTDI_BASE)

# Target for enable boot UART via ISP app
uart_boot: $(UTILITIES)
	@echo "Enable UART boot mode..."
	@python $(UTILITIES) --gpio --uart_boot $(FTDI_BASE)

#*******************************************************************************
#                               CLEAN
#*******************************************************************************
# For HAL
ifeq ($(LAYER), HAL)
clean:
	@rm -rf bin
	@if [ -f ./swap_mak/hal/AsicFwS1.makbk ]; 									\
	then 																		\
		mv ./swap_mak/hal/AsicFwS1.makbk $(DEV_BASE)/AsicFwS1/Makefile;			\
	fi
	@if [ -f ./swap_mak/hal/AsicFwS2.makbk ];									\
	then																		\
		mv ./swap_mak/hal/AsicFwS2.makbk $(DEV_BASE)/AsicFwS2/Makefile;			\
	fi
	@cd    $(DEV_BASE) ; make clean
endif
# For drivers
ifeq ($(LAYER), DRIVER)
clean:
	@rm -rf bin
	@if [ -f ./swap_mak/drivers/AsicFwS1.makbk ]; 								\
	then 																		\
		mv ./swap_mak/drivers/AsicFwS1.makbk $(DEV_BASE)/AsicFwS1/Makefile;		\
	fi
	@if [ -f ./swap_mak/drivers/AsicFwS2.makbk ];								\
	then																		\
		mv ./swap_mak/drivers/AsicFwS2.makbk $(DEV_BASE)/AsicFwS2/Makefile;		\
	fi
	@if [ -f ./swap_ld/drivers/AsicFwS1.ldbk ]; 								\
	then 																		\
		mv ./swap_ld/drivers/AsicFwS1.ldbk $(DEV_LINKER)/AsicFwS1/linker.ld;	\
	fi
	@if [ -f ./swap_ld/drivers/AsicFwS2.ldbk ]; 								\
	then 																		\
		mv ./swap_ld/drivers/AsicFwS2.ldbk $(DEV_LINKER)/AsicFwS2/linker.ld;	\
	fi
	@cd    $(DEV_BASE) ; make clean
endif
#*******************************************************************************
# 								END OF FILE
#*******************************************************************************