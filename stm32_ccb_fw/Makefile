################################################################################
# Makefile for the LightCCB_STM32F429_Firmware project.
#
# Created by Duong Vuong.
#
# For use on the Light_CCB board, uses the GCC toolchain for ARM. 
################################################################################

################################################################################
#
# Compile target definition
#
################################################################################
# Derive the project name from the current folder.
ifeq ($(TARGET),)
TARGET        = $(OBJDIR)/LightCCB
endif
# Set up CPU and FPU Usage.
USE_HARD_FPU	= 1
MCU				= cortex-m4
RELEASE_VER		= 0

# Start of default section.
# List all default C defines here, like _DEBUG=1. -D is automatically added.
DCDEFS+=STM32F429xx
DCDEFS+=HSE_VALUE=8000000

DCDEFS+=RELEASE=$(RELEASE_VER)
DCDEFS+=BUILD_TIME=__TIMESTAMP__
DCDEFS+=LOG_LEVEL=3

ifeq ($(BOARD_VERSION), )
DCDEFS+=BOARD_VERSION_P1
else

ifeq ($(BOARD_VERSION), p1)
DCDEFS+=BOARD_VERSION_P1
endif
ifeq ($(BOARD_VERSION), p1.1)
DCDEFS+=BOARD_VERSION_P1_1
endif

endif
ifeq ($(RELEASE_VER), 0)
DCDEFS+=USE_FULL_ASSERT=1
endif

# List all default ASM defines here, like _DEBUG=1 -D is automatically added.
#
# List all default directories to look for include files here
DINCDIR+= $(STM32F4xx)/Inc
DINCDIR+= $(STM32F4xx)/Inc/CMSIS
DINCDIR+= $(FREERTOS)/include
DINCDIR+= $(FREERTOS)/portable/GCC/ARM_CM4F
DINCDIR+= $(PERIPH_DRIVER)/Inc
DINCDIR+= $(LIGHTCCB_API)
DINCDIR+= $(LIGHTCCB_DRV)/Inc
DINCDIR+= $(LIGHTCCB_APP)/Inc
DINCDIR+= $(LIGHTCCB_APP)/Inc/Config


# List the default directory to look for the libraries here
DLIBDIR =
# List all default libraries here
DLIBS = 

################################################################################

################################################################################
#
# Source directories
#	Directory source code of LightCCB firmware
#
################################################################################
# Relative path to LightCCB-FW root.
ifeq ($(LIGHTCCB_FW),)
LIGHTCCB_FW=
endif
# STM32F4xx CMSIS Path
STM32F4xx=Drivers/STM32F4xx
# Standard Peripheral Driver Path
PERIPH_DRIVER=Drivers/Periph_Driver
# FreeRTOS
FREERTOS=FreeRTOS/Source
# LightCCB Application Path
LIGHTCCB_APP=Light
# LightCCB board BSP path
LIGHTCCB_DRV=Drivers/LightCCB
# LightCCB board BSP path
LIGHTCCB_API=Drivers/Api
# LightCCB flash scripts
LIGHTCCB_SCRIPTS=$(STM32F4xx)/Scripts
# Compiled Output Path
OBJDIR = Output
LSTDIR = Lst

#
# Create output directories
#
$(shell mkdir -p $(OBJDIR)/$(LSTDIR) 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/$(STM32F4xx)/Startup 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/$(FREERTOS)/portable/MemMang 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/$(FREERTOS)/portable/GCC/ARM_CM4F 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/$(PERIPH_DRIVER)/Src 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/$(LIGHTCCB_DRV)/Src 2>/dev/null)
$(shell mkdir -p $(OBJDIR)/$(LIGHTCCB_APP)/Src 2>/dev/null)

# Include the dependency files
$(wildcard $(OBJDIR)/*.d)

################################################################################
#
#	Specifies compiler settings for LightCCB board
#
################################################################################

#
# Linker Scripts.
#
LDSCRIPT = $(STM32F4xx)/Linker/stm32f4xxxg_flash.ld
#
# ASM source files here
#
ASRC+= $(STM32F4xx)/Startup/startup_stm32f429xx.s
#
# C source files here
#
# List of FreeRTOS files.
SRC+= $(FREERTOS)/tasks.c
SRC+= $(FREERTOS)/queue.c
SRC+= $(FREERTOS)/list.c
SRC+= $(FREERTOS)/croutine.c
SRC+= $(FREERTOS)/portable/GCC/ARM_CM4F/port.c 
SRC+= $(FREERTOS)/timers.c
#Memory management
SRC+= $(FREERTOS)/portable/MemMang/heap_2.c

# List of gcc system functions call files.
SRC+= $(STM32F4xx)/Startup/syscalls.c
# List of all the board related files.
SRC+= $(PERIPH_DRIVER)/Src/misc.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_gpio.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_rcc.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_system.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_usart.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_i2c.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_flash.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_spi.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_dma.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_tim.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_exti.c
SRC+= $(PERIPH_DRIVER)/Src/stm32f4xx_syscfg.c

# List of all LightCCB driver files.
SRC+= $(LIGHTCCB_DRV)/Src/drv_usart.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_i2c_ex.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_cci.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_spi_slave.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_flash_ex.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_piezo.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_piezo_hall.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_piezo_hwinit.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_piezo_pwm.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_piezo_move.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_vcm.c
SRC+= $(LIGHTCCB_DRV)/Src/drv_cpld.c

# List of all application files.
SRC+= $(LIGHTCCB_APP)/Src/platform.c
SRC+= $(LIGHTCCB_APP)/Src/log.c
SRC+= $(LIGHTCCB_APP)/Src/mems.c
SRC+= $(LIGHTCCB_APP)/Src/cam_ctrl.c
SRC+= $(LIGHTCCB_APP)/Src/af_ctrl.c
SRC+= $(LIGHTCCB_APP)/Src/ucid.c
SRC+= $(LIGHTCCB_APP)/Src/fpga.c
SRC+= $(LIGHTCCB_APP)/Src/Interpolators.c
SRC+= $(LIGHTCCB_APP)/Src/cam_eeprom.c
SRC+= $(LIGHTCCB_APP)/Src/main.c

################################################################################

################################################################################
#
#	Compiler settings for ARM cortex-m4
#
################################################################################
## Compiler programs.
ifeq ($(TRGT),)
ifeq ($(OS),Windows_NT)
TRGT = arm-none-eabi-
else ifeq ($(shell uname), Linux)
#TRGT = /opt/gcc-arm-none-eabi/bin/arm-none-eabi-
TRGT = arm-none-eabi-
else
TRGT = arm-none-eabi-
endif
endif
# Program names.
CC		= $(TRGT)gcc
CP		= $(TRGT)objcopy
AS		= $(TRGT)gcc -x assembler-with-cpp
OD		= $(TRGT)objdump
SZ  	= $(TRGT)size
NM  	= $(TRGT)nm
OPENOCD = openocd
DOXYGEN	= doxygen

# Compile Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
# 
OPT = 3

################################################################################

################################################################################
#
#	Flags
#		Linker, compiler, assembler flags for LightCCB board..
#
################################################################################

#
# Define FPU settings here
#
ifeq ($(USE_HARD_FPU), 0)
# Specifying ‘soft’ causes GCC to generate output containing library calls for floating-point operations
FPU+= -mfloat-abi=soft
else
# ‘Hard’ allows generation of floating-point instructions and uses FPU-specific calling conventions.
FPU+=-mfloat-abi=hard 
# Specifies what floating-point hardware (or hardware emulation) is available on the target. 
FPU+=-mfpu=fpv4-sp-d16
# __FPU_USED sets registry to enable FPU.
FPU+=-D__FPU_USED=1
endif

#
# Add prefixe to defines
#
INCDIR  = $(patsubst %,-I%,$(sort $(DINCDIR) $(UINCDIR)))
LIBDIR  = $(patsubst %,-L%,$(sort $(DLIBDIR) $(ULIBDIR)))

#
# Add -D prefix to defines.
#
CDEFS=$(patsubst %,-D%,$(sort $(DCDEFS) $(UCDEFS)))
ADEFS=$(patsubst %,-D%,$(sort $(DADEFS) $(UADEFS)))

#
# Combine Libraries. 
#
LIBS   +=$(DLIBS)
LIBS   +=$(ULIBS)

#
# MCU Flags
#
MCUFLAGS+=-mthumb
MCUFLAGS+=-mcpu=$(MCU)
MCUFLAGS+=$(FPU)

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler
#  -adhlns:   create listing
#----------------------------------------------------

#
# Assembly Flags
#
ASFLAGS+=$(MCUFLAGS) 
ASFLAGS+=$(ADEFS)
ASFLAGS+=-Wa,-amhls=$(OBJDIR)/$(LSTDIR)/$(*F).lst
ASFLAGS+=-O$(OPT)

#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
#----------------------------------------------------

#
# C Flags
#CPFLAGS+=-DSTM_CCB
CPFLAGS+=-DP1_CCB
CPFLAGS+=$(MCUFLAGS)
CPFLAGS+=$(CDEFS)
CPFLAGS+=-O$(OPT)
CPFLAGS+=-Wall
CPFLAGS+=-std=gnu99
CPFLAGS+=-gdwarf-2
CPFLAGS+=-Werror
#CPFLAGS+=-Wextra
#CPFLAGS+=-Wpointer-arith
#CPFLAGS+=-Wstrict-prototypes
#CPFLAGS+=-Winline
#CPFLAGS+=-Wunreachable-code
#CPFLAGS+=-Wundef
CPFLAGS+=-Wstrict-prototypes
CPFLAGS+=-ffunction-sections
CPFLAGS+=-fdata-sections
CPFLAGS+=-Wa,-adhlns=$(OBJDIR)/$(LSTDIR)/$(*F).lst
# Optimize use of the single-precision FPU
#
CPFLAGS+=-fsingle-precision-constant

CPFLAGS+=-MD
CPFLAGS+=-MP
CPFLAGS+=-MF
CPFLAGS+=$(OBJDIR)/$(LSTDIR)/$(*F).d

#---------------- Linker Options --------------------
#  -Wl,...:     tell GCC to pass this to linker
#    -Map:      create map file
#    --cref:    add cross reference to  map file
#----------------------------------------------------

#
# Linker Flags
#
LDFLAGS+=$(MCUFLAGS)
LDFLAGS+=-nostartfiles
LDFLAGS+=-T$(LDSCRIPT)
LDFLAGS+=-Wl,-Map=$(TARGET).map,--cref,--gc-sections,--no-warn-mismatch
LDFLAGS+=$(LIBDIR)

################################################################################

################################################################################
#
# Paths
#	Paths for LightCCB board
#
################################################################################
## Flashing Program & Flags
FLASH      = st-flash
FLASHFLAGS+= write $(TARGET).bin 0x08000000

#FLASH      = openocd
#FLASHFLAGS+= -s /opt/openocd/share/openocd/scripts
#FLASHFLAGS+= -f board/stm32f4discovery.cfg
#FLASHFLAGS+= -f $(LIGHTCCB_SCRIPTS)/stm32f4_flash.cfg
#FLASHFLAGS+= -c "stm32f4_Flash $(TARGET).elf"
################################################################################

################################################################################
#
# Makefile targets.
#	Targets for LightCCB board
#
################################################################################

#
# Get objects 
#
OBJS	+= $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SRC))))
OBJS	+= $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(ASRC))))

# Make all
all: gccversion updatetime build showsize

# Update build time
updatetime:
	@touch $(LIGHTCCB_APP)/Src/main.c

# Build process
build: elf hex lss sym bin

elf: $(TARGET).elf
hex: $(TARGET).hex
bin: $(TARGET).bin
lss: $(TARGET).lss
sym: $(TARGET).sym

# Creating Doxygen documentation
doxygen:
	@echo
	@echo Creating Doxygen documentation
	@$(DOXYGEN)
	
# Display compiler version information
gccversion: 
	@$(CC) --version

# Show the final program size
showsize: elf
	@echo
	@$(SZ) $(TARGET).elf 2>/dev/null

# Compile: create object files from C source files
$(OBJDIR)/%.o : %.c
	@echo
	@echo Compiling C: $<
	$(CC) -c $(CPFLAGS) $(INCDIR) $< -o $@

# Assemble: create object files from assembler source files
$(OBJDIR)/%.o : %.s
	@echo
	@echo Assembling: $<
	$(AS) -c $(ASFLAGS) $< -o $@
	
# Link: create ELF output file from object files
.SECONDARY: $(TARGET).elf
.PRECIOUS:  $(OBJS)
$(TARGET).elf: $(OBJS)
	@echo
	@echo Linking: $@
	$(CC) $^ $(LDFLAGS) $(LIBS) -o $@

# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo Creating hex file: $@
	$(CP) -O ihex $< $@

# Create bin file :
%.bin: %.elf
	@echo
	@echo Creating bin file: $@
	$(CP) -O binary $< $@

# Flash hex file to the board.
flash: $(TARGET).hex
	@echo Flashing $(TARGET)
	$(FLASH) $(FLASHFLAGS)

clean:
	@echo Cleaning project:
	rm -rf $(OBJDIR)

# Create extended listing file from ELF output file
%.lss: %.elf
	@echo
	@echo Creating Extended Listing: $@
	$(OD) -h -S -z $< > $@

# Create a symbol table from ELF output file
%.sym: %.elf
	@echo
	@echo Creating Symbol Table: $@
	$(NM) -n $< > $@

# Listing of phony targets
.PHONY: all build flash clean \
        doxygen elf lss sym \
        showsize gccversion
################################################################################
