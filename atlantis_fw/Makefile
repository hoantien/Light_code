#******************************************************************************
#
# The compiler to be used.
#
#******************************************************************************

export PROJECT_ROOT		?=${PWD}

export OUTPUT_DIR		?=${PROJECT_ROOT}/Output
export OBJ_BUILD		?=${OUTPUT_DIR}/build
export OPTIMIZE			?=s
export CPU_NAME			?=cortex-r4
export CERT_NAME		?="lightcert"
export CERT_PASS		?="lightco"
export CERT_VERSION		?="0x00000001"
export CERT_SIGN		?="/distinguishedName=light.co"
#******************************************************************************
#
# Bootflow State
#
#******************************************************************************
export BOOT_S1_DIR		?=AsicFwS1
export BOOT_S2_DIR		?=AsicFwS2

##############################################################################
#
#   Platform Include Directories
#
##############################################################################

export DIR_PLATFORM		?=${PROJECT_ROOT}/platform
export DIR_ASIC			?=${DIR_PLATFORM}/asic
export DIR_DEVICES		?=${DIR_PLATFORM}/driver
export DIR_HAL			?=${DIR_ASIC}/hal
export DIR_OS			?=${DIR_PLATFORM}/os
export STARTUP_DIR		?=${DIR_ASIC}/${CPU_NAME}/startup

## APPLICATION
export CFLAGS_INC+=-I ${PROJECT_ROOT}/${BOOT_S2_DIR}/include

## ASIC
CFLAGS_INC+=-I ${DIR_ASIC}/${CPU_NAME}/include

## HAL
CFLAGS_INC+=-I ${DIR_HAL}/include

## DEVICE DRIVER
CFLAGS_INC+=-I ${DIR_DEVICES}/include

## OS
CFLAGS_INC+=-I ${DIR_PLATFORM}/common
CFLAGS_INC+=-I ${DIR_PLATFORM}/common/sblib
CFLAGS_INC+=-I ${DIR_PLATFORM}/common/lightheader
CFLAGS_INC+=-I ${DIR_PLATFORM}/common/lightheader/include
CFLAGS_INC+=-I ${DIR_PLATFORM}/common/lightheader/include/protobuf-c
CFLAGS_INC+=-I ${DIR_PLATFORM}/common/lightheader/src
CFLAGS_INC+=-I ${DIR_OS}/Source/include
CFLAGS_INC+=-I ${DIR_OS}/Source/portable/GCC/${CPU_NAME}
ifdef BUILD_EVT
CFLAGS_INC+=-DREWORK
endif


##############################################################################
#
#   Source directories
#
##############################################################################

## ASIC
export VPATH+=:${DIR_ASIC}/${CPU_NAME}
VPATH+=:${DIR_ASIC}/${CPU_NAME}/startup


## DEVICES
VPATH+=:${DIR_DEVICES}/src

## HAL
VPATH+=:${DIR_HAL}/src
VPATH+=:${DIR_PLATFORM}/test

## OS
VPATH+=:${DIR_OS}/Source
VPATH+=:${DIR_OS}/Source/portable/GCC/${CPU_NAME}
VPATH+=:${DIR_OS}/Source/portable/MemMang
VPATH+=:${DIR_PLATFORM}/common
VPATH+=:${DIR_PLATFORM}/common/lightheader/src



##############################################################################
#
#   The compiler to be used.
#
##############################################################################


## Floating Point Uinit
export FPU_DEF+=-mfloat-abi=soft
FPU_DEF+=-mfpu=vfpv3-d16


export ARCH_DEF=-march=armv7-r
# Not use this option in current, will fix it in future
# ARCH_DEF+=-mthumb


## Define CPU settings here
export CPU_DEF=-mcpu=${CPU_NAME}
CPU_DEF+=${ARCH_DEF} ${FPU_DEF}
CPU_DEF+=-O${OPTIMIZE}
CPU_DEF+=-MD -MP
#CPU_DEF+=-fsingle-precision-constant // gsun: this makes <algorithm> fail. comment it out for now
CPU_DEF+=-Wall
CPU_DEF+=-Werror
CPU_DEF+=-gdwarf-2
CPU_DEF+=-ffunction-sections
CPU_DEF+=-fdata-sections
CPU_DEF+=-DASIC1=1 -DASIC2=2 -DASIC3=3

export CPU_DEF_C=-std=gnu99
CPU_DEF_C+=-Wstrict-prototypes

export CPU_DEF_CC=-fno-rtti -std=c++11

## ASIC BUILD DEFINE
export CFLAGS_DEF+=-D_USE_FULL_ASSERT

## The flags passed to the assembler.
export AFLAGS=-mcpu=${CPU_NAME}
AFLAGS+=${ARCH_DEF} ${FPU_DEF}
AFLAGS+=-O${OPTIMIZE}
AFLAGS+=-x assembler-with-cpp

## The command for calling the compiler.
export CC=arm-none-eabi-gcc
export PP=arm-none-eabi-g++
## The flags passed to the compiler.
export CFLAGS+= ${CFLAGS_INC} ${CFLAGS_DEF} ${CPU_DEF} ${CPU_DEF_C}
export CCFLAGS+= ${CFLAGS_INC} ${CFLAGS_DEF} ${CPU_DEF} ${CPU_DEF_CC}

## The command for calling the linker.
export LD=arm-none-eabi-gcc -mcpu=${CPU_NAME} -nostartfiles

## The flags passed to the linker.
export LDFLAGS+= --specs=nosys.specs -lnosys -u _printf_float

## The command for extracting images from the linked executables.
export OBJCOPY=arm-none-eabi-objcopy

## The command for dumping output file
export OBJDUMP=arm-none-eabi-objdump

## The command for show size #
export OBJSIZE=arm-none-eabi-size


#******************************************************************************
#
# The rule for building the object file from each C source file.
#
#******************************************************************************


#******************************************************************************
#
# Make/clean all targets
#
#******************************************************************************
export ASIC_NUM=
export BUILD_TARGET=
export SBOOT_BUILD=
export BUILD_ID?=0
export LOG_VERBOSE_DEFAULT?=0x1F

$(shell mkdir -p ${OUTPUT_DIR} 2>/dev/null)
$(shell mkdir -p ${OBJ_BUILD} 2>/dev/null)


all:
	@export SBOOT_BUILD=DISABLE;\
	echo "\n\r>>>>> BUILD FOR ${BOOT_S1_DIR}\n\t" ;\
	cd ${BOOT_S1_DIR} && $(MAKE);\
	cd ..;\
	for i in 1 2 3 ;\
	 do \
		echo "\n\r>>>>> BUILD FOR ${BOOT_S2_DIR}_ASIC$$i\n\t" ;\
		export ASIC_NUM=ASIC$$i;\
		cd ${BOOT_S2_DIR} && $(MAKE); cd ../; \
	 done
	@echo "done..."
	
s1:
	@export SBOOT_BUILD=DISABLE;\
	echo "\n\r>>>>> BUILD FOR ${BOOT_S1_DIR}\n\t" ;\
	cd ${BOOT_S1_DIR} && $(MAKE)
	@echo "done..."

s1clean:
	@rm -rf ${OUTPUT_DIR}/AsicFwS1*
	@rm -rf ${OUTPUT_DIR}/build/AsicFwS1*

	
docs:
	@mkdir doc; doxygen Doxygen

clean:
	@rm -rf ${OUTPUT_DIR}
	@rm -rf ${CERT_NAME}
