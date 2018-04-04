#*******************************************************************************
# @File's name	: Makefile
# @Company		: Light Co.
# @Author 		: Tien Luong -- Light's QC team
# @Revision		: 1.0.0
# @Date			: Apr-2016
#*******************************************************************************

# ASIC_FW_PATH
ASIC_FW_PATH=../../../atlantis_fw/
# ASIC_DRV Path
ASIC_DRV=platform/driver
# ASIC_FW_S1
ASIC_FW_S1=AsicFwS1
# ASIC_FW_S2
ASIC_FW_S2=AsicFwS2
# Common path
COMMON_PATH=platform/common
# Hardware abstract layer
ASIC_HAL=platform/asic/hal
# ARM Cortex R4
CORTEX_R4=platform/asic/cortex-r4
# Common path
COMMON_PATH=platform/common
# OS
OS_PATH=platform/os/Source
# UT Driver path
UT_DRIVER=UT_Driver
# Mock object path
MOCK_OBJ=mock_objects

DINCDIR+= $(ASIC_FW_PATH)/$(ASIC_HAL)/include
DINCDIR+= $(ASIC_FW_PATH)/$(ASIC_HAL)/src
DINCDIR+= $(ASIC_FW_PATH)/$(ASIC_DRV)/include
DINCDIR+= $(ASIC_FW_PATH)/$(ASIC_DRV)/src
DINCDIR+= $(ASIC_FW_PATH)/$(COMMON_PATH)
DINCDIR+= $(ASIC_FW_PATH)/$(CORTEX_R4)/include
DINCDIR+= $(ASIC_FW_PATH)/$(CORTEX_R4)/include/startup
DINCDIR+= $(ASIC_FW_PATH)/$(OS_PATH)
DINCDIR+= $(ASIC_FW_PATH)/$(OS_PATH)/include
DINCDIR+= $(ASIC_FW_PATH)/$(OS_PATH)/portable/GCC/cortex-r4
DINCDIR+= $(ASIC_FW_PATH)/$(ASIC_FW_S2)/include
DINCDIR+= $(MOCK_OBJ)

CC= gcc

CPFLAGS+=-std=gnu99
CPFLAGS+=-gdwarf-2
CPFLAGS+=-Wattributes
CPFLAGS+=-g -Wl,--wrap=GPIO_SetBits
#CPFLAGS+=-Werror
CPFLAGS+=-D__UNIT_TEST
CPFLAGS+=-DNO_DEBUG

CPFLAGS+=-fprofile-arcs
CPFLAGS+=-ftest-coverage
CFLAGS+=-lgcov
CPFLAGS +=--coverage
CFLAGS +=--coverage
CFLAGS+=-lcmocka

INCDIR  = $(patsubst %,-I%,$(sort $(DINCDIR) $(UINCDIR)))

I2C_SLAVE_SRC += $(UT_DRIVER)/test_i2c_slave.c
I2C_SLAVE_SRC += $(MOCK_OBJ)/mock_hal_i2c.c
I2C_SLAVE_OBJS+= $(addprefix ,$(addsuffix .o,$(basename $(I2C_SLAVE_SRC))))
I2C_SLAVE_EXE+= $(UT_DRIVER)/test_i2c_slave

LOG_SRC += $(UT_DRIVER)/test_log.c
LOG_SRC += $(MOCK_OBJ)/mock_hal_com.c
LOG_OBJS+= $(addprefix ,$(addsuffix .o,$(basename $(LOG_SRC))))
LOG_EXE+= $(UT_DRIVER)/test_log

TEMP_SENSOR_SRC += $(UT_DRIVER)/test_temp_sensor.c
TEMP_SENSOR_SRC += $(MOCK_OBJ)/mock_tasks.c
TEMP_SENSOR_SRC += $(MOCK_OBJ)/mock_hal_i2c.c
TEMP_SENSOR_OBJS+= $(addprefix ,$(addsuffix .o,$(basename $(TEMP_SENSOR_SRC))))
TEMP_SENSOR_EXE+= $(UT_DRIVER)/test_temp_sensor

SPI_SLAVE_SRC += $(UT_DRIVER)/test_spi_slave.c
SPI_SLAVE_SRC += $(MOCK_OBJ)/mock_hal_spi.c
SPI_SLAVE_SRC += $(MOCK_OBJ)/mock_queue.c
SPI_SLAVE_SRC += $(MOCK_OBJ)/mock_task_list.c
SPI_SLAVE_SRC += $(MOCK_OBJ)/mock_tasks.c
SPI_SLAVE_SRC += $(MOCK_OBJ)/mock_port.c
SPI_SLAVE_OBJS+= $(addprefix ,$(addsuffix .o,$(basename $(SPI_SLAVE_SRC))))
SPI_SLAVE_EXE+= $(UT_DRIVER)/test_spi_slave

CAMERA_SRC += $(UT_DRIVER)/test_camera.c
CAMERA_SRC += $(MOCK_OBJ)/mock_ar1335.c
CAMERA_SRC += $(MOCK_OBJ)/mock_log.c
CAMERA_SRC += $(MOCK_OBJ)/mock_hal_i2c.c
CAMERA_SRC += $(MOCK_OBJ)/mock_i2c_master.c
CAMERA_SRC += $(MOCK_OBJ)/mock_heap_1.c
CAMERA_SRC += $(MOCK_OBJ)/mock_tasks.c
CAMERA_OBJS+= $(addprefix ,$(addsuffix .o,$(basename $(CAMERA_SRC))))
CAMERA_EXE+= $(UT_DRIVER)/test_camera

VCM_SRC += $(UT_DRIVER)/test_vcm.c
VCM_SRC += $(MOCK_OBJ)/mock_hal_i2c.c
VCM_SRC += $(MOCK_OBJ)/mock_tasks.c
VCM_OBJS+= $(addprefix ,$(addsuffix .o,$(basename $(VCM_SRC))))
VCM_EXE+= $(UT_DRIVER)/test_vcm

ALL_EXE+= $(I2C_SLAVE_EXE) $(LOG_EXE) $(TEMP_SENSOR_EXE) $(SPI_SLAVE_EXE)
ALL_EXE+= $(CAMERA_EXE) $(VCM_EXE)

all: test_utils $(ALL_EXE)
	@echo $(MODE)

%.o : %.c
	@echo
	@echo Compiling C: $<
	$(CC) -c $(CPFLAGS) $(INCDIR) $< -o $@

$(I2C_SLAVE_EXE): $(I2C_SLAVE_OBJS)
	$(CC) -o $@  $^ $(CFLAGS)

$(LOG_EXE): $(LOG_OBJS)
	$(CC) -o $@  $^ $(CFLAGS)

$(TEMP_SENSOR_EXE): $(TEMP_SENSOR_OBJS)
	$(CC) -o $@  $^ $(CFLAGS)

$(SPI_SLAVE_EXE): $(SPI_SLAVE_OBJS)
	$(CC) -o $@  $^ $(CFLAGS)

$(CAMERA_EXE): $(CAMERA_OBJS)
	$(CC) -o $@  $^ $(CFLAGS)

$(VCM_EXE): $(VCM_OBJS)
	$(CC) -o $@  $^ $(CFLAGS)

test_utils:
	cp ./UT_util/portmacro.h $(ASIC_FW_PATH)/$(OS_PATH)/portable/GCC/cortex-r4/portmacro.h
	cp ./UT_util/cortex_r4.h $(ASIC_FW_PATH)/$(CORTEX_R4)/include/cortex_r4.h

clean:
	rm -f $(ALL_EXE) $(UT_DRIVER)/$(ALL_EXE)
	rm -rf *.c.gcov *.info XML_OUTPUT HTML_OUTPUT
	rm -f $(UT_DRIVER)/*.o $(UT_DRIVER)/*.gcda
	rm -f $(UT_DRIVER)/*.gcno $(UT_DRIVER)/*.c.gcov
	rm -f $(MOCK_OBJ)/*.gcda $(MOCK_OBJ)/*.gcno $(MOCK_OBJ)/*.o

run:
	./$(UT_DRIVER)/test_i2c_slave
	gcov $(UT_DRIVER)/test_i2c_slave.c
	./$(UT_DRIVER)/test_log
	gcov $(UT_DRIVER)/test_log.c
	./$(UT_DRIVER)/test_temp_sensor
	gcov $(UT_DRIVER)/test_temp_sensor.c
	./$(UT_DRIVER)/test_spi_slave
	gcov $(UT_DRIVER)/test_spi_slave.c
	./$(UT_DRIVER)/test_camera
	gcov $(UT_DRIVER)/test_camera.c
	./$(UT_DRIVER)/test_vcm
	gcov $(UT_DRIVER)/test_vcm.c

	./$(I2C_SLAVE_EXE) 		>	$(I2C_SLAVE_EXE).xml
	./$(LOG_EXE) 			>	$(LOG_EXE).xml
	./$(TEMP_SENSOR_EXE) 	>	$(TEMP_SENSOR_EXE).xml
	./$(SPI_SLAVE_EXE) 		>	$(SPI_SLAVE_EXE).xml
	./$(CAMERA_EXE) 		>	$(CAMERA_EXE).xml
	./$(VCM_EXE) 			>	$(VCM_EXE).xml

	$(shell mkdir -p XML_OUTPUT 2>/dev/null)
	mv $(UT_DRIVER)/*.xml XML_OUTPUT/
	lcov --capture --directory ./ --output-file coverage.info
	genhtml coverage.info --output-directory HTML_OUTPUT

#*******************************************************************************
# 								END OF FILE
#*******************************************************************************