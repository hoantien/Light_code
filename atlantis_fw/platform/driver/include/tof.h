/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    flash_led.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jun-1-2016
 * @brief   Header file of LM3644 current/power sensor driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LM3644_H__
#define __LM3644_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lcc_system.h"

/* Private define ------------------------------------------------------------*/
/* #define USE_MULTI_VERSION */
/* Exported define -----------------------------------------------------------*/
/* LM3644 Register addresses*/
#define LM3644_REG_ENABLE				0x01
#define LM3644_REG_IVFM					0x02
#define LM3644_REG_LED1_FLASH_BRIGHT	0x03
#define LM3644_REG_LED2_FLASH_BRIGHT	0x04
#define LM3644_REG_LED1_TORCH_BRIGHT	0x05
#define LM3644_REG_LED2_TORCH_BRIGHT	0x06
#define LM3644_REG_BOOST_CONFIG			0x07
#define LM3644_REG_TIMING_CONFIG		0x08
#define LM3644_REG_TEMP					0x09
#define LM3644_REG_FLAGS1				0x0A
#define LM3644_REG_FLAGS2				0x0B
#define LM3644_REG_DEVICE_ID			0x0C
#define LM3644_REG_LAST_FLASH			0x0D
/* Exported typedef ----------------------------------------------------------*/
#define LM3644_VALID_DEVICDE_ID			0
#define LM3644_TYPE_LM3644_STANDARD		2
#define LM3644_TYPE_LM2644_TT			4
#define FLASH_TOGGLE_TORCH_TRIG_PIN		0
#define FLASH_TOGGLE_FLASH_TRIG_PIN		1
#define FLASH_FLASH_MODE				1
#define FLASH_TORCH_MODE				2
#define FLASH_IR_MODE					4
#define FLASH_MAX_DURATION				1600
#define FLASH_MAX_FLASH_CURRENT			1500
#define FLASH_MAX_TORCH_CURRENT			360
/*
 * @brief flash_light_status_t
 * LM3644 return status
 */
typedef enum
{
	LM3644_OK = 			0,
	LM3644_I2C_BUSY = 		1,
	LM3644_ALERT = 			2,
	LM3644_VALUE_INVALID =	3,
} flash_light_status_t;

/*
 * @brief flash_light_mode_t
 * LM3644 mode
 */
typedef enum
{
	LM3644_MODE_STANDBY =		0,
	LM3644_MODE_IR_DRIVE =		1,
	LM3644_MODE_TORCH =			2,
	LM3644_MODE_FLASH =			3,
} flash_light_mode_t;

/*
 * @brief lm3644_ivfm_level_t
 * LM3644 ivfm mode
 */
typedef enum
{
	IVFM_LEVEL_2V9 =	0,
	IVFM_LEVEL_3V0 =	1,
	IVFM_LEVEL_3V1 =	2,
	IVFM_LEVEL_3V2 =	3,
	IVFM_LEVEL_3V3 =	4,
	IVFM_LEVEL_3V4 =	5,
	IVFM_LEVEL_3V5 =	6,
	IVFM_LEVEL_3V6 =	7,
} lm3644_ivfm_level_t;

/*
 * @brief lm3644_ivfm_selection_t
 * LM3644 ivfm selection
 */
typedef enum
{
	IVFM_SELECTION_DISABLE =	0,
	IVFM_SELECTION_STOP_HOLD =	1,
	IVFM_SELECTION_DOWN =		2,
	IVFM_SELECTION_UP_DOWN =	3,
} lm3644_ivfm_selection_t;

/*
 * @brief lm3644_boost_frequency_t
 * LM3644 boost frequency selection
 */
typedef enum
{
	BOOST_FREQUENCY_2MHZ =	0,
	BOOST_FREQUENCY_4MHZ =	1,
} lm3644_boost_frequency_t;

/*
 * @brief lm3644_torch_current_ramp_t
 * LM3644 torch current ramp selection
 */
typedef enum
{
	TORCH_CURRENT_RAMP_NO =		0,
	TORCH_CURRENT_RAMP_1MS =	1,
	TORCH_CURRENT_RAMP_32MS =	2,
	TORCH_CURRENT_RAMP_64MS =	3,
	TORCH_CURRENT_RAMP_128MS =	4,
	TORCH_CURRENT_RAMP_256MS =	5,
	TORCH_CURRENT_RAMP_512MS =	6,
	TORCH_CURRENT_RAMP_1024MS =	7,
} lm3644_torch_current_ramp_t;

/*
 * @brief lm3644_flash_time_out_duration_t
 * LM3644 flash time out duration selection
 */
typedef enum
{
	FLASH_TIME_OUT_DURATION_10MS =	0,
	FLASH_TIME_OUT_DURATION_20MS =	1,
	FLASH_TIME_OUT_DURATION_30MS =	2,
	FLASH_TIME_OUT_DURATION_40MS =	3,
	FLASH_TIME_OUT_DURATION_50MS =	4,
	FLASH_TIME_OUT_DURATION_60MS =	5,
	FLASH_TIME_OUT_DURATION_70MS =	6,
	FLASH_TIME_OUT_DURATION_80MS =	7,
	FLASH_TIME_OUT_DURATION_90MS =	8,
	FLASH_TIME_OUT_DURATION_100MS =	9,
	FLASH_TIME_OUT_DURATION_150MS =	10,
	FLASH_TIME_OUT_DURATION_200MS =	11,
	FLASH_TIME_OUT_DURATION_250MS =	12,
	FLASH_TIME_OUT_DURATION_300MS =	13,
	FLASH_TIME_OUT_DURATION_350MS =	14,
	FLASH_TIME_OUT_DURATION_400MS =	15,
} lm3644_flash_time_out_duration_t;

/*
 * @brief lm3644_torch_temp_function_select_t
 * LM3644 TORCH/TEMP function selection
 */
typedef enum
{
	TORCH_FUNCTION =	0,
	TEMP_FUNCTION =		1,
} lm3644_torch_temp_function_select_t;

/*
 * @brief lm3644_temp_detect_volt_t
 * LM3644 TEMP Detect Voltage Threshold selection
 */
typedef enum
{
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V2 =		0,
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V3 =		1,
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V4 =		2,
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V5 =		3,
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V6 =		4,
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V7 =		5,
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V8 =		6,
	TEMP_DETECT_VOLTAGE_THRESHOLD_0V9 =		7,
} lm3644_temp_detect_volt_t;

/*
 * @brief lm3644_strobe_mode_t
 * LM3644 return status
 */
typedef enum
{
	MODE_STROBE_LEVEL =		0,
	MODE_STROBE_EDGE =		1,
} lm3644_strobe_mode_t;

typedef enum
{
	FLASH_FLAG_DONE = 0,
	FLASH_FLAG_FLASH,
	FLASH_FLAG_BUSY,
	FLASH_FLAG_ERROR,
	FLASH_FLAG_TRIGGER_FLASH
} LM3644_flag_t;

typedef struct
{
	uint8_t IVFM_sel : 2; /* IVFM selection */
	uint8_t IVFM_hys : 1; /* IVFM Hysteresis */
	uint8_t IVFM_lvl : 3; /* IVFM levels */
	uint8_t IVFM_cir : 1; /* IVFM Circuitry */
	uint8_t reserve : 1; /* resert bit */
} lm3644_IVFM_reg_t;

typedef struct
{
	uint8_t LED1 : 1; /* LED 1 enable bit */
	uint8_t LED2 : 1; /* LED 2 enable bit */
	uint8_t mode : 2;
	uint8_t torch_temp_enable : 1; /* Torch/Temp enable pin */
	uint8_t strobe_enable : 1; /* Strobe PIN enable */
	uint8_t strobe_type : 1; /* Strobe PIN type */
	uint8_t tx_enable : 1; /* TX PIN enable */
} lm3644_enable_reg_t;

typedef struct
{
	uint8_t flash_timeout : 4; /* Flash time out bits */
	uint8_t torch_ramp : 3; /* Torch current ramp bits */
	uint8_t reserve : 1; /* Reserve bit */
} lm3644_time_reg_t;

typedef struct
{
	uint8_t boost_cur : 1; /* Boost current limit setting */
	uint8_t boost_frq : 1; /* Boost frequency */
	uint8_t boost_mode : 1; /* Boost mode */
	uint8_t led_fault : 1; /* LED pin short fault detection */
	uint8_t reserve : 3; /* Reserve bits */
	uint8_t reset : 1; /* Software reset */
} lm3644_boost_reg_t;

/*
 * @brief flash_light_status_t
 * LM3644 return status
 */
typedef struct
{
	uint8_t flags1;				/* get flags1 from LM3644 */
	uint8_t flags2;				/* get flags2 from LM3644 */
	uint8_t i2c_addr;			/* Flash LED slave address */
	lm3644_enable_reg_t enable_reg;
	lm3644_IVFM_reg_t IVFM_reg;
	uint8_t led1_bright;	/* mA, 1500mA max, 10.9mA min */
	uint8_t led2_bright;	/* mA, 1500mA max, 10.9mA min */
	uint16_t led1_cur_mA;
	uint16_t led2_cur_mA;
	lm3644_time_reg_t torch_flash_time;	/* ms */
	uint32_t flash_time;
	uint8_t flash_mode;
	lm3644_boost_reg_t boost_mode;
	uint8_t flag;
} lm3644_info_t;
/*
 * @brief tof_sensor_t
 * Time-of-Flight Ranging Sensor interface, ASIC1 only
 */
typedef struct
{
	gpio_t flash_ctrl;			/* [in]  FLASH_CTRL signal from host */
	gpio_t flash_hwen;
	gpio_t flash_trig;			/* [out] FLASH_TRIG pin */
	gpio_t torch_trig;			/* [out] TORCH_TRIG pin */
	gpio_t tof_enable;			/* [out] ToF enable pin */
	gpio_t tof_intr;
	void (*cb_tof_intr)(void);
} tof_sensor_t;

/* Exported functions --------------------------------------------------------*/
/*
 * @brief flash_light_init
 * Configure the Flash LED devices
 * @param chid I2C channel ID
 * @param *info Flash LED device pointer
 * @return flash_light_status_t
 */
int flash_light_init(i2c_t chid, lm3644_info_t *info);

/*
 * @brief flash_light_deinit
 * Shutdown the Flash LED devices
 * @param chid I2C channel ID
 * @param *info Flash LED device pointer
 * @return flash_light_status_t
 */
int flash_light_deinit(i2c_t chid, lm3644_info_t *info);

/*
 * @brief flash_trigger_pin
 * Using to turn on/off flash trigger pin
 * @param state ON/OFF
 * return none
 */
void flash_triger_pin(uint8_t state);
/*
 * @brief flash_light_control
 * Control the LM3644 to set mode/LED current/timeout
 * @param chid I2C channel ID
 * @param info Flash LED device struct pointer need to update
 * @return flash_light_status_t
 */
uint8_t flash_light_control(uint8_t chid, lm3644_info_t *info);

#ifdef USING_TOF_FUNC
/*******************************************************************************
 *    The code ported from ST VL53L0x source code use to control ToF device    *
 ******************************************************************************/
/*---------------------------Exported define----------------------------------*/
#define VL53L0X_SPECIFICATION_VER_MAJOR				1
#define VL53L0X_SPECIFICATION_VER_MINOR				2
#define VL53L0X_SPECIFICATION_VER_SUB				7
#define VL53L0X_SPECIFICATION_VER_REVISION			1440
#define VL53L0X_IMPLEMENTATION_VER_MAJOR			1
#define VL53L0X_IMPLEMENTATION_VER_MINOR			1
#define VL53L0X_IMPLEMENTATION_VER_SUB				20
#define VL53L0X_IMPLEMENTATION_VER_REVISION			4606

#define VL53L010_SPECIFICATION_VER_MAJOR				1
#define VL53L010_SPECIFICATION_VER_MINOR				2
#define VL53L010_SPECIFICATION_VER_SUB					7
#define VL53L010_SPECIFICATION_VER_REVISION				1440
#define VL53L010_IMPLEMENTATION_VER_MAJOR				1
#define VL53L010_IMPLEMENTATION_VER_MINOR				0
#define VL53L010_IMPLEMENTATION_VER_SUB					9
#define VL53L010_IMPLEMENTATION_VER_REVISION			3673

#define VL53L0X_DEFAULT_MAX_LOOP						200
#define VL53L0X_MAX_STRING_LENGTH						32
#define VL53L0X_HISTOGRAM_BUFFER_SIZE					24
#define VL53L0X_REF_SPAD_BUFFER_SIZE					6
#define VL53L0X_SEQUENCESTEP_NUMBER_OF_CHECKS			5

#define REF_ARRAY_SPAD_0				0
#define REF_ARRAY_SPAD_5				5
#define REF_ARRAY_SPAD_10				10

#define VL53L0X_SPEED_OF_LIGHT_IN_AIR					2997
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV	0x0089
#define VL53L0X_REG_ALGO_PHASECAL_LIM					0x0030
#define VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT		0x0030

#define VL53L0X_MODE_STANDARD			0x01
#define VL53L0X_MODE_HIGH_ACCUARACY		0x02
#define VL53L0X_MODE_LONG_RANGE			0x03
#define VL53L0X_MODE_HIGH_SPEED			0x04
#define VL53L0X_CALIB_REF				0x81
#define VL53L0X_CALIB_SPAD				0x82
#define VL53L0X_CALIB_OFFSET			0x83
#define VL53L0X_CALIB_XTALK				0x84

#define VL53L0X_REF_CALIBRATION			BIT0
#define VL53L0X_SPAD_CALIBRATION		BIT1
#define VL53L0X_OFFSET_CALIBRATION		BIT2
#define VL53L0X_XTALK_CALIBRATION		BIT3

#define VL53L0X_FLAG_DONE				0X00
#define VL53L0X_FLAG_EXECUTE			0x01
#define VL53L0X_FLAG_BUSY				0x02
#define VL53L0X_FLAG_ERROR				0x03

#define VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE			0
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE		1
#define VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP				2
#define VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD		3
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC			4
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE		5
#define VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS			6

#define VL53L010_CHECKENABLE_SIGMA_FINAL_RANGE			0
#define VL53L010_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE	1
#define VL53L010_CHECKENABLE_NUMBER_OF_CHECKS			2

#define VL53L0X_REG_SYSRANGE_START							0x000

#define VL53L0X_REG_SYSRANGE_MODE_MASK						0x0F
#define VL53L0X_REG_SYSRANGE_MODE_START_STOP				0x01
#define VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT				0x00
#define VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK				0x02
#define VL53L0X_REG_SYSRANGE_MODE_TIMED						0x04
#define VL53L0X_REG_SYSRANGE_MODE_HISTOGRAM					0x08

#define VL53L0X_REG_SYSTEM_THRESH_HIGH						0x000C
#define VL53L0X_REG_SYSTEM_THRESH_LOW						0x000E

#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG					0x0001
#define VL53L0X_REG_SYSTEM_RANGE_CONFIG						0x0009
#define VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD			0x0004

#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO			0x000A
#define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_DISABLED			0x00
#define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_LOW			0x01
#define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_HIGH		0x02
#define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_OUT_OF_WINDOW		0x03
#define VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY	0x04

#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH					0x0084
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR					0x000B

#define VL53L0X_REG_RESULT_INTERRUPT_STATUS					0x0013
#define VL53L0X_REG_RESULT_RANGE_STATUS						0x0014

#define VL53L0X_REG_RESULT_CORE_PAGE  1
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN	0x00BC
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN	0x00C0
#define VL53L0X_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF	0x00D0
#define VL53L0X_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF	0x00D4
#define VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF				0x00B6

#define VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM		0x0028

#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS				0x008a

/* Check Limit registers */
#define VL53L0X_REG_MSRC_CONFIG_CONTROL						0x0060

#define VL53L0X_REG_PRE_RANGE_CONFIG_MIN_SNR					0X0027
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW			0x0056
#define VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH			0x0057
#define VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT			0x0064

#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_SNR					0X0067
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW			0x0047
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH			0x0048
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT	0x0044


#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI		0X0061
#define VL53L0X_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO		0X0062

/* PRE RANGE registers */
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD			0x0050
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI		0x0051
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO		0x0052

#define VL53L0X_REG_SYSTEM_HISTOGRAM_BIN					0x0081
#define VL53L0X_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT	0x0033
#define VL53L0X_REG_HISTOGRAM_CONFIG_READOUT_CTRL			0x0055

#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD			0x0070
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI	0x0071
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO	0x0072
#define VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS	0x0020

#define VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP			0x0046

#define VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N			0x00bf
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID				0x00c0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID			0x00c2
#define VL53L0X_REG_OSC_CALIBRATE_VAL					0x00f8

#ifdef USE_MULTI_VERSION
#define VL53L010_REG_SYSRANGE_START						0x000
#define VL53L010_REG_SYSRANGE_MODE_MASK					0x0F
#define VL53L010_REG_SYSRANGE_MODE_START_STOP			0x01
#define VL53L010_REG_SYSRANGE_MODE_SINGLESHOT			0x00
#define VL53L010_REG_SYSRANGE_MODE_BACKTOBACK			0x02
#define VL53L010_REG_SYSRANGE_MODE_TIMED				0x04
#define VL53L010_REG_SYSRANGE_MODE_HISTOGRAM			0x08

#define VL53L010_REG_SYSTEM_THRESH_HIGH					0x000C
#define VL53L010_REG_SYSTEM_THRESH_LOW					0x000E

/* FPGA bitstream */
#define VL53L010_REG_SYSTEM_SEQUENCE_CONFIG				0x0001
#define VL53L010_REG_SYSTEM_INTERMEASUREMENT_PERIOD		0x0004

#define VL53L010_REG_SYSTEM_REPORT_REQUEST				0x0009
#define	VL53L010_REG_SYSTEM_RANGEA_DATA					0x04
#define	VL53L010_REG_SYSTEM_RANGEB_DATA					0x05

#define VL53L010_REG_SYSTEM_INTERRUPT_CONFIG_GPIO				0x000A
#define VL53L010_REG_SYSTEM_INTERRUPT_GPIO_DISABLED				0x00
#define VL53L010_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_LOW			0x01
#define VL53L010_REG_SYSTEM_INTERRUPT_GPIO_LEVEL_HIGH			0x02
#define VL53L010_REG_SYSTEM_INTERRUPT_GPIO_OUT_OF_WINDOW		0x03
#define VL53L010_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY		0x04
#define VL53L010_REG_GPIO_HV_MUX_ACTIVE_HIGH			0x0084
#define VL53L010_REG_SYSTEM_INTERRUPT_CLEAR				0x000B

/* Result registers */
#define VL53L010_REG_RESULT_INTERRUPT_STATUS			0x0013
#define VL53L010_REG_RESULT_RANGE_STATUS				0x0014

#define VL53L010_REG_RESULT_SIGNAL_COUNT_RATE_RET		0x001A
#define VL53L010_REG_RESULT_AMBIENT_COUNT_RATE_RET		0x001C
#define VL53L010_REG_RESULT_FINAL_RANGE					0x001E

/* Algo register */
#define VL53L010_REG_ALGO_CROSSTALK_COMPENSATION_RATE	0x0020
#define VL53L010_REG_ALGO_RANGE_IGNORE_VALID_HEIGHT		0x0025
#define VL53L010_REG_ALGO_RANGE_IGNORE_THRESHOLD		0x0026
#define VL53L010_REG_ALGO_SNR_RATIO						0x0027
#define VL53L010_REG_ALGO_RANGE_CHECK_ENABLES			0x0028
#define VL53L010_REG_ALGO_PART_TO_PART_RANGE_OFFSET		0x0029
#define VL53L010_REG_I2C_SLAVE_DEVICE_ADDRESS			0x008a

/* MSRC registers */
#define VL53L010_REG_MSRC_CONFIG_COUNT					0x0044
#define VL53L010_REG_MSRC_CONFIG_TIMEOUT				0x0046
#define VL53L010_REG_MSRC_CONFIG_MIN_SNR				0x0055
#define VL53L010_REG_MSRC_CONFIG_VALID_PHASE_LOW		0x0047
#define VL53L010_REG_MSRC_CONFIG_VALID_PHASE_HIGH		0x0048

/* RANGE A registers */
#define VL53L010_REG_RNGA_CONFIG_VCSEL_PERIOD			0x0050
#define VL53L010_REG_RNGA_TIMEOUT_MSB					0x0051
#define VL53L010_REG_RNGA_TIMEOUT_LSB					0x0052
#define VL53L010_REG_RNGA_CONFIG_VALID_PHASE_LOW		0x0056
#define VL53L010_REG_RNGA_CONFIG_VALID_PHASE_HIGH		0x0057

/* RANGE B1 registers */
#define VL53L010_REG_RNGB1_CONFIG_VCSEL_PERIOD			0x0060
#define VL53L010_REG_RNGB1_TIMEOUT_MSB					0x0061
#define VL53L010_REG_RNGB1_TIMEOUT_LSB					0x0062
#define VL53L010_REG_RNGB1_CONFIG_VALID_PHASE_LOW		0x0066
#define VL53L010_REG_RNGB1_CONFIG_VALID_PHASE_HIGH		0x0067

/* RANGE B2 registers */
#define VL53L010_REG_RNGB2_CONFIG_VCSEL_PERIOD			0x0070
#define VL53L010_REG_RNGB2_TIMEOUT_MSB					0x0071
#define VL53L010_REG_RNGB2_TIMEOUT_LSB					0x0072
#define VL53L010_REG_RNGB2_CONFIG_VALID_PHASE_LOW		0x0076
#define VL53L010_REG_RNGB2_CONFIG_VALID_PHASE_HIGH		0x0077

#define VL53L010_REG_SOFT_RESET_GO2_SOFT_RESET_N		0x00bf
#define VL53L010_REG_IDENTIFICATION_MODEL_ID			0x00c0
#define VL53L010_REG_IDENTIFICATION_REVISION_ID			0x00c2
#define VL53L010_REG_IDENTIFICATION_MODULE_ID			0x00c3
#define VL53L010_REG_OSC_CALIBRATE_VAL					0x00f8
#define VL53L010_REG_FIRMWARE_MODE_STATUS				0x00C5
#define VL53L010_REG_DYNAMIC_SPAD_ACTUAL_RTN_SPADS_INT	0x0016
#endif /* USE_MULTI_VERSION */

#define VL53L0X_SIGMA_ESTIMATE_MAX_VALUE				65535
#define VL53L010_SIGMA_ESTIMATE_MAX_VALUE				65535

#define VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH			0x032
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0	0x0B0
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1	0x0B1
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2	0x0B2
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3	0x0B3
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4	0x0B4
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5	0x0B5

#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT		0xB6
#define VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD		0x4E
#define VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET		0x4F
#define VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE		0x80

#define VL53L0X_STRING_DEVICE_INFO_NAME				"VL53L0X cut1.0"
#define VL53L0X_STRING_DEVICE_INFO_NAME_TS0			"VL53L0X TS0"
#define VL53L0X_STRING_DEVICE_INFO_NAME_TS1			"VL53L0X TS1"
#define VL53L0X_STRING_DEVICE_INFO_NAME_TS2			"VL53L0X TS2"
#define VL53L0X_STRING_DEVICE_INFO_NAME_ES1			"VL53L0X ES1 or later"
#define VL53L0X_STRING_DEVICE_INFO_TYPE				"VL53L0X"

#define VL53L010_STRING_DEVICE_INFO_NAME			"VL53L0 cut1.0"
#define VL53L010_STRING_DEVICE_INFO_NAME_TS0		"VL53L0 TS0"
#define VL53L010_STRING_DEVICE_INFO_NAME_TS1		"VL53L0 TS1"
#define VL53L010_STRING_DEVICE_INFO_NAME_TS2		"VL53L0 TS2"
#define VL53L010_STRING_DEVICE_INFO_NAME_ES1		"VL53L0 ES1 or later"
#define VL53L010_STRING_DEVICE_INFO_TYPE			"VL53L0"

/* PAL ERROR strings */
#define VL53L0X_STRING_ERROR_NONE					"No Error"
#define VL53L0X_STRING_ERROR_CALIBRATION_WARNING	"Calibration Warning Error"
#define VL53L0X_STRING_ERROR_MIN_CLIPPED			"Min clipped error"
#define VL53L0X_STRING_ERROR_UNDEFINED				"Undefined error"
#define VL53L0X_STRING_ERROR_INVALID_PARAMS			"Invalid parameters error"
#define VL53L0X_STRING_ERROR_NOT_SUPPORTED			"Not supported error"
#define VL53L0X_STRING_ERROR_RANGE_ERROR			"Range error"
#define VL53L0X_STRING_ERROR_TIME_OUT				"Time out error"
#define VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED		"Mode not supported error"
#define VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL		"Buffer too small"
#define VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING		"GPIO not existing"
#define VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED \
		"GPIO funct not supported"
#define VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED	"Interrupt not Cleared"
#define VL53L0X_STRING_ERROR_CONTROL_INTERFACE		"Control Interface Error"
#define VL53L0X_STRING_ERROR_INVALID_COMMAND		"Invalid Command Error"
#define VL53L0X_STRING_ERROR_DIVISION_BY_ZERO		"Division by zero Error"
#define VL53L0X_STRING_ERROR_REF_SPAD_INIT			"Reference Spad Init Error"
#define VL53L0X_STRING_ERROR_NOT_IMPLEMENTED		"Not implemented error"
#define VL53L0X_STRING_UNKNOWN_ERROR_CODE			"Unknown Error Code"

/* Range Status */
#define VL53L0X_STRING_RANGESTATUS_NONE				"No Update"
#define VL53L0X_STRING_RANGESTATUS_RANGEVALID		"Range Valid"
#define VL53L0X_STRING_RANGESTATUS_SIGMA			"Sigma Fail"
#define VL53L0X_STRING_RANGESTATUS_SIGNAL			"Signal Fail"
#define VL53L0X_STRING_RANGESTATUS_MINRANGE			"Min Range Fail"
#define VL53L0X_STRING_RANGESTATUS_PHASE			"Phase Fail"
#define VL53L0X_STRING_RANGESTATUS_HW				"Hardware Fail"

/* Range Status */
#define VL53L0X_STRING_STATE_POWERDOWN				"POWERDOWN State"
#define VL53L0X_STRING_STATE_WAIT_STATICINIT		"Wait for staticinit State"
#define VL53L0X_STRING_STATE_STANDBY				"STANDBY State"
#define VL53L0X_STRING_STATE_IDLE					"IDLE State"
#define VL53L0X_STRING_STATE_RUNNING				"RUNNING State"
#define VL53L0X_STRING_STATE_UNKNOWN				"UNKNOWN State"
#define VL53L0X_STRING_STATE_ERROR					"ERROR State"

/* Device Specific */
#define VL53L0X_STRING_DEVICEERROR_NONE				"No Update"
#define VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE \
		"VCSEL Continuity Test Failure"
#define VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE \
		"VCSEL Watchdog Test Failure"
#define VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND	"No VHV Value found"
#define VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET		"MSRC No Target Error"
#define VL53L0X_STRING_DEVICEERROR_SNRCHECK			"SNR Check Exit"
#define VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK	"Range Phase Check Error"
#define VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK \
		"Sigma Threshold Check Error"
#define VL53L0X_STRING_DEVICEERROR_TCC				"TCC Error"
#define VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY	"Phase Consistency Error"
#define VL53L0X_STRING_DEVICEERROR_MINCLIP			"Min Clip Error"
#define VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE	"Range Complete"
#define VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW	"Range Algo Underflow Error"
#define VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW		"Range Algo Overlow Error"
#define VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD \
		"Range Ignore Threshold Error"
#define VL53L0X_STRING_DEVICEERROR_UNKNOWN			"Unknown error code"

#define VL53L010_STRING_DEVICEERROR_MSRCMINIMUMSNR	"MSRC Minimum SNR Error"
#define VL53L010_STRING_DEVICEERROR_MSRCWRAPAROUND	"MSRC Wraparound Error"
#define VL53L010_STRING_DEVICEERROR_RANGEAWRAPAROUND \
		"Range A Wraparound Error"
#define  VL53L010_STRING_DEVICEERROR_RANGEBWRAPAROUND \
		"Range B Wraparound Error"
#define VL53L010_STRING_DEVICEERROR_FINALSNRLIMIT	"Final Minimum SNR Error"
#define VL53L010_STRING_DEVICEERROR_NOTARGETIGNORE	"No Target Ignore Error"

#define VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE \
		"SIGMA FINAL RANGE"
#define VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE \
		"SIGNAL RATE FINAL RANGE"
#define VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP \
		"SIGNAL REF CLIP"
#define VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD \
		"RANGE IGNORE THRESHOLD"
#define VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC \
		"SIGNAL RATE MSRC"
#define VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE \
		"SIGNAL RATE PRE RANGE"

#define VL53L0X_STRING_SEQUENCESTEP_TCC				"TCC"
#define VL53L0X_STRING_SEQUENCESTEP_DSS				"DSS"
#define VL53L0X_STRING_SEQUENCESTEP_MSRC			"MSRC"
#define VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE		"PRE RANGE"
#define VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE		"FINAL RANGE"

	/* Check Enable */
#define  VL53L010_STRING_CHECKENABLE_SIGMA			"SIGMA"
#define  VL53L010_STRING_CHECKENABLE_SIGNAL_RATE	"SIGNAL RATE"
/* Define functions utilities V1.0 */
#define VL53L010_SETPARAMETERFIELD(Dev, field, value) \
	do { \
		if (Status == VL53L0X_ERROR_NONE) {\
			CurrentParameters = \
				PALDevDataGet(Dev, CurrentParameters); \
			CurrentParameters.field = value; \
			CurrentParameters =	\
				PALDevDataSet(Dev, CurrentParameters, \
					CurrentParameters); \
		} \
	} while (0)
#define VL53L010_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
	do { \
		if (Status == VL53L0X_ERROR_NONE) {\
			CurrentParameters = \
				PALDevDataGet(Dev, CurrentParameters); \
			CurrentParameters.field[index] = value; \
			CurrentParameters = \
				PALDevDataSet(Dev, CurrentParameters, \
					CurrentParameters); \
		} \
	} while (0)
#define VL53L010_GETPARAMETERFIELD(Dev, field, variable) \
	do { \
		if (Status == VL53L0X_ERROR_NONE) { \
			CurrentParameters = \
				PALDevDataGet(Dev, CurrentParameters); \
			variable = CurrentParameters.field; \
		} \
	} while (0)
#define VL53L010_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
	do { \
		if (Status == VL53L0X_ERROR_NONE) { \
			CurrentParameters = \
				PALDevDataGet(Dev, CurrentParameters); \
			variable = CurrentParameters.field[index]; \
		} \
	} while (0)
#define VL53L010_SETDEVICESPECIFICPARAMETER(Dev, field, value) \
	do { \
		if (Status == VL53L0X_ERROR_NONE) { \
			DeviceSpecificParameters = \
				PALDevDataGet(Dev, DeviceSpecificParameters); \
			DeviceSpecificParameters.field = value; \
			DeviceSpecificParameters = \
				PALDevDataSet(Dev, DeviceSpecificParameters, \
				DeviceSpecificParameters); \
		} \
	} while (0)
#define VL53L010_BUILDCASESTRING(BUFFER, CODE, STRINGVALUE) \
	do { \
		case CODE: \
			VL53L0X_COPYSTRING(BUFFER, STRINGVALUE); \
			break; \
	} while (0)
#define VL53L010_GETDEVICESPECIFICPARAMETER(Dev, field) \
		PALDevDataGet(Dev, DeviceSpecificParameters).field
#define VL53L010_FIXPOINT1616TOFIXPOINT97(Value) \
			(uint16_t)((Value >> 9) & 0xFFFF)
#define VL53L010_FIXPOINT97TOFIXPOINT1616(Value) \
			(FixPoint1616_t)(Value << 9)
#define VL53L010_FIXPOINT1616TOFIXPOINT412(Value) \
			(uint16_t)((Value >> 4) & 0xFFFF)
#define VL53L010_FIXPOINT412TOFIXPOINT1616(Value) \
			(FixPoint1616_t)(Value << 4)
#define VL53L010_FIXPOINT1616TOFIXPOINT08(Value) \
			(uint8_t)((Value >> 8) & 0x00FF)
#define VL53L010_FIXPOINT08TOFIXPOINT1616(Value) \
			(FixPoint1616_t)(Value << 8)
#define VL53L010_MAKEUINT16(lsb, msb) \
			(uint16_t)((((uint16_t)msb) << 8) + (uint16_t)lsb)

/* Define functions utilities V1.1 */
#define PALDevDataGet(Dev, field) (Dev->Data.field)
#define PALDevDataSet(Dev, field, data) (Dev->Data.field) = (data)
/* Define using for main API*/
#define VL53L0X_SETPARAMETERFIELD(Dev, field, value) \
	PALDevDataSet(Dev, CurrentParameters.field, value)
#define VL53L0X_GETPARAMETERFIELD(Dev, field, variable) \
	variable = PALDevDataGet(Dev, CurrentParameters).field
#define VL53L0X_COPYSTRING(str, ...) strcpy(str, ##__VA_ARGS__)
#define VL53L0X_SETARRAYPARAMETERFIELD(Dev, field, index, value) \
	PALDevDataSet(Dev, CurrentParameters.field[index], value)
#define VL53L0X_GETARRAYPARAMETERFIELD(Dev, field, index, variable) \
	variable = PALDevDataGet(Dev, CurrentParameters).field[index]
#define VL53L0X_SETDEVICESPECIFICPARAMETER(Dev, field, value) \
		PALDevDataSet(Dev, DeviceSpecificParameters.field, value)
#define VL53L0X_GETDEVICESPECIFICPARAMETER(Dev, field) \
		PALDevDataGet(Dev, DeviceSpecificParameters).field
#define VL53L0X_PollingDelay(Dev)	vTaskDelay(25);
#define VL53L0X_FIXPOINT1616TOFIXPOINT97(Value) \
	(uint16_t)((Value>>9)&0xFFFF)
#define VL53L0X_FIXPOINT97TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<9)
#define VL53L0X_FIXPOINT1616TOFIXPOINT88(Value) \
	(uint16_t)((Value>>8)&0xFFFF)
#define VL53L0X_FIXPOINT88TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<8)
#define VL53L0X_FIXPOINT1616TOFIXPOINT412(Value) \
	(uint16_t)((Value>>4)&0xFFFF)
#define VL53L0X_FIXPOINT412TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<4)
#define VL53L0X_FIXPOINT1616TOFIXPOINT313(Value) \
	(uint16_t)((Value>>3)&0xFFFF)
#define VL53L0X_FIXPOINT313TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<3)
#define VL53L0X_FIXPOINT1616TOFIXPOINT08(Value) \
	(uint8_t)((Value>>8)&0x00FF)
#define VL53L0X_FIXPOINT08TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<8)
#define VL53L0X_FIXPOINT1616TOFIXPOINT53(Value) \
	(uint8_t)((Value>>13)&0x00FF)
#define VL53L0X_FIXPOINT53TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<13)
#define VL53L0X_FIXPOINT1616TOFIXPOINT102(Value) \
	(uint16_t)((Value>>14)&0x0FFF)
#define VL53L0X_FIXPOINT102TOFIXPOINT1616(Value) \
	(FixPoint1616_t)(Value<<12)
#define VL53L0X_MAKEUINT16(lsb, msb) (uint16_t)((((uint16_t)msb)<<8) + \
		(uint16_t)lsb)

/*---------------------------Exported typedef---------------------------------*/
typedef uint32_t FixPoint1616_t;
typedef int8_t VL53L0X_Error;
typedef uint8_t VL53L0X_DeviceModes;
typedef uint8_t VL53L0X_HistogramModes;
typedef uint8_t VL53L0X_PowerModes;
typedef uint8_t VL53L0X_State;
typedef uint8_t VL53L0X_InterruptPolarity;
typedef uint8_t VL53L0X_VcselPeriod;
typedef uint8_t VL53L0X_SequenceStepId;
typedef uint8_t VL53L0X_DeviceError;
typedef uint8_t VL53L0X_GpioFunctionality;
typedef uint8_t VL53L010_DeviceError;
typedef uint8_t VL53L010_GpioFunctionality;
typedef struct {
	uint32_t     revision;
	uint8_t      major;
	uint8_t      minor;
	uint8_t      build;
} VL53L0X_Version_t;

typedef struct {
	char Name[VL53L0X_MAX_STRING_LENGTH];
	char Type[VL53L0X_MAX_STRING_LENGTH];
	char ProductId[VL53L0X_MAX_STRING_LENGTH];
	uint8_t ProductType;
	uint8_t ProductRevisionMajor;
	uint8_t ProductRevisionMinor;
} VL53L0X_DeviceInfo_t;

typedef struct {
	VL53L0X_DeviceModes DeviceMode;
	VL53L0X_HistogramModes HistogramMode;
	uint32_t MeasurementTimingBudgetMicroSeconds;
	uint32_t InterMeasurementPeriodMilliSeconds;
	uint8_t XTalkCompensationEnable;
	uint16_t XTalkCompensationRangeMilliMeter;
	FixPoint1616_t XTalkCompensationRateMegaCps;
	int32_t RangeOffsetMicroMeters;
	uint8_t LimitChecksEnable[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS];
	uint8_t LimitChecksStatus[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS];
	FixPoint1616_t LimitChecksValue[VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS];
	uint8_t WrapAroundCheckEnable;
} VL53L0X_DeviceParameters_t;

typedef struct {
	int32_t AmbTuningWindowFactor_K;
	int32_t RetSignalAt0mm;
} VL53L0X_DMaxData_t;

typedef struct {
	uint32_t TimeStamp;
	uint32_t MeasurementTimeUsec;
	uint16_t RangeMilliMeter;
	uint16_t RangeDMaxMilliMeter;
	FixPoint1616_t SignalRateRtnMegaCps;
	FixPoint1616_t AmbientRateRtnMegaCps;
	uint16_t EffectiveSpadRtnCount;
	uint8_t ZoneId;
	uint8_t RangeFractionalPart;
	uint8_t RangeStatus;
} VL53L0X_RangingMeasurementData_t;

typedef struct {
	uint32_t HistogramData[VL53L0X_HISTOGRAM_BUFFER_SIZE];
	uint8_t HistogramType;
	uint8_t FirstBin;
	uint8_t BufferSize;
	uint8_t NumberOfBins;
	VL53L0X_DeviceError ErrorStatus;
} VL53L0X_HistogramMeasurementData_t;

typedef struct {
	uint8_t RefSpadEnables[VL53L0X_REF_SPAD_BUFFER_SIZE];
	uint8_t RefGoodSpadMap[VL53L0X_REF_SPAD_BUFFER_SIZE];
} VL53L0X_SpadData_t;

typedef struct {
	FixPoint1616_t OscFrequencyMHz;
	uint16_t LastEncodedTimeout;
	VL53L0X_GpioFunctionality Pin0GpioFunctionality;
	uint32_t FinalRangeTimeoutMicroSecs;
	uint8_t FinalRangeVcselPulsePeriod;
	uint32_t PreRangeTimeoutMicroSecs;
	uint8_t PreRangeVcselPulsePeriod;
	uint16_t SigmaEstRefArray;
	uint16_t SigmaEstEffPulseWidth;
	uint16_t SigmaEstEffAmbWidth;
	uint8_t ReadDataFromDeviceDone;
	uint8_t ModuleId;
	uint8_t Revision;
	char ProductId[VL53L0X_MAX_STRING_LENGTH];
	uint8_t ReferenceSpadCount;
	uint8_t ReferenceSpadType;
	uint8_t RefSpadsInitialised;
	uint32_t PartUIDUpper;
	uint32_t PartUIDLower;
	FixPoint1616_t SignalRateMeasFixed400mm;
} VL53L0X_DeviceSpecificParameters_t;

typedef struct {
	VL53L0X_DMaxData_t DMaxData;
	int32_t  Part2PartOffsetNVMMicroMeter;
	int32_t  Part2PartOffsetAdjustmentNVMMicroMeter;
	VL53L0X_DeviceParameters_t CurrentParameters;
	VL53L0X_RangingMeasurementData_t LastRangeMeasure;
	VL53L0X_HistogramMeasurementData_t LastHistogramMeasure;
	VL53L0X_DeviceSpecificParameters_t DeviceSpecificParameters;
	VL53L0X_SpadData_t SpadData;
	uint8_t SequenceConfig;
	uint8_t RangeFractionalEnable;
	VL53L0X_State PalState;
	VL53L0X_PowerModes PowerMode;
	uint16_t SigmaEstRefArray;
	uint16_t SigmaEstEffPulseWidth;
	uint16_t SigmaEstEffAmbWidth;
	uint8_t StopVariable;
	uint16_t targetRefRate;
	FixPoint1616_t SigmaEstimate;
	FixPoint1616_t SignalEstimate;
	FixPoint1616_t LastSignalRefMcps;
	uint8_t *pTuningSettingsPointer;
	uint8_t UseInternalTuningSettings;
	uint16_t LinearityCorrectiveGain;
	uint16_t DmaxCalRangeMilliMeter;
	FixPoint1616_t DmaxCalSignalRateRtnMegaCps;
} VL53L0X_DevData_t;

typedef struct {
	VL53L0X_DevData_t Data;

	uint8_t I2cDevAddr;
	uint8_t	Mode;
	uint8_t Flag;
	uint8_t RangeStatus;
	uint8_t CalibStep;
	uint16_t CalibDistance;
	uint8_t InitStatus;
#ifdef USE_MULTI_VERSION
	uint8_t DevVersion;
#endif /* USE_MULTI_VERSION */
	/* int Id;
	int Present;
	int Enabled;
	int Ready;
	int LeakyRange;
	int LeakyFirst;
	uint8_t PreviousRangeStatus;
	FixPoint1616_t SignalRateRtnMegaCps;
	uint16_t EffectiveSpadRtnCount; */
} VL53L0X_Dev_t;
typedef VL53L0X_Dev_t *VL53L0X_DEV;

typedef struct {
	uint8_t TccOn;
	uint8_t MsrcOn;
	uint8_t DssOn;
	uint8_t PreRangeOn;
	uint8_t FinalRangeOn;
} VL53L0X_SchedulerSequenceSteps_t;

/* Define typical */
#define VL53L0X_ERROR_NONE					((int8_t) 0)
#define VL53L0X_ERROR_CALIBRATION_WARNING	((int8_t) -1)
#define VL53L0X_ERROR_MIN_CLIPPED			((int8_t) -2)
#define VL53L0X_ERROR_UNDEFINED				((int8_t) -3)
#define VL53L0X_ERROR_INVALID_PARAMS		((int8_t) -4)
#define VL53L0X_ERROR_NOT_SUPPORTED			((int8_t) -5)
#define VL53L0X_ERROR_RANGE_ERROR			((int8_t) -6)
#define VL53L0X_ERROR_TIME_OUT				((int8_t) -7)
#define VL53L0X_ERROR_MODE_NOT_SUPPORTED	((int8_t) -8)
#define VL53L0X_ERROR_BUFFER_TOO_SMALL		((int8_t) -9)
#define VL53L0X_ERROR_GPIO_NOT_EXISTING		((int8_t) -10)
#define VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED	((int8_t) -11)
#define VL53L0X_ERROR_INTERRUPT_NOT_CLEARED	((int8_t) -12)
#define VL53L0X_ERROR_CONTROL_INTERFACE		((int8_t) -20)
#define VL53L0X_ERROR_INVALID_COMMAND		((int8_t) -30)
#define VL53L0X_ERROR_DIVISION_BY_ZERO		((int8_t) -40)
#define VL53L0X_ERROR_REF_SPAD_INIT			((int8_t) -50)
#define VL53L0X_ERROR_NOT_IMPLEMENTED		((int8_t) -99)

#define VL53L0X_DEVICEMODE_SINGLE_RANGING	((uint8_t)  0)
#define VL53L0X_DEVICEMODE_CONTINUOUS_RANGING	((uint8_t)  1)
#define VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM	((uint8_t)  2)
#define VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING	((uint8_t) 3)
#define VL53L0X_DEVICEMODE_SINGLE_ALS		((uint8_t) 10)
#define VL53L0X_DEVICEMODE_GPIO_DRIVE		((uint8_t) 20)
#define VL53L0X_DEVICEMODE_GPIO_OSC			((uint8_t) 21)

#define VL53L0X_HISTOGRAMMODE_DISABLED		((uint8_t) 0)
#define VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY	((uint8_t) 1)
#define VL53L0X_HISTOGRAMMODE_RETURN_ONLY	((uint8_t) 2)
#define VL53L0X_HISTOGRAMMODE_BOTH			((uint8_t) 3)

#define VL53L0X_POWERMODE_STANDBY_LEVEL1	((uint8_t) 0)
#define VL53L0X_POWERMODE_STANDBY_LEVEL2	((uint8_t) 1)
#define VL53L0X_POWERMODE_IDLE_LEVEL1		((uint8_t) 2)
#define VL53L0X_POWERMODE_IDLE_LEVEL2		((uint8_t) 3)

#define VL53L0X_STATE_POWERDOWN				((uint8_t) 0)
#define VL53L0X_STATE_WAIT_STATICINIT		((uint8_t) 1)
#define VL53L0X_STATE_STANDBY				((uint8_t) 2)
#define VL53L0X_STATE_IDLE					((uint8_t) 3)
#define VL53L0X_STATE_RUNNING				((uint8_t) 4)
#define VL53L0X_STATE_UNKNOWN				((uint8_t) 98)
#define VL53L0X_STATE_ERROR					((uint8_t) 99)

#define VL53L0X_INTERRUPTPOLARITY_LOW		((uint8_t) 0)
#define VL53L0X_INTERRUPTPOLARITY_HIGH		((uint8_t) 1)

#define VL53L0X_VCSEL_PERIOD_PRE_RANGE		((uint8_t) 0)
#define VL53L0X_VCSEL_PERIOD_FINAL_RANGE	((uint8_t) 1)

#define VL53L0X_SEQUENCESTEP_TCC			((uint8_t) 0)
#define VL53L0X_SEQUENCESTEP_DSS			((uint8_t) 1)
#define VL53L0X_SEQUENCESTEP_MSRC			((uint8_t) 2)
#define VL53L0X_SEQUENCESTEP_PRE_RANGE		((uint8_t) 3)
#define VL53L0X_SEQUENCESTEP_FINAL_RANGE	((uint8_t) 4)

#define VL53L0X_DEVICEERROR_NONE						((uint8_t) 0)
#define VL53L0X_DEVICEERROR_VCSELCONTINUITYTESTFAILURE	((uint8_t) 1)
#define VL53L0X_DEVICEERROR_VCSELWATCHDOGTESTFAILURE	((uint8_t) 2)
#define VL53L0X_DEVICEERROR_NOVHVVALUEFOUND				((uint8_t) 3)
#define VL53L0X_DEVICEERROR_MSRCNOTARGET				((uint8_t) 4)
#define VL53L0X_DEVICEERROR_SNRCHECK					((uint8_t) 5)
#define VL53L0X_DEVICEERROR_RANGEPHASECHECK				((uint8_t) 6)
#define VL53L0X_DEVICEERROR_SIGMATHRESHOLDCHECK			((uint8_t) 7)
#define VL53L0X_DEVICEERROR_TCC							((uint8_t) 8)
#define VL53L0X_DEVICEERROR_PHASECONSISTENCY			((uint8_t) 9)
#define VL53L0X_DEVICEERROR_MINCLIP						((uint8_t) 10)
#define VL53L0X_DEVICEERROR_RANGECOMPLETE				((uint8_t) 11)
#define VL53L0X_DEVICEERROR_ALGOUNDERFLOW				((uint8_t) 12)
#define VL53L0X_DEVICEERROR_ALGOOVERFLOW				((uint8_t) 13)
#define VL53L0X_DEVICEERROR_RANGEIGNORETHRESHOLD		((uint8_t) 14)

#define VL53L0X_GPIOFUNCTIONALITY_OFF						((uint8_t) 0)
#define VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW		((uint8_t) 1)
#define VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH	((uint8_t) 2)
#define VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT		((uint8_t) 3)
#define VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY			((uint8_t) 4)

#define VL53L010_DEVICEERROR_NONE						((uint8_t) 0)
#define VL53L010_DEVICEERROR_VCSELCONTINUITYTESTFAILURE	((uint8_t) 1)
#define VL53L010_DEVICEERROR_VCSELWATCHDOGTESTFAILURE	((uint8_t) 2)
#define VL53L010_DEVICEERROR_NOVHVVALUEFOUND			((uint8_t) 3)
#define VL53L010_DEVICEERROR_MSRCNOTARGET				((uint8_t) 4)
#define VL53L010_DEVICEERROR_MSRCMINIMUMSNR				((uint8_t) 5)
#define VL53L010_DEVICEERROR_MSRCWRAPAROUND				((uint8_t) 6)
#define VL53L010_DEVICEERROR_TCC						((uint8_t) 7)
#define VL53L010_DEVICEERROR_RANGEAWRAPAROUND			((uint8_t) 8)
#define VL53L010_DEVICEERROR_RANGEBWRAPAROUND			((uint8_t) 9)
#define VL53L010_DEVICEERROR_MINCLIP					((uint8_t) 10)
#define VL53L010_DEVICEERROR_RANGECOMPLETE				((uint8_t) 11)
#define VL53L010_DEVICEERROR_ALGOUNDERFLOW				((uint8_t) 12)
#define VL53L010_DEVICEERROR_ALGOOVERFLOW				((uint8_t) 13)
#define VL53L010_DEVICEERROR_FINALSNRLIMIT				((uint8_t) 14)
#define VL53L010_DEVICEERROR_NOTARGETIGNORE				((uint8_t) 15)

#define VL53L010_GPIOFUNCTIONALITY_OFF						((uint8_t) 0)
#define VL53L010_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW	((uint8_t) 1)
#define VL53L010_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH	((uint8_t) 2)
#define VL53L010_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT	((uint8_t) 3)
#define VL53L010_GPIOFUNCTIONALITY_NEW_MEASURE_READY		((uint8_t) 4)

VL53L0X_Error VL53L0X_get_info_from_device(VL53L0X_DEV Dev, uint8_t option);
VL53L0X_Error VL53L0X_GetMeasurementDataReady(VL53L0X_DEV Dev,
	uint8_t *pMeasurementDataReady);
VL53L0X_Error VL53L0X_GetInterruptMaskStatus(VL53L0X_DEV Dev,
	uint32_t *pInterruptMaskStatus);
VL53L0X_Error VL53L0X_ClearInterruptMask(VL53L0X_DEV Dev,
	uint32_t InterruptMask);
VL53L0X_Error VL53L0X_GetVcselPulsePeriod(VL53L0X_DEV Dev,
	VL53L0X_VcselPeriod VcselPeriodType, uint8_t *pVCSELPulsePeriodPCLK);
VL53L0X_Error VL53L0X_GetSequenceStepEnables(VL53L0X_DEV Dev,
	VL53L0X_SchedulerSequenceSteps_t *pSchedulerSequenceSteps);
VL53L0X_Error VL53L0X_SetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
	uint32_t MeasurementTimingBudgetMicroSeconds);
VL53L0X_Error VL53L0X_GetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
	uint32_t *pMeasurementTimingBudgetMicroSeconds);
VL53L0X_Error VL53L0X_perform_phase_calibration(VL53L0X_DEV Dev,
	uint8_t *pPhaseCal, const uint8_t get_data_enable,
	const uint8_t restore_config);
VL53L0X_Error VL53L0X_GetXTalkCompensationEnable(VL53L0X_DEV Dev,
	uint8_t *pXTalkCompensationEnable);
VL53L0X_Error VL53L0X_SetXTalkCompensationEnable(VL53L0X_DEV Dev,
	uint8_t XTalkCompensationEnable);
VL53L0X_Error VL53L0X_GetLimitCheckEnable(VL53L0X_DEV Dev,
	uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);
VL53L0X_Error VL53L0X_GetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	FixPoint1616_t *pLimitCheckValue);
VL53L0X_Error VL53L0X_SetLimitCheckEnable(VL53L0X_DEV Dev,
	uint16_t LimitCheckId, uint8_t LimitCheckEnable);
VL53L0X_Error VL53L0X_GetInterruptThresholds(VL53L0X_DEV Dev,
	VL53L0X_DeviceModes DeviceMode, FixPoint1616_t *pThresholdLow,
	FixPoint1616_t *pThresholdHigh);
VL53L0X_Error VL53L0X_SetWrapAroundCheckEnable(VL53L0X_DEV Dev,
	uint8_t WrapAroundCheckEnable);
VL53L0X_Error VL53L0X_SetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
	VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality,
	VL53L0X_InterruptPolarity Polarity);
VL53L0X_Error VL53L0X_PerformSingleRangingMeasurement(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);
VL53L0X_Error VL53L0X_PerformSingleMeasurement(VL53L0X_DEV Dev);
VL53L0X_Error VL53L0X_SetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
	FixPoint1616_t XTalkCompensationRateMegaCps);
VL53L0X_Error VL53L0X_SetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev,
	int32_t OffsetCalibrationDataMicroMeter);
VL53L0X_Error VL53L0X_GetSequenceStepEnable(VL53L0X_DEV Dev,
	VL53L0X_SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled);
VL53L0X_Error VL53L0X_SetSequenceStepEnable(VL53L0X_DEV Dev,
	VL53L0X_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled);
VL53L0X_Error VL53L0X_GetOffsetCalibrationDataMicroMeter(VL53L0X_DEV Dev,
	int32_t *pOffsetCalibrationDataMicroMeter);
VL53L0X_Error VL53L0X_StartMeasurement(VL53L0X_DEV Dev);
VL53L0X_Error VL53L0X_perform_ref_calibration(VL53L0X_DEV Dev,
	uint8_t *pVhvSettings, uint8_t *pPhaseCal, uint8_t get_data_enable);
VL53L0X_Error VL53L0X_GetPalSpecVersion(VL53L0X_Version_t *pPalSpecVersion);
VL53L0X_Error VL53L0X_StaticInit(VL53L0X_DEV Dev);
VL53L0X_Error VL53L0X_GetDeviceParameters(VL53L0X_DEV Dev,
	VL53L0X_DeviceParameters_t *pDeviceParameters);
VL53L0X_Error VL53L0X_SetLimitCheckValue(VL53L0X_DEV Dev, uint16_t LimitCheckId,
	FixPoint1616_t LimitCheckValue);
VL53L0X_Error VL53L0X_GetFractionEnable(VL53L0X_DEV Dev, uint8_t *pEnabled);
VL53L0X_Error VL53L0X_GetSequenceStepTimeout(VL53L0X_DEV Dev,
	VL53L0X_SequenceStepId SequenceStepId, FixPoint1616_t *pTimeOutMilliSecs);
VL53L0X_Error VL53L0X_SetDeviceMode(VL53L0X_DEV Dev,
	VL53L0X_DeviceModes DeviceMode);
VL53L0X_Error VL53L0X_SetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
	uint32_t InterMeasurementPeriodMilliSeconds);
VL53L0X_Error VL53L0X_GetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
	uint32_t *pInterMeasurementPeriodMilliSeconds);
VL53L0X_Error VL53L0X_GetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
	FixPoint1616_t *pXTalkCompensationRateMegaCps);
VL53L0X_Error VL53L0X_GetWrapAroundCheckEnable(VL53L0X_DEV Dev,
	uint8_t *pWrapAroundCheckEnable);
VL53L0X_Error VL53L0X_GetDeviceMode(VL53L0X_DEV Dev,
	VL53L0X_DeviceModes *pDeviceMode);
VL53L0X_Error VL53L0X_ResetDevice(VL53L0X_DEV Dev);

#ifdef USE_MULTI_VERSION
VL53L0X_Error VL53L010_StaticInit(VL53L0X_DEV Dev);
VL53L0X_Error VL53L010_GetDeviceParameters(VL53L0X_DEV Dev,
	VL53L0X_DeviceParameters_t *pDeviceParameters);
VL53L0X_Error VL53L010_SetLimitCheckEnable(VL53L0X_DEV Dev,
	uint16_t LimitCheckId, uint8_t LimitCheckEnable);
VL53L0X_Error VL53L010_SetLimitCheckValue(VL53L0X_DEV Dev,
	uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue);
VL53L0X_Error VL53L010_SetGpioConfig(VL53L0X_DEV Dev, uint8_t Pin,
	VL53L0X_DeviceModes DeviceMode, VL53L0X_GpioFunctionality Functionality,
	VL53L0X_InterruptPolarity Polarity);
VL53L0X_Error VL53L010_PerformRefCalibration(VL53L0X_DEV Dev);
VL53L0X_Error VL53L010_SetDeviceMode(VL53L0X_DEV Dev,
	VL53L0X_DeviceModes DeviceMode);
VL53L0X_Error VL53L010_SetDeviceParameters(VL53L0X_DEV Dev,
	const VL53L0X_DeviceParameters_t *pDeviceParameters);
VL53L0X_Error VL53L010_SetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
	uint32_t InterMeasurementPeriodMilliSeconds);
VL53L0X_Error VL53L010_SetXTalkCompensationEnable(VL53L0X_DEV Dev,
	uint8_t XTalkCompensationEnable);
VL53L0X_Error VL53L010_SetWrapAroundCheckEnable(VL53L0X_DEV Dev,
	uint8_t WrapAroundCheckEnable);
VL53L0X_Error VL53L010_GetDeviceMode(VL53L0X_DEV Dev,
	VL53L0X_DeviceModes *pDeviceMode);
VL53L0X_Error VL53L010_GetXTalkCompensationEnable(VL53L0X_DEV Dev, uint8_t*
	pXTalkCompensationEnable);
VL53L0X_Error VL53L010_SetHistogramMode(VL53L0X_DEV Dev,
	VL53L0X_HistogramModes HistogramMode);
VL53L0X_Error VL53L010_SetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
	FixPoint1616_t XTalkCompensationRateMegaCps);
VL53L0X_Error VL53L010_SetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
	uint32_t MeasurementTimingBudgetMicroSeconds);
VL53L0X_Error VL53L010_GetHistogramMode(VL53L0X_DEV Dev,
	VL53L0X_HistogramModes *pHistogramMode);
VL53L0X_Error VL53L010_GetInterMeasurementPeriodMilliSeconds(VL53L0X_DEV Dev,
	uint32_t *pInterMeasurementPeriodMilliSeconds);
VL53L0X_Error VL53L010_GetXTalkCompensationRateMegaCps(VL53L0X_DEV Dev,
	FixPoint1616_t *pXTalkCompensationRateMegaCps);
VL53L0X_Error VL53L010_GetLimitCheckValue(VL53L0X_DEV Dev,
	uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue);
VL53L0X_Error VL53L010_GetWrapAroundCheckEnable(VL53L0X_DEV Dev, uint8_t
	*pWrapAroundCheckEnable);
VL53L0X_Error VL53L010_GetMeasurementTimingBudgetMicroSeconds(VL53L0X_DEV Dev,
	uint32_t *pMeasurementTimingBudgetMicroSeconds);
VL53L0X_Error VL53L010_GetLimitCheckEnable(VL53L0X_DEV Dev,
	uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);
VL53L0X_Error VL53L010_StartMeasurement(VL53L0X_DEV Dev);
VL53L0X_Error VL53L010_GetMeasurementDataReady(VL53L0X_DEV Dev,
	uint8_t *pMeasurementDataReady);
VL53L0X_Error VL53L010_ClearInterruptMask(VL53L0X_DEV Dev,
	uint32_t InterruptMask);
VL53L0X_Error VL53L010_PerformSingleRangingMeasurement(VL53L0X_DEV Dev,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);
VL53L0X_Error VL53L010_GetInterruptMaskStatus(VL53L0X_DEV Dev, uint32_t
	*pInterruptMaskStatus);
VL53L0X_Error VL53L010_get_pal_range_status(VL53L0X_DEV Dev,
	uint8_t DeviceRangeStatus, FixPoint1616_t SignalRate,
	FixPoint1616_t CrosstalkCompensation, uint16_t EffectiveSpadRtnCount,
	VL53L0X_RangingMeasurementData_t *pRangingMeasurementData,
	uint8_t *pPalRangeStatus);
VL53L0X_Error VL53L010_device_read_strobe(VL53L0X_DEV Dev);
#endif /* USE_MULTI_VERSION */
VL53L0X_Error Init_ToF_Device(VL53L0X_DEV Dev);
VL53L0X_Error Perform_ToF_Device(VL53L0X_DEV Dev);
#endif /* USING_TOF_FUNC */
#ifdef __cplusplus
}
#endif
#endif /* __LM3644_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
