/********************************************************************************
 * Copyright (c) 2015, The LightCo * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
/* System includes */
#include <stdio.h>
#include <stdint.h>

#define BOARD_VERSION_P1_1

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* CCB drivers includes. */
#include "Clk_register_map.h"
#ifdef BOARD_VERSION_P1
#include "ar0835_35mm_cam.h"
#endif /* BOARD_VERSION_P1 */
#include "hal_i2c_ex.h"
#include "cam_ctrl.h"
#ifdef BOARD_VERSION_P1_1
#include "ar1335_cam.h"
#endif /* BOARD_VERSION_P1_1 */
#include "af_ctrl.h"
#include "ucid.h"
#include "mems.h"
#include "log.h"
#include "errors.h"
#include "drv_piezo.h"
#include "drv_piezo_hall.h"
#include "drv_piezo_move.h"
#include "drv_piezo_pwm.h"
#include "drv_spi.h"
#include "drv_vcm.h"
#include "fpga.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"


/* Private typedef -----------------------------------------------------------*/
#ifdef BOARD_VERSION_P1
typedef struct {
	msm_camera_i2c_reg_array *regs;
	UInt16 reg_size;
	UInt16 data_size;
} cam_reg_array;
#endif /* BOARD_VERSION_P1 */

typedef struct {

	UInt16 tid;
	UInt16 cmd;
	UInt16 ucid;
	UInt16 data_len;
	UInt8 data[128];
	UInt8 bitmask[3];
	UInt8 flags;
	UInt16 tolerance;
} cam_cmd_t;

typedef struct {
	UInt16 *data;
	UInt16 len;
} I2C_Buf_t;

/* I2C message structure */
/* Address - Slave Address */
/* mode - Address size Data size */
/* txbuf - Data to be transmitted */
typedef struct
{
	CamSlaveAddress_TypeDef Address;
	I2C_Buf_t txbuf;
	I2C_Buf_t rxbuf;
}I2C_Message_t;

#ifdef BOARD_VERSION_P1_1
typedef enum
{
	UPDATE_FULL_SETTING = 0,
	UPDATE_RESOLUTION = 1
} update_resolution;
#endif
extern AFData AFInfo[MAX_SUPPORTED_CAM];
extern volatile uint8_t cpld_ver_major, cpld_ver_minor;
/* Private define ------------------------------------------------------------*/
#undef STM_CCB

/*
 * TODO: this define used to convert endianness,
 * if don't use, comment define bellow
 * have two define in two files: cam_ctrl.c and drv_cci.c
 */
#define CONVERT_TO_LITTLE_ENDIAN

// This is true for the AR0835
#define WAR_SENSOR_FAILS_TO_SHIFT_IN_X	1

#define ENABLE_FPGA_SENSOR_INIT			0

// Enable the following to insert test reads into the Set_Exposure() function.
#define TEST_I2CREAD_A16D16 0
#define TEST_I2CREAD_A8D8	0

#define INTERRUPT_ENABLE		1
#define INTERRUPT_DISABLE		0
#define LOCK_MASK				0x15
#define LOS_MASK				0x04
#define NUMBER_OF_RESOLUTION_SUPPORT 2
/* Private macro -------------------------------------------------------------*/
#ifdef BOARD_VERSION_P1_1
#define RESOLUTION_13M		(uint64_t)((((uint64_t)4208 << 32) & 0xffffffff00000000) | ((uint64_t)3120 & 0x00000000ffffffff))
#define RESOLUTION_1080		(uint64_t)((((uint64_t)1920 << 32) & 0xffffffff00000000) | ((uint64_t)1080 & 0x00000000ffffffff))
#endif

#define CAM_CSI0_SELECT_35MM()	GPIO_ResetBits(CAM1_70MM_EN_GPIO_PORT , CAM1_70MM_EN_PIN )
#define CAM_CSI0_SELECT_70MM()	GPIO_SetBits(CAM1_70MM_EN_GPIO_PORT , CAM1_70MM_EN_PIN )

#define CAM_CSI2_SELECT_35MM()	GPIO_ResetBits(CAM2_70MM_EN_GPIO_PORT , CAM2_70MM_EN_PIN )
#define CAM_CSI2_SELECT_70MM()	GPIO_SetBits(CAM2_70MM_EN_GPIO_PORT , CAM2_70MM_EN_PIN )

#ifdef BOARD_VERSION_P1
#define CAM_EXPOSURE_DEFAULT	2510
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
//#define CAM_EXPOSURE_DEFAULT	0xDFC
#define CAM_EXPOSURE_DEFAULT	0xE55
#endif /* BOARD_VERSION_P1_1 */
#define LENS_MAX_FAILURE_TRY	10
#define LENS_MOVE_ACCURACY		2


#define CMD_HEADER()   printf("\e[36m[DEBUG: %s] ", __FUNCTION__)
#define log_cmd(...) if (LOG_LEVEL <= LOG_LEVEL_ERROR){\
									CMD_HEADER();\
									printf(__VA_ARGS__);\
									DEBUG_ENDL();\
								 }

#ifdef BOARD_VERSION_P1_1

//#define VT_PIX_CLK (220000000)
////todo:read the LLPCLK from register 0x342
//#define LINE_LENGTH_PCLK (0x3000 / 2 )
//#define FRAME_LENGTH_LINES 	3580
//#define CONVERT_TO_PLL(x)	   (UInt64)((x * VT_PIX_CLK / LINE_LENGTH_PCLK) / 1000000000)
//#define GET_FIRST_SET_BIT(x)	((x) & ~((x) -1))
//todo : handle this better, using macros for the bring up

#define VT_PIX_CLK (220000000)
//#define VT_PIX_CLK_ (170000000)

//todo:read the LLPCLK (line_length_pclk) from register 0x342
#define LINE_LENGTH_PCLK_REG_ADDR		0x0342
#define LINE_LENGTH_PCLK_13M			(0x3000 / 2)
#define LINE_LENGTH_PCLK_1080P			(0x176C / 2)
#define LINE_LENGTH_PCLK_LONG_EXP		0x7FF0

//todo: read frame_length_lines form register REG=0x0340
#define FRAME_LENGTH_LINES_REG_ADDR		0x0340
#define FRAME_LENGTH_LINES_13M			0xDFC
#define FRAME_LENGTH_LINES_1080P		0xE55
#define FRAME_LENGTH_LINES_LONG_EXP		0x53E

#define CONVERT_TO_PLL(ll_pclk, x)	   (UInt64)(((x) * VT_PIX_CLK / (ll_pclk)) / 1000000000)
#endif

#ifdef BOARD_VERSION_P1
//todo : handle this better, using macros for the bring up
#define VT_PIX_CLK (80000000)

//todo:read the LLPCLK from register 0x342
#define LINE_LENGTH_PCLK (0x13D4 / 2)

//todo: read frame_length_lines form register REG=0x0340
#define FRAME_LENGTH_LINES 	3150
#define CONVERT_TO_PLL(x)	   (UInt64)((x * VT_PIX_CLK / LINE_LENGTH_PCLK) / 1000000000)
#endif

#define GET_FIRST_SET_BIT(x)	((x) & ~((x) -1))


/* Private variables ---------------------------------------------------------*/
__IO Int8 _status = 0;
unsigned int cam_bitmask = 0, stream_bitmask = 0;
UInt8 cam_ctrl = 0, stream_ctrl = 0;
UInt16 ucid = 0;
UInt32 status_master=0;
xQueueHandle cam_cmd_queue;
cam_cmd_log_t cam_cmd_log;
UInt16 streaming_count = 0; /*WA to skip VCM init. */

UInt8 fpga_debug_cmd[FPGA_MAX_DATA_SIZE + 3]; /* Command, Asic type, transaction and data len */
UInt8 cpld_debug_cmd[FPGA_MAX_DATA_SIZE + FPGA_MAX_DATA_SIZE];	  // 64 bytes
CPLD_CONFIG cpld_config_buffer;  // 50 bytes

#ifdef BOARD_VERSION_P1_1
/* this variable using for backup old resolution */
__IO update_resolution setting_resolution = UPDATE_FULL_SETTING;
uint8_t long_exposure_flag = 0;
#endif

uint8_t CPLD_data_size = 0;
volatile int gyro_read_flag = 0;
#ifdef BOARD_VERSION_P1_1
static uint8_t cam_trigger = 2 , cam_frame = 2;
static uint16_t  cam_lines = 114;
#endif
volatile ccb_cmd_base_t * cam_m_status = (ccb_cmd_base_t *) CCB_M_WRITE;
static uint8_t get_cam_csi_channel(uint32_t cam_idx);
static int cam_stream_off(int cam_idx);

/*
		 name	 type			 orientation			 i2c_channel  addr	ID		   CPLD    close   far
*/
#ifdef BOARD_VERSION_P1
CAM_INIT(CAM_A1, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A1,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_A2, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A2,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_A3, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A3,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_A4, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A4,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_A5, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A5,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_B4, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_V, I2CEx_PZT0,  0x6C, CAM_ID_B4, 0x0001, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_B5, CAM_TYPE_70MM,  CAM_ORIENTATION_NORMAL, I2CEx_PZT1,  0x6C, CAM_ID_B5, 0x0002, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_B3, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_V, I2CEx_PZT2,  0x6C, CAM_ID_B3, 0x0004, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_B2, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_V, I2CEx_PZT3,  0x6C, CAM_ID_B2, 0x0008, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_B1, CAM_TYPE_70MM,  CAM_ORIENTATION_NORMAL, I2CEx_PZT4,  0x6C, CAM_ID_B1, 0x0010, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_C1, CAM_TYPE_150MM, CAM_ORIENTATION_NORMAL, I2CEx_PZT5,  0x6C, CAM_ID_C1, 0x0100, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_C2, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT6,  0x6C, CAM_ID_C2, 0x0200, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_C4, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT7,  0x6C, CAM_ID_C4, 0x0400, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_C3, CAM_TYPE_150MM, CAM_ORIENTATION_NORMAL, I2CEx_PZT8,  0x6C, CAM_ID_C3, 0x0800, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_C6, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT9,  0x6C, CAM_ID_C6, 0x1000, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
CAM_INIT(CAM_C5, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT10, 0x6C, CAM_ID_C5, 0x2000, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0);
#endif
#ifdef BOARD_VERSION_P1_1
CAM_INIT(CAM_A1, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A1,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_A2, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A2,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_A3, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A3,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_A4, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A4,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_A5, CAM_TYPE_35MM,  CAM_ORIENTATION_NORMAL,		 -1,  0x6C, CAM_ID_A5,		0, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_B4, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_V, I2CEx_PZT0,  0x6C, CAM_ID_B4, 0x0001, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_B5, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_H, I2CEx_PZT1,  0x6C, CAM_ID_B5, 0x0002, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_B3, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_V, I2CEx_PZT2,  0x6C, CAM_ID_B3, 0x0004, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_B2, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_V, I2CEx_PZT3,  0x6C, CAM_ID_B2, 0x0008, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_B1, CAM_TYPE_70MM,  CAM_ORIENTATION_FLIP_H, I2CEx_PZT4,  0x6C, CAM_ID_B1, 0x0010, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_C1, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_H, I2CEx_PZT5,  0x6C, CAM_ID_C1, 0x0100, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_C2, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT6,  0x6C, CAM_ID_C2, 0x0200, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_C4, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT7,  0x6C, CAM_ID_C4, 0x0400, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_C3, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_H, I2CEx_PZT8,  0x6C, CAM_ID_C3, 0x0800, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_C6, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT9,  0x6C, CAM_ID_C6, 0x1000, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
CAM_INIT(CAM_C5, CAM_TYPE_150MM, CAM_ORIENTATION_FLIP_V, I2CEx_PZT10, 0x6C, CAM_ID_C5, 0x2000, 0x0000, 0xFFFF, HALL_POLARITY_NORMAL, 0x0000, 0xFFFF, 0, LINE_LENGTH_PCLK_13M, FRAME_LENGTH_LINES_13M);
#endif


#define CMD_HEADER()   printf("\e[36m[DEBUG: %s] ", __FUNCTION__)
#define log_cmd(...) if (LOG_LEVEL <= LOG_LEVEL_ERROR){\
									CMD_HEADER();\
									printf(__VA_ARGS__);\
									DEBUG_ENDL();\
								 }

static UInt32 __cam_status [CAM_NUM_CAMERA_MAX];
static volatile UInt32 cam_focus_status;
static volatile UInt32 cam_mirror_status;
static volatile UInt8 cam_streaming_array[48];
CamDevice_TypeDef *FTMCamDeviceTbl[CAM_NUM_CAMERA_MAX] = {
		&CAM_A2,		/* A1 */
		&CAM_A1,		/* A2 */
		&CAM_A4,		/* A3 */
		&CAM_A5,		/* A4 */
		&CAM_A3,		/* A5 */
		&CAM_B4,		/* B1 */
		&CAM_B5,		/* B2 */
		&CAM_B3,		/* B3 */
		&CAM_B2,		/* B4 */
		&CAM_B1,		/* B5 */
		&CAM_C1,		/* C1 */
		&CAM_C2,		/* C2 */
		&CAM_C4,		/* C3 */
		&CAM_C3,		/* C4 */
		&CAM_C6,		/* C5 */
		&CAM_C5,		/* C6 */
};

CamDevice_TypeDef *CamDeviceTbl[CAM_NUM_CAMERA_MAX] = {
		&CAM_A1,		/* A1 */
		&CAM_A2,		/* A2 */
		&CAM_A3,		/* A3 */
		&CAM_A4,		/* A4 */
		&CAM_A5,		/* A5 */
		&CAM_B1,		/* B1 */
		&CAM_B2,		/* B2 */
		&CAM_B3,		/* B3 */
		&CAM_B4,		/* B4 */
		&CAM_B5,		/* B5 */
		&CAM_C1,		/* C1 */
		&CAM_C2,		/* C2 */
		&CAM_C3,		/* C3 */
		&CAM_C4,		/* C4 */
		&CAM_C5,		/* C5 */
		&CAM_C6,		/* C6 */
};


CamDevice_TypeDef *CurrentCamera = NULL; // Single camera that is currently running.  Temporary.
#ifdef BOARD_VERSION_P1
/* This array is only used ONCE in the CAM_MODULE_OPEN command */
cam_reg_array CamOpenRegs[] = {
	{start_reg_array,				ARRAY_COUNT(start_reg_array),				1},
	{stop_reg_array,				ARRAY_COUNT(stop_reg_array),				1},
	{init_reg_array,				ARRAY_COUNT(init_reg_array),				2},
	{init_pll_settings_reg_array,	ARRAY_COUNT(init_pll_settings_reg_array),	2},
	{init_mipi_settings_reg_array,	ARRAY_COUNT(init_mipi_settings_reg_array),	2},
	{init_recommended_reg_array,	ARRAY_COUNT(init_recommended_reg_array),	1},
	{res0_reg_array,				ARRAY_COUNT(res0_reg_array),				2},
//	  {reset_reg_array,				  ARRAY_COUNT(reset_reg_array),				  2},
	{slave_mode_control_reg_array,	ARRAY_COUNT(slave_mode_control_reg_array),	2},
};

cam_reg_array CamCloseRegs[] = {
	{stop_reg_array,				ARRAY_COUNT(stop_reg_array),				1},
};

cam_reg_array CamStreamOnRegs[] = {
	{start_reg_array,				ARRAY_COUNT(start_reg_array),				1},
};

cam_reg_array CamStreamOffRegs[] = {
	{stop_reg_array,				ARRAY_COUNT(stop_reg_array),				1},
};

cam_reg_array CamResolutionRegs[NUMBER_OF_RESOLUTION_SUPPORT] = {
	{res0_reg_array,				ARRAY_COUNT(res0_reg_array),				2}, //8MP
#if 0
	{res1_reg_array,				ARRAY_COUNT(res1_reg_array),				2}, //1080
#endif
};

/*
	These tables contain the CPLD configuration values necessary to drive
	certain PWM signals. The coarse tables drive the lenses over long
	distances, are used for getting lenses reasonably close to the focus
	point. The fine tables are used for fine-tuning the focus point. The
	hard stop tables are designed to drive the actuator to each hard stop
	as quickly as possible.

	At the moment, the hard stop tables are exactly the same as the coarse
	tables. Ideally, these values would be tuned for each individual module,
	and even tuned for the direction each module is travelling.
 */

static CPLD_CONFIG hard_stop_retract_values =
{
	// 10 75 97 75 97 10 0 1 100 1 100 0 1000 1000 1 128
	.duty_cycle1.byte	 = {0x27, 0x00},
	.duty_cycle2.byte	 = {0x25, 0x01},  // 75
	.duty_cycle3.byte	 = {0x25, 0x01},  // 75
	.duty_cycle4.byte	 = {0x25, 0x01},  // 75
	.duty_cycle5.byte	 = {0x25, 0x01},  // 75
	.duty_cycle6.byte	 = {0x27, 0x00},
	.rep_cnt1.byte		 = {0x00, 0x00, 0x00, 0x00},
	.rep_cnt2.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt3.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt4.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt5.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt6.byte		 = {0x00, 0x00, 0x00, 0x00},
	.rep_group_cnt1.byte= {0xe8, 0x03, 0x00, 0x00},
	.rep_group_cnt2.byte= {0xe8, 0x03, 0x00, 0x00},
	.rep_ovr_cnt1.byte	= {0x01, 0x00, 0x00, 0x00},
	.freq.byte			= {0x87, 0x01}
};

static CPLD_CONFIG coarse_retract_values =
{
    // 10 75 97 75 97 10 0 1 100 1 100 0 1000 1000 1 128
    .duty_cycle1.byte    = {0x27, 0x00},
    .duty_cycle2.byte    = {0x25, 0x01},  // 75
    .duty_cycle3.byte    = {0x25, 0x01},  // 75
    .duty_cycle4.byte    = {0x25, 0x01},  // 75
    .duty_cycle5.byte    = {0x25, 0x01},  // 75
    .duty_cycle6.byte    = {0x27, 0x00},
    .rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt1.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt2.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_ovr_cnt1.byte   = {0xC8, 0x00, 0x00, 0x00},
    .freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG fine_retract_values =
{
	// 10 75 97 75 97 10 0 1 100 1 100 0 10 10 1 128
	.duty_cycle1.byte    = {0x27, 0x00},
	.duty_cycle2.byte    = {0x25, 0x01},
	.duty_cycle3.byte    = {0x7b, 0x01},
	.duty_cycle4.byte    = {0x25, 0x01},
	.duty_cycle5.byte    = {0x7b, 0x01},
	.duty_cycle6.byte    = {0x27, 0x00},
	.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_cnt2.byte       = {0x02, 0x00, 0x00, 0x00},
	.rep_cnt3.byte       = {0x32, 0x00, 0x00, 0x00},
	.rep_cnt4.byte       = {0x02, 0x00, 0x00, 0x00},
	.rep_cnt5.byte       = {0x32, 0x00, 0x00, 0x00},
	.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_group_cnt1.byte = {0x0a, 0x00, 0x00, 0x00},
	.rep_group_cnt2.byte = {0x0a, 0x00, 0x00, 0x00},
	.rep_ovr_cnt1.byte   = {0x14, 0x00, 0x00, 0x00},
	.freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG hard_stop_extend_values =
{
	// 10 25 3 25 3 10 0 1 100 1 100 0 10 10 1 128
	.duty_cycle1.byte    = {0x27, 0x00},
	.duty_cycle2.byte    = {0x62, 0x00},  // 25
	.duty_cycle3.byte    = {0x62, 0x00},  // 25
	.duty_cycle4.byte    = {0x62, 0x00},  // 25
	.duty_cycle5.byte    = {0x62, 0x00},  // 25
	.duty_cycle6.byte    = {0x27, 0x00},
	.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_group_cnt1.byte = {0xE8, 0x03, 0x00, 0x00},
	.rep_group_cnt2.byte = {0xE8, 0x03, 0x00, 0x00},
	.rep_ovr_cnt1.byte   = {0x01, 0x00, 0x00, 0x00},
	.freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG coarse_extend_values =
{
    // 10 25 3 25 3 10 0 1 100 1 100 0 10 10 1 128
    .duty_cycle1.byte    = {0x27, 0x00},
    .duty_cycle2.byte    = {0x62, 0x00},  // 25
    .duty_cycle3.byte    = {0x62, 0x00},  // 25
    .duty_cycle4.byte    = {0x62, 0x00},  // 25
    .duty_cycle5.byte    = {0x62, 0x00},  // 25
    .duty_cycle6.byte    = {0x27, 0x00},
    .rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt1.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt2.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_ovr_cnt1.byte   = {0xC8, 0x00, 0x00, 0x00},
    .freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG fine_extend_values =
{
	// 10 25 3 25 3 10 0 1 100 1 100 0 10 10 1 128
	.duty_cycle1.byte    = {0x27, 0x00},
	.duty_cycle2.byte    = {0x62, 0x00}, // 25%
	.duty_cycle3.byte    = {0x0c, 0x00}, // 3%
	.duty_cycle4.byte    = {0x62, 0x00}, // 25%
	.duty_cycle5.byte    = {0x0c, 0x00}, // 3%
	.duty_cycle6.byte    = {0x27, 0x00},
	.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00}, // 0 reps
	.rep_cnt2.byte       = {0x02, 0x00, 0x00, 0x00}, // 2 reps
	.rep_cnt3.byte       = {0x32, 0x00, 0x00, 0x00}, // 50 reps
	.rep_cnt4.byte       = {0x02, 0x00, 0x00, 0x00}, // 2 reps
	.rep_cnt5.byte       = {0x32, 0x00, 0x00, 0x00}, // 50 reps
	.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00}, // 0 reps
	.rep_group_cnt1.byte = {0x0a, 0x00, 0x00, 0x00}, // 10 reps
	.rep_group_cnt2.byte = {0x0a, 0x00, 0x00, 0x00}, // 10 reps
	.rep_ovr_cnt1.byte   = {0x14, 0x00, 0x00, 0x00}, // 20 reps
	.freq.byte           = {0x87, 0x01}
};
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1

msm_camera_stream_type stream_types[] = {
	{stream_13m_30fps,			4208,	3120,	30,	ARRAY_COUNT(stream_13m_30fps)},
	{stream_13m_30fps_border,	4224,	3136,	30,	ARRAY_COUNT(stream_13m_30fps_border)},
	{stream_13m_30fps_DPCM8,	4208,	3120,	30,	ARRAY_COUNT(stream_13m_30fps_DPCM8)},
	{stream_13m_24fps,			4208,	3120,	24,	ARRAY_COUNT(stream_13m_24fps)},
	{stream_13m_15fps,			4208,	3120,	15,	ARRAY_COUNT(stream_13m_15fps)},
	{stream_3M_30fps_HQ,		2104,	1560,	30,	ARRAY_COUNT(stream_3M_30fps_HQ)},
	{stream_3M_30fps_LP,		2104,	1560,	30,	ARRAY_COUNT(stream_3M_30fps_LP)},
	{stream_1080p_30fps_HQ,		1920,	1080,	30,	ARRAY_COUNT(stream_1080p_30fps_HQ)},
	{stream_1080p_30fps_LP,		1920,	1080,	30,	ARRAY_COUNT(stream_1080p_30fps_LP)},
	{stream_1080p_60fps_HQ,		1920,	1080,	60,	ARRAY_COUNT(stream_1080p_60fps_HQ)},
	{stream_1080p_60fps_LP,		1920,	1080,	60,	ARRAY_COUNT(stream_1080p_60fps_LP)},
	{stream_720p_30fps_HQ,		1280,	720,	30,	ARRAY_COUNT(stream_720p_30fps_HQ)},
	{stream_720p_30fps_LP,		1280,	720,	30,	ARRAY_COUNT(stream_720p_30fps_LP)},
	{stream_720p_60fps_HQ,		1280,	720,	60,	ARRAY_COUNT(stream_720p_60fps_HQ)},
	{stream_720p_60fps_LP,		1280,	720,	60,	ARRAY_COUNT(stream_720p_60fps_LP)},
	{stream_4k_uhd,				3840,	2160,	30,	ARRAY_COUNT(stream_4k_uhd)},
	{stream_4k_cinema,			4096,	2160,	30,	ARRAY_COUNT(stream_4k_cinema)},
};

cam_reg_array CamOpenRegs[] = {
	{start_reg_array,			ARRAY_COUNT(start_reg_array),				1 },
	{stop_reg_array,			ARRAY_COUNT(stop_reg_array),				1 },
	{correction_recommend,		ARRAY_COUNT(correction_recommend),			2 },
	{pll_setup_recommended,		ARRAY_COUNT(pll_setup_recommended),			2 },
	{pixel_timing_recommended,	ARRAY_COUNT(pixel_timing_recommended),		2 },
	{analog_setup_recommended,	ARRAY_COUNT(analog_setup_recommended),		2 },
	{mipi_timing_recommended,	ARRAY_COUNT(mipi_timing_recommended),		2 },
	{res13_4208_3120_recommended,	ARRAY_COUNT(res13_4208_3120_recommended),		2 },
	{defect_correction,			ARRAY_COUNT(defect_correction),				2 },
	//{reset_reg_array,			ARRAY_COUNT(reset_reg_array),				2 },
	{slave_mode_control_reg_array,	ARRAY_COUNT(slave_mode_control_reg_array),2},
	{ar1335_black_level_correction,	ARRAY_COUNT(ar1335_black_level_correction),2}

};

cam_reg_array CamSlaveEnable[] = {
	{slave_mode_enable_reg_array,	ARRAY_COUNT(slave_mode_enable_reg_array), 2}
};

cam_reg_array CamSlaveDisable[] = {
	{slave_mode_disable_reg_array,	ARRAY_COUNT(slave_mode_disable_reg_array),2}
};

cam_reg_array CamCloseRegs[] = {
	{stop_reg_array,				ARRAY_COUNT(stop_reg_array),				1},
};

cam_reg_array CamStreamOnRegs[] = {
	{analog_control_on,			ARRAY_COUNT(analog_control_on),	2},
	{start_reg_array,			ARRAY_COUNT(start_reg_array),	1}
};

cam_reg_array CamStreamOffRegs[] = {
	{analog_control_off,	ARRAY_COUNT(analog_control_off),	2},
	{stop_reg_array,		ARRAY_COUNT(stop_reg_array),		1},
	{analog_control_reset,	ARRAY_COUNT(analog_control_reset),	2}
};

/*
	These tables contain the CPLD configuration values necessary to drive
	certain PWM signals. The coarse tables drive the lenses over long
	distances, are used for getting lenses reasonably close to the focus
	point. The fine tables are used for fine-tuning the focus point. The
	hard stop tables are designed to drive the actuator to each hard stop
	as quickly as possible.

	At the moment, the hard stop tables are exactly the same as the coarse
	tables. Ideally, these values would be tuned for each individual module,
	and even tuned for the direction each module is travelling.
 */

static CPLD_CONFIG hard_stop_retract_values =
{
	// 10 75 97 75 97 10 0 1 100 1 100 0 1000 1000 1 128
	.duty_cycle1.byte	 = {0x27, 0x00},
	.duty_cycle2.byte	 = {0x25, 0x01},  // 75
	.duty_cycle3.byte	 = {0x25, 0x01},  // 75
	.duty_cycle4.byte	 = {0x25, 0x01},  // 75
	.duty_cycle5.byte	 = {0x25, 0x01},  // 75
	.duty_cycle6.byte	 = {0x27, 0x00},
	.rep_cnt1.byte		 = {0x00, 0x00, 0x00, 0x00},
	.rep_cnt2.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt3.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt4.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt5.byte		 = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt6.byte		 = {0x00, 0x00, 0x00, 0x00},
	.rep_group_cnt1.byte = {0xe8, 0x03, 0x00, 0x00},
	.rep_group_cnt2.byte = {0xe8, 0x03, 0x00, 0x00},
	.rep_ovr_cnt1.byte	 = {0x01, 0x00, 0x00, 0x00},
	.freq.byte			 = {0x87, 0x01}
};

static CPLD_CONFIG coarse_retract_values =
{
    // 10 75 97 75 97 10 0 1 100 1 100 0 1000 1000 1 128
    .duty_cycle1.byte    = {0x27, 0x00},
    .duty_cycle2.byte    = {0x25, 0x01},  // 75
    .duty_cycle3.byte    = {0x25, 0x01},  // 75
    .duty_cycle4.byte    = {0x25, 0x01},  // 75
    .duty_cycle5.byte    = {0x25, 0x01},  // 75
    .duty_cycle6.byte    = {0x27, 0x00},
    .rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt1.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt2.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_ovr_cnt1.byte   = {0xC8, 0x00, 0x00, 0x00},
    .freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG fine_retract_values =
{
		// 10 75 97 75 97 10 0 1 100 1 100 0 10 10 1 128
		.duty_cycle1.byte    = {0x27, 0x00},
		.duty_cycle2.byte    = {0x1d, 0x01},
		.duty_cycle3.byte    = {0x1d, 0x01},
		.duty_cycle4.byte    = {0x1d, 0x01},
		.duty_cycle5.byte    = {0x1d, 0x01},
		.duty_cycle6.byte    = {0x27, 0x00},
		.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
		.rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
		.rep_group_cnt1.byte = {0x01, 0x00, 0x00, 0x00},
		.rep_group_cnt2.byte = {0x01, 0x00, 0x00, 0x00},
		.rep_ovr_cnt1.byte   = {0x01, 0x00, 0x00, 0x00},
		.freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG hard_stop_extend_values =
{
	// 10 25 3 25 3 10 0 1 100 1 100 0 10 10 1 128
	.duty_cycle1.byte    = {0x27, 0x00},
	.duty_cycle2.byte    = {0x62, 0x00},  // 25
	.duty_cycle3.byte    = {0x62, 0x00},  // 25
	.duty_cycle4.byte    = {0x62, 0x00},  // 25
	.duty_cycle5.byte    = {0x62, 0x00},  // 25
	.duty_cycle6.byte    = {0x27, 0x00},
	.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_group_cnt1.byte = {0xE8, 0x03, 0x00, 0x00},
	.rep_group_cnt2.byte = {0xE8, 0x03, 0x00, 0x00},
	.rep_ovr_cnt1.byte   = {0x01, 0x00, 0x00, 0x00},
	.freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG coarse_extend_values =
{
    // 10 25 3 25 3 10 0 1 100 1 100 0 10 10 1 128
    .duty_cycle1.byte    = {0x27, 0x00},
    .duty_cycle2.byte    = {0x62, 0x00},  // 25
    .duty_cycle3.byte    = {0x62, 0x00},  // 25
    .duty_cycle4.byte    = {0x62, 0x00},  // 25
    .duty_cycle5.byte    = {0x62, 0x00},  // 25
    .duty_cycle6.byte    = {0x27, 0x00},
    .rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
    .rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt1.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_group_cnt2.byte = {0x00, 0x00, 0x00, 0x00},
    .rep_ovr_cnt1.byte   = {0xC8, 0x00, 0x00, 0x00},
    .freq.byte           = {0x87, 0x01}
};

static CPLD_CONFIG fine_extend_values =
{
		// 10 25 3 25 3 10 0 1 100 1 100 0 10 10 1 128
		.duty_cycle1.byte    = {0x27, 0x00},
		.duty_cycle2.byte    = {0x69, 0x00}, // 27%
		.duty_cycle3.byte    = {0x69, 0x00}, // 27%
		.duty_cycle4.byte    = {0x69, 0x00}, // 27%
		.duty_cycle5.byte    = {0x69, 0x00}, // 27%
		.duty_cycle6.byte    = {0x27, 0x00},
		.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
		.rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
		.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
		.rep_group_cnt1.byte = {0x01, 0x00, 0x00, 0x00},
		.rep_group_cnt2.byte = {0x01, 0x00, 0x00, 0x00},
		.rep_ovr_cnt1.byte   = {0x01, 0x00, 0x00, 0x00},
		.freq.byte           = {0x87, 0x01}
};


// Test for nudging - 8 pulses at 73%
static CPLD_CONFIG fine_retract_nudge_values =
{
	// 10 75 97 75 97 10 0 1 100 1 100 0 10 10 1 128
	.duty_cycle1.byte    = {0x27, 0x00},
	.duty_cycle2.byte    = {0x1d, 0x01},
	.duty_cycle3.byte    = {0x1d, 0x01},
	.duty_cycle4.byte    = {0x1d, 0x01},
	.duty_cycle5.byte    = {0x1d, 0x01},
	.duty_cycle6.byte    = {0x27, 0x00},
	.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_group_cnt1.byte = {0x01, 0x00, 0x00, 0x00},
	.rep_group_cnt2.byte = {0x01, 0x00, 0x00, 0x00},
	.rep_ovr_cnt1.byte   = {0x01, 0x00, 0x00, 0x00},
	.freq.byte           = {0x87, 0x01}
};

// Test for nudging - 8 pulses at 27%
static CPLD_CONFIG fine_extend_nudge_values =
{
	// 10 25 3 25 3 10 0 1 100 1 100 0 10 10 1 128
	.duty_cycle1.byte    = {0x27, 0x00},
	.duty_cycle2.byte    = {0x69, 0x00}, // 27%
	.duty_cycle3.byte    = {0x69, 0x00}, // 27%
	.duty_cycle4.byte    = {0x69, 0x00}, // 27%
	.duty_cycle5.byte    = {0x69, 0x00}, // 27%
	.duty_cycle6.byte    = {0x27, 0x00},
	.rep_cnt1.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_cnt2.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt3.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt4.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt5.byte       = {0x01, 0x00, 0x00, 0x00},
	.rep_cnt6.byte       = {0x00, 0x00, 0x00, 0x00},
	.rep_group_cnt1.byte = {0x01, 0x00, 0x00, 0x00},
	.rep_group_cnt2.byte = {0x01, 0x00, 0x00, 0x00},
	.rep_ovr_cnt1.byte   = {0x01, 0x00, 0x00, 0x00},
	.freq.byte           = {0x87, 0x01}
};


#endif /* BOARD_VERSION_P1_1 */

/* Private functions prototype -----------------------------------------------*/
/* Private task functions prototype ------------------------------------------*/
#ifdef BOARD_VERSION_P1
STATIC UInt8 IncrementSensorReg(msm_camera_i2c_reg_array *reg, cam_reg_array *regs, UInt16 address);
STATIC UInt8 ConfigureSensorFlips(CamDevice_TypeDef *cam, cam_reg_array *ResolutionRegs);
#endif
#ifdef BOARD_VERSION_P1_1
static uint8_t update_resolution_register(CamDevice_TypeDef *cam, cam_reg_array *regs);
STATIC cam_reg_array* resolution_selection(UInt16 width, UInt16 height, UInt16 fps, UInt8* size);
UInt8 SetSensorStreaming(CamID ChannelID, UInt8 SlaveAddress, Int8 Intr, UInt8 StreamOn);
STATIC UInt8 IncrementSensorReg(msm_camera_i2c_reg_array *reg, cam_reg_array *regs, UInt16 address, UInt8 size);
STATIC UInt8 ConfigureSensorFlips(CamDevice_TypeDef *cam, cam_reg_array *ResolutionRegs, UInt8 regsize);
#endif /* BOARD_VERSION_P1_1 */
STATIC UInt8 SensorConfig(CamDevice_TypeDef *cam, cam_reg_array *regs, UInt16 size);
STATIC UInt8 get_require_data_size(UInt32 bitmask, UInt32 cmd);
STATIC Bool  check_require_data_size(UInt32 bitmask, UInt32 cmd, UInt16 received_size);
STATIC void cam_init_queue(void);
void cam_cmd_task(void *pvParameters);
STATIC UInt8 SensorsSetting(unsigned int cam_bitmask, cam_reg_array *regs, UInt16 size);
STATIC UInt8 SensorTransferFrame(UInt32 i2c_channel_mask);
STATIC Bool read_fpga_version(UInt8 *major, UInt8 *minor);
STATIC cam_cmd_status get_command_status(uint16_t tid);
STATIC char *cmd_status_to_string(cam_cmd_status status);

UInt8 Config_I2C_message(UInt32 I2C_Channel_Mask, UInt16 reg_addr, UInt16 reg_data, I2CMESSAGETYPE mode);
__IO UInt8 af_command;

void CPLD_SPI_Transfer(uint8_t* tx_buffer, uint8_t* rx_buffer,
					   uint16_t num_bytes);
void start_PWM_control(PiezoActuator* actuator, uint16_t CPLD_select, uint8_t is_lens, uint32_t delay);
void calibrate_freq(CamDevice_TypeDef* pCam, PiezoActuator* actuator);
int run_parallel_lens_move(uint8_t * data, unsigned int cam_mask, uint8_t tolerance);
int run_parallel_mirror_move(uint8_t * data, unsigned int cam_mask);
void init_hall(void);
void fine_nudge(PiezoActuator* actuator, uint16_t CPLD_select, uint8_t direction, uint16_t multiplier, uint8_t is_lens);
extern unsigned int stm_ver;

/* Export task functions -----------------------------------------------------*/
#ifdef BOARD_VERSION_P1_1
static cam_reg_array* resolution_selection(UInt16 width, UInt16 height, UInt16 fps, UInt8* size)
{
	int i = 0;
	msm_camera_stream_type *stream_type = NULL;
	for(i = 0; i < ARRAY_COUNT(stream_types); i++)
	{
		stream_type = (msm_camera_stream_type *)&(stream_types[i]);
		if(stream_type->fps == fps &&
			stream_type->height == height &&
			stream_type->width == width)
		{
			if(size)
				*size = stream_type->sizeofconfig;
			log_debug("stream_type id %d\r\n", i);
			return stream_type->config;
		}
	}
	return NULL;
}

static uint8_t update_resolution_register(CamDevice_TypeDef *cam, cam_reg_array *regs)
{
	ENTER_FUNC;

	if(cam == NULL || regs == NULL)
		return 0;

	cam_reg_array *pReg = regs;
	I2CMESSAGETYPE mode;
	int i;

	pReg += 2; /* jump to resolution struct */

	/* select I2C mode */
	if(pReg->data_size == 1)
		mode = ADDR16_DATA8;
	else
		mode = ADDR16_DATA16;

	for(i=0; i < pReg->reg_size; i++)
	{
		fpga_write_i2c_value(cam->camera_id, cam->Address, pReg->regs[i].reg_addr, pReg->regs[i].reg_val, mode, ETrue);
	}

	EXIT_FUNC;
	return 1;
}

#endif /* BOARD_VERSION_P1_1 */
#ifdef BOARD_VERSION_P1
static UInt16 read_sensor_temp(CamDevice_TypeDef *pCam)
{
	UInt16 T1,T2,temp_data;
	float real_temp,S;

	/* Clear data */
	fpga_write_i2c_value(pCam->camera_id, pCam->Address,TEMP_SENSOR_CTRL_REG,
			TEMP_SENSOR_RESET_DATA, ADDR16_DATA16, ETrue);

	/* Turn on temperature sensor and conversion */
	fpga_write_i2c_value(pCam->camera_id, pCam->Address,TEMP_SENSOR_CTRL_REG,
			TEMP_SENSOR_START_READ, ADDR16_DATA16, ETrue);

	/* Need some delay to collect temperature data */
	vTaskDelay(2);

	/* Read temperature data */
	temp_data = fpga_read_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_DATA_REG, ADDR16_DATA16);

	/* Get last 10 bits */
	temp_data &= 0x3ff;

	/* Read calibration data T1 */
	T1 =  fpga_read_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_CAL1_REG, ADDR16_DATA16);

	/* Read calibration data T2 */
	T2 =  fpga_read_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_CAL2_REG, ADDR16_DATA16);

	log_debug("Temp sens: %d Temp cal1: %d Temp cal2: %d \n", temp_data, T1, T2);

	/* Calculate Celsius degree */
	S = (T1 - T2) / 15;
	real_temp = fabs(temp_data - (T2 - S*55))/S;

	fpga_write_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_CTRL_REG, TEMP_SENSOR_RESET, ADDR16_DATA16, ETrue);

	return (UInt16) (real_temp);
}
#else /* BOARD_VERSION_P1_1 */
static UInt16 read_sensor_temp(CamDevice_TypeDef *pCam)
{
	float T1;
	UInt16 test_reg, T2;

	fpga_write_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_CTRL_REG, 0x1, ADDR16_DATA16, ETrue);
	fpga_write_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_CTRL_REG, 0x21, ADDR16_DATA16, ETrue);
	fpga_write_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_CTRL_REG, 0x11, ADDR16_DATA16, ETrue);
	vTaskDelay(2);
	test_reg = fpga_read_i2c_value(pCam->camera_id, pCam->Address,
				TEMP_SENSOR_DATA_REG, ADDR16_DATA16);
	fpga_write_i2c_value(pCam->camera_id, pCam->Address,
				TEMP_SENSOR_CTRL_REG, 0x0, ADDR16_DATA16, ETrue);
	T2 = fpga_read_i2c_value(pCam->camera_id, pCam->Address,
			TEMP_SENSOR_CAL1_REG, ADDR16_DATA16);

	T1 = (float)((test_reg - T2 + 90)/1.51);
	log_debug("Temp sens: %d Temp test: %d Temp cal2: %d \n", (UInt16) T1, test_reg, T2);
	return (UInt16)T1;

}
#endif

void cam_sensor_temp_monitor(void *pvParameters)
{
	ENTER_FUNC;

	(void)pvParameters;
	CamDevice_TypeDef *pCam = CamDeviceTbl[0];
	UInt16 ctemp = 0;

	while(1)
	{
		log_printf("SENSOR TEMPERATURE \n\r");
		for (int i = 0; i < CAM_NUM_CAMERA_MAX; i++)
		{
			 pCam = CamDeviceTbl[i];
			 if(pCam == NULL)
				 continue;
			 ctemp = read_sensor_temp(pCam);
			 log_printf("[%s]: %d \n\r", pCam->Name, ctemp);
		}
		vTaskDelay(59965);
	}

	EXIT_FUNC;
}
xTaskHandle xGyroHandle = NULL;
static UInt8 gyro_read(UInt8 reg)
{
	UInt16 fpga_tx[3];
	UInt16 fpga_rx[3];
	fpga_cmd_error_t fpga_state;
	fpga_tx[0] = 0x1281;
	fpga_tx[1] = 0xd100;
	fpga_tx[2] = (0x00 << 8) | reg;

	fpga_state = fpga_send_command( fpga_tx, fpga_rx, 3, IRQ_FROM_FPGA_TIMEOUT );

	if(fpga_state == FPGA_CMD_TOUT || fpga_state == FPGA_CMD_WAITING)
	{
		log_error("Timeout when sending data to FPGA");
	}
	else
	{
		/* Do nothing*/
	}

	/* Create dummy bytes to read from FPGA */
	for(int i = 0; i < 3; i ++)
	{
		fpga_tx[i] = 0xFFFF;
	}

	/* Send command to FPGA */
	fpga_state = fpga_send_command(fpga_tx, fpga_rx, 3, IRQ_FROM_FPGA_TIMEOUT);
	if(fpga_state == FPGA_CMD_TOUT || fpga_state == FPGA_CMD_WAITING)
	{
		log_error("Timeout when reading data from FPGA");
	}
	else
	{
		/* Delay 1ms for the sensor ready for the new command */
		vTaskDelay(1);
	}
	return (UInt8) fpga_rx[0];
}
void vGyro(void *pvParameters)
{
	UInt16 X_axis;
	UInt16 Y_axis;
	UInt16 Z_axis;
	for(;;)
	{

		*((int *)pvParameters) = 0;

		 /* Read X axis */
		X_axis = gyro_read(0x03) << 8;
		X_axis |= gyro_read(0x02);
		log_printf("X_axis = %04X\n\r", X_axis);
        save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_gyro), 2,
                                LITTLE_ENDIAN, (uint8_t*)&X_axis);
        /*
		cam_m_status->cam_m_gyro[0] = X_axis >> 8;
		cam_m_status->cam_m_gyro[1] = X_axis & 0xFF;
        */

		/* Read Y axis */
		Y_axis = gyro_read(0x05) << 8;
		Y_axis |= gyro_read(0x04);
		log_printf("Y_axis = %04X\n\r", Y_axis);
        save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_gyro + 2), 2,
                                LITTLE_ENDIAN, (uint8_t*)&Y_axis);
        /*
		cam_m_status->cam_m_gyro[2] = Y_axis >> 8;
		cam_m_status->cam_m_gyro[3] = Y_axis & 0xFF;
        */

		/* Read Y axis */
		Z_axis = gyro_read(0x07) << 8;
		Z_axis |= gyro_read(0x06);
		log_printf("Z_axis = %04X\n\r", Z_axis);
        save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_gyro + 4), 2,
                                LITTLE_ENDIAN, (uint8_t*)&Z_axis);
        /*
		cam_m_status->cam_m_gyro[4] = Z_axis >> 8;
		cam_m_status->cam_m_gyro[5] = Z_axis & 0xFF;
        */

		log_printf("\n\r");

		*((int *)pvParameters) = 1;
		taskYIELD();
	}
}

int cam_set_property(unsigned int cam_bitmask, uint16_t ucid, uint8_t * data, ELEMENT element)
{
	UInt16 vcm_position = 0;
	int data_idx = 0;
	int isGlobal = cam_bitmask & 0x01;
	int i;
    AFData* af;
#ifdef BOARD_VERSION_P1_1
	int ret =0;
#endif /* BOARD_VERSION_P1_1*/
	CamDevice_TypeDef *pCam;

	ENTER_FUNC;
    int j = 0;
	for (i = 0; i < CAM_ID_B1 - 1 ; i++)
	{
		if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
		{
			vcm_position = 0;
			pCam = CamDeviceTbl[i];
			vcm_position = (data[data_idx + 0] << 0x8) | data[data_idx + 1] ;
            af = &AFInfo[pCam->camera_id-1];

			if(!isGlobal)
				data_idx += 2;
			if(pCam->Type == CAM_TYPE_35MM)
			{
#ifdef BOARD_VERSION_P1
				switch (element)
				{
				case VCM_POSITION:
					add_setting(element, vcm_position, i, get_active_ucid());
					log_info("[%s] Applied %s setting", pCam->Name, element_to_string(element));
                    save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_lens_hall + j), 2,
                                            LITTLE_ENDIAN, (uint8_t*)&(af->CurrentPosition));
                    j += 2;
					break;
				default:
					log_debug("%s is not yet supported for module [%s]\n", element_to_string(element), pCam->Name);
					break;
				}
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
				log_debug("[%s] VCM position: [0x%x], UCID: [%x] ", pCam->Name, vcm_position, ucid);
				add_setting(VCM_POSITION, vcm_position, i, ucid);
				if(ucid != get_active_ucid())
				{
					log_info("[%s] Saved VCM position setting for %s", pCam->Name, ucid_to_text(ucid));
				}
				else
				{
					log_info("[%s] Applied VCM position setting for %s", pCam->Name, ucid_to_text(ucid));
                    save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_lens_hall + j), 2,
                                            LITTLE_ENDIAN, (uint8_t*)&(af->CurrentPosition));
                    j += 2;
				}
#endif /* BOARD_VERSION_P1_1 */
			}
			else
			{
				log_warning("%s is not supported for module [%s]\n", element_to_string(element), pCam->Name);
			}
		}
	}

	EXIT_FUNC;
#ifdef BOARD_VERSION_P1
	return data_idx;
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
	return ret;
#endif /* BOARD_VERSION_P1_1 */
}

void init_hall()
{
	ENTER_FUNC;
	int i;
	CamDevice_TypeDef *pCam;
	PiezoActuator *lens;
	PiezoActuator *mirror;
	PiezoModule * p_module;
	for (i = 1 ; i < CAM_ID_MAX; i ++)
	{
		pCam = CamDeviceTbl[i - 1];
		p_module = CameraIdToModule(pCam->camera_id);

		if (p_module == NULL)
		{
			log_debug("No piezo actuators connected: %s\n",pCam->Name);
			continue;
		}
		lens = p_module->Lens;
		mirror = p_module->Mirror;
		if ((!lens->IsConnected) || (!mirror->IsConnected))
		{
			log_debug("No piezo actuator connected: %s\n",pCam->Name);
			continue;
		}

		ReadHallCalibration(pCam, pCam->camera_id, &(p_module->Lens->CalibData),
				&(p_module->Mirror->CalibData));


		InitHallSensor(&(lens->Hall));
		InitHallSensor(&(mirror->Hall));

	}
	EXIT_FUNC;
}

void cam_read_lens_position(unsigned int cam_bitmask)
{
	int i;
	int isGlobal = cam_bitmask & 0x1;
	CamDevice_TypeDef * pCam;
	uint16_t data;
	UInt8 j = 0;
	//unsigned long time_start = 0, time_end = 0, total_time = 0;
	for(i = 1; i < CAM_ID_MAX; i++)
	{

		if (((cam_bitmask) & 0x1<<i) != 0 || isGlobal)
		{
			pCam = CamDeviceTbl[i -1];
            //time_start = xTaskGetTickCount();

			if(pCam->Type == CAM_TYPE_35MM)
			{

				uint16_t position;
				AFData* af;
				af = &AFInfo[pCam->camera_id-1];
				//position  = af_controller_get_current_pos(af,
				//			pCam->camera_id);
				position  = af->CurrentPosition;

                save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_lens_hall + j), 2,
                                        LITTLE_ENDIAN, (uint8_t*)&position);
                j += 2;
                /*
				cam_m_status->cam_m_lens_hall[j++] = (uint8_t)(position >> 8);
				cam_m_status->cam_m_lens_hall[j++]= (uint8_t)position;
                */

				log_printf("\nModule %s ", pCam->Name);
				log_printf("LENS value	: 0x%04x\n", position);
				log_printf("\r\n");


			}
			else if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
			{
				PiezoActuator *a;
				PiezoModule   *m;

				m = CameraIdToModule(pCam->camera_id); // +1 converts from i to bitmask index
				a = m->Lens;
				if (!a->IsConnected)
				{
					log_error("The lens actuator for module %s is not connected.", pCam->Name);
					continue;
				}
				// If the hall sensor has been initialized, use the existing values.
				// If not, read the EEPROM values for the lens, and use defaults
				// for the mirrors to do calibration.
				//if (m->IsCalibrated) // calibration? check initialization instead
				if (!(a->Hall.IsInitialized))
				{
					ReadHallCalibration(pCam, i, &(m->Lens->CalibData), &(m->Mirror->CalibData));
					InitHallSensor(&(a->Hall));
				}

				data = ReadHallSensor(&(a->Hall));
                save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_lens_hall + j), 2,
                                        LITTLE_ENDIAN, (uint8_t*)&data);
                j += 2;
                /*
				cam_m_status->cam_m_lens_hall[j++] = (uint8_t)(data >> 8);
				cam_m_status->cam_m_lens_hall[j++] = (uint8_t)data;
                */

                log_printf("\nModule %s: ", pCam->Name);
                log_printf("LENS value          : 0x%04x\r\n", data);
                log_printf("RETRACTED hard stop : 0x%04x\r\n", a->CalibData.NearPosition);
                log_printf("EXTENDED  hard stop : 0x%04x\r\n", a->CalibData.FarPosition);
            }
            else
            {
                log_debug("Command not supported for module %s", pCam->Name);
            }

            /*
            time_end = xTaskGetTickCount();
            total_time = (time_end - time_start);
            log_time("Time taken to read %s lens position: %d milliseconds\n", pCam->Name, (unsigned int)total_time);
            */
        }
	}
}

void cam_read_mirror_position(unsigned int cam_bitmask)
{
	int i;
	int isGlobal = cam_bitmask & 0x1;
	CamDevice_TypeDef * pCam = NULL;
	uint16_t data;
	PiezoActuator *a;
	PiezoModule   *m;
	UInt8 j = 0;
	// Don't need to check for 70mm/150mm because we start at B1
	for(i = CAM_ID_B1; i < CAM_ID_MAX; i++)
	{
		if (((cam_bitmask) & 0x1<<i) != 0 || isGlobal)
		{
			pCam = CamDeviceTbl[i - 1];

			m = CameraIdToModule(pCam->camera_id); // +1 converts from i to bitmask index
			a = m->Mirror;

			if ((!a->IsConnected) || (i == CAM_ID_B4) || (i == CAM_ID_C5) || (i == CAM_ID_C6))
			{
				log_error("The mirror actuator for module %s is not connected.", pCam->Name);
                data = 0xdead;
			}
            else
            {
                // If the hall sensor has been initialized, use the existing values.
                // If not, read the EEPROM values for the lens, and use defaults
                // for the mirrors to do calibration.
                //if (m->IsCalibrated) // calibration? check initialization instead
                if (!(a->Hall.IsInitialized))
                {
                    ReadHallCalibration(pCam, i, &(m->Lens->CalibData), &(m->Mirror->CalibData));
                    InitHallSensor(&(a->Hall));
                }

                data = ReadHallSensor(&(a->Hall));
            }

            save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_mirror_hall + j), 2,
                                    LITTLE_ENDIAN, (uint8_t*)&data);
            j += 2;
            /*
			cam_m_status->cam_m_mirror_hall[j++] = (uint8_t)(data >> 8);
			cam_m_status->cam_m_mirror_hall[j++] = (uint8_t)data;
            */

			log_printf("\nModule %s: ", pCam->Name);
			log_printf("MIRROR value     : 0x%04x\r\n", data);
			log_printf("WIDE hard stop   : 0x%04x\r\n", a->CalibData.NearPosition);
			log_printf("NARROW hard stop : 0x%04x\r\n", a->CalibData.FarPosition);

		}
	}
}
UInt8 set_vc_capture(UInt8 count, UInt8 *CamOrder, UInt8 *ControlStatusReg, UInt8 *VirtualChannelIdentifier, UInt8 *DataType)
{
	ENTER_FUNC;

	UInt8 ret = 0;
	UInt8 len = 5;
	UInt16 tx[5];
	UInt16 rx[5];
	fpga_cmd_error_t tmp = 0;

	log_debug("Sending capture settings command to the FPGA \n\r");
	/*0x10B00010 to 0x00300FF0 */
	/* for p1.1 0x10B00010 to 0x00301040 */
	tx[0] = CAPTURE_VC_CHANNEL_CMD;
	tx[1] = (CAPTURE_VC_REG_ADDR >> 16) & 0xFFFF;
	tx[2] = CAPTURE_VC_REG_ADDR & 0xFFFF;
#ifdef BOARD_VERSION_P1_1
	tx[4] = 0x1040;
#else
	tx[4] = 0x0FF0;
#endif

	switch(VirtualChannelIdentifier[0] & 0x0F)
	{
	case CAM_VC_0:
		log_debug("VC channel VC0 selected \n");
		 tx[3] = (0x0 | (FPGA_CAPTURE_VC0 << 0x4)) ;
		break;
	case CAM_VC_1:
		log_debug("VC channel VC1 selected \n");
		tx[3] = (0x0 | (FPGA_CAPTURE_VC1 << 0x4));
		break;
	case CAM_VC_2:
	case CAM_VC_3:
	default:
		log_debug("Default VC channel VC0 selected \n");
		tx[3] = (0x0 | (FPGA_CAPTURE_VC0 << 0x4));
		break;
	}
	tmp = fpga_send_command(tx, rx, len, IRQ_FROM_FPGA_TIMEOUT);
	if(tmp == FPGA_CMD_TOUT)
	{
		log_printf("capture VC failed: no response from the FPGA!\n\r");
		ret = 3;
		goto EXIT;
	}
	ret = 1;
EXIT:
	EXIT_FUNC;
	return ret;
}

uint32_t get_interrupt_mask(uint16_t cmd_tid, uint16_t status)
{
	uint32_t ret = 0x8000FFFF;

	ret = ((0x0000FFFF & status) << 16 | cmd_tid);
	log_debug("setting the interrupt to 0x%x\n\n",(unsigned int)ret);
	return ret;

}

void vCamController(void *pvParameters)
{
	ENTER_FUNC;

	(void)pvParameters;
	CamDevice_TypeDef *pCam = CamDeviceTbl[0];
	UInt8 i, sta = 0, data_idx = 0;
	UInt16 data;
	UInt16 fll;
	double fll_msec = 0;
	uint32_t fll_msec_int = 0;
	portBASE_TYPE xStatus;
	cam_cmd_t CurrentCmd;
	UInt8 isGlobal = 0;
	UInt16 tid = 0;
	unsigned long time_start = 0, time_end = 0, total_time = 0;

	cam_ctrl = *CCB_ASIC_CAM_RD = 0;
	cam_bitmask = *CCB_M_BITMASK = 0;
	stream_bitmask = *CCB_M_BITMASK = 0;
	/*reset all memories*/
	memset((void *)cam_m_status, 0, sizeof(ccb_cmd_base_t));

	/* Reset all status*/
	StatusResetAll();
	/*Reset all focus status*/
	cam_focus_status_reset_all();

	/*
	 * Check Power Good signal of FPGA and Camera sensors at start up
	 */
	if(fpga_check_cam_pg() == FPGA_PWR_GOOD)
	{
		log_printf("CAM PG OK \n\r");
	}
	else
	{
		log_error("CAM PG is not OK!\n\r");
		log_error("Please turn on the FPGA!\n\r");
	}

	// Init the Piezo hall sensors
	init_hall();
	cam_init_queue();

	/* Create one of the two tasks. */
	xTaskCreate(	cam_cmd_task,
					(const signed char * const)"CamCmd",
					300,
					NULL,
					1,
					NULL);
#ifdef BOARD_VERSION_P1
	xTaskCreate(	vGyro,
	(const signed char * const)"vGyro",
	200,
	(void*) &gyro_read_flag,
	tskIDLE_PRIORITY + 1,
	&xGyroHandle);

	vTaskSuspend(xGyroHandle);
#endif
/*
	xTaskCreate(	cam_sensor_temp_monitor,
						(const signed char * const)"CamSensorMonitor",
						300,
						NULL,
						tskIDLE_PRIORITY+4,
						NULL);
*/
	/* Load use-case from flash */
	initialize_usecase_manager();
	/* As per most tasks, this task is implemented in an infinite loop. */
	while(1)
	{
		do{
			xStatus = xQueueReceive(cam_cmd_queue, &CurrentCmd, 10 /   portTICK_RATE_MS);
		} while (xStatus != pdPASS);

		log_debug("Command received! Command ID = %x\n", CurrentCmd.cmd);
		log_debug("0x%x is Interrupt enabled ? %s \n\n",
				CurrentCmd.flags, CurrentCmd.flags & 0x08 ? "YES" : "NO");
		tid = CurrentCmd.tid;
		switch (CurrentCmd.cmd)
		{
			/*Open cmd*/
			case CAM_MODULE_OPEN:
			{
				// Messy, but we need to maintain separate variables for each state
				// because the user can specify certain cameras for SW_STANDBY,
				// HW_STANDBY, or CLOSE
				cam_open_data_t cam_open_data;

				cam_open_data.hw_standby_bitmask     = 0;
				cam_open_data.sw_standby_bitmask     = 0;
				cam_open_data.close_bitmask          = 0;
				cam_open_data.hw_standby_slave_order = 0;
				cam_open_data.sw_standby_slave_order = 0;
				cam_open_data.close_slave_order      = 0;
				cam_open_data.current_cam_status     = 0;

				data_idx = 0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, CMD_INVALID_ARG);
					break;
				}
				if(!CurrentCmd.data_len)
				{
					log_info("Reading out current module states:\n");
					// Read the current module statuses
					UInt8 j = 0;
					for(i = 0; i < CAM_NUM_CAMERA_MAX; i++)
					{
						if(((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
						{
							pCam = CamDeviceTbl[i];
							// The MODULE_OPEN status only needs the first 6 bits
                            // Does not need save_cam_m_status_field because it
                            // is one byte
							cam_m_status->cam_m_open[j++] = (uint8_t) (__cam_status [i] & 0xFF);
							cam_open_data.current_cam_status = __cam_status [i];
							log_debug("%s: %s mode", pCam->Name, cam_open_data.current_cam_status & MODULE_HW_STANDBY ? "HW_STANDBY":
								cam_open_data.current_cam_status & MODULE_SW_STANDBY ? "SW_STANDBY": "CLOSED");
						}
					}
					save_command_log(tid, CMD_SUCCESS);
					break;
				}
				else // Write the current module status
				{
					for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
					{
						if (((cam_bitmask >> 1) & 1 << i) != 0 || isGlobal)
						{
							vTaskDelay(300);
							cam_ctrl = CurrentCmd.data[data_idx];
							pCam = CamDeviceTbl[i];
							cam_module_open_iterate(i, cam_ctrl, pCam, &cam_open_data, tid);

							if (!isGlobal)
								data_idx++;
						}
					}
					cam_module_open_apply(isGlobal, &cam_open_data);
				}
				/* CMD_PENDING is the default status of transaction when coming
				 * into the command parser. If the command status at this point
				 * is PENDING means there is no error occur. So, the
				 * transaction would be a SUCCESS transaction. In case of some
				 * errors occurred, that errors have been logged so that
				 * we do not reset it here */
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			/*Stream cmd*/
			case CAM_STREAMING:
			{
				time_start = xTaskGetTickCount();
				UInt8 ControlStatusReg[CurrentCmd.data_len/3];
				UInt8 VirtualChannelIdentifier[CurrentCmd.data_len/3];
				UInt8 DataType[CurrentCmd.data_len/3];
				UInt8 CamOrder[CurrentCmd.data_len/3];
				UInt64 max_exposure = 0;
				UInt32 i2c_channel_mask = 0;
				UInt8 j = 0;
				data_idx = 0;
				stream_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;

				i2c_channel_mask = isGlobal;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(stream_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				if(!CurrentCmd.data_len)
				{
					for(i = 0; i < CAM_NUM_CAMERA_MAX; i++)
					{
						if(((stream_bitmask >> 1) & 1<<i) != 0 || isGlobal)
						{
							pCam = CamDeviceTbl[i];
							/* save all setting of camera*/
                            save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_stream + j),
                                                    3, LITTLE_ENDIAN, (uint8_t*)&cam_streaming_array[i * 3]);
                            j += 3;

                            /*
							cam_m_status->cam_m_stream[j++] = cam_streaming_array[i*3];
							cam_m_status->cam_m_stream[j++] = cam_streaming_array[i*3 + 1];
							cam_m_status->cam_m_stream[j++] = cam_streaming_array[i*3 + 2];
                            */

							log_debug("%s : Data type = %x",pCam->Name, cam_streaming_array[i*3]);
							log_debug("%s : Virtual channel identtifier = %x", pCam->Name,cam_streaming_array[i*3 + 1]);
							log_debug("%s : Control status register = %x",pCam->Name, cam_streaming_array[i*3 + 2]);
						}
					}
					save_command_log(tid, CMD_SUCCESS);
					break;
				}
#ifdef BOARD_VERSION_P1
				j = 0;
#endif /* BOARD_VERSION_P1 */
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((stream_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						stream_ctrl = CurrentCmd.data[data_idx + 2] & 0x1;
						/* cache current setting*/
						cam_streaming_array[i*3] = CurrentCmd.data[data_idx];
						cam_streaming_array[i*3 + 1] = CurrentCmd.data[data_idx + 1];
						cam_streaming_array[i*3 + 2] = CurrentCmd.data[data_idx + 2];

						if (stream_ctrl == CAM_STREAM_ON)
						{
							if(!(StatusActivated(i, MODULE_SW_STANDBY)))
							{
								log_error("%s is not in SW_STANDBY \r\n", pCam->Name);
								log_error("Need to open %s in SW_STANDBY before streaming on \r\n", pCam->Name);
								continue;
							}
							else if (StatusActivated(i, MODULE_STREAM_ON)  && get_active_ucid() == UC_PREVIEW)
							{
								log_info("%s is already streamed on \r\n", pCam->Name);

								/* TODO: Need more discussion about re-setting for Preview after take snapshot
								 * The preview be restored only if resetting trigger and send preview command
								 */
								//	continue;
							}
							i2c_channel_mask |=  (1 << (i+1));

							ControlStatusReg[j]			= CurrentCmd.data[data_idx + 2];
							VirtualChannelIdentifier[j] = CurrentCmd.data[data_idx + 1];
							DataType[j]					= CurrentCmd.data[data_idx];
							CamOrder[j]					= pCam->camera_id;
							j++;

						}
						else if(stream_ctrl == CAM_STREAM_OFF)
						{
							if(!(StatusActivated(pCam->camera_id - 1, MODULE_STREAM_ON)))
							{
								log_debug("%s cam stream off\n", pCam->Name);
								continue;
							}

							if (get_active_ucid() == UC_PREVIEW)
								cam_stream_off(pCam->camera_id);

							Trigger(1 << (i + 1), 0);
							StatusDisable(i, MODULE_STREAM_ON);
							StatusDisable(i, CSI_CAM_CHANNEL_0);
							StatusDisable(i , CSI_CAM_CHANNEL_1);
							log_printf("%s stream off successful\n\r", pCam->Name );
							CurrentCamera = NULL;
						}
						else
						{
							log_error("Invalid input data!!!");
							*(CCB_ASIC_CAM_RD + 2) = -1;
							save_command_log(tid, ERROR_INVALID_ARG);
						}
						if(!isGlobal)
						  data_idx += 3;
					}
				}

				if((i2c_channel_mask & 0x01) && (stream_ctrl == CAM_STREAM_OFF))
					i2c_channel_mask = 0;

				if(i2c_channel_mask != 0 && get_active_ucid() == UC_HIRES_CAPTURE)
				{
					log_debug("i2c_channel_mask 0x%x\n", (unsigned int )i2c_channel_mask);
					//i2c_channel_mask = GROUP_AB;
					sta = Preview_Off(i2c_channel_mask);
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to turn off preview!");

					/* Stop Trigger for all requested camera */
					sta = Trigger(i2c_channel_mask, 0);
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to stop the trigger!");

					/* Set up the virtual channel */
					sta = set_vc_capture(j, CamOrder, ControlStatusReg, VirtualChannelIdentifier, DataType);
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to set VC channel for capture!");
					/* Configure Offsets based on Capture group requested*/
#ifdef BOARD_VERSION_P1
					sta = Configure_offsets_trigger(i2c_channel_mask);
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
					sta = Configure_offsets_trigger_p1_1(i2c_channel_mask);
#endif /* BOARD_VERSION_P1 */
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to configure trigger offsets!");


					/* Configure Double LLPCLK for A cameras if capture group is AB */
					/* Configure Double LLPCLK for B cameras if capture group is BC */
					/* Configure Single LLPCLK for C cameras if capture group is C */
#ifdef BOARD_VERSION_P1
					Configure_LLPCLK(i2c_channel_mask);
#endif
#ifdef BOARD_VERSION_P1_1
					stream_bitmask &= 0xFFFFFFFE;
					log_printf("stream_bitmask 0x%x\n",stream_bitmask);
					Configure_LLPCLK(stream_bitmask, i2c_channel_mask);
#endif
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to configure line length pixel clocks!");

					/* Configure Half CIT for A cameras if capture group is AB */
					/* Configure Half CIT for B cameras if capture group is BC */
					/* Configure Full CIT for C cameras if capture group is C */
#ifdef BOARD_VERSION_P1
					Configure_CIT(i2c_channel_mask);
#endif
#ifdef BOARD_VERSION_P1_1
					Configure_CIT(stream_bitmask, i2c_channel_mask);
#endif
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to configure coarse integration time!");

					/* max_exposure is the maximum exposure set for the requested modules */
					/* Find max. exposure set from the UCID settings for the requested modules AB,BC or C */
					CamID cam_id = find_max_exposure(i2c_channel_mask, &max_exposure);

					/*If max exposure is in A group then, FLL(A) = max_exposure + 2 ; FLL(B) = FLL(A) */
					/*If max exposure is in B group then FLL(B) = max_exposure + 2 ; FLL(A) = FLL(B) / 2 */
					fll = max_exposure + 2;
					log_debug("fll %x \n\r", fll);
					/* Set FLL for all sensors */
#ifdef BOARD_VERSION_P1
					sta = Configure_FLL( &fll, i2c_channel_mask, cam_id );
#endif
#ifdef BOARD_VERSION_P1_1
					sta = Configure_FLL(stream_bitmask, &fll, i2c_channel_mask, cam_id );
#endif
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to configure frame length lines!");

#ifdef BOARD_VERSION_P1
					fll_msec = calculate_fll_msec(fll);
#endif
#ifdef BOARD_VERSION_P1_1
					pCam = CamDeviceTbl[cam_id - 1];
					fll_msec = calculate_fll_msec(pCam, fll);
#endif

					fll_msec_int = (uint32_t) fll_msec;
					log_debug("Calculated fll_msec in msec: %f \n\r", fll_msec);
					log_debug("Calculated fll_msec in msec: %d \n\r", (unsigned int) fll_msec_int);

#if 1
					/* Configure trigger interval based off FLL*/
					/* Hard code I2C channel mask for Configure Trigger as module bit mask
					  is not used at all by FPGA for this command */
#ifdef BOARD_VERSION_P1
					sta = Config_Trigger(fll, 0xFFFFFFFF);
#endif
#ifdef BOARD_VERSION_P1_1
					sta = Config_Trigger(pCam, fll, 0xFFFFFFFF);
#endif
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to configure the trigger!");



					/* Send Trigger to flush out frames */
					sta = Trigger(i2c_channel_mask, 1);
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to start the trigger!");

					/* Let first few frames flush out from the sensor to get it adjusted to CIT */
                    /* Flush 5 frames for P1, flush 2 frames for P1.1.  */

#ifdef BOARD_VERSION_P1
					/* 5 * fll */
					vTaskDelay(5 * fll_msec_int);
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
					/* 2 * fll */
					vTaskDelay(2 * fll_msec_int);
#endif /* BOARD_VERSION_P1_1 */

					/* Trigger Off for flushing frames */
					sta = Trigger(i2c_channel_mask, 0);
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to stop the trigger!");
#endif


					/* Configure which frame to dump. Use 2nd frame */
#ifdef BOARD_VERSION_P1
					sta = ConfigureFrameToCapture(2);
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
					printf("cam_trigger %d cam_frame %d \n", cam_trigger, cam_frame);
					sta = ConfigureFrameToCapture(cam_frame);
#endif
					if (sta != FPGA_CMD_SUCCESS)
						log_error("Failed to configure the frame to capture!");
#ifdef BOARD_VERSION_P1
					log_printf("Configuring the trigger to send 2 pulses\n");
					sta = Config_Trigger(fll, 0x2);
#endif
#ifdef BOARD_VERSION_P1_1
					log_printf("Configuring the trigger to send %d pulses and %d frame\n", cam_trigger, cam_frame);
					sta = Config_Trigger(pCam, fll, cam_trigger);
#endif



				}
				else if (i2c_channel_mask != 0 && get_active_ucid() == UC_PREVIEW)
				{
					/* Configure trigger pulses in Preview UC */
					for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
					{
						if (i2c_channel_mask & (1 << (i + 1)))
						{
							pCam = CamDeviceTbl[i];
							/* TODO: Read FLL for requested camera module & then send that value*/
							/* Change function prototype as described above*/
							/* FLL = READ_FLL(I2C_CHANNEL_MASK)*/
							fll = fpga_read_i2c_value(pCam->camera_id, 0x6c, 0x0340, ADDR16_DATA16);
							log_debug("FLL = %x\n\r", fll);
#ifdef BOARD_VERSION_P1
							sta = Config_Trigger(fll, 0xFFFFFFFF);
#endif
#ifdef BOARD_VERSION_P1_1
							sta = Config_Trigger(pCam, fll, 0xFFFFFFFF);
#endif
							log_printf("Configure frame for Preview %s %s\n\r", pCam->Name, (sta == 1) ? "ok" : "fail");
#ifdef BOARD_VERSION_P1_1
							fpga_write_i2c_value(pCam->camera_id, 0x6C, 0x3F3C, 0x0003, ADDR16_DATA16, EFalse);
							fpga_write_i2c_value(pCam->camera_id, 0x6C, 0x301A, 0x031C, ADDR16_DATA16, EFalse);
#endif /* BOARD_VERSION_P1_1 */
						}
					}

					/* Configure for preview */
					if (j > 0)
					{
						uint32_t preview_stream_cams = i2c_channel_mask;
						if (j > 2)
						{
							preview_stream_cams = GET_FIRST_SET_BIT(i2c_channel_mask);
							preview_stream_cams |= GET_FIRST_SET_BIT((i2c_channel_mask) & (i2c_channel_mask - 1));
							log_debug("Received more than 2 cameras defaulting it to 2\n");
							j = 2;
						}
						log_debug("preview_stream_cams  mask 0x%x\n", (unsigned int)preview_stream_cams);

						for (i = 1; i < CAM_NUM_CAMERA_MAX + 1; i++)
						{
							if ((~preview_stream_cams) & ((0x1) << i))
							{
								if(!(StatusActivated(i - 1, MODULE_STREAM_ON)))
								{
									continue;
								}
								StatusDisable(i - 1, MODULE_STREAM_ON);
								StatusDisable(i - 1, CSI_CAM_CHANNEL_0);
								StatusDisable(i - 1, CSI_CAM_CHANNEL_1);
							}
						}

						/* Send Trigger for PREVIEW */
						Trigger(preview_stream_cams, 1);

						/*FIXME: For now we support only 2 camera preview */
						sta = Preview(INTERRUPT_ENABLE, j,  CamOrder, ControlStatusReg,
							VirtualChannelIdentifier, DataType);
						/* turn off others after turning preview as fpga hangs! */
						Trigger(~preview_stream_cams & 0x0001FFFE, 0);
						log_debug("Preview %d cameras: status = %d\n", j, sta);
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);

				time_end = xTaskGetTickCount();
				total_time = (time_end - time_start);
				log_debug("time taken in cam_streaming 0x%x \n", (unsigned int)total_time);
				break;
			}
			case RDI_CAPTURE:
			{

				// start time stamp
				time_start = xTaskGetTickCount();

				UInt32 i2c_channel_mask=0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;

				log_printf("Starting RDI capture...\n");

				/*
				*Snapshot
				*Trigger On
				*Delay for FPGA to dump the frames in DDR
				*Transfer Frames
				*Trigger Off
				*
				* */

				i2c_channel_mask = cam_bitmask;
				log_printf("i2c_channel_mask = 0x%x\n", (unsigned int) i2c_channel_mask);

				if( (i2c_channel_mask != GROUP_AB) &&
					(i2c_channel_mask != GROUP_BC) &&
					(i2c_channel_mask != GROUP_C)  )
				{
					log_warning("\n Invalid camera group -- ignoring the request.");
					log_warning("\n Please provide an AB, BC, or C camera group.");
					break;
				}

				/* Send snapshot command to FPGA */
				Snapshot(i2c_channel_mask, INTERRUPT_ENABLE);

//#ifdef BOARD_VERSION_P1
#if 1
					vTaskDelay(fll_msec_int);
#endif

				/* Send Trigger for snapshot */
				sta = Trigger(i2c_channel_mask, 1);

				/* Delay for a while camera store image to memory */
				// TODO: 3 * max exposure + offset delay + time to dump 100MB in memory
				/* Request transfer frame from memory to streaming bus */
				log_debug("fll_msec in msec: %u \n\r", (unsigned int) fll_msec_int);
#ifdef BOARD_VERSION_P1
				log_debug("Starting to read the gyroscope: \n\r");
				gyro_read_flag = 0;
				vTaskResume(xGyroHandle);
				vTaskDelay( (fll_msec_int * 3));
				/* delay 1 ms if Gyro reading hasn't finished yet */
				do
				{
					vTaskDelay(1);
				}
				while(!gyro_read_flag);
				vTaskSuspend(xGyroHandle);
				log_debug("Finished reading the gyroscope! \n\r");
#endif
#ifdef BOARD_VERSION_P1_1

				vTaskDelay(fll_msec_int * (cam_frame + 2));//(fll_msec_int * 10 + 500)
				log_printf("Finished waiting now starting transfer \n\r");
#endif
				sta = SensorTransferFrame(i2c_channel_mask);

				/* Trigger off */
				/*Not needed anymore since only 2 pulses are sent in the Trigger Signal*/
				//sta = Trigger(i2c_channel_mask, 0);
				log_printf("Finished RDI capture\n\r");
				break;
			}
			/*Set Resolution*/
			case CAM_MODULE_RESOLUTION:
			{
				UInt64 resolution;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				data_idx = 0;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				UInt8 j = 0;
				ucid = CurrentCmd.ucid;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						resolution = 0;
						if(!CurrentCmd.data_len)
						{
							get_last_setting(i, ucid, RESOLUTION, &resolution);
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_resolution[j]), 8, LITTLE_ENDIAN, (uint8_t*)&resolution);
                            j += 8;
                            /*
							for(int k = 56 ; k >= 0; k -= 8)
							{
								cam_m_status->cam_m_resolution[j++] = (UInt8)(resolution >> k);
							}
                            */
							log_debug("[%s] Resolution: [X: 0x%x, Y: 0x%x], UCID: [%x] ", pCam->Name, (unsigned int)(resolution >> 32), (unsigned int)(resolution), ucid);
							continue;
						}

						for(j = 0; j < 8; j++)
						{
							/* Shift left each byte to get 64 bit data */
							resolution |= ((UInt64)CurrentCmd.data[data_idx + j]) << (64 - 8*(j+1));
						}
#ifdef BOARD_VERSION_P1_1
						UInt64 old_resolution = 0;
						/* get last resolution */
						get_last_setting(i, ucid, RESOLUTION, &old_resolution);

						if((old_resolution == RESOLUTION_13M && resolution == RESOLUTION_1080) ||
							(old_resolution == RESOLUTION_1080 && resolution == RESOLUTION_13M))
						{
							log_info("[%s] Switch Resolution: [X: 0x%x, Y: 0x%x] to [X: 0x%x, Y: 0x%x], UCID: [%x] ",
									pCam->Name,
									(unsigned int)(old_resolution >> 32), (unsigned int)(old_resolution),
									(unsigned int)(resolution >> 32), (unsigned int)(resolution),
									ucid);

							setting_resolution = UPDATE_RESOLUTION;
						}
						else
						{
							setting_resolution = UPDATE_FULL_SETTING;
						}
#endif

						if(!isGlobal)
							data_idx += 8;
						log_debug("[%s] Resolution: [X: 0x%x, Y: 0x%x], UCID: [%x] ", pCam->Name, (unsigned int)(resolution >> 32), (unsigned int)(resolution), ucid);
						add_setting(RESOLUTION, resolution, i, ucid);
						if(ucid != get_active_ucid())
						{
							log_info("[%s] Saved resolution setting for  %s",pCam->Name, ucid_to_text(ucid));
						}
						else
						{
							log_info("[%s] Applied resolution setting for %s",pCam->Name, ucid_to_text(ucid));
						}
#ifdef BOARD_VERSION_P1_1
						/* Change frame_length_lines and line_length_pclk */
						if(resolution == RESOLUTION_1080)
						{
							pCam->frame_length_lines = FRAME_LENGTH_LINES_1080P;
							pCam->line_length_pclk = LINE_LENGTH_PCLK_1080P;
						}
						else if (resolution == RESOLUTION_13M)
						{
							pCam->frame_length_lines = FRAME_LENGTH_LINES_13M;
							pCam->line_length_pclk = LINE_LENGTH_PCLK_13M;
						}
						else
						{
							pCam->frame_length_lines = fpga_read_i2c_value(pCam->camera_id, pCam->Address, FRAME_LENGTH_LINES_REG_ADDR, ADDR16_DATA16);
							pCam->line_length_pclk = fpga_read_i2c_value(pCam->camera_id, pCam->Address, LINE_LENGTH_PCLK_REG_ADDR, ADDR16_DATA16);
							pCam->line_length_pclk = pCam->line_length_pclk / 2;
						}
#endif
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}

			/* Set Sensitivity*/
			case CAM_MODULE_SENSITIVITY:
			{
				UInt32 sensitivity = 0;
				data_idx = 0;
				UInt8 j = 0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = CurrentCmd.ucid;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						sensitivity = 0;


						if(!CurrentCmd.data_len)
						{
							get_last_setting(i, ucid, SENSITIVITY, (UInt64 *)&sensitivity);
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_sensitivity[j]), 4, LITTLE_ENDIAN, (uint8_t*)&sensitivity);
                            j += 4;

                            /*
							cam_m_status->cam_m_sensitivity[j++] = (UInt8)(sensitivity >> 24);
							cam_m_status->cam_m_sensitivity[j++] = (UInt8)(sensitivity >> 16);
							cam_m_status->cam_m_sensitivity[j++] = (UInt8)(sensitivity >> 8);
							cam_m_status->cam_m_sensitivity[j++] = (UInt8)sensitivity;
                            */


							log_debug("[%s] Sensitivity: [0x%08x], UCID: [%x] ", pCam->Name, (unsigned int)sensitivity, ucid);
							continue;
						}
						for(j = 0; j < 4; j++)
						{
							/* Shift left each byte to get 16 bit data */
							sensitivity |= CurrentCmd.data[data_idx + j] << (32 - 8*(j+1));
						}
						if(!isGlobal)
							data_idx += 4;
						log_debug("[%s] Sensitivity: [0x%08x], UCID: [%x] ", pCam->Name, (unsigned int)sensitivity, ucid);
						add_setting(SENSITIVITY, sensitivity, i, ucid);
						if(ucid != get_active_ucid())
						{
							log_info("[%s] Saved sensitivity setting for %s", pCam->Name, ucid_to_text(ucid));
						}
						else
						{
							log_info("[%s] Applied sensitivity setting for %s", pCam->Name, ucid_to_text(ucid));
						}
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			/*Set Exposure*/
			case CAM_MODULE_EXPOSURE_TIME:
			{
				UInt64 exposure_time;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				data_idx = 0;
				UInt8 j = 0;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = CurrentCmd.ucid;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
                //log_printf("cam_module_exposure_time called\n");
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						exposure_time = 0;
						pCam = CamDeviceTbl[i];

						if(!CurrentCmd.data_len)
						{
                            //log_info("Reading exposure for module %s", pCam->Name);
							get_last_setting(i, ucid, EXPOSURE_TIME, &exposure_time);
                            //log_info("Previous exposure time setting was %llu", (unsigned long long) exposure_time);
                            //log_info("j = %d\n", j);
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_exposure_time[j]), 8, LITTLE_ENDIAN, (uint8_t*)&exposure_time);
                            j += 8;
                            //log_debug("cam_m_status->cam_m_exposure_time[%d] = 0x%x\n", j-8, cam_m_status->cam_m_exposure_time[j-8]);
                            //log_debug("Exposure time:[0x%x%8x] (ns)", (unsigned int) (exposure_time >> 32), (unsigned int)exposure_time);
                            //log_info("j = %d\n", j);
                            /*
							for(int k = 56 ; k >= 0; k -= 8)
							{
								cam_m_status->cam_m_exposure_time[j++] = (UInt8)(exposure_time >> k);
							}
                            */
							//log_info("[%s] Exposure time:[0x%x%8x] (ns), UCID: [%x] ", pCam->Name, (unsigned int) (exposure_time >> 32), (unsigned int)exposure_time, ucid);
							continue;
						}

						for(j = 0; j < 8; j++)
						{
							/* Shift left each byte to get 16 bit data */
							exposure_time |= ((UInt64)CurrentCmd.data[data_idx + j]) << (64 - 8*(j+1));
						}


#ifdef BOARD_VERSION_P1_1
						UInt64 resolution = 0;
						get_last_setting(i, ucid, RESOLUTION, &resolution);

                        if (exposure_time >= 1000000000 ) // 1.0 seconds in nanoseconds
                        {
                            long_exposure_flag = 1;
                            pCam->line_length_pclk = LINE_LENGTH_PCLK_LONG_EXP;
                            pCam->frame_length_lines = FRAME_LENGTH_LINES_LONG_EXP; // This should be around 100msec

                        }
                        else
                        {
                            long_exposure_flag = 0;
                            if (resolution == RESOLUTION_13M)
                            {
                            	log_debug("setting it to 13MP");
                            	pCam->line_length_pclk = LINE_LENGTH_PCLK_13M;
                            	pCam->frame_length_lines = FRAME_LENGTH_LINES_13M;
                            }

                            else if (resolution == RESOLUTION_1080)
                            {
                            	log_debug("Setting default 1080p");
                            	pCam->line_length_pclk = LINE_LENGTH_PCLK_1080P;
                            	pCam->frame_length_lines = FRAME_LENGTH_LINES_1080P;
                            }
                            else
                            {
                            	log_error("setting to default value");
                            	pCam->line_length_pclk = LINE_LENGTH_PCLK_1080P;
                            	pCam->frame_length_lines = FRAME_LENGTH_LINES_1080P;
                            }


                        }

                        fpga_write_i2c_value(pCam->camera_id, pCam->Address, 0x0342,2 * pCam->line_length_pclk, ADDR16_DATA16, ETrue);
                        fpga_write_i2c_value(pCam->camera_id, pCam->Address, 0x0340, pCam->frame_length_lines, ADDR16_DATA16, ETrue);

                        //log_info("Setting exposure time for module %s", pCam->Name);
                        //log_info("sizeof(unsigned long) = %d", sizeof(unsigned long));
                        //log_info("sizeof(unsigned long long) = %d", sizeof(unsigned long long));

						if(!isGlobal)
							data_idx += 8;
						log_debug("[%s] Exposure time:[0x%x%8x] (ns), UCID: [%x] ", pCam->Name, (unsigned int) (exposure_time >> 32), (unsigned int)exposure_time, ucid);
						exposure_time = CONVERT_TO_PLL(pCam->line_length_pclk, exposure_time);
#endif
#ifdef BOARD_VERSION_P1
						exposure_time = CONVERT_TO_PLL(exposure_time);
#endif

						log_debug("Exposure time after converting to PLL: [0x%x%8x] \n" ,(unsigned int) (exposure_time >> 32), (unsigned int)exposure_time);
						add_setting(EXPOSURE_TIME, exposure_time, i, ucid);
						if(ucid != get_active_ucid())
						{
							log_info("[%s] Saved exposure time setting for %s", pCam->Name, ucid_to_text(ucid));
						}
						else
						{
							log_info("[%s] Applied exposure time setting for %s", pCam->Name, ucid_to_text(ucid));
						}
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}

			case CAM_MODULE_FPS:
			{
				UInt16 fps = 0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				data_idx = 0;
				UInt8 j = 0;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = CurrentCmd.ucid;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						fps = 0;
						pCam = CamDeviceTbl[i];
						if(!CurrentCmd.data_len)
						{
							get_last_setting(i, ucid, FPS, (UInt64 *)&fps);
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_fps[j]), 2, LITTLE_ENDIAN, (uint8_t*)&fps);
                            j += 2;

                            /*
							cam_m_status->cam_m_fps[j++] = (UInt8)(fps >> 8);
							cam_m_status->cam_m_fps[j++] = (UInt8)fps;
                            */
							log_debug("[%s] FPS: [0x%x], UCID: [%x] ", pCam->Name, fps, ucid);
							continue;
						}

						for(j = 0; j < 2; j++)
						{
							/* Shift left each byte to get 16 bit data */
							fps |= CurrentCmd.data[data_idx + j] << (16 - 8*(j+1));
						}
						if(!isGlobal)
							data_idx += 2;
						log_debug("[%s] FPS: [0x%x], UCID: [%x] ", pCam->Name, fps, ucid);
						add_setting(FPS, fps, i, ucid);
						if(ucid != get_active_ucid())
						{
							log_info("[%s] Saved FPS setting for %s", pCam->Name, ucid_to_text(ucid));
						}
						else
						{
							log_info("[%s] Applied FPS setting for %s", pCam->Name, ucid_to_text(ucid));
						}
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_MODULE_VCM_POSITION:
			{
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				data_idx = 0;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = CurrentCmd.ucid;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) != EFalse)
				{
					cam_set_property(cam_bitmask, ucid, CurrentCmd.data, VCM_POSITION);
				}
				else
				{
					log_error("Invalid input \n");
					save_command_log(tid, ERROR_INVALID_ARG);
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}

			case CAM_MODULE_FOCUS_DISTANCE:
			{
				//log_printf("reached CAM_MODULE_FOCUS_DISTANCE\n");
				UInt32 focus_distance = 0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				data_idx = 0;
				UInt8 j = 0;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = CurrentCmd.ucid;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						focus_distance = 0;
						pCam = CamDeviceTbl[i];
						if(!CurrentCmd.data_len)
						{
							get_last_setting(i, ucid, FOCUS_DISTANCE, (UInt64 *)&focus_distance);
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_focus_distance[j]), 4, LITTLE_ENDIAN, (uint8_t*)&focus_distance);
                            j += 8;

                            /*
							cam_m_status->cam_m_focus_distance[j++] = (UInt8)(focus_distance >> 24);
							cam_m_status->cam_m_focus_distance[j++] = (UInt8)(focus_distance >> 16);
							cam_m_status->cam_m_focus_distance[j++] = (UInt8)(focus_distance >> 8);
							cam_m_status->cam_m_focus_distance[j++] = (UInt8)focus_distance;
                            */
							log_debug("[%s] Focus distance: [0x%x], UCID: [%x] ", pCam->Name, (unsigned int) focus_distance, ucid);
							continue;
						}
						for(j = 0; j < 4; j++)
						{
							/* Shift left each byte to get 32 bit data */
							focus_distance |= CurrentCmd.data[data_idx + j] << (32 - 8*(j+1));
						}
						if(!isGlobal)
							data_idx += 4;
						log_debug("[%s] Focus distance: [0x%x], UCID: [%x] ", pCam->Name, (unsigned int) focus_distance, ucid);
						add_setting(FOCUS_DISTANCE, focus_distance, i, ucid);
						if(ucid != get_active_ucid())
						{
							log_info("[%s] Saved focus distance setting for %s", pCam->Name, ucid_to_text(ucid));
						}
						else
						{
							log_info("[%s] Applied focus distance setting for %s", pCam->Name, ucid_to_text(ucid));
						}
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_MODULE_LENS_POSITION:
			{
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
                //printf("Module bitmask in LENS_POSITION: %x\n", cam_bitmask);
				data_idx = 0;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = CurrentCmd.ucid;
				int data_index = 0;
				uint8_t tolerance = 0;
				save_command_log(tid, CMD_PENDING);
				if (CurrentCmd.data_len == 0) // This should be a read
				{
					cam_read_lens_position(cam_bitmask);
					save_command_log(tid, CMD_SUCCESS);
				}
				else if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) != EFalse)
				{
					tolerance = CurrentCmd.tolerance;
					// Do the 35mm setting
					if (cam_bitmask & 0x00003f)
						data_index = cam_set_property(cam_bitmask, ucid, CurrentCmd.data,VCM_POSITION);

					// Do the 70mm and 150mm setting
					if (cam_bitmask & 0x01FFC1)
					{
						run_parallel_lens_move(&CurrentCmd.data[data_index], cam_bitmask, tolerance);
					}
					save_command_log(tid, CMD_SUCCESS);
				}
				else
				{
					log_error("Invalid input");
					save_command_log(tid, ERROR_INVALID_ARG);
				}

				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_COMMAND_STATUS:
			{
				tid = CurrentCmd.tid;
				cam_cmd_status status = get_command_status(tid);
				if(status == CMD_UNKNOWN)
				{
					log_warning("Can't find status for transaction id 0x%x \n\r",tid);
				}
				else
				{
                    save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_command), 4, LITTLE_ENDIAN, (uint8_t*)&status);
                    /*
					cam_m_status->cam_m_command[0] = (uint8_t)((uint32_t)status << 24);
					cam_m_status->cam_m_command[1] = (uint8_t)((uint32_t)status << 16);
					cam_m_status->cam_m_command[2] = (uint8_t)((uint32_t)status << 8);
					cam_m_status->cam_m_command[3] = (uint8_t)(status);
                    */
					log_info("Command status for TID 0x%x is %s \r\n",
							tid, cmd_status_to_string(status));
				}
				break;
			}
			case CAM_MODULE_LENS_POSITION2:
			{
				UInt16 DutyCycle = 90; // DUTY_LO
				UInt16 Tolerance =	5;
				data = (UInt16)(CurrentCmd.data[0] << 8 | CurrentCmd.data[1]);
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = (UInt16)(CurrentCmd.data[0] << 8 | CurrentCmd.data[1]);
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				if (CurrentCmd.cmd == CAM_MODULE_LENS_POSITION2)
				{
					// Use extra data bytes for duty cycle.
					Tolerance = (UInt16)(CurrentCmd.data[2]);
					DutyCycle = (UInt16)(CurrentCmd.data[3]);
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
						{
							Error_t e;
							PiezoModule *m;

							m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
							e = MovePiezoToPosition(m->Lens, data, Tolerance, DutyCycle);
							if (e)
							{
								log_debug("MovePiezoToPosition(Lens) returned %d", e);
							}
						}
						else
						{
							log_debug("Unsupported for cam %s", pCam->Name);
						}
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_MODULE_FOCUS_STATUS:
			{
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;
				UInt8 j = 0;
				ucid = CurrentCmd.ucid;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];


						if(pCam->Type == CAM_TYPE_35MM)
						{
							/*
							Strictly speaking, you should not be reading the
							hall sensors for the 35mm cameras in this function.
							However, given that the 35mm cameras do in fact
							have hall sensors, I'll leave this functionality
							here.
							*/
							uint16_t position;
							AFData* af;
							af = &AFInfo[pCam->camera_id-1];
							position  = af_controller_get_current_pos(af,
										pCam->camera_id);
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_lens_hall[j]), 2, LITTLE_ENDIAN, (uint8_t*)&position);
                            j += 2;

                            /*
							cam_m_status->cam_m_lens_hall[j++] = (uint8_t)(position >> 8);
							cam_m_status->cam_m_lens_hall[j++] = (uint8_t)position;
                            */

							log_printf("\nModule %s ", pCam->Name);
							log_printf("LENS value	: 0x%04x\n", position);
							log_printf("\r\n");
						}
						else if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
						{

							// avoiding add_setting for the moment, as it doesn't support
							// commands that don't need use case IDs

							PiezoActuator *a;
							PiezoModule   *m;

							m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
							a = m->Lens;
							if (!a->IsConnected)
							{
								log_error("The lens actuator for module %s is disconnected.", pCam->Name);
								continue;
							}
							// If the hall sensor has been initialized, use the existing values.
							// If not, read the EEPROM values for the lens, and use defaults
							// for the mirrors to do calibration.
							//if (m->IsCalibrated) // calibration? check initialization instead
							if (!(a->Hall.IsInitialized))
							{
								//log_debug("Initializing the hall sensor...\n");
								// Using John's ReadHallCalibration, as it
								// makes only two EEPROM reads, rather than the
								// numerous slow reads in ReadEepromCalibration
								//ReadHallCalibration(pCam, i, &(m->Lens->CalibData), &(m->Mirror->CalibData));
								InitHallSensor(&(a->Hall));
							}

							data = ReadHallSensor(&(a->Hall));
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_lens_hall[j]), 2, LITTLE_ENDIAN, (uint8_t*)&data);
                            j += 2;

                            /*
							cam_m_status->cam_m_lens_hall[j++] = (uint8_t)(data >> 8);
							cam_m_status->cam_m_lens_hall[j++] = (uint8_t)data;
                            */

							log_printf("\nModule %s: ", pCam->Name);
							log_printf("LENS value : 0x%04x\r\n", data);
						}
						else
						{
							/* If the camera is not 35mm or 70mm/150mm, then
							   the user has made an error. */
							log_error("Invalid bitmask passed to CAM_MODULE_FOCUS_STATUS: %x", cam_bitmask);
							save_command_log(tid, ERROR_INVALID_MBITMASK);
						}

						uint32_t status_master = read_cam_focus_status(i);

                        save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_focus_status), 4, LITTLE_ENDIAN, (uint8_t*)&status_master);
                        /*
						cam_m_status->cam_m_focus_status[0] = (UInt8)(status_master >> 24);
						cam_m_status->cam_m_focus_status[1] = (UInt8)(status_master >> 16);
						cam_m_status->cam_m_focus_status[2] = (UInt8)(status_master >> 8);
						cam_m_status->cam_m_focus_status[3] = (UInt8)(status_master);
                        */
						log_printf("%s FOCUS STATUS [%08x]\r\n",pCam->Name, (unsigned int)status_master);
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_MODULE_LENS_HALL:
			case CAM_MODULE_MIRROR_HALL:
			{
				//log_debug("Reading hall sensor values \n");
				UInt8 IsLens = (CurrentCmd.cmd == CAM_MODULE_LENS_HALL) ? 1 : 0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;
				ucid		= CurrentCmd.ucid;
				UInt8 j = 0, k = 0;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						if(pCam->Type == CAM_TYPE_35MM)
						{
							/*
							Strictly speaking, you should not be reading the
							hall sensors for the 35mm cameras in this function.
							However, given that the 35mm cameras do in fact
							have hall sensors, I'll leave this functionality
							here.
							*/
							uint16_t position;
							AFData* af;
							af = &AFInfo[pCam->camera_id-1];
							position  = af_controller_get_current_pos(af,
										pCam->camera_id);
                            save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_lens_hall[j]), 2, LITTLE_ENDIAN, (uint8_t*)&position);
                            j += 2;
                            /*
							cam_m_status->cam_m_lens_hall[j++]= (uint8_t)(position >> 8);
							cam_m_status->cam_m_lens_hall[j++] = (uint8_t)position;
                            */
							log_debug("\nModule %s ", pCam->Name);
							log_debug("LENS value  : 0x%04x\n", position);
						}
						else if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
						{
							// avoiding add_setting for the moment, as it doesn't support
							// commands that don't need use case IDs


							//add_setting(IsLens ? LENS_HALL : MIRROR_HALL, 0, i, ucid);

							PiezoActuator *a;
							PiezoModule   *m;

							m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
							a = IsLens ? m->Lens : m->Mirror;
#ifdef BOARD_VERSION_P1_1
							if (!IsLens && (pCam->camera_id == CAM_ID_B4 || pCam->camera_id == CAM_ID_C5
									|| pCam->camera_id == CAM_ID_C6))
							{
								log_error("Mirrors for B4/C5/C6 are glued");
								continue;
							}
#endif
							if (!a->IsConnected)
							{
								log_error("The %s actuator for module %s is disconnected.", IsLens ? "lens" : "mirror", pCam->Name);
								continue;
							}

							// If the hall sensor has been initialized, use the existing values.
							// If not, read the EEPROM values for the lens, and use defaults
							// for the mirrors to do calibration.
							//if (m->IsCalibrated) // calibration? check initialization instead
							if (!(a->Hall.IsInitialized))
							{
								//log_debug("Initializing the hall sensor...\n");
								// Using John's ReadHallCalibration, as it
								// makes only two EEPROM reads, rather than the
								// numerous slow reads in ReadEepromCalibration
							   // ReadHallCalibration(pCam, i, &(m->Lens->CalibData), &(m->Mirror->CalibData));

								// ReadEepromCalibration(i, &(m->Lens->CalibData), &(m->Mirror->CalibData));
								if (!IsLens)
								{
									a->Hall.Sensitivity = m->Mirror->CalibData.Sensitivity;
									a->Hall.Polarity = m->Mirror->CalibData.Polarity;
								}
								//log_printf("m->Mirror->CalibData.Sensitivity: %d\n", (int)(m->Mirror->CalibData.Sensitivity));
								InitHallSensor(&(a->Hall));
							}

							data = ReadHallSensor(&(a->Hall));
							if (IsLens)
							{
                                save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_lens_hall[j]), 2, LITTLE_ENDIAN, (uint8_t*)&data);
                                j += 2;
                                /*
								cam_m_status->cam_m_lens_hall[j++] = (uint8_t)(data >> 8);
								cam_m_status->cam_m_lens_hall[j++] = (uint8_t)data;
                                */
								log_printf("\nModule %s: ", pCam->Name);
								log_printf("LENS value : 0x%04x\n", data);
								log_printf("Retracted hard stop : 0x%04x\n", pCam->retracted_hard_stop);
								log_printf("Extended  hard stop : 0x%04x\n", pCam->extended_hard_stop);
								log_printf("Range				: 0x%04x\n",
											abs(pCam->extended_hard_stop - pCam->retracted_hard_stop));
								log_printf("\n\n");
							}
							else
							{
                                save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_mirror_hall[k]), 2, LITTLE_ENDIAN, (uint8_t*)&data);
                                k += 2;
                                /*
								cam_m_status->cam_m_mirror_hall[k++] = (uint8_t)(data >> 8);
								cam_m_status->cam_m_mirror_hall[k++] = (uint8_t)data;
                                */
								log_printf("\nModule %s: ", pCam->Name);
								log_printf("MIRROR value : 0x%04x\n", data);
								log_printf("Wide   hard stop : 0x%04x\n", pCam->wide_hard_stop);
								log_printf("Narrow hard stop : 0x%04x\n", pCam->narrow_hard_stop);
								log_printf("Range			 : 0x%04x\n",
											abs(pCam->narrow_hard_stop - pCam->wide_hard_stop));
								log_printf("\n\n");
							   /*
								*(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 0) = (data>>8) & 0x0ff;
								*(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 1) =  data		& 0x0ff;
								log_printf("%s %s value : 0x%04x, S:%d, P:%d\n\r",
										   pCam->Name,
										   IsLens ? "LENS" : "MIRROR",
										   data,
										   a->Hall.Sensitivity, // these fields are valid, see drv_piezo_hall.h
										   a->Hall.Polarity);
							   */
							}

						}
						else
						{
							log_debug("Unsupported for camera %s", pCam->Name);
							save_command_log(tid, ERROR_INVALID_MBITMASK);
						}
					}
				}
				log_printf("\n");
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}

			// Debug command (lens):   0x0043 <m_bitmask> <dir> <duty cycle [MSBs]> <duty cycle [LSBs]> <count>
			// Debug command (mirror): 0x0047 <m_bitmask> <dir> <duty cycle [MSBs]> <duty cycle [LSBs]> <count>
			case CAM_MODULE_LENS_NUDGE:
			case CAM_MODULE_MIRROR_NUDGE:
			{
				// switch to the default multiplexer mode
				uint8_t tx_buffer[3], rx_buffer[3];
				tx_buffer[0] = 0x02;
				tx_buffer[1] = 0x06;
				tx_buffer[2] = 0x00;
				CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
				UInt8  Direction;
				UInt16 DutyCycle;
				UInt8  Count;

				Direction	= CurrentCmd.data[0];
				DutyCycle	= (UInt16)(CurrentCmd.data[1] << 8 | CurrentCmd.data[2]);
				Count		= CurrentCmd.data[3];
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;

				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
#ifdef BOARD_VERSION_P1_1
						if ( (CurrentCmd.cmd == CAM_MODULE_MIRROR_NUDGE) && (pCam->camera_id == CAM_ID_B4 || pCam->camera_id == CAM_ID_C5
								|| pCam->camera_id == CAM_ID_C6))
						{
							log_error("Mirrors for B4/C5/C6 are glued");
							continue;
						}
#endif

						if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
						{
							PiezoActuator *a;
							PiezoModule   *m;
							Error_t		   e;

							m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
							a = (CurrentCmd.cmd == CAM_MODULE_LENS_NUDGE) ? m->Lens : m->Mirror;
							e = PiezoNudge(a, Direction, DutyCycle, Count);
							if (e)
							{
								log_debug("NudgePiezo(Lens) returned %d", e);
							}
						}
						else
						{
							log_debug("Unsupported for cam %s", pCam->Name);
						}
					}
				}
				// switch back to the debug_test mode to generate PWM from
				// the CPLD
				tx_buffer[0] = 0x02;
				tx_buffer[1] = 0x06;
				tx_buffer[2] = 0x01;
				CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
				break;
			}

			case CAM_MODULE_FINE_NUDGE_LENS:
			case CAM_MODULE_FINE_NUDGE_MIRROR:
			{
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) |
							   (CurrentCmd.bitmask[1] << 8)  |
								CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	=	CurrentCmd.bitmask[0]  & 0x01;
				uint8_t  direction	 =	CurrentCmd.data[2]; // 0 or 1 only
				uint16_t multiplier  = (CurrentCmd.data[0] << 8)  |
										CurrentCmd.data[1];

				uint16_t CPLD_select = 0;
				PiezoActuator* a	 = NULL;
				PiezoModule*   m	 = NULL;
				uint8_t is_lens		 = (CurrentCmd.cmd == CAM_MODULE_FINE_NUDGE_LENS);

				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						if(pCam->Type == CAM_TYPE_35MM)
						{
							if(is_lens)
							{
								log_printf("Nudging 35mm lens\n\n");
								// read the current value
								uint16_t position;
								AFData* af;
								af = &AFInfo[pCam->camera_id-1];
								position = af_controller_get_current_pos(af,
										   pCam->camera_id);

								log_printf("Current position: 0x%04x\n\n", position);

								// write a new value incremented or decremented
								// from the previous value according to the
								// multiplier/direction

								uint16_t destination = direction ? position + multiplier : position - multiplier;
								log_printf("Destination: 0x04%x\n\n", destination);

								Bool ret = af_controller_move(pCam, destination, EFalse);
								if(ret == EFalse)
								{
									reset_cam_focus_status(i, MOVING);
									set_cam_focus_status(i, MOVING_ERROR);
								}
								else
								{
									reset_cam_focus_status(i, MOVING);
									set_cam_focus_status(i, IDLE);
								}
								//log_printf("\nModule %s ", pCam->Name);
								//log_printf("LENS value  : 0x%04x\n", position);
							}
							else
								log_debug("Nudging mirrors is not supported for 35mm modules.\n");

						}
						else if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
						{
							// gather all the CPLD bits for the relevant modules
							CPLD_select |= pCam->CPLD_select;
							m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
							a = (is_lens) ? m->Lens : m->Mirror;
#ifdef BOARD_VERSION_P1_1
							if ( (!is_lens) && (pCam->camera_id == CAM_ID_B4 || pCam->camera_id == CAM_ID_C5
									|| pCam->camera_id == CAM_ID_C6))
							{
								log_error("Mirrors for B4/C5/C6 are glued");
								continue;
							}
#endif
							GPIO_SetBits(a->Channel->Port, a->Channel->Pin);
						}
						else
						{
							log_debug("Unsupported for cam %s", pCam->Name);
						}
					}
				}

				if(cam_bitmask & 0x1FFC1) // 70mm or 150mm
				{
					// move all lenses/mirrors at once
					fine_nudge(a, CPLD_select, direction, multiplier, is_lens);

					// need a separate for loop to disable the NJW pins
					for (i = CAM_ID_B1-1; i < CAM_NUM_CAMERA_MAX; i++)
					{
						if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
						{
							pCam = CamDeviceTbl[i];
							m = CameraIdToModule(pCam->camera_id); // +1 converts from i to bitmask index
							a = (is_lens) ? m->Lens : m->Mirror;
							GPIO_ResetBits(a->Channel->Port, a->Channel->Pin);
							log_printf("%s actuator position: 0x%04x\n", pCam->Name, ReadHallSensor(&(a->Hall)));
						}
					}
				}
				break;
			}

			case CAM_MODULE_MIRROR_POSITION:
			{
				log_debug("In CAM_MODULE_MIRROR_POSITION\n");
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				data_idx = 0;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				save_command_log(tid, CMD_PENDING);
				if (CurrentCmd.data_len == 0) // This should be a read
				{
					log_debug("Reading the mirror position\n");
					cam_read_mirror_position(cam_bitmask);
					save_command_log(tid, CMD_SUCCESS);
				}
				else if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) != EFalse)
				{

					// Do the 70mm and 150mm settings
					run_parallel_mirror_move(CurrentCmd.data, cam_bitmask);
					save_command_log(tid, CMD_SUCCESS);
				}
				else
				{
					log_error("Invalid input");
					save_command_log(tid, CMD_INVALID_ARG);
				}
				break;
			}

			case CAM_MODULE_MIRROR_POSITION2:
			{
				UInt16 DutyCycle = 90; // DUTY_LO
				UInt16 Tolerance =	5;
				data = (UInt16)(CurrentCmd.data[0] << 8 | CurrentCmd.data[1]);
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				ucid = (UInt16)(CurrentCmd.data[0] << 8 | CurrentCmd.data[1]);
				save_command_log(tid, CMD_PENDING);
				if (CurrentCmd.cmd == CAM_MODULE_MIRROR_POSITION2)
				{
					// Use extra data bytes for duty cycle.
					Tolerance = (UInt16)(CurrentCmd.data[2]);
					DutyCycle = (UInt16)(CurrentCmd.data[3]);
				}
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
#ifdef BOARD_VERSION_P1_1
						if (pCam->camera_id == CAM_ID_B4 || pCam->camera_id == CAM_ID_C5
								|| pCam->camera_id == CAM_ID_C6)
						{
							log_error("Mirrors for B4/C5/C6 are glued");
							continue;
						}
#endif

						if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
						{
							Error_t e;
							PiezoModule *m;

							m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
							e = MovePiezoToPosition(m->Mirror, data, Tolerance, DutyCycle);
							if (e)
							{
								log_debug("MovePiezoToPosition(Mirror) returned %d", e);
							}
						}
						else
						{
							log_debug("Unsupport to cam %s", pCam->Name);
						}
					}
				}
				save_command_log(tid, CMD_SUCCESS);
				break;
			}

			// Debug command: 0x0080 <m_bitmask> <slave id (preshifted)> <size byte> <addr[MSBs]> <addr[LSBs]> <match[MSBs]> <match[LSBs]>
			// A match value of 0xffff indicates don't care - consider any value legal.
			case CAM_MODULE_DEBUG_I2C_READ:
			{
				UInt8  slave;
				UInt8  size;
				UInt16 addr;
				UInt16 data;
				UInt16 expected;

				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	=	CurrentCmd.bitmask[0] & 0x01;
				slave		=	CurrentCmd.data[0];
				size		=	CurrentCmd.data[1] & 0x3;
				addr		=  (CurrentCmd.data[2] << 8) | CurrentCmd.data[3];
				expected	=  (CurrentCmd.data[4] << 8) | CurrentCmd.data[5];

				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						data = fpga_read_i2c_value(pCam->camera_id, slave, addr, size);
						log_printf("%s I2C READ Slave:0x%02x A%dD%d A:0x%04x\n", pCam->Name, slave, size & 0x1 ? 16:8, size & 0x2 ? 16: 8, addr);
						log_printf("				Read: 0x%04x  Expected: 0x%04x\n", data, expected);
						log_printf("	  %s\n", ((data == expected) || (expected == 0xffff)) ? "match" : "FAIL");
					}
				}
				break;
			}

			// Debug command: 0x0082 <m_bitmask> <slave id (preshifted)> <size byte> <addr[MSBs]> <addr[LSBs]> <data[MSBs]> <data[LSBs]>
			case CAM_MODULE_DEBUG_I2C_WRITE:
			{
				UInt8  slave;
				UInt8  size;
				UInt16 addr;
				UInt16 data;

				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	=	CurrentCmd.bitmask[0] & 0x01;
				slave		=	CurrentCmd.data[0];
				size		=	CurrentCmd.data[1] & 0x3;
				addr		=  (CurrentCmd.data[2] << 8) | CurrentCmd.data[3];
				data		=  (CurrentCmd.data[4] << 8) | CurrentCmd.data[5];

				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];

						fpga_write_i2c_value(pCam->camera_id, slave, addr, data, size, 1);
						log_printf("%s I2C WRITE Slave:0x%02x A%dD%d A:0x%04x: 0x%04x\n", pCam->Name, slave, size & 0x1 ? 16:8, size & 0x2 ? 16: 8, addr, data);
					}
				}
				break;
			}

			case LIGHT_ACTIVE_UCID:
				ucid = CurrentCmd.ucid;

                save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_light_active_ucid), 2, LITTLE_ENDIAN, (uint8_t*)&ucid);
                /*
				cam_m_status->cam_m_light_active_ucid[0] = (UInt8)(ucid >> 8);
				cam_m_status->cam_m_light_active_ucid[1] = (UInt8)ucid;
                */
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				log_debug("Received active ucid: %x",ucid);
				activate_ucid(ucid & 0xf);
				save_command_log(tid, CMD_SUCCESS);
				break;

			case 0x00B0:
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				save_command_log(tid, CMD_PENDING);
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						log_printf("EEPROM of %s \n\r", pCam->Name);
						vAR0835_EEPROM(pCam);

					}
				}
				save_command_log(tid, CMD_SUCCESS);
				break;

			case CAM_MODULE_LENS_CALIBRATION:
			{
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) |
							   (CurrentCmd.bitmask[1] << 8) |
								CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;
				save_command_log(tid, CMD_PENDING);
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1 << i ) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						if((pCam->Type == CAM_TYPE_70MM)||
                           (pCam->Type == CAM_TYPE_150MM))
						{
							PiezoModule*   m;
							PiezoActuator* a;
							m = CameraIdToModule(i+1);
							a = m->Lens;
                            PiezoCalibrationData LensData;
                            hall_calib_t lens_calib;
                            start_lens_calibration(i, pCam, a, &LensData, lens_calib);

							//calibrate_lens(pCam, a);
							//hall_calibration_save(pCam, LENS_CALIBRATION);

                            a->Hall.Sensitivity       = LensData.Sensitivity;
                            a->Hall.Polarity          = LensData.Polarity;
                            log_debug("lens->CalibData.NearPosition = 0x%04x\n", a->CalibData.NearPosition);
                            log_debug("lens->CalibData.FarPosition  = 0x%04x\n", a->CalibData.FarPosition);

                            log_debug("Calibrated lens for module %s", pCam->Name);
						}
						else
						{
							log_warning("This calibration procedure is valid for\
 70mm and 150mm modules only. \n");
						}
					}
				}
				save_command_log(tid, CMD_SUCCESS);
				break;
			}

			case CAM_MODULE_LENS_CALIBRATION2:
			{
				UInt16 Iterations = 1;
				UInt16 DutyCycle = 280;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				if (CurrentCmd.cmd == CAM_MODULE_LENS_CALIBRATION2)
				{
					// Use extra data bytes for duty cycle.
					Iterations = (UInt16)(CurrentCmd.data[0]);
					DutyCycle  = (UInt16)((CurrentCmd.data[1] << 8) | CurrentCmd.data[2]);
				}
				save_command_log(tid, CMD_PENDING);
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						if((pCam->Type == CAM_TYPE_70MM)||(pCam->Type == CAM_TYPE_150MM))
						{
							//UInt16 min = 0, max = 0;
							Error_t e;
							PiezoModule *m;

							m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
							e = CalibrateModule(m, Iterations, DutyCycle);
							log_debug("CalibrateModule(m) returned %d", e);
						}
						else
						{
							log_debug("Unsupported for camera %s", pCam->Name);
						}
					}
				}
				save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_MODULE_MIRROR_CALIBRATION:
			{
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) |
							   (CurrentCmd.bitmask[1] << 8) |
								CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;
				save_command_log(tid, CMD_PENDING);
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1 << i ) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						/*
						If the module is 70mm or 150mm, calibrate the
						lens. This logic may be repeated for the mirrors
						in the future.
						 */
						if((pCam->Type == CAM_TYPE_70MM)||
						   (pCam->Type == CAM_TYPE_150MM))
						{
							PiezoModule*   m;
							PiezoActuator* a;
							m = CameraIdToModule(i+1);
							a = m->Mirror;

							//set the polarity to normal
				            a->Hall.Polarity = HALL_POLARITY_NORMAL;
				            a->Hall.Sensitivity = (UInt8) 3;
				            InitHallSensor(&(a->Hall));

							calibrate_mirror(pCam, a);
							a->Hall.Sensitivity = (UInt8) 3;
							InitHallSensor(&(a->Hall));

							hall_calibration_save(pCam, MIRROR_CALIBRATION);
                            log_debug("Calibrated mirror for module %s", pCam->Name);
						}
						else
						{
							log_warning("This calibration procedure is valid for\
 70mm and 150mm cameras only. \n");
						}
					}
				}
				save_command_log(tid, CMD_SUCCESS);
				break;
			}

			case CAM_MODULE_DEBUG_PIEZO_MONITOR:
			case CAM_MODULE_DEBUG_PIEZO_MONITOR2:
			{
				unsigned int monitor_bitmask = 0x1ffc0; // Default to all piezo modules
				UInt8  on;
				if (CurrentCmd.cmd == CAM_MODULE_DEBUG_PIEZO_MONITOR2)
				{
					monitor_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				}
				log_printf("PiezoMonitor: 0x%02x 0x%02x%04x\n",
						CurrentCmd.data[0],
						monitor_bitmask >> 16,
						monitor_bitmask & 0xffff);
				on = CurrentCmd.data[0];
				SetPiezoMonitor(on, monitor_bitmask);
				log_printf("PIEZO MONITOR %s\n", on ? "ON" : "OFF");

				break;
			}

			// 0x0063  <m_bitmask>	<focus distance>  <focus position>
			// Overwrites position data at that focus distance (if the point exists)
			// Adds the data point otherwise.
			case CAM_MODULE_FOCUS_CALIBRATION_DATA:
			{
				Error_t e;
				PiezoModule *m;
				UInt32 FocusDistance;
				UInt16 FocusPosition;

				cam_bitmask   = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	  = CurrentCmd.bitmask[0] & 0x01;

				FocusDistance = (CurrentCmd.data[0] << 24) | (CurrentCmd.data[1] << 16) |
								(CurrentCmd.data[2] <<	8) |  CurrentCmd.data[3];
				FocusPosition = (CurrentCmd.data[4] <<	8) |  CurrentCmd.data[5];
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						m = CameraIdToModule(i+1); // +1 converts from i to bitmask index
						e = SetFocusCalibration(m, FocusDistance, FocusPosition);
						log_debug("SetFocusCalibration() returned %d", e);
					}
				}
				break;
			}

			case CAM_MODULE_STATUS:
			{
				UInt8 index=0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for(i = 0; i< CAM_NUM_CAMERA_MAX; i++)
				{
					if(((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						status_master = StatusMask(i);
                        save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_status[index]), 4, LITTLE_ENDIAN, (uint8_t*)&status_master);
                        index += 4;
                        /*
						cam_m_status->cam_m_status[index++] =(UInt8)(status_master>>24);
						cam_m_status->cam_m_status[index++] =(UInt8)(status_master>>16);
						cam_m_status->cam_m_status[index++] =(UInt8)(status_master>>8);
						cam_m_status->cam_m_status[index++] =(UInt8)(status_master);
                        */
						log_printf("Status cam [%s]= [%02x%02x%02x%02x]\n\r",CamDeviceTbl[i]->Name,(UInt8)(status_master>>24),(UInt8)(status_master>>16),(UInt8)(status_master>>8),(UInt8)(status_master));
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}

			case SPI_TRANSFER_FPGA:
			{
				/* Header of this message contain 3 bytes:
				 * - Asic ID: 0
				 * - Transaction: 0:read/1:write
				 * - Data len
				 */
				fpga_cmd_error_t fpga_state = FPGA_CMD_SUCCESS;
				UInt8 len = fpga_debug_cmd[2];

				/* Word alighment */
				if(len % 2 != 0)
					len++;

				UInt16 tx[len/2];
				UInt16 rx[len/2];

				/* Write data to fpga */

				/* Reformatting as our fpga_send_command support UInt16 */
				for(int i = 0; i < len/2; i ++)
				{
					tx[i] = fpga_debug_cmd[i*2+4] << 8 | fpga_debug_cmd[i*2+3];
				}
				/* Now, send to FPGA */
				log_debug("Transferring %d bytes of data to FPGA\n\r", fpga_debug_cmd[2]);
				fpga_state = fpga_send_command( tx, rx, len / 2, IRQ_FROM_FPGA_TIMEOUT );
				if(fpga_state == FPGA_CMD_TOUT || fpga_state == FPGA_CMD_WAITING)
				{
					log_error("Timed out while sending data to FPGA...");
				}
				else
				{
					log_debug("Finished sending data to FPGA!\n\r");
				}

				if(fpga_debug_cmd[1] == 0)
				{
					/* Create dummy bytes to read from FPGA */
					for(int i = 0; i < len/2; i ++)
					{
						tx[i] = 0xFFFF;
					}
					/* Send command to FPGA */
					log_debug("Reading data from the FPGA \n\r ");
					fpga_state = fpga_send_command(tx, rx, len / 2, IRQ_FROM_FPGA_TIMEOUT);
					if(fpga_state == FPGA_CMD_TOUT || fpga_state == FPGA_CMD_WAITING)
					{
						log_error("Timed out while reading data from FPGA");
					}
					else
					{
						/* Done. Write to RAM to support host to read data */
						for(int i = 0; i < len; i += 2)
						{
							*(CCB_ASIC_CAM_RD + CurrentCmd.cmd + i) = (UInt8)(rx[i/2] & 0xff);
							*(CCB_ASIC_CAM_RD + CurrentCmd.cmd + i + 1) = (UInt8)((rx[i/2] >> 8) & 0xff);
						}
						log_debug("Finished reading data from FPGA\n\r");
						log_debug("Returned Byte ");
#if 1
						for(i = 0; i < len; i++)
						{
							log_printf("  [%x]", *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + i));
						}
#endif
					}
				}
				break;
			}

			case SPI_TRANSFER_CPLD_SINGLE:
			{
				UInt8 dataTx[4], dataRx[4];
				UInt16 num_bytes =0;

				dataTx[0] = cpld_debug_cmd[0];
				dataTx[1] = cpld_debug_cmd[1];
				dataTx[2] = 0xFF;

				if(cpld_debug_cmd[0] == 2)
				{
					num_bytes = 3;
					dataTx[2] = cpld_debug_cmd[2];
				}
				else if(cpld_debug_cmd[0] == 0x0B)
				{
					num_bytes = 4;
					dataTx[3] = 0xFF;
				}
				else
				{
					log_error("\nInvalid transaction type. 0x02: Write 0x0B: Read");
					break;
				}

				CPLD_SPI_Transfer((UInt8*)dataTx, (UInt8*)dataRx, num_bytes);
				log_debug("SPI WRITE: [0x%02x 0x%02x 0x%02x]\n", dataTx[0], dataTx[1], dataTx[2]);
				log_debug("SPI READ : [0x%02x 0x%02x 0x%02x 0x%02x]\n", dataRx[0], dataRx[1], dataRx[2], dataRx[3]);

				if(cpld_debug_cmd[0] == 0x0B)
				{
					log_debug("Address: 0x%02x\t", dataTx[1]);
					log_debug("Data: 0x%02x\n", dataRx[3]);
				}

				break;

			case SPI_TRANSFER_CPLD:
			{
				if (CPLD_data_size == 0)
				{
					log_error("Invalid size for CPLD transfer. \n\n");
					break;
				}

				uint8_t tx_buffer[CPLD_data_size + 3], rx_buffer[CPLD_data_size + 3];
				uint8_t address = 0;
				uint8_t word_count;
				uint8_t remainder;

				// Allocate the 8 byte buffer first, use it multiple times
				// Add +3 for the transaction type, address, and dummy byte
				/*
				for (int i = 2; i < MAX_CPLD_TXRX_SIZE + 3; i++)
					tx_buffer[i] = 0xFF;
				*/

				// Reads need one dummy byte of 0xFF before the actual
				// reads start, so increase the transfer size
				//CPLD_data_size += 1; // temporarily increase size
				tx_buffer[0] = CPLD_READ;
				tx_buffer[1] = cpld_debug_cmd[1]; // address
				address = cpld_debug_cmd[1];

				// Reads are capped at 8 bytes ("word size") at a time
				word_count = (CPLD_data_size / MAX_CPLD_TXRX_SIZE);
				log_printf("word_count = %d\n", word_count);
				remainder = CPLD_data_size % MAX_CPLD_TXRX_SIZE;
				log_printf("remainder = %d\n", remainder);

				log_printf("%d byte transaction\n", CPLD_data_size);

				// Start with 8 byte transactions, remainder later
				for (int i = 0; i < word_count; i++)
				{
					log_printf("Sending word index %d\n", i);
					// Change the address to read from
					tx_buffer[1] = cpld_debug_cmd[1] + i * MAX_CPLD_TXRX_SIZE;
					// Fill the buffer appropriately, initiate transaction
					if(cpld_debug_cmd[0] == CPLD_WRITE)
					{
						log_printf("CPLD_WRITE request\n\n");
						for (int j = 2; j < MAX_CPLD_TXRX_SIZE + 2; j++)
							tx_buffer[j] = cpld_debug_cmd[2 + (i * MAX_CPLD_TXRX_SIZE) + j];
						CPLD_SPI_Transfer(tx_buffer, rx_buffer, MAX_CPLD_TXRX_SIZE + 2);
					}
					else if(cpld_debug_cmd[0] == CPLD_READ)
					{
						log_printf("CPLD_READ request\n\n");
						for (int j = 2; j < MAX_CPLD_TXRX_SIZE + 3; j++)
							tx_buffer[j] = 0xFF;
						CPLD_SPI_Transfer(tx_buffer, rx_buffer, MAX_CPLD_TXRX_SIZE + 3);
						// Print the bytes read
						for (int j = 0; j < MAX_CPLD_TXRX_SIZE; j++)
						{
							log_printf("Address: 0x%02x \t", address++);
							log_printf("Data: 0x%02x\n", rx_buffer[j + 3]);
						}
					}
					else
					{
						log_printf("cpld_debug_cmd[0] == 0x%02x\n\n", cpld_debug_cmd[0]);
					}
					log_printf("tx_buffer: [");
					for (int i = 0; i < MAX_CPLD_TXRX_SIZE + 3; i++)
						log_printf("0x%02x	", tx_buffer[i]);
					log_printf("\b\b]\n\n");

					log_printf("rx_buffer: [");
					for (int i = 0; i < MAX_CPLD_TXRX_SIZE + 3; i++)
						log_printf("0x%02x	", rx_buffer[i]);
					log_printf("\b\b]\n\n");
				}
				if (remainder > 0) // not a multiple of MAX_CPLD_TXRX_SIZE
				{
					log_printf("Sending remainder\n\n");
					// Clear the transmit buffer
					for (int i = 2; i < MAX_CPLD_TXRX_SIZE + 3; i++)
						tx_buffer[i] = 0x00;

					// Send the remainder
					if(cpld_debug_cmd[0] == CPLD_WRITE)
					{
						for (int i = 2; i < remainder + 2; i++)
							tx_buffer[i] = cpld_debug_cmd[2 + (word_count * MAX_CPLD_TXRX_SIZE) + i];
						CPLD_SPI_Transfer(tx_buffer, rx_buffer, remainder + 2);
					}
					else if(cpld_debug_cmd[0] == CPLD_READ)
					{
						for (int i = 2; i < remainder + 3; i++)
							tx_buffer[i] = 0xFF;
						CPLD_SPI_Transfer(tx_buffer, rx_buffer, remainder + 3);
						// Print the remaining bytes
						for (int i = 0; i < remainder ; i++)
						{
							log_printf("Address: 0x%02x \t", address++);
							log_printf("Data: 0x%02x\n", rx_buffer[i + 3]);
						}
					}

					log_printf("tx_buffer: [");
					for (int i = 0; i < remainder + 3; i++)
						log_printf("0x%02x	", tx_buffer[i]);
					log_printf("\b\b]\n\n");

					log_printf("rx_buffer: [");
					for (int i = 0; i < remainder + 3; i++)
						log_printf("0x%02x	", rx_buffer[i]);
					log_printf("\b\b]\n\n");
				}
				log_printf("\n");

				// delay for 5 microseconds between transactions
				DelayUSec(5);

				}

				log_printf("\n");
				break;
			}

			case CONFIGURE_CPLD:
			{
			/*
			registers to write:

			0x06		: debug_test, always 1
			0x0a - 0x15 : duty cycles, 2 bytes each, 12 bytes total
			0x20 - 0x37 : repetition counts, 4 bytes each, 24 bytes total
			0x50 - 0x57 : group repetition counts, 4 bytes each, 8 bytes total
			0x70 - 0x73 : overall repetition count, 4 bytes total
			0x80 - 0x81 : frequency, 2 bytes total

			50 bytes total, not including 0x06
			*/
				// Testing out the multi-byte CPLD write.
				// Currently set at 4 bytes.
				// Contiguous reads of unspecified size are supported,
				// but must write an equivalent number of bytes to provide
				// clock pulses.
				log_printf("Configuring CPLD...\n\n");
				configure_CPLD(USER_DEFINED_CONTROL, RETRACT, NULL, 0);

				//log_printf("i = %d\n\n", i);
				break;
			}


			case CAM_MODULE_NUDGE_CPLD:
			{
			/*
				This command should only be called after the CPLD has been
				configured. It gathers all the modules whose bitmask has been
				set, writes the module selection registers on the CPLD
				accordingly, then sends the start signal for PWM generation.

				If the CPLD has not been configured, this sends the start
				signal for the modules given, and the CPLD will generate
				pulses according to whatever its default values are.
			 */
				log_printf("Moving the lens with CAM_MODULE_NUDGE_CPLD\n\n");
				uint8_t tx_buffer[4], rx_buffer[4];
				uint8_t module_select_regs[2];
				module_select_regs[0] = 0;
				module_select_regs[1] = 0;

				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) |
							   (CurrentCmd.bitmask[1] << 8)  |
								CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;

				// Iterate through the module bitmask, turn on the NJW enables
				// OR in the corresponding bits to the module selection
				// register buffers.
				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
#ifdef BOARD_VERSION_P1_1
						pCam = CamDeviceTbl[i];
#endif /* BOARD_VERSION_P1_1 */
						if ((pCam->Type == CAM_TYPE_70MM) ||
							(pCam->Type == CAM_TYPE_150MM))
						{
							PiezoModule*   m;
							PiezoActuator* a;
							m = CameraIdToModule(i+1);
							// +1 converts from i to bitmask index
							a = m->Lens;

							GPIO_SetBits(a->Channel->Port, a->Channel->Pin);
							log_printf("Port: 0x%08x  Pin: %d",
										(int)a->Channel->Port, a->Channel->Pin);

							if (pCam->Type == CAM_TYPE_70MM)
								module_select_regs[0] |=  (pCam->CPLD_select)
														   && 0xFF;
							else if (pCam->Type == CAM_TYPE_150MM)
								module_select_regs[1] |= ((pCam->CPLD_select)
														   >> 8) && 0xFF;
							else // control should not reach here
								log_debug("Invalid camera given in CAM_MODULE_NUDGE_CPLD.\n");
						}
						else
						{
							log_debug("Unsupported for cam %s", pCam->Name);
						}
					}
				}
				// configure for write
				tx_buffer[0] = 0x02;

				log_printf("Writing the module selection registers...\n");
				// write the B module selection register
				tx_buffer[1] = 0x02;
				tx_buffer[2] = module_select_regs[0];
				CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

				// write the C module selection register
				tx_buffer[1] = 0x03;
				tx_buffer[2] = module_select_regs[1];
				CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

				log_printf("Moving module[s]...\n");
				// move multiple modules at the same time
				CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
				break;
			}

			case CAM_MODULE_PIEZO_LENS_CONTROL:
			{
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) |
							   (CurrentCmd.bitmask[1] << 8)  |
								CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;


				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
					break;

				run_parallel_lens_move(CurrentCmd.data, cam_bitmask, 0);
				break;
			}

			case CAM_MODULE_PIEZO_MIRROR_CONTROL:
			{
				uint16_t destination = 0;
				uint16_t CPLD_select;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) |
							   (CurrentCmd.bitmask[1] << 8)  |
								CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;

				configure_CPLD(FINE_CONTROL, WIDE, NULL, 0);

				for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
				{
					if (((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						// Check for the default values to see if
						// the lens has been calibrated
#ifdef BOARD_VERSION_P1_1
						if ((pCam->camera_id == CAM_ID_B4 || pCam->camera_id == CAM_ID_C5
								|| pCam->camera_id == CAM_ID_C6))
						{
							log_error("Mirrors for B4/C5/C6 are glued");
							continue;
						}
#endif /* BOARD_VERSION_P1_1 */
						if((pCam->Type == CAM_TYPE_70MM)||
						   (pCam->Type == CAM_TYPE_150MM))
						{
							PiezoModule*   m;
							PiezoActuator* a;
							m = CameraIdToModule(i+1);
							// +1 converts from i to bitmask index
							a = m->Mirror;

							log_printf("CAM_MODULE_PIEZO_CONTROL:\n\n");
							log_printf("Module: %s\n", pCam->Name);
							CPLD_select = pCam->CPLD_select;
							//log_printf("CPLD_select: %04x\n", CPLD_select);
							destination = (CurrentCmd.data[1] << 8) |
										   CurrentCmd.data[0];
							start_closed_control_mirror(pCam, a, CPLD_select, destination, FALSE);
						}
						else
							log_debug("Unsupported for module %s", pCam->Name);
					}
				}
				break;
			}

			case CAM_MODULE_LENS_FREQ_CALIBRATION:
			{
			   cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
			   isGlobal    = CurrentCmd.bitmask[0] & 0x01;
			   for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
			   {
				   //log_printf("Module : %s\n\n", CamDeviceTbl[i]->Name);
				   //log_printf("cam_bitmask: 0x%x\n\n", cam_bitmask);
				   if (((cam_bitmask >> 1) & 1 << i ) != 0 || isGlobal)
				   {
					   //log_printf("Entered if statement for module %s\n\n", CamDeviceTbl[i]->Name);
					   pCam = CamDeviceTbl[i];
					   //log_printf("Calibrating module %s...\n", pCam->Name);
					   PiezoModule*   m;
					   PiezoActuator* a;
					   m = CameraIdToModule(i+1);
					   a = m->Lens;
					   /*
					   If the module is 70mm or 150mm, calibrate the
					   lens. This logic may be repeated for the mirrors
					   in the future.
						*/
					   if((pCam->Type == CAM_TYPE_70MM)||
						  (pCam->Type == CAM_TYPE_150MM))
					   {
						   calibrate_freq(pCam, a);
					   }
					   else
					   {
						   log_warning("This calibration procedure is valid for\
70mm and 150mm modules only. \n");
					   }
				   }
			   }
			   break;
			}

			case CAM_MODULE_MIRROR_FREQ_CALIBRATION:
			{
				log_debug("Mirror freq calibration is now part of 'calibration'");
				log_debug("Use CAM_MODULE_LENS_FREQ_CALIBRATION (0x%04x) instead.", CAM_MODULE_LENS_FREQ_CALIBRATION);
				break;
			}
			case STM_VERSION:
			{
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}

#ifdef BUILD_ID
				/* Display and write correct STM build id to memory */

				log_console("  Build ID: %08x \n\r", stm_ver);

#endif
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case SOFTWARE_VERSION:
			{
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				UInt8 fpga_version_major = 0, fpga_version_minor = 0;
				read_fpga_version(&fpga_version_major, &fpga_version_minor);
				log_console("SOFTWARE VERSIONS \n\r");
				log_console("------------------------\n\r");
#ifdef BUILD_ID
				log_console("  Build ID: %08x \n\r", stm_ver);
#endif
				log_console("CPLD version: %02d.%02d \n\r", cpld_ver_major, cpld_ver_minor);
				log_console("FPGA version: %02d.%02d \n\r", fpga_version_major, fpga_version_minor);
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_MODULE_TEMP:
			{
				UInt8 index = 0;
				UInt16 tempdata = 0;
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal = CurrentCmd.bitmask[0] & 0x01;
				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}
				for(i = 0; i< CAM_NUM_CAMERA_MAX; i++)
				{
					if(((cam_bitmask >> 1) & 1<<i) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i];
						tempdata = read_sensor_temp(pCam);
						log_debug("[%s] Temp %d \n", pCam->Name, tempdata);
						save_cam_m_status_field((uint8_t*)&(cam_m_status->cam_m_temp[index]), 2, LITTLE_ENDIAN, (uint8_t*)&tempdata);
						index += 2;
					}
				}
				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case CAM_COMMAND_BOARD_TEMPERATURE:
			{
				const uint8_t MAX_TEMP = 4; /* Have 4 sensors on board */
				UInt16 tmp;
				UInt8 idx=0;
				uint16_t SlaveAddr[] = {0x48,0x49,0x4A,0x4B};

				save_command_log(tid, CMD_PENDING);
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
				{
					save_command_log(tid, ERROR_INVALID_ARG);
					break;
				}

				for(int i=0; i< MAX_TEMP;i++)
				{
					tmp=Get_TempBoad(SlaveAddr[i]);
					log_info("Temperature sensor %d: %d \r\n", (i+1), tmp);
					save_cam_m_status_field((uint8_t *)&(cam_m_status->cam_m_board_temp[idx]), 2,LITTLE_ENDIAN, (uint8_t*)&tmp);
					idx += 2;
				}

				if(get_command_status(tid) == CMD_PENDING)
					save_command_log(tid, CMD_SUCCESS);
				break;
			}
			case 0x2003:
				break;
#ifdef BOARD_VERSION_P1_1
			case CAM_MODULE_SET_CAPTURE_PARAM:
				if(check_require_data_size(cam_bitmask,CurrentCmd.cmd, CurrentCmd.data_len) == EFalse)
					break;
				cam_trigger = CurrentCmd.data[0];
				cam_frame = CurrentCmd.data[1];
				cam_lines = CurrentCmd.data[2] << 0x08 | CurrentCmd.data[3] ;

				printf("cam_trigger %d cam_frame %d cam_lines = %d \n ",cam_trigger, cam_frame, cam_lines);
				break;
#endif
			case CAM_MODULE_SAVE_POSITION:
				log_debug("In CAM_MODULE_SAVE_POSITION\n");
				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;

				for (i = 1; i < CAM_ID_MAX; i++)
				{
					if ((cam_bitmask & (0x1 << i) ) != 0 || isGlobal)
					{
						pCam = CamDeviceTbl[i-1];
						hall_calibration_update_position( pCam, LENS_CALIBRATION);
						hall_calibration_update_position( pCam, MIRROR_CALIBRATION);
					}

				}
				break;

			case CAM_MODULE_MIRROR_FOCAL_STATUS:
#ifdef BOARD_VERSION_P1
				log_debug("In CAM_MODULE_MIRROR_STATUS \n");
				unsigned int status = 0;

				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) | (CurrentCmd.bitmask[1] << 8) | CurrentCmd.bitmask[0]) & 0x00FFFFFF;
				isGlobal	= CurrentCmd.bitmask[0] & 0x01;
				log_debug("cam_bitmask 0x%x\n", cam_bitmask);
				cam_read_mirror_position(cam_bitmask);
				status = read_cam_mirror_status();

                save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_mirror_status), 4, LITTLE_ENDIAN, (uint8_t*)&status);
                /*
				cam_m_status->cam_m_mirror_status[0] = (UInt8)(status >> 24);
				cam_m_status->cam_m_mirror_status[1] = (UInt8)(status >> 16);
				cam_m_status->cam_m_mirror_status[2] = (UInt8)(status >> 8);
				cam_m_status->cam_m_mirror_status[3] = (UInt8)(status);
                */
				log_debug("mirror status %d", status);
#endif /* BOARD_VERSION_P1 */
				break;

			default:
				log_error("Command 0x%x is not supported !!!", CurrentCmd.cmd);
				break;
		}
		if (CurrentCmd.flags & CCB_INTR_EN)
		{
			log_debug("Generating Interrupt...");

			Trigger_STM_IRQ(get_interrupt_mask(CurrentCmd.tid, get_command_status(tid)));
			log_debug("Generated Interrupt for transaction ID 0x%x\n", CurrentCmd.tid);
		}
	}
}

/* Private task functions ----------------------------------------------------*/
STATIC Bool  check_require_data_size(UInt32 bitmask, UInt32 cmd, UInt16 received_size)
{
	UInt16 req_size = get_require_data_size(bitmask, cmd);
	if(!received_size)
	{
		/*Determine a command is read or write*/
		return ETrue;
	}
	else if(received_size < req_size)
	{
		log_error("Not enough data: required %d bytes - received %d bytes", req_size, received_size );
		return EFalse;
	}
	else if(received_size > req_size)
	{
		log_warning("Redundant data: required %d bytes - received %d byte", req_size, received_size);
	}

	return ETrue;
}

STATIC UInt8 get_require_data_size(UInt32 bitmask, UInt32 cmd)
{
	UInt8 cam_count = 0, i = 0, cmd_size = 0;

	if((bitmask & 0x1) != 0)
	{
		cam_count = 1;
	}
	else
	{
		for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
		{
			if (((bitmask >> 1) & 1<<i) != 0)
				cam_count ++;
		}
	}
	switch(cmd)
	{
		case CAM_MODULE_LENS_HALL:
		case CAM_MODULE_MIRROR_HALL:
		case CAM_MODULE_LENS_CALIBRATION:
		case CAM_MODULE_MIRROR_CALIBRATION:
		case CAM_MODULE_LENS_FREQ_CALIBRATION:
		case CAM_MODULE_MIRROR_FREQ_CALIBRATION:
		case CAM_MODULE_FOCUS_STATUS:
		case CAM_MODULE_STATUS:
		{
			cmd_size = 0;
			break;
		}
		case CAM_MODULE_OPEN:
		case SOFTWARE_VERSION:
		case STM_VERSION:
#ifdef BOARD_VERSION_P1
		case CAM_MODULE_MIRROR_FOCAL_STATUS:
#endif /* BOARD_VERSION_P1 */
		{
			cmd_size = 1;
			break;
		}
		case CAM_MODULE_FINE_NUDGE_LENS:
		case CAM_MODULE_FINE_NUDGE_MIRROR:
		case CAM_STREAMING:
		{
			cmd_size = 3;
			break;
		}
		case CAM_MODULE_FOCAL_LEN:
		case CAM_MODULE_VCM_POSITION:
		case CAM_MODULE_LENS_POSITION:
		case CAM_MODULE_MIRROR_POSITION:
		case CAM_MODULE_FPS:
		case CAM_SNAPSHOT_UUID:
		case CAM_MODULE_PIEZO_LENS_CONTROL:
		case CAM_MODULE_TEMP:
		{
			cmd_size = 2;
			break;
		}
		case CAM_MODULE_SET_CAPTURE_PARAM:
		{
			cmd_size = 4;
			break;
		}
		case CAM_MODULE_EXPOSURE_TIME:
		case CAM_MODULE_RESOLUTION:
		{
			cmd_size = 8;
			break;
		}
		case CAM_MODULE_FOCUS_DISTANCE:
		case CAM_MODULE_SENSITIVITY:
		{
			cmd_size = 4;
			break;
		}
		case LIGHT_ACTIVE_UCID:
		case CAM_COMMAND_STATUS:
		{
			cmd_size = 0;
			break;
		}
		case CAM_MODULE_DEBUG_I2C_READ:
		case CAM_MODULE_DEBUG_I2C_WRITE:
		{
			cmd_size = 6;
			break;
		}
		default:
		{
			cmd_size = 0;
			break;
		}
	}

	if(cmd == LIGHT_ACTIVE_UCID || cmd == CAM_COMMAND_STATUS
			|| cmd == SOFTWARE_VERSION || cmd == CAM_MODULE_MIRROR_FOCAL_STATUS
			|| cmd == CAM_MODULE_SET_CAPTURE_PARAM || cmd == CAM_MODULE_DEBUG_I2C_READ
			|| cmd == CAM_MODULE_DEBUG_I2C_WRITE || cmd == STM_VERSION)
	{
		return cmd_size;
	}
	else
	{
		return cmd_size * cam_count;
	}
}
UInt8 I2C_8BitAddr_ReadByte(I2CEx_Msg_t *msg, UInt8 addr)
{
	msg->txbuf.data[0] = addr;
	msg->txbuf.len = 1;
	msg->rxbuf.len = 1;
	HAL_I2CEx_Transfer(msg);
	while(*msg->is_completed == EFalse){};
	return msg->rxbuf.data[0];
}

Int8 I2C_8BitAddr_WriteByte(I2CEx_Msg_t *msg, UInt8 addr, UInt8 data)
{
	msg->txbuf.data[0] = addr;
	msg->txbuf.data[1] = data;
	msg->txbuf.len = 2;
	msg->rxbuf.len = 0;
	HAL_I2CEx_Transfer(msg);
	while(*msg->is_completed == 0){};
	return *msg->is_completed;
}

Int8 I2C_8BitAddr_WriteWord(I2CEx_Msg_t *msg, UInt8 addr, UInt16 data)
{
	msg->txbuf.data[0] = addr;
	msg->txbuf.data[1] = (UInt8)(data>>8);
	msg->txbuf.data[2] = (UInt8)(data);
	msg->txbuf.len = 3;
	msg->rxbuf.len = 0;
	HAL_I2CEx_Transfer(msg);
	while(*msg->is_completed == 0){};
	return *msg->is_completed;
}

UInt16 I2C_8BitAddr_ReadWord(I2CEx_Msg_t *msg, UInt8 addr)
{
	msg->txbuf.data[0] = (UInt8)(addr);
	msg->txbuf.len = 1;
	msg->rxbuf.len = 2;
	HAL_I2CEx_Transfer(msg);
	while(*msg->is_completed == EFalse){};
	return (msg->rxbuf.data[0] << 8 | msg->rxbuf.data[1]);
}

Int8 I2C_16BitAddr_WriteWord(I2CEx_Msg_t *msg, UInt16 addr, UInt16 data)
{
	msg->txbuf.data[0] = (UInt8)(addr>>8);
	msg->txbuf.data[1] = (UInt8)(addr);
	msg->txbuf.data[2] = (UInt8)(data>>8);
	msg->txbuf.data[3] = (UInt8)(data);
	msg->txbuf.len = 4;
	msg->rxbuf.len = 0;
	HAL_I2CEx_Transfer(msg);
	while(*msg->is_completed == 0){};
	return *msg->is_completed;
}

Int8 I2C_16BitAddr_WriteByte(I2CEx_Msg_t *msg, UInt16 addr, UInt16 data)
{
	msg->txbuf.data[0] = (UInt8)(addr>>8);
	msg->txbuf.data[1] = (UInt8)(addr);
	msg->txbuf.data[2] = (UInt8)data;
	msg->txbuf.len = 3;
	msg->rxbuf.len = 0;
	HAL_I2CEx_Transfer(msg);
	while(*msg->is_completed == 0){};
	return *msg->is_completed;
}

UInt8 I2C_16BitAddr_ReadByte(I2CEx_Msg_t *msg, UInt16 addr)
{
	msg->txbuf.data[0] = (UInt8)(addr>>8);
	msg->txbuf.data[1] = (UInt8)(addr);
	msg->txbuf.len = 2;
	msg->rxbuf.len = 1;
	HAL_I2CEx_Transfer(msg);
	while(*msg->is_completed == 0){};
	return (msg->rxbuf.data[0]);
}

/* Private task functions ----------------------------------------------------*/

STATIC UInt8 SensorConfig(CamDevice_TypeDef *cam, cam_reg_array *regs, UInt16 size)
{
	int TransactionCount = 0;

	ENTER_FUNC;

	if(cam == NULL || regs == NULL)
		return 0;

	UInt16 i = 0, j = 0;
	cam_reg_array *pReg = regs;

	/* The number of register array */
	for (i = 0; i < size; i++ )
	{
		/* Size of each register array */
		for (j = 0; j < pReg->reg_size; j++)
		{
			TransactionCount++;
			if ((TransactionCount % 10) == 0) log_debug("%d transactions", TransactionCount);

			if (pReg->data_size == 1)
			{
				fpga_write_i2c_value( cam->camera_id, cam->Address, pReg->regs[j].reg_addr, pReg->regs[j].reg_val, ADDR16_DATA8, ETrue );
			}
			else
			{
				fpga_write_i2c_value( cam->camera_id, cam->Address, pReg->regs[j].reg_addr, pReg->regs[j].reg_val, ADDR16_DATA16, ETrue );
			}
		}
		pReg++;
	}

	EXIT_FUNC;
	return 1;
}

UInt8 Set_Exposure(CamDevice_TypeDef *cam, UInt16 data)
{
	ENTER_FUNC;
	UInt8  ret	= 0;
	UInt16 fll_value = CAM_EXPOSURE_DEFAULT + 1;

	//Check NULL camera
	if (cam == NULL) return ret;
	log_debug("Channel[%d] - Set_Exposure 0x%x",cam->camera_id, data);

	// check that the FLL value does not drop below the pulse width interval
#ifdef BOARD_VERSION_P1
	if (data > 2510){
		fll_value = data + 1;
	}
#endif
#ifdef BOARD_VERSION_P1_1
	if (data > cam->frame_length_lines)
		fll_value = data + 1;
    else
        fll_value = cam->frame_length_lines;
#endif
	fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0104, 0x01, ADDR16_DATA8, ETrue);
	fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0340, fll_value, ADDR16_DATA16, ETrue);
	log_debug("frame_length_lines = 0x%x\n", fll_value);
	/*Configure Trigger interval based on new FLL*/
#ifdef BOARD_VERSION_P1
	Config_Trigger(fll_value, 0xFFFFFFFF);
#endif
#ifdef BOARD_VERSION_P1_1
	Config_Trigger(cam, fll_value, 0xFFFFFFFF);
#endif

	fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0202, data, ADDR16_DATA16, ETrue);
    log_debug("coarse_integration_time = 0x%x\n", data);
	fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0104, 0x00, ADDR16_DATA8, ETrue);

	UInt16 llpclk = fpga_read_i2c_value(cam->camera_id, cam->Address, 0x0342, ADDR16_DATA16);
	log_debug("llpclk = 0x%x\n", llpclk);


#ifdef BOARD_VERSION_P1
	if (llpclk != 0x0ED8)
		fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0342, 0x0ED8, ADDR16_DATA16, ETrue);
#endif
#ifdef BOARD_VERSION_P1_1
	if (llpclk != (cam->line_length_pclk * 2) && (UC_PREVIEW == get_active_ucid()))
		fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0342,
				(cam->line_length_pclk * 2), ADDR16_DATA16, ETrue);
    log_printf("line_length_pclk = 0x%x\n", cam->line_length_pclk * 2);
#endif
	EXIT_FUNC;
	return 1;
}

UInt8 Set_Sensitivity(CamDevice_TypeDef *cam, UInt16 data)
{
	ENTER_FUNC;
	UInt8  ret	= 0;
	//Check NULL camera
	if (cam == NULL) return ret;
	log_debug("Channel[%d] - Set_Sensitivity 0x%x",cam->camera_id, data);
	ret = fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0104, 0x01, ADDR16_DATA8, ETrue);
	ret = fpga_write_i2c_value(cam->camera_id, cam->Address, 0x305E, data, ADDR16_DATA16, ETrue);
	ret = fpga_write_i2c_value(cam->camera_id, cam->Address, 0x0104, 0x00, ADDR16_DATA8, ETrue);

	EXIT_FUNC;
	return ret;
}

#ifdef BOARD_VERSION_P1
// Find value written to address in regs and provide (address, value+1) in reg.
STATIC UInt8 IncrementSensorReg(msm_camera_i2c_reg_array *reg, cam_reg_array *regs, UInt16 address)
{
	UInt8 Status = 1; // Return an error unless address is found.
	int i;

	ENTER_FUNC;

	reg->reg_addr = address;
	reg->reg_val  = 0;
	for (i = 0; i < regs->reg_size; i++)
	{
		if (regs->regs[i].reg_addr == address)
		{

			reg->reg_val = regs->regs[i].reg_val + 1;
			Status = 0;
			log_debug("  Register address = 0x%04x	Register value = 0x%04x", reg->reg_addr, reg->reg_val);
			// Do not exit yet in case the register is
			// set more than once in regs.
		}
	}

	EXIT_FUNC;

	return Status;
}

STATIC UInt8 ConfigureSensorFlips(CamDevice_TypeDef *cam, cam_reg_array *ResolutionRegs)
{
	UInt8 Status = 0;
	CamOrientation Orientation;

	msm_camera_i2c_reg_array  OrientationReg[1];
	msm_camera_i2c_reg_array  FlipHRegs[2];
	msm_camera_i2c_reg_array  FlipVRegs[2];

	cam_reg_array FlipRegsArrays[3] =
	{
			{ OrientationReg, ARRAY_COUNT(OrientationReg), 1},
			{ FlipHRegs,	  ARRAY_COUNT(FlipHRegs),	   2},
			{ FlipVRegs,	  ARRAY_COUNT(FlipVRegs),	   2},
	};

	ENTER_FUNC;

	Orientation = cam->Orientation;
#if WAR_SENSOR_FAILS_TO_SHIFT_IN_X
	Orientation &= CAM_ORIENTATION_FLIP_V; // Only flip vertically.
#endif // #if WAR_SENSOR_FAILS_TO_SHIFT_IN_X

	log_debug("Orientation=%d", Orientation);

	if (Orientation != CAM_ORIENTATION_NORMAL)
	{
		// Generate the register settings to handle flip cases.
		// Note that this code should work with all sensors - not just the AR0835.

		// image_orientation register
		OrientationReg[0].reg_addr = 0x0101;
#if WAR_SENSOR_FAILS_TO_SHIFT_IN_X
		// Add back in the horizontal flip, as the default register
		// settings set this bit in R0x0101.  If the X shift worked,
		// the standard settings could be modified to make this all much
		// simpler.
		OrientationReg[0].reg_val = cam->Orientation | CAM_ORIENTATION_FLIP_H;
#else // #if WAR_SENSOR_FAILS TO_SHIFT_IN_X
		OrientationReg[0].reg_val  = cam->Orientation;
#endif // #if WAR_SENSOR_FAILS_TO_SHIFT_IN_X
		Status |= SensorConfig(cam, &FlipRegsArrays[0], 1);

		// Shift X by one pixel to help correct the Bayer phase.
		// NOTE: This appears to be broken on the AR0835!
		if (Orientation & CAM_ORIENTATION_FLIP_H)
		{
			// Increment x_addr_start and x_addr_end.
			IncrementSensorReg(&FlipHRegs[0], ResolutionRegs, 0x0344);
			IncrementSensorReg(&FlipHRegs[1], ResolutionRegs, 0x0348);
			Status |= SensorConfig(cam, &FlipRegsArrays[1], 1);
		}

		// Shift Y by one pixel to help correct the Bayer phase.
		if (Orientation & CAM_ORIENTATION_FLIP_V)
		{
			// Increment y_addr_start and y_addr_end.
			IncrementSensorReg(&FlipVRegs[0], ResolutionRegs, 0x0346);
			IncrementSensorReg(&FlipVRegs[1], ResolutionRegs, 0x034a);
			Status |= SensorConfig(cam, &FlipRegsArrays[2], 1);
		}
	}

	return Status;
}
#endif/* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1

STATIC UInt8 IncrementSensorReg(msm_camera_i2c_reg_array *reg, cam_reg_array *regs, UInt16 address, UInt8 size)
{
	UInt8 Status = 1; // Return an error unless address is found.
	int i, j;
	msm_camera_i2c_reg_array *msm_regs;
	ENTER_FUNC;

	reg->reg_addr = address;
	reg->reg_val  = 0;
	for (i = 0; i < size; i++)
	{
		msm_regs = regs[i].regs;
		for (j = 0 ; j< regs[i].reg_size; j++)
		{
			if (msm_regs[j].reg_addr == address)
			{

				reg->reg_val = msm_regs[j].reg_val + 1;
				Status = 0;
				log_debug("  Register address = 0x%04x	Register value = 0x%04x", reg->reg_addr, reg->reg_val);
				// Do not exit yet in case the register is
				// set more than once in regs.
			}
		}
	}

	EXIT_FUNC;

	return Status;
}


STATIC UInt8 ConfigureSensorFlips(CamDevice_TypeDef *cam, cam_reg_array *ResolutionRegs, UInt8 size)
{

	UInt8 Status = 0;
	CamOrientation Orientation;

	msm_camera_i2c_reg_array  OrientationReg[1];
	msm_camera_i2c_reg_array  FlipHRegs[2];
	msm_camera_i2c_reg_array  FlipVRegs[2];

	cam_reg_array FlipRegsArrays[3] =
	{
			{ OrientationReg, ARRAY_COUNT(OrientationReg), 1},
			{ FlipHRegs,	  ARRAY_COUNT(FlipHRegs),	   2},
			{ FlipVRegs,	  ARRAY_COUNT(FlipVRegs),	   2},
	};

	ENTER_FUNC;

	Orientation = cam->Orientation;

	OrientationReg[0].reg_val = 0x0;


	OrientationReg[0].reg_addr = 0x0101;

	//FIXME: For now this is done as horizontal flip leads to bayer phase shift
	// which cannot be corrected by shifting column
	Orientation &= CAM_ORIENTATION_FLIP_V;


	if(Orientation & CAM_ORIENTATION_FLIP_V)
	{
		OrientationReg[0].reg_val = 0x0  | CAM_ORIENTATION_FLIP_V;
	}
	else if (Orientation & CAM_ORIENTATION_FLIP_H)
	{
		OrientationReg[0].reg_val = 0x0 | CAM_ORIENTATION_FLIP_H;
	}
	else if (Orientation & CAM_ORIENTATION_FLIP_HV)
	{
		OrientationReg[0].reg_val = 0x0 | CAM_ORIENTATION_FLIP_V | CAM_ORIENTATION_FLIP_H ;
	}

	Status |= SensorConfig(cam, &FlipRegsArrays[0], 1);

	// Shift x by one pixel to help correct the Bayer phase.
	if (Orientation & CAM_ORIENTATION_FLIP_H)
	{
		// Increment x_addr_start and x_addr_end
		IncrementSensorReg(&FlipHRegs[0], ResolutionRegs, 0x0344, size);
		IncrementSensorReg(&FlipHRegs[1], ResolutionRegs, 0x0348, size);
		Status |= SensorConfig(cam, &FlipRegsArrays[1], 1);
	}

	// Shift Y by one pixel to help correct the Bayer phase.
	if (Orientation & (CAM_ORIENTATION_FLIP_V) )
	{
		// Increment y_addr_start and y_addr_end.
		IncrementSensorReg(&FlipVRegs[0], ResolutionRegs, 0x0346, size);
		IncrementSensorReg(&FlipVRegs[1], ResolutionRegs, 0x034a, size);
		Status |= SensorConfig(cam, &FlipRegsArrays[2], 1);
	}



	EXIT_FUNC;

	return Status;
}

#endif/* BOARD_VERSION_P1_1 */
//							  A1| A2|A3.....
uint8_t fpga_cam_reg_offset[]={1, 0, 4, 2, 3, 9, 8, 7, 5, 6, 10, 11, 13, 12, 15, 14};

UInt8 update_fpga_sensor_resolution(CamDevice_TypeDef *cam, UInt32 res_x, UInt32 res_y)
{
	ENTER_FUNC;

	UInt8 ret = 0;
	UInt8 len = 5;
	UInt16 tx[5];
	UInt16 rx[5];
	fpga_cmd_error_t tmp = 0;

	log_debug("Sending resolution settings command to the FPGA \n\r");
	tx[0] = CAPTURE_VC_CHANNEL_CMD;
	tx[1] = 0x1020 + fpga_cam_reg_offset[cam->camera_id - 1];
	tx[2] = 0x8004;
	tx[3] = 0x0000;

	if(res_x == 1920 && res_y == 1080)
	{
		log_debug("FPGA 1080P res change");
		tx[4] = 0x0001;
	}
	else if(res_x == 4208 && res_y == 3120)
	{
		log_debug("FPGA 13MP res change");
		tx[4] = 0x0000;
	}
	else
	{
		tx[4] = 0x0000;
		log_debug("FPGA Change not required for res change");
	}

	tmp = fpga_send_command(tx, rx, len, IRQ_FROM_FPGA_TIMEOUT);
	if(tmp == FPGA_CMD_TOUT)
	{
		log_printf("resolution settings failed: no response from the FPGA!\n\r");
		ret = 3;
		goto EXIT;
	}
EXIT:
	return ret;

}
/*
 * TODO: Host has not implemented yet.
 * This function just support 2 resolutions (8MP and 1080)
 */
UInt8 Set_Resolution(CamDevice_TypeDef *cam, UInt32 res_x, UInt32 res_y)
{
#ifdef BOARD_VERSION_P1
	cam_reg_array *regs = NULL;
	UInt8 Status = 0;

	ENTER_FUNC;

	//Check NULL camera
	if (cam == NULL) return 0;

	if(res_x == 3264 && res_y == 2448)
	{
		regs = &CamResolutionRegs[0];
	}
	else if(res_x == 1920 && res_y == 1080)
	{
		regs = &CamResolutionRegs[1];
	}
	else
	{
		log_error("[%s] Unsupported resolution [0x%x - 0x%x]", cam->Name, (unsigned int) res_x, (unsigned int) res_y);
		EXIT_FUNC;
		return 0;
	}

	if (regs)
	{
		Status	= SensorConfig(cam, regs, 1);
		Status |= ConfigureSensorFlips(cam, regs);
	}

	log_info("[%s] Set resolution to [0x%x - 0x%x] %s",  cam->Name, (unsigned int) res_x, (unsigned int) res_y, Status == 1 ? "success" : "fail");


	EXIT_FUNC;

	return Status;
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
	UInt8 Status = 0;
	UInt8 reg_size = 0;
	ENTER_FUNC;

	//Check NULL camera
	if (cam == NULL) return 0;

	cam_reg_array *preg = resolution_selection(res_x,res_y,30,&reg_size);
	if(!preg)
	{
		log_error("[%s] Unsupported resolution [0x%x - 0x%x]", cam->Name, (unsigned int) res_x, (unsigned int) res_y);
		EXIT_FUNC;
		return 0;
	}


	update_fpga_sensor_resolution(cam, res_x, res_y);

	/* just update some registers when switch resolution form 1080p to full resolution or opposite */
	if(setting_resolution == UPDATE_RESOLUTION)
	{
		log_debug("Update resolution");
		Status = update_resolution_register(cam, preg);
	}
	else
	{
		log_debug("Set new resolution");
		Status	= SensorConfig(cam, preg, reg_size);
	}

	Status |= ConfigureSensorFlips(cam, preg, reg_size);

	log_info("[%s] Set resolution to [0x%x - 0x%x] %s",  cam->Name, (unsigned int) res_x, (unsigned int) res_y, Status == 1 ? "success" : "fail");

	EXIT_FUNC;

	return Status;
#endif /* BOARD_VERSION_P1_1 */
}

/*
 *	TODO: Implement FPS settings. It should be defined by Host.
 */

UInt8 Set_FPS(CamDevice_TypeDef *cam, UInt16 fps)
{
	if(cam == NULL) return 0;
	if(fps == 30)
	{
		log_info("[%s] FPS value has been set to 0x%x", cam->Name, fps);
	}
	else
	{
		log_error("[%s] Unsupported FPS value 0x%x", cam->Name, fps);
	}
	/*TO DO:*/
	return 1;
}



UInt8 vAR0835_EEPROM(CamDevice_TypeDef *cam)
{
	I2CEx_Msg_t msg;
	UInt8 txbuf[8], rxbuf[8];

	//Check NULL camera
	if (cam == NULL) return 0;

	msg.txbuf.data = &txbuf[0];
	msg.rxbuf.data = &rxbuf[0];
	msg.ch = cam->I2CChannel;
	msg.is_completed = &_status;

	UInt8 EEPROM_IDX = 0;
	UInt8 temp=0;
	// Select CSI channel

	msg.addr = 0xA0;

	if((temp=I2C_8BitAddr_ReadByte(&msg, 0))!=0x01)
	{
		log_printf("Data addr [0]=[0x%02x]\n\r",temp);
		//return 0;
	}
	for(int i=0xA0;i<0xAF;i=i+2)
	{
		msg.addr = i;

		for(int j=0;j<256;j++)
		{
			*(CCB_M_EEPROM + (EEPROM_IDX*256) + j) = I2C_8BitAddr_ReadByte(&msg, j);
			log_printf("0x%02X	%02X  0x%02X \r\n",i,j, I2C_8BitAddr_ReadByte(&msg, j) );

		}
		EEPROM_IDX ++;
	}
return 1;
}
void StatusResetAll(void)
{
	memset(__cam_status,0, sizeof(__cam_status));
#ifdef BOARD_VERSION_P1
	cam_mirror_status = 0;
#endif /* BOARD_VERSION_P1 */
}
void StatusEnable(UInt8 idx,status_t mask)
{
//	  log_debug("Enter %s function", __FUNCTION__);
	if(idx < CAM_NUM_CAMERA_MAX)
		__cam_status[idx] |=(UInt32)mask;
	else
	{

	}
//	  log_debug("Exit %s function", __FUNCTION__);
	return;
}
void StatusDisable(UInt8 idx,status_t mask)
{
//	  log_debug("Enter %s function", __FUNCTION__);
	if(idx < CAM_NUM_CAMERA_MAX)
		__cam_status[idx] &=(UInt32)(~mask);
	else
	{

	}
//	  log_debug("Exit %s function", __FUNCTION__);
	return;
}
UInt32 StatusMask(UInt8 idx)
{
	if(idx < CAM_NUM_CAMERA_MAX)
	return __cam_status[idx];
	else
		return 0xffffffff;
}
Bool StatusActivated(UInt8 idx, status_t mask)
{
	if(idx < CAM_NUM_CAMERA_MAX)
		return (__cam_status[idx] & (UInt32)mask) ? 0x01 : 0x0;
	else
		return 0;
}
void cam_focus_status_reset_all(void)
{
	cam_focus_status = 0;
}
Bool check_cam_focus_status_actived(UInt8 index, cam_focus_status_t mask)
{
	if((index < CAM_NUM_CAMERA_MAX) && ((cam_focus_status >> (index*2)) & mask))
	{
		return ETrue ;
	}
	else
	{
		return EFalse;
	}
}
void set_cam_focus_status(UInt8 index, cam_focus_status_t mask)
{
	if((index < CAM_NUM_CAMERA_MAX))
	{
		cam_focus_status  |= (UInt32)((UInt8)mask << (index*2));
	}
}
void reset_cam_focus_status(UInt8 index, cam_focus_status_t mask)
{
	if((index < CAM_NUM_CAMERA_MAX))
	{
		cam_focus_status &= (UInt32)(~((UInt8)mask << (index*2)));
	}
}
UInt32 read_cam_focus_status(UInt8 index)
{
	if((index < CAM_NUM_CAMERA_MAX))
	{
		return (UInt32)cam_focus_status;
	}
	else
	{
		return 0xFF;
	}
}
Bool check_cam_mirror_status_actived(UInt8 index, cam_focus_status_t mask)
{
	if((index < CAM_NUM_CAMERA_MAX) && ((cam_mirror_status >> (index*2)) & mask))
	{
		return ETrue ;
	}
	else
	{
		return EFalse;
	}
}
void set_cam_mirror_status(UInt8 index, cam_focus_status_t mask)
{
	if((index < CAM_NUM_CAMERA_MAX))
	{
		cam_mirror_status  |= (UInt32)((UInt8)mask << (index*2));
	}
}
void reset_cam_mirror_status(UInt8 index, cam_focus_status_t mask)
{
	if((index < CAM_NUM_CAMERA_MAX))
	{
		cam_mirror_status &= (UInt32)(~((UInt8)mask << (index*2)));
	}
}
UInt32 read_cam_mirror_status(void)
{
   return (UInt32)cam_mirror_status;
}

static void cam_init_queue(void)
{
	log_debug("Enter %s function", __FUNCTION__);
	cam_cmd_queue = xQueueCreate(CAM_CMD_QUEUE_SIZE, sizeof(cam_cmd_t));
	log_debug("Exit %s function", __FUNCTION__);
}

void cam_cmd_task(void *pvParameters)
{
    portBASE_TYPE xStatus;
    cam_cmd_t CurrentCmd;
    UInt8 i = 0;
    uint8_t array_size = 0;

    (void)pvParameters;
    /* As per most tasks, this task is implemented in an infinite loop. */
    while(1)
    {
        // Waiting for new command from host
        while (ReceiveCmd == 0xFFFF);
        CurrentCmd.cmd = ReceiveCmd;
        CurrentCmd.data_len = ReceiveDataSize;
        CurrentCmd.flags = ReceivedFlags;
        CurrentCmd.tid = ReceiveTid;

        if ((CurrentCmd.cmd == CAM_MODULE_OPEN)                   || /* Open */
            (CurrentCmd.cmd == CAM_STREAMING)                     || /* Stream */
			(CurrentCmd.cmd == RDI_CAPTURE)                       || /* Capture */
            (CurrentCmd.cmd == CAM_MODULE_SENSITIVITY)            || /* Global gain */
            (CurrentCmd.cmd == CAM_MODULE_EXPOSURE_TIME)          || /* Exposure */
            (CurrentCmd.cmd == CAM_MODULE_RESOLUTION)             || /* Resolution */
            (CurrentCmd.cmd == CAM_MODULE_VCM_POSITION)           || /* AF DAC */
            (CurrentCmd.cmd == CAM_MODULE_FOCUS_DISTANCE)         || /* AF Distance*/
            (CurrentCmd.cmd == CAM_MODULE_LENS_POSITION)          || /* FPS */
            (CurrentCmd.cmd == CAM_MODULE_STATUS)                 || /* Module Status*/
            (CurrentCmd.cmd == CAM_MODULE_LENS_CALIBRATION)       || /* Lens calibration*/
            (CurrentCmd.cmd == CAM_MODULE_LENS_CALIBRATION2)      || /* Lens calibration*/
            (CurrentCmd.cmd == CAM_MODULE_MIRROR_CALIBRATION)     || /* Mirror calibration*/
            (CurrentCmd.cmd == CAM_MODULE_MIRROR_HALL)            ||
            (CurrentCmd.cmd == CAM_MODULE_LENS_HALL)              ||
            (CurrentCmd.cmd == CAM_MODULE_LENS_POSITION)          ||
            (CurrentCmd.cmd == CAM_MODULE_LENS_NUDGE)             || /* Lens nudge */
            (CurrentCmd.cmd == CAM_MODULE_MIRROR_POSITION)        ||
            (CurrentCmd.cmd == CAM_MODULE_MIRROR_NUDGE)           || /* Mirror nudge */
            (CurrentCmd.cmd == CAM_MODULE_DEBUG_I2C_READ)         ||
            (CurrentCmd.cmd == CAM_MODULE_DEBUG_I2C_WRITE)        ||
            (CurrentCmd.cmd == CAM_MODULE_DEBUG_PIEZO_MONITOR2)   ||
            (CurrentCmd.cmd == CAM_MODULE_FOCUS_CALIBRATION_DATA) ||
            (CurrentCmd.cmd == CAM_MODULE_FPS)                    ||
            (CurrentCmd.cmd == CAM_MODULE_FINE_NUDGE_LENS)        ||
            (CurrentCmd.cmd == CAM_MODULE_FINE_NUDGE_MIRROR)      ||
            (CurrentCmd.cmd == CAM_MODULE_PIEZO_LENS_CONTROL)     ||
            (CurrentCmd.cmd == CAM_MODULE_PIEZO_MIRROR_CONTROL)   ||
            (CurrentCmd.cmd == CAM_MODULE_LENS_FREQ_CALIBRATION)  || /* Lens frequence calibration*/
            (CurrentCmd.cmd == CAM_MODULE_MIRROR_FREQ_CALIBRATION)|| /* Mirror frequence calibration*/
            (CurrentCmd.cmd == CAM_MODULE_FOCUS_STATUS)           ||
			(CurrentCmd.cmd == SOFTWARE_VERSION)				  ||
			(CurrentCmd.cmd == STM_VERSION)						  ||
			(CurrentCmd.cmd == CAM_MODULE_TEMP)					  ||
			(CurrentCmd.cmd == CAM_MODULE_SAVE_POSITION)		  ||
			(CurrentCmd.cmd == CAM_MODULE_SET_CAPTURE_PARAM)	  ||
			(CurrentCmd.cmd == CAM_MODULE_MIRROR_FOCAL_STATUS))
        {
        	array_size = ARRAY_COUNT(CurrentCmd.bitmask);
            for(i = 0; i < array_size; i++)
            {
                CurrentCmd.bitmask[i] = *(CCB_M_BITMASK + i);
            }

            CurrentCmd.ucid = (*(CCB_M_UCID + 1) << 8) | (*CCB_M_UCID);
            if  (CurrentCmd.cmd == CAM_MODULE_LENS_POSITION)
            {
            	CurrentCmd.tolerance = *(CCB_M_TOLERANCE);
            }
            //log_debug("CurrentCmd.ucid = 0x%02x\n", CurrentCmd.ucid);
            //log_printf("CurrentCmd.bitmask = 0x%02x 0x%02x 0x%02x\n", CurrentCmd.bitmask[0], CurrentCmd.bitmask[1], CurrentCmd.bitmask[2]);

            /* Some commands don't require data, skip them to increase performance */
            if( (CurrentCmd.cmd != CAM_MODULE_STATUS)                  &&
                (CurrentCmd.cmd != CAM_MODULE_LENS_CALIBRATION)        &&
                (CurrentCmd.cmd != CAM_MODULE_LENS_CALIBRATION2)       &&
                (CurrentCmd.cmd != CAM_MODULE_MIRROR_CALIBRATION)      &&
                (CurrentCmd.cmd != CAM_MODULE_MIRROR_HALL)             &&
                (CurrentCmd.cmd != CAM_MODULE_LENS_HALL)               &&
                (CurrentCmd.cmd != CAM_MODULE_NUDGE_CPLD)			   &&
                (CurrentCmd.cmd != CAM_MODULE_LENS_FREQ_CALIBRATION)   &&
                (CurrentCmd.cmd != CAM_MODULE_MIRROR_FREQ_CALIBRATION) &&
                (CurrentCmd.cmd != CAM_MODULE_FOCUS_STATUS)            &&
				(CurrentCmd.cmd != SOFTWARE_VERSION)				   &&
				(CurrentCmd.cmd != STM_VERSION)						   &&
				(CurrentCmd.cmd != CAM_MODULE_TEMP)					   &&
				(CurrentCmd.cmd != CAM_MODULE_SAVE_POSITION)		   &&
				(CurrentCmd.cmd != CAM_MODULE_MIRROR_FOCAL_STATUS))
            {
#ifdef CONVERT_TO_LITTLE_ENDIAN
            	uint32_t cam_bitmask = 0;
				uint8_t i = 0, j = 0;
				uint8_t index = 0;
				uint8_t data_size = 0;
				uint8_t valid_data_length = 0;

				cam_bitmask = ((CurrentCmd.bitmask[2] << 16) |
						       (CurrentCmd.bitmask[1] << 8)  |
							    CurrentCmd.bitmask[0])	& 0x00FFFFFF;

				/* number of byte in one data with 1 camera */
				data_size = get_require_data_size(0x01, CurrentCmd.cmd);
				/* valid total number of byte have to receives */
				valid_data_length = get_require_data_size(cam_bitmask, CurrentCmd.cmd);

				if ((valid_data_length != CurrentCmd.data_len) &&
					(CurrentCmd.data_len != 0))
				{
					log_error("Number of data received is not match with number of required data");
					log_error("Number of received data: %d", CurrentCmd.data_len);
					log_error("Number of required data: %d", valid_data_length);
					ReceiveCmd = 0xFFFF;
					continue;
				}
				else
				{
					if (data_size != 0)
					{
						i = 0;
						while (i < CurrentCmd.data_len)
						{
							index = i + data_size - 1;
							j = i;
							while (j < i + data_size)
							{
								CurrentCmd.data[j] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + index);
								//log_debug("CurrentCmd.data[%d] = 0x%02x", j, CurrentCmd.data[j]);
								index--;
								j++;
							}
							i += data_size;
						}
					}
				}
#else
	        	array_size = ARRAY_COUNT(CurrentCmd.data);
	            for(i = 0; i < array_size; i++)
	            {
	                CurrentCmd.data[i] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + i);
	                //log_printf(" CurrentCmd.data[%d] = 0x%02x\n", i, CurrentCmd.data[i]);
	            }
#endif
            }
        }
        else if(CurrentCmd.cmd == LIGHT_ACTIVE_UCID)
        {
#ifdef CONVERT_TO_LITTLE_ENDIAN
        	CurrentCmd.ucid = (*(CCB_M_UCID + 1) << 8) | (*CCB_M_UCID);
#else
            CurrentCmd.ucid = ((*CCB_M_UCID) << 8) | (*(CCB_M_UCID + 1));
#endif
        }
        else if(CurrentCmd.cmd == SPI_TRANSFER_FPGA)
        {
            /* This is specified handler for SPI_TRANSFER_FPGA
             * Because current support data of other commands is 8 bytes
             * SPI_TRANSFER_FPGA does not need a m_bitmask
             */
            fpga_debug_cmd[0] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd); /* asic_id */
            fpga_debug_cmd[1] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 1); /* transaction_type */
            fpga_debug_cmd[2] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 2); /* data_len */

            if(fpga_debug_cmd[2] > FPGA_MAX_DATA_SIZE)
                log_debug("Maximum data len is 64 bytes. Others will be ignored");

            fpga_debug_cmd[2] = (fpga_debug_cmd[2] > FPGA_MAX_DATA_SIZE ) ? FPGA_MAX_DATA_SIZE : fpga_debug_cmd[2];

            for(i = 0; i < fpga_debug_cmd[2]; i++)
                fpga_debug_cmd[3+i] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 3 + i); /* data */
        }
        else if( CurrentCmd.cmd == SPI_TRANSFER_CPLD_SINGLE )
        {
            cpld_debug_cmd[0] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd); /*transaction_type*/
            cpld_debug_cmd[1] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 1); /*1 Byte Offset*/
            cpld_debug_cmd[2] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 2); /*1 Byte Value*/
        }
        else if( CurrentCmd.cmd == SPI_TRANSFER_CPLD )
        {
            CPLD_data_size = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd);
            // CPLD_data_size refers to just the data, wihtout the
            // transaction type or the address

            //log_printf("CPLD_data_size = %d\n\n", CPLD_data_size);
            /*
            if (CPLD_data_size > FPGA_MAX_DATA_SIZE - 3)
            {
                log_debug("Transaction size is too large. Maximum size is 2'd%d bytes.\n", FPGA_MAX_DATA_SIZE - 1);
            }
            else
            */
            {
                for (int i = 0; i < CPLD_data_size + 3; i++)
                {
                    cpld_debug_cmd[i] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + 1 + i);
                    //log_printf("cpld_debug_cmd[%d] = %02x\n\n", i, cpld_debug_cmd[i]);
                }
            }
        }
        else if( CurrentCmd.cmd == CONFIGURE_CPLD )
        {
            // copy all the data into the buffer to use later
            memcpy((uint8_t *) &cpld_config_buffer,
                   (uint8_t *) (CCB_ASIC_CAM_RD + CurrentCmd.cmd), CPLD_CONFIG_SIZE);
                   /*
            for(int i = 0; i < CPLD_CONFIG_SIZE; i++)
            {
                log_debug("cpld_config_buffer[%d]: 0x%02x\n\n", i,
                        ((uint8_t *)&cpld_config_buffer)[i]);
            }
            */
        }
        else
        {
        	array_size = ARRAY_COUNT(CurrentCmd.data);
            for(i = 0; i < array_size; i++)
            {
                CurrentCmd.data[i] = *(CCB_ASIC_CAM_RD + CurrentCmd.cmd + i);
                //log_printf(" CurrentCmd.data[%d] = 0x%02x\n", i, CurrentCmd.data[i]);
            }
        }

        xStatus = xQueueSendToBack(cam_cmd_queue, &CurrentCmd, 0);
        if (xStatus == errQUEUE_FULL)
        {
            /* Resume the scheduler as method of mutual exclusion. */
            log_error("The camera control queue is full!");
        }

        ReceiveCmd = 0xFFFF;
    }
}


void FTM_cam_modules(void)
{
	ENTER_FUNC;
	I2CEx_Msg_t msg;
	UInt8 i = 0;
	UInt8 txbuf[2], rxbuf[2];

	UInt16 device_id = 0;

	UInt16 spi_tx[16], spi_rx[16];

	msg.txbuf.data = &txbuf[0];
	msg.rxbuf.data = &rxbuf[0];
	msg.is_completed = &_status;

	if (fpga_check_cam_pg() == FPGA_PWR_FAIL)
		goto FAIL;

	log_printf("----Hall sensor\n\r");
	for(i = 0; i < CAM_NUM_CAMERA_MAX; i++)
	{
		if(FTMCamDeviceTbl[i] == NULL) continue;

		if(FTMCamDeviceTbl[i]->Type == CAM_TYPE_70MM)
		{
			vTaskDelay(1);
			StatusEnable(i,MODULE_SW_STANDBY);
			/* PIEZO */
			/* Hall Sensor */
			for(UInt8 id = 0; id < 2; id++)
			{
				msg.ch = FTMCamDeviceTbl[i]->I2CChannel;
				UInt8 c = (id%2) ? MIRROR_ADDR_WR: LENS_ADDR_WR;
				msg.addr = c;
				I2C_8BitAddr_ReadByte(&msg,0x0);
				if(*msg.is_completed == -1)
				{
					StatusDisable(i,MODULE_SW_STANDBY);
					if(msg.addr==LENS_ADDR_WR)
					{
						StatusEnable(i,ERROR_LENS_DETECT);
					}
					else
					{
						StatusEnable(i,ERROR_MIRRORS_DETECT);
					}
				}
				log_printf("[%s - %s] %s\n\r", FTMCamDeviceTbl[i]->Name,
									   (c & 0x02) ? "MIRR" : "LENS",
									   (*msg.is_completed == -1) ? "Not connected" : "Connected");
			}
		}

		if(FTMCamDeviceTbl[i]->Type == CAM_TYPE_150MM)
		{
			StatusEnable(i,MODULE_SW_STANDBY);
			vTaskDelay(1);
			/* PIEZO */
			/* Hall Sensor */
			for(UInt8 id = 0; id < 2; id++)
			{
				msg.ch = CamDeviceTbl[i]->I2CChannel;
				UInt8 c = (id%2) ? MIRROR_ADDR_WR: LENS_ADDR_WR;
				msg.addr = c;
				I2C_8BitAddr_ReadByte(&msg,0x0);
				if(_status == -1)
				{
					StatusDisable(i,MODULE_SW_STANDBY);
					if(msg.addr==LENS_ADDR_WR)
					{
						StatusEnable(i,ERROR_LENS_DETECT);
					}
					else
					{
						StatusEnable(i,ERROR_MIRRORS_DETECT);
					}
				}
				log_printf("[%s - %s] %s\n\r", CamDeviceTbl[i]->Name,
									   (c & 0x02) ? "MIRR" : "LENS",
									   (*msg.is_completed == -1) ? "Not connected" : "Connected");
			}
		}
	}

	log_printf("----FPGA\n\r");

	/*Check Power Good Signal from FPGA and Camera sensors*/
	if (fpga_check_pg() == FPGA_PWR_FAIL)
		goto FAIL;

	// Send enable command to FPGA
	spi_tx[0] = 0x1001;
	// Send FTM (Quick check) command
	log_printf("Waiting for FPGA init sensor . . .\n\r");
	if (fpga_send_command(spi_tx, spi_rx, 1, IRQ_FROM_FPGA_TIMEOUT) == FPGA_CMD_TOUT)
		goto FAIL;

	spi_tx[0] = 0xFF01;
	fpga_send_command(spi_tx, spi_rx, 1, IRQ_FROM_FPGA_TIMEOUT);	// Send FTM (Quick check) command

	spi_tx[0] = 0x0000;
	fpga_send_command(spi_tx, spi_rx, 16, IRQ_FROM_FPGA_TIMEOUT);	// Send FTM (Quick check) command
	log_printf("FPGA returned: ");
	for (i = 0; i < 16; i++)
	{
		log_printf("[0x%04x]", *(spi_rx + i));
	}
	log_printf("\n\r");

	for (i = 0; i < 16; i++)
	{
		device_id = spi_rx[i];	// m_bitmask
		if (device_id != 0)
		{
			StatusEnable(i, MODULE_SW_STANDBY);
			StatusDisable(i, MODULE_HW_STANDBY);
			log_debug("DeviceID[%s]: %04x\n\r", FTMCamDeviceTbl[i]->Name, device_id);
		}
		else
		{
			StatusEnable(i, ERROR_SENSOR_I2C_DETECT_FAILURE);
			StatusDisable(i, MODULE_OPEN | MODULE_SW_STANDBY | MODULE_HW_STANDBY);
			log_error("DeviceID[%s] is not present\n\r", FTMCamDeviceTbl[i]->Name);
		}
	}
	goto EXIT;

FAIL:
	log_printf("FPGA communication failed!\n\r");

EXIT:
	EXIT_FUNC;
	return;

}

STATIC Bool read_fpga_version(UInt8 *major, UInt8 *minor)
{
	UInt16 tx[2] = {0}, rx[2] = {0};
	fpga_cmd_error_t fpga_state;
	tx[0] = 0x0191;
	tx[1] = 0x0000;
	fpga_state = fpga_send_command( tx, rx, 2, IRQ_FROM_FPGA_TIMEOUT );
	if(fpga_state == FPGA_CMD_TOUT || fpga_state == FPGA_CMD_WAITING)
	{
		log_error("Timed out while sending data to the FPGA");
		return EFalse;
	}
	else
	{
		log_debug("Sending read version command to FPGA...\n\r");
	}
	fpga_state = fpga_send_command(tx, rx, 2, IRQ_FROM_FPGA_TIMEOUT);
	if(fpga_state == FPGA_CMD_TOUT || fpga_state == FPGA_CMD_WAITING)
	{
		log_error("Timed out while reading data from the FPGA");
		return EFalse;
	}
	else
	{
		*major = (UInt8)((rx[0] & 0x00F0) >> 4);
		*minor = (UInt8)(rx[0] & 0x000F);
		log_debug("FPGA read version data %x \n\r", rx[0]);
	}
	return ETrue;
}

STATIC UInt8 SensorTransferFrame(UInt32 i2c_channel_mask)
{
	UInt16 tx[6];
	UInt16 rx[8];
	fpga_cmd_error_t tmp = 0;
	UInt8 ret = 0;

	ENTER_FUNC;

	tx[0] = 0x01B1; // Transfer Frames Command
	tx[1] = 0x0000;
	tx[3] = 0x0000;
	tx[4] = 0x0000;
	tx[5] = 0x0000;

	switch (i2c_channel_mask)
	{
		case GROUP_AB:
			tx[2] = 0x2000;
			break;
		case GROUP_BC:
			tx[2] = 0x2A00;
			break;
		case GROUP_C:
			tx[2] = 0x3400;
			break;
		default:
			break;
	}

	log_debug("Transfer frame\n\r");
	tmp = fpga_send_command(tx, rx, 6, IRQ_FROM_FPGA_TIMEOUT);

	if(tmp == FPGA_CMD_TOUT)
	{
			log_error("Transfer frames failed: no response from the FPGA!\n\r");
			ret = 3;
			goto EXIT;
	}
	ret = 1;
EXIT:
	EXIT_FUNC;
	return ret;

}

UInt8 Snapshot(UInt32 I2C_Channel_Mask, UInt8 Intr)
{
	ENTER_FUNC;
	UInt8 ret = 0;
	UInt8 i = 0;
	UInt16 tx[3];
	UInt16 rx[3];
	fpga_cmd_error_t tmp = 0;

	if(Intr != 0 && Intr != 1)
	{
		log_error("Invalid Interrupt status !!");
		ret = 2;
		goto EXIT;
	}

	tx[0] = 0x00FF & ((Intr << 7) | 0x032);
	tx[1] = (UInt16)(I2C_Channel_Mask & 0xFFFF);
	tx[2] = (UInt16)((I2C_Channel_Mask >> 16) & 0x00FF);

#if 1
	for(i = 0; i < 3 ; i++)
	{
		log_printf("  [%04x]", *(tx + i));
	}
#endif
	log_printf("\r\n");
	tmp = fpga_send_command(tx, rx, 3, IRQ_FROM_FPGA_TIMEOUT);
	if(tmp == FPGA_CMD_TOUT)
	{
		log_error("SNAPSHOT failed: no response from the FPGA!\n\r");
		ret = 3;
		goto EXIT;
	}
	ret = 1;
EXIT:
	EXIT_FUNC;
	return ret;
}
static int set_cam_csi_channel(uint32_t cam_idx, uint8_t csi_channel, uint8_t enable_stream)
{
	int ret = 0;

	if (enable_stream)
	{
		switch (csi_channel) {
		case FPGA_CSI_CHANNEL_0:
			StatusEnable(cam_idx - 1, CSI_CAM_CHANNEL_0);
			StatusDisable(cam_idx - 1, CSI_CAM_CHANNEL_1);
			break;
		case FPGA_CSI_CHANNEL_1:
			StatusEnable(cam_idx - 1, CSI_CAM_CHANNEL_1);
			StatusDisable(cam_idx - 1, CSI_CAM_CHANNEL_0);
			break;
		default:
			log_debug("Channel not supported\n")
			break;
		}
	}
	else
	{
		StatusDisable(cam_idx - 1, CSI_CAM_CHANNEL_0);
		StatusDisable(cam_idx - 1, CSI_CAM_CHANNEL_1);
	}

	return ret;

}

static uint8_t get_cam_csi_channel(uint32_t cam_idx)
{
	uint8_t ret = 0;

	ret = (StatusActivated(cam_idx - 1, CSI_CAM_CHANNEL_0) ? 0x1 : 0x0) |
			(StatusActivated(cam_idx - 1, CSI_CAM_CHANNEL_1) ? 0x2 : 0x0 );

	return ret;

}
UInt8 Preview(UInt8 Intr, UInt8 count, UInt8 *CamOrder, UInt8 *ControlStatusReg, UInt8 *VirtualChannelIdentifier, UInt8 *DataType)
{
	ENTER_FUNC;

	UInt8 *ControlStatusReg_val = ControlStatusReg;
	UInt8 *CamOrder_val = CamOrder;
	UInt8 ret = 0;
	UInt8 cs_channel = 0;
	CamDevice_TypeDef *pCam = NULL;
	UInt8 len = 0;
	len = (count + 1);
	UInt16 tx[len];
	UInt16 rx[len];
	fpga_cmd_error_t tmp = 0;

	log_debug("Sending PREVIEW command to the FPGA \n\r");

	if(Intr != 0 && Intr != 1)
	{
		log_error("Invalid Interrupt status !!");
		ret = 2;
		goto EXIT;
	}

	tx[0] = ((count & 0x0F) << 8) | (Intr << 7) | 0x33;
	for (int i = 1; i <= count; i++)
	{
		pCam = CamDeviceTbl[CamOrder_val[i-1] - 1];
		switch (((ControlStatusReg_val[i-1] & 0xF0)) >> 4) {
			case 0x1:
				cs_channel = FPGA_CSI_CHANNEL_0;
				log_debug("Setting CS0 Interrupt status !! \n");
				break;
			case 0x2:
				cs_channel = FPGA_CSI_CHANNEL_1;
				log_debug("Setting CS1 Interrupt status !! \n");
				break;
			default:
				log_debug("default case setting CS0 Interrupt status !! \n");
				cs_channel = FPGA_CSI_CHANNEL_0;
				break;
		}
		log_debug("CamOrder_val 0x%x ControlStatusReg_val 0x%x channel: 0x%x\n", CamOrder_val[i-1], ControlStatusReg_val[i-1], cs_channel);
		if (ControlStatusReg_val[i-1] & 0x0F)
		{
			tx[i] = (cs_channel) | (CamOrder_val[i-1] << 8);
		}
		else
		{
			tx[i] = (cs_channel) | (0x0 << 8);
		}

		/* enable or disable the stream */
		if (ControlStatusReg_val[i-1] & 0x0F){
			 StatusEnable(pCam->camera_id -1 , MODULE_STREAM_ON);
			 set_cam_csi_channel(pCam->camera_id, cs_channel,1);
			 log_printf("%s stream on successful\n\r", pCam->Name );
		}
		else
		{
			 StatusDisable(pCam->camera_id - 1, MODULE_STREAM_ON);
			 set_cam_csi_channel(pCam->camera_id, cs_channel,0);
			 log_printf("%s stream off successful\n\r", pCam->Name );
		}
	}
	tmp = fpga_send_command(tx, rx, len, IRQ_FROM_FPGA_TIMEOUT);
	if(tmp == FPGA_CMD_TOUT)
	{
		log_printf("PREVIEW failed: no response from the FPGA!\n\r");
		ret = 3;
		goto EXIT;
	}
	ret = 1;
EXIT:
	EXIT_FUNC;
	return ret;
}


UInt8 PreviewSimple(CamID ChannelID, UInt8 Intr)
{
	ENTER_FUNC;
	UInt8 ret = 0;
	UInt16 tx = 0;
	UInt16 rx = 0;
	fpga_cmd_error_t tmp = 0;
	if(ChannelID < 1 || ChannelID > 23)
	{
		log_error("Invalid Channel ID !!");
		ret = 2;
		goto EXIT;
	}
	if(Intr != 0 && Intr != 1)
	{
		log_error("Invalid Interrupt status !!");
		ret = 2;
		goto EXIT;
	}
	log_printf("Send PREVIEW_SIMPLE cmd to FPGA, channel %d \n\r", ChannelID);
	tx = (ChannelID << 8) | (Intr << 7);
	tx &= 0xff80;
	tx |= 0x34;
	tmp = fpga_send_command(&tx, &rx, 1,IRQ_FROM_FPGA_TIMEOUT);
	if(tmp == FPGA_CMD_TOUT)
	{
		log_printf("PREVIEW_SIMPLE failed: no response from the FPGA!\n\r");
		ret = 3;
		goto EXIT;
	}
	ret = 1;
EXIT:
	EXIT_FUNC;
	return ret;
}

void ReadTemperature(void)
{
	UInt8 temp_slave_addr[TEMPERATURE_MAX_NUM] = {0x48, 0x49, 0x4A, 0x4B};
	Int16 temp100, temp10, temp1 = 0;
	log_printf("----Temperatures\n\r");
	for (int i = 0; i < TEMPERATURE_MAX_NUM; i++)
	{
		temp100 = (Int16)Get_TempBoad(temp_slave_addr[i]);
		temp100 = temp100*625/1000;
		temp10 = temp100/10;
		temp1 = temp100%10;
		log_printf("Temperature[%d]: %d.%d\n\r", i+1, temp10, temp1);
	}
}

Bool fpga_write_i2c_value(UInt8 chanid, UInt8 slave_addr, UInt16 reg_addr, UInt16 reg_data, I2CMESSAGETYPE mode, Bool interupt)
{
	UInt8 len = 0; /* Header 2 bytes */
	UInt16 tx_buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	UInt16 rx_buf[8];
	tx_buf[0] = chanid << 8;
	tx_buf[0] |= (interupt ? 1 : 0 ) << 7;
	tx_buf[1] = slave_addr<<8;
	// tx_buf[1] &= 0xFF80;
	tx_buf[1] &= 0xFF00;  // johns

	switch(mode)
	{
		case ADDR8_DATA8:
			tx_buf[2]  = reg_addr & 0x00ff;
			tx_buf[2] |= reg_data << 8;
			len = 3;
			break;
		case ADDR16_DATA8:
			tx_buf[1] |= 1;
			tx_buf[2]  = reg_addr;
			tx_buf[3]  = reg_data & 0x0ff;
			len = 4;
			break;
		case ADDR8_DATA16:
			tx_buf[1] |= 2;
			tx_buf[2]  = (reg_addr & 0x00ff);
			tx_buf[2] |= (reg_data & 0x00ff) << 8;
			tx_buf[3]  = (reg_data & 0xff00) >> 8;
			len = 4;
			break;
		case ADDR16_DATA16:
			tx_buf[1] |= 3;
			tx_buf[2]  = reg_addr;
			tx_buf[3]  = reg_data;
			len = 4;
			break;
		default:
			break;
	}
	/* Word alignment */
	/* TODO: sending SPI and checking interrupt */
	UInt8 ret = fpga_send_command(tx_buf, rx_buf, len, IRQ_FROM_FPGA_TIMEOUT);
	//vTaskDelay(20 / portTICK_RATE_MS); // Wait for 150ms.

	if (ret == FPGA_CMD_SUCCESS)
		return ETrue;
	else
		return EFalse;
}

UInt16 fpga_read_i2c_value(UInt8 chanid, UInt8 slave_addr, UInt16 reg_addr, I2CMESSAGETYPE mode)
{
	UInt16 retVal = 0;
	fpga_cmd_error_t tmp = 0;
	UInt8 len = 0; /* Header 2 bytes */
	UInt16 tx_buf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	UInt16 rx_buf[8];
	tx_buf[0] = chanid << 8;
	tx_buf[0] |= 0x81;
	tx_buf[1] = (slave_addr | 0x01)<<8;
	tx_buf[1] &= 0xFF00;

	switch(mode)
	{
		case ADDR8_DATA8:
			tx_buf[2] = reg_addr & 0x0ff;
			len = 3;
			break;
		case ADDR16_DATA8:
			tx_buf[1] |= 1;
			tx_buf[2] = reg_addr;
			len = 3;
			break;
		case ADDR8_DATA16:
			tx_buf[1] |= 2;
			tx_buf[2] = reg_addr & 0x0ff;
			len = 3;
			break;
		case ADDR16_DATA16:
			tx_buf[1] |= 3;
			tx_buf[2] = reg_addr;
			len = 3;
			break;
		default:
			break;
	}

	tmp = fpga_send_command(tx_buf, rx_buf, len, IRQ_FROM_FPGA_TIMEOUT);
	//log_printf("tmp %d \n\r", tmp);
	if(tmp == FPGA_CMD_TOUT)
	{
		log_printf("No irq signal from FPGA \n\r");
	}
	//log_printf("Return value from FPGA: 0x%x\n\r", rx_buf[0]);

	// Place a known pattern in the receive buffer to confirm
	// which data values were actually received.
	tx_buf[0] = 0xFFFF;
	tx_buf[1] = 0xFFFF;
	tx_buf[2] = 0xFFFF;//send 2 bytes dummy. Temporary is 0xFFFF.

	tmp = fpga_send_command(tx_buf, rx_buf, 1, IRQ_FROM_FPGA_TIMEOUT); // Receive data from FPGA
	//log_printf("tmp %d \n\r", tmp);
	if(tmp == FPGA_CMD_TOUT)
	{
		log_error("No interrupt from the FPGA!\n\r");
	}
	//log_printf("Return value from FPGA: 0x%x\n\r", rx_buf[0]);

	// Prepare the return data.
	retVal = rx_buf[0];

	// Wait for 150ms to avoid overflowing the FPGA's SPI input queue.
	// Skip this for reads, as there is already sync due to the interrupt.
	// vTaskDelay(150 / portTICK_RATE_MS);

	return retVal;
}

STATIC UInt8 SensorsSetting(unsigned int cam_bitmask, cam_reg_array *regs, UInt16 size)
{
	int TransactionCount = 0;

	//ENTER_FUNC;

	UInt16 i = 0, j = 0;
	cam_reg_array *pReg = regs;

	if (cam_bitmask & 0x01)
		cam_bitmask = 0x01FFFE;
	/* The number of register array */
	for (i = 0; i < size; i++ )
	{
		/* Size of each register array */
		for (j = 0; j < pReg->reg_size; j++)
		{
			TransactionCount++;
			if ((TransactionCount % 10) == 0)
				log_debug("%d transactions", TransactionCount);
			//log_debug("%d transactions", TransactionCount);
			if (pReg->data_size == 1)
			{
				Config_I2C_message( cam_bitmask, pReg->regs[j].reg_addr, pReg->regs[j].reg_val, 2);
			}
			else
			{
				Config_I2C_message( cam_bitmask, pReg->regs[j].reg_addr, pReg->regs[j].reg_val, 3);
			}
		}
		pReg++;
	}

	//EXIT_FUNC;
	return 1;
}

UInt8 Config_I2C_message(UInt32 I2C_Channel_Mask, UInt16 reg_addr, UInt16 reg_data, I2CMESSAGETYPE mode)
{
	//ENTER_FUNC;
	//message->Address = 0x6C;

	UInt16 tx[8];
	UInt16 rx[8];

	tx[0] = (0 << 8) | 0x82;
	tx[1] = (UInt16)(I2C_Channel_Mask & 0xFFFE);
	tx[2] = (UInt16)((I2C_Channel_Mask >> 16) & 0x00FF);
	tx[3] = ((UInt8)mode << 8) | 0x6C;
	tx[4] = (reg_addr & 0xFF00) >>	8 | (reg_addr & 0x00FF) << 8;
	if(mode == 3)
		tx[5] = (reg_data & 0xFF00) >>	8 | (reg_data & 0x00FF) << 8;
	else
		tx[5] = reg_data & 0x00FF;

/*	log_debug("===============tx[0] = %x ", tx[0]);
	log_debug("===============tx[1] = %x ", tx[1]);
	log_debug("===============tx[2] = %x ", tx[2]);
	log_debug("===============tx[3] = %x ", tx[3]);
	log_debug("===============tx[4] = %x ", tx[4]);
	log_debug("===============tx[5] = %x ", tx[5]);*/

	//LOAD_I2C_BUFFER
	UInt8 ret = fpga_send_command(tx, rx, 6, IRQ_FROM_FPGA_TIMEOUT);

	tx[0] = (0 << 8) | 0x83;

	//XMIT_I2C_BUFFER
	if (ret == FPGA_CMD_SUCCESS)
		ret = fpga_send_command(tx, rx, 3, IRQ_FROM_FPGA_TIMEOUT);

	//EXIT_FUNC;
	return ret;
}

/*trigger: 1-> Send trigger
		  0-> Stop Trigger
*/
UInt8 Trigger(UInt32 I2C_Channel_Mask, UInt8 trigger)
{
	ENTER_FUNC;

	UInt16 tx[8];
	UInt16 rx[8];

	tx[0] = (trigger == 1) ? 0x00E2 : 0x00E4;
	tx[1] = (UInt16)(I2C_Channel_Mask & 0xFFFE);
	tx[2] = (UInt16)((I2C_Channel_Mask >> 16) & 0x0001);

	// LOAD_I2C_BUFFER
	UInt8 ret = fpga_send_command(tx, rx, 3, IRQ_FROM_FPGA_TIMEOUT);

	EXIT_FUNC;
	return ret;
}

UInt8 Preview_Off(uint32_t i2c_channel_mask)
{
	uint8_t ret = FPGA_CMD_SUCCESS;
	CamDevice_TypeDef *pCam = NULL;
	uint8_t ControlStatusReg_off[2];
	uint8_t CamOrder_off[2];
	int i = 0, cam_off_count = 0;

	for (i = 1 ; i <= CAM_NUM_CAMERA_MAX; i++)
	{
		if (i2c_channel_mask & (0x1 << i))
		{
			pCam = CamDeviceTbl[i -1];

			/* for now we support only 2 active cam */
			if (StatusActivated(pCam->camera_id - 1, MODULE_STREAM_ON) && (cam_off_count < 2) )
			{
				CamOrder_off[cam_off_count] = pCam->camera_id;
				ControlStatusReg_off[cam_off_count] =  (get_cam_csi_channel(pCam->camera_id) << 0x4) | 0x00;
				cam_off_count +=1;
				log_debug("Turning off camera %s \n", pCam->Name);
			}

			StatusDisable(pCam->camera_id - 1, MODULE_STREAM_ON);
			StatusDisable(pCam->camera_id - 1, CSI_CAM_CHANNEL_0);
			StatusDisable(pCam->camera_id - 1, CSI_CAM_CHANNEL_1);
		}
	}

	if (cam_off_count > 0)
		ret = Preview(INTERRUPT_ENABLE, cam_off_count,  CamOrder_off,
				ControlStatusReg_off, NULL, NULL);

	return ret;
}
static int cam_stream_off(int cam_idx)
{
	CamDevice_TypeDef *pCam = CamDeviceTbl[cam_idx -1];
	int stream_on = 0;
	uint8_t ControlStatusReg_off[2];
	uint8_t CamOrder_off[2];
	int cam_off_count = 0, index;
	CamDevice_TypeDef *second_cam = NULL;
	int ret = 0;

	log_debug("%s is being turned off \r\n", pCam->Name);

	if(!(StatusActivated(pCam->camera_id - 1 , MODULE_STREAM_ON)))
	{
		log_warning("%s is already streamed off \r\n", pCam->Name);
		return ret;
	}
	/* now turn off stream for at least one camera
	 */
	CamOrder_off[cam_off_count] = pCam->camera_id;
	// 0xf0 mask for CSI mask and 0x0F enable/disable
	ControlStatusReg_off[cam_off_count] =  (get_cam_csi_channel(pCam->camera_id) << 0x4) | 0x00;
	cam_off_count +=1;
	/* check if there is any other camera streaming else turn off */
	for ( index = 0; index < CAM_NUM_CAMERA_MAX; index++)
	{
		if( (pCam->camera_id - 1) != index)
			stream_on = StatusActivated(index, MODULE_STREAM_ON);

		/* for now we support only 2 active cam */
		if (stream_on && cam_off_count < 2 )
		{
			second_cam = CamDeviceTbl[index];
			CamOrder_off[cam_off_count] = second_cam->camera_id;
			ControlStatusReg_off[cam_off_count] =  (get_cam_csi_channel(second_cam->camera_id) << 0x4) | 0x01;
			cam_off_count +=1;
			log_debug("While turning off found cam %s ON\n", second_cam->Name);
		}
	}
	/* send the preview command that truns off the camera */
	log_debug("turning preview off %s\n", pCam->Name);
	ret = Preview(INTERRUPT_ENABLE, cam_off_count,  CamOrder_off, ControlStatusReg_off,
			NULL, NULL);

	return ret;

}

#ifdef BOARD_VERSION_P1
//TODO: Have arguments for pulse width,pulse interval & pulse count.
UInt8 Config_Trigger(UInt16 fll, UInt32 num_pulses)
{
	UInt16 tx[16];
	UInt16 rx[16];
	UInt32 pulse_interval = 0;
	UInt32 I2C_Channel_Mask = 0;

	/*Configure Trigger command doesn't take I2C channel mask into effect
	 So hard coding global value for it */
	I2C_Channel_Mask = 0x00FEFF01;

	ENTER_FUNC;

	pulse_interval = calculate_pulse_interval(fll);

	tx[0] = (0 << 8) | 0xE0;
	tx[1] = (UInt16)(I2C_Channel_Mask & 0xFFFF);
	tx[2] = (UInt16)((I2C_Channel_Mask >> 16) & 0x00FF);
	tx[3] = 0x0001;
	tx[4] = 0x86A0;
	tx[5] = ( pulse_interval >> 16 ) & 0xFFFF;
	tx[6] = (UInt16)(pulse_interval & 0xFFFF);
	tx[7] = ( num_pulses >> 16 ) & 0xFFFF;
	tx[8] = (UInt16)(num_pulses & 0xFFFF);

	UInt8 ret = fpga_send_command(tx, rx, 9, IRQ_FROM_FPGA_TIMEOUT);

	EXIT_FUNC;
	return ret;
}
#endif
#ifdef BOARD_VERSION_P1_1
//TODO: Have arguments for pulse width,pulse interval & pulse count.
UInt8 Config_Trigger(CamDevice_TypeDef *cam, UInt16 fll, UInt32 num_pulses)
{
	UInt16 tx[16];
	UInt16 rx[16];
	UInt32 pulse_interval = 0;
	UInt32 I2C_Channel_Mask = 0;

	/*Configure Trigger command doesn't take I2C channel mask into effect
	 So hard coding global value for it */
	I2C_Channel_Mask = 0x00FEFF01;

	ENTER_FUNC;

	pulse_interval = calculate_pulse_interval(cam, fll);

	tx[0] = (0 << 8) | 0xE0;
	tx[1] = (UInt16)(I2C_Channel_Mask & 0xFFFF);
	tx[2] = (UInt16)((I2C_Channel_Mask >> 16) & 0x00FF);
	tx[3] = 0x0001;
	tx[4] = 0x86A0;
	tx[5] = ( pulse_interval >> 16 ) & 0xFFFF;
	tx[6] = (UInt16)(pulse_interval & 0xFFFF);
	tx[7] = ( num_pulses >> 16 ) & 0xFFFF;
	tx[8] = (UInt16)(num_pulses & 0xFFFF);

	UInt8 ret = fpga_send_command(tx, rx, 9, IRQ_FROM_FPGA_TIMEOUT);

	EXIT_FUNC;
	return ret;
}
#endif


#ifdef BOARD_VERSION_P1
double calculate_fll_msec(UInt16 fll_value)
{
	double fll_nsec = 0;
	//printf("before converting pulse interval, fll = %d\n", fll_value);
	fll_nsec = ((((double)((double)( fll_value * 3800 ) / 158400000)) * 1000 * 1000 * 1000 ) + 1000 - 1000000);
	log_debug("\n Calculated fll_msec in nsec float %f\n", fll_nsec);
	return (double) (fll_nsec / ((double)(1000 * 1000)));
}

UInt32 calculate_pulse_interval(UInt16 fll_value)
{
	double pulse_interval = 0;
	//printf("before converting pulse interval, fll = %d\n", fll_value);
	pulse_interval = ((((double)((double)( fll_value * 3800 ) / 158400000)) * 1000 * 1000 * 1000 ) + 1000 - 1000000)/10;
	//printf("\n Calculated pulse_interval in float %f\n", pulse_interval);
	return pulse_interval;
}
#endif

#ifdef BOARD_VERSION_P1_1
double calculate_fll_msec(CamDevice_TypeDef *cam, UInt16 fll_value)
{
    double fll_nsec = 0;
    //printf("before converting pulse interval, fll = %d\n", fll_value);

    fll_nsec = ((((double)((double)( fll_value * cam->line_length_pclk ) / VT_PIX_CLK)) * 1000 * 1000 * 1000 ) + 1000 - 1000000);

    log_debug("\n Calculated fll_msec in nsec float %f\n", fll_nsec);
    return (double) (fll_nsec / ((double)(1000 * 1000)));
}

UInt32 calculate_pulse_interval(CamDevice_TypeDef *cam, UInt16 fll_value)
{
    double pulse_interval = 0;
    //printf("before converting pulse interval, fll = %d\n", fll_value);
    log_printf("fll_value = 0x%x\n", fll_value);
    log_printf("line_length_pclk = 0x%x\n", cam->line_length_pclk);
    log_printf("VT_PIX_CLK = 0x%x\n", VT_PIX_CLK);
    pulse_interval = ((((double)((double)( fll_value * cam->line_length_pclk ) / VT_PIX_CLK)) * 1000 * 1000 * 1000 ) + 1000 - 1000000)/10;
    //printf("\n Calculated pulse_interval in float %f\n", pulse_interval);
    return pulse_interval;
}
#endif

#ifdef BOARD_VERSION_P1
UInt8 Configure_FLL( UInt16 *fll, UInt32 i2c_channel_mask, CamID cam_id )
{
	UInt8 status = 0;
	UInt16 fll_A = *fll;
	UInt16 fll_B = *fll;
	UInt16 fll_C = *fll;

	if (i2c_channel_mask == GROUP_AB)
	{
		// check that the FLL value does not fall below the pulse width interval
		if (*fll < 0x139E)
			*fll = 0x139E;
		fll_B = *fll;
		fll_A = *fll/2;

		status = Config_I2C_message( GROUP_A, 0x340, fll_A, 3);
		status = Config_I2C_message( GROUP_B, 0x340, fll_B, 3);
		log_printf("fll_A %x fll_B %x\n\r", fll_A, fll_B);
	}
	else if (i2c_channel_mask == GROUP_BC)
	{
		// check that the FLL value does not fall below the pulse width interval
		if (*fll < 0x139E)
			*fll = 0x139E;
		fll_C = *fll;
		fll_B = *fll/2;

		status = Config_I2C_message( GROUP_B, 0x340, fll_B, 3);
		status = Config_I2C_message( GROUP_C, 0x340, fll_C, 3);
		log_printf("fll_B %x fll_C %x\n\r", fll_B, fll_C);
	}
	else if (i2c_channel_mask == GROUP_C)
	{
		if (*fll < (0x139E/2))
			*fll = (0x139E/2);
		fll_C = *fll;
		status = Config_I2C_message( GROUP_C, 0x340, fll_C, 3);
		log_printf("fll_C %x\n\r", fll_C);
	}
	else
	{
		return 1;
	}

	return status;
}
/*Configure Line Length pclk for all A cameras*/
/*Instead of hard coding (2 * 3800) .Read reg 0x0342
 * of the camera whose llpclk doesn't change & multiple by 2*/
UInt8 Configure_LLPCLK(UInt32 i2c_channel_mask)
{
	UInt8 status = 0;
	UInt16 llpclk = 0x0ED8;

	// Remask i2c channel mask to camera effected
	switch (i2c_channel_mask)
	{
		case GROUP_AB:
			i2c_channel_mask = GROUP_A;
			llpclk = llpclk*2;
			break;
		case GROUP_BC:
			i2c_channel_mask = GROUP_B;
			llpclk = llpclk*2;
			break;
		case GROUP_C:
			i2c_channel_mask = GROUP_C;
			break;
		default:
			break;
	}
	status = Config_I2C_message( i2c_channel_mask, 0x342, llpclk, 3);
	return status;
}

/*Half CIT for Camera group A if AB capture is requested*/
/*Half CIT for Camera group B if BC capture is requested*/
UInt8 Configure_CIT(UInt32 i2c_channel_mask)
{
	UInt8 status = 0;
	UInt8 i = 0;
	UInt16 cit_value = 0, rate = 1;
	CamDevice_TypeDef *pCam = CamDeviceTbl[0];

	// Remask i2c channel mask to camera effected
	switch (i2c_channel_mask)
	{
		case GROUP_AB:
			i2c_channel_mask = GROUP_A;
			rate = 2;
			break;
		case GROUP_BC:
			i2c_channel_mask = GROUP_B;
			rate = 2;
			break;
		case GROUP_C:
			i2c_channel_mask = GROUP_C;
			rate = 1;
			break;
		default:
			break;
	}

	for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
	{
		pCam = CamDeviceTbl[i];
		if (((i2c_channel_mask >> 1) & 1<<i) != 0)
		{
			cit_value = fpga_read_i2c_value(pCam->camera_id, 0x6c, 0x0202, ADDR16_DATA16);
			status = fpga_write_i2c_value(pCam->camera_id, 0x6c, 0x0202, cit_value/rate, ADDR16_DATA16, ETrue);
		}
	}
	return status;
}
#endif

#ifdef BOARD_VERSION_P1_1
UInt8 Configure_FLL( UInt32 cam_group, UInt16 *fll, UInt32 i2c_channel_mask, CamID cam_id )
{
	ENTER_FUNC;

	UInt8 status = 0;
	UInt16 fll_A = *fll;
	UInt16 fll_B = *fll;
	UInt16 fll_C = *fll;
	log_debug("i2c_channel_mask 0x%x\n",(unsigned int)i2c_channel_mask);
	if (cam_group == GROUP_AB) //Assumes it's AB group
	{
		log_debug("GROUP_AB\n");
		// check that the FLL value does not fall below the pulse width interval
		if (*fll < (0xDFC * 2))
			*fll = (0xDFC * 2);
		fll_B = *fll;
		fll_A = *fll/2;

		status = Config_I2C_message( i2c_channel_mask & GROUP_A, 0x340, fll_A, 3);
		status = Config_I2C_message( i2c_channel_mask & GROUP_B, 0x340, fll_B, 3);
		log_printf("fll_A %x fll_B %x\n\r", fll_A, fll_B);
	}
	else if (cam_group == GROUP_BC)// Assumes it's BC group
	{
		log_debug("GROUP_BC\n");
		// check that the FLL value does not fall below the pulse width interval
		if (*fll < (0xDFC * 2))
			*fll = (0xDFC * 2);
		fll_C = *fll;
		fll_B = *fll/2;

		status = Config_I2C_message( i2c_channel_mask & GROUP_B, 0x340, fll_B, 3);
		status = Config_I2C_message( i2c_channel_mask & GROUP_C, 0x340, fll_C, 3);
		log_printf("fll_B %x fll_C %x\n\r", fll_B, fll_C);
	}
	else if (cam_group == GROUP_C)
	{
		log_debug("GROUP_C\n");
		if (*fll < (0xDFC))
			*fll = (0xDFC);
		fll_C = *fll;
		status = Config_I2C_message( i2c_channel_mask & GROUP_C, 0x340, fll_C, 3);
		log_printf("fll_C %x\n\r", fll_C);
	}


	EXIT_FUNC;
	return status;
}
/*Configure Line Length pclk for all A cameras*/
/*Instead of hard coding (2 * 3800) .Read reg 0x0342
 * of the camera whose llpclk doesn't change & multiple by 2*/
UInt8 Configure_LLPCLK(UInt32 camera_group,UInt32 i2c_channel_mask)
{
	ENTER_FUNC;
	UInt8 status = 0;
	CamDevice_TypeDef *pCam = CamDeviceTbl[0];

	//UInt16 llpclk = 0x0ED8;
	/* Hard coding this for now. In the future, we will read
	   this value from a C camera's 0x0342 register, as it
	   does not change. */

	UInt16 double_llpclk = 0;
	/* In the AR1335, 0x0342 contains DOUBLE the llpclk value,
	unlike AR0835. */


	// Remask i2c channel mask to camera effected
	switch (camera_group)
	{
		case GROUP_AB:
			log_debug("GROUP_AB\n");
			i2c_channel_mask &= GROUP_A;
			/* Read value from register 0x0342 of camera A1 */
			pCam = CamDeviceTbl[0];
			double_llpclk = fpga_read_i2c_value(pCam->camera_id, 0x6c, 0x0342, ADDR16_DATA16);
			double_llpclk = double_llpclk * 2;
			log_printf("double_llpclk in  group AB = 0x%x\n", double_llpclk);
			break;
		case GROUP_BC:
			log_debug("GROUP_BC\n");
			i2c_channel_mask &= GROUP_B;
			/* Read value from register 0x0342 of camera B1 */
			pCam = CamDeviceTbl[5];
			double_llpclk = fpga_read_i2c_value(pCam->camera_id, 0x6c, 0x0342, ADDR16_DATA16);
			double_llpclk = double_llpclk * 2;
			break;
		case GROUP_C:
			log_debug("GROUP_C\n");
			/* Read value from register 0x0342 of camera C1 */
			pCam = CamDeviceTbl[10];
			double_llpclk = fpga_read_i2c_value(pCam->camera_id, 0x6c, 0x0342, ADDR16_DATA16);
			i2c_channel_mask &= GROUP_C;
			break;
		default:
			break;
	}

	// Temporarily hard code the value of double_llpclk, since
	// we don't have AB group yet
	// double_llpclk = 2 * double_llpclk;

	log_printf("double_llpclk = 0x%x\n", double_llpclk);
	status = Config_I2C_message( i2c_channel_mask, 0x342, double_llpclk, 3);
	EXIT_FUNC;
	return status;
}

/*Half CIT for Camera group A if AB capture is requested*/
/*Half CIT for Camera group B if BC capture is requested*/
UInt8 Configure_CIT(UInt32 cam_group, UInt32 i2c_channel_mask)
{
	UInt8 status = 0;
	UInt8 i = 0;
	UInt16 cit_value = 0, rate = 1;
	CamDevice_TypeDef *pCam = CamDeviceTbl[0];

	// Remask i2c channel mask to camera effected
	switch (cam_group)
	{
		case GROUP_AB:
			log_debug("GROUP_AB\n");
			i2c_channel_mask &= GROUP_A;
			rate = 2;
			break;
		case GROUP_BC:
			log_debug("GROUP_BC\n");
			i2c_channel_mask &= GROUP_B;
			rate = 2;
			break;
		case GROUP_C:
			log_debug("GROUP_C\n");
			i2c_channel_mask &= GROUP_C;
			rate = 1;
			break;
		default:
			break;
	}

	for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
	{
		pCam = CamDeviceTbl[i];
		if (((i2c_channel_mask >> 1) & 1<<i) != 0)
		{
			cit_value = fpga_read_i2c_value(pCam->camera_id, 0x6c, 0x0202, ADDR16_DATA16);
			status = fpga_write_i2c_value(pCam->camera_id, 0x6c, 0x0202, cit_value/rate, ADDR16_DATA16, ETrue);
		}
	}
	return status;
}
#endif

CamID find_max_exposure(UInt32 I2C_Channel_Mask, UInt64 *exposure)
{
   CamID cam_id = CAM_ID_A1;
   UInt8 i = 0;
   UInt64 exposure_tbl[CAM_NUM_CAMERA_MAX];
   *exposure = 0;

   for (i = 0; i < CAM_NUM_CAMERA_MAX; i++)
   {
	   if (I2C_Channel_Mask & (1 << (i + 1)))
	   {
		   if (get_last_setting(i, UC_PREVIEW, EXPOSURE_TIME, &exposure_tbl[i]) == 0)
			   exposure_tbl[i] = CAM_EXPOSURE_DEFAULT;
		   if (*exposure < exposure_tbl[i])
		   {
			   *exposure = exposure_tbl[i];
			   cam_id = i + 1;
		   }
	   }
   }
   log_debug("Max value = 0x%x%x, CamID : %d", (unsigned int)(*exposure>>32), (unsigned int)*exposure, cam_id);

   return cam_id;
}

UInt8 ConfigureFrameToCapture(UInt8 frame_index)
{
	UInt16 tx[16];
	UInt16 rx[16];

	ENTER_FUNC;

	tx[0] = 0x00F3;
	tx[1] = 0x00FF & frame_index;

	UInt8 ret = fpga_send_command(tx, rx, 2, IRQ_FROM_FPGA_TIMEOUT);

	EXIT_FUNC;
	return ret;
}

/*Program Trigger Offsets for B cameras */
/*Command F0 for A*/
/*Command F1 for B*/
/*Command F2 for C*/
UInt8 Configure_offsets_trigger(UInt32 i2c_channel_mask)
{
	UInt16 tx[16];
	UInt16 rx[16];
	UInt8 size = 0;

	ENTER_FUNC;

	if (i2c_channel_mask == GROUP_C)
	{
		log_debug("Found group_c capture in configure_offsets_trigger\n");
		return 1;
	}

	tx[0] = (i2c_channel_mask == GROUP_AB) ? 0x00F1 : 0x00F2;
	tx[1] = 0x0001;
	tx[2] = 0x9C54;
	tx[3] = 0x0001;
	tx[4] = 0x9C54;
	tx[5] = 0x005B;
	tx[6] = 0x38AB;
	tx[7] = 0x002E;
	tx[8] = 0x6A7F;
	tx[9] = 0x005B;
	tx[10] = 0x38AB;

	size = 11;

	if (i2c_channel_mask == GROUP_BC)
	{
		tx[11] = 0x0000;
		tx[12] = 0x0000;
		size = 13;
	}

	UInt8 ret = fpga_send_command(tx, rx, size, IRQ_FROM_FPGA_TIMEOUT);

	EXIT_FUNC;
	return ret;
}

/*Program Trigger Offsets for B cameras */
/*Command F0 for A*/
/*Command F1 for B*/
/*Command F2 for C*/
/*
 * adb shell "echo 27 0x0000 0x4E 0x00 0x00 0x01 0x16
 * 0xF1 0x00 0x00 0x00 0x7F 0xAD 0x00 0x00 0x7F 0xAD 0x97 0x00 0x57 0xB6 0x4C 0x00 0xEB 0x31 0x97 0x00 0x57 0xB6   > /sys/class/light_ccb/i2c_interface/i2c_w"
 */
UInt8 Configure_offsets_trigger_p1_1(UInt32 i2c_channel_mask)
{
	UInt16 tx[16];
	UInt16 rx[16];
	UInt8 size = 0;

	ENTER_FUNC;

	if (i2c_channel_mask == GROUP_C)
	{
		log_debug("Found group_c capture in configure_offsets_trigger\n");
		return 1;
	}

	tx[0] = (i2c_channel_mask == GROUP_AB) ? 0x00F1 : 0x00F2;
	tx[1] = 0x0000;//0x00 0x00
	tx[2] = 0xAD7F;//0x7F 0xAD
	tx[3] = 0x0000;//0x00 0x00
	tx[4] = 0xAD7F;//0x7F 0xAD
	tx[5] = 0x0097;//0x97 0x00
	tx[6] = 0xB657;//0x57 0xB6
	tx[7] = 0x004C;//0x4C 0x00
	tx[8] = 0x31EB;//0xEB 0x31
	tx[9] = 0x0097;//0x97 0x00
	tx[10] = 0xB657;//0x57 0xB6

	size = 11;

	if (i2c_channel_mask == GROUP_BC)
	{
		tx[11] = 0x0097;
		tx[12] = 0xB657;
		size = 13;
	}

	UInt8 ret = fpga_send_command(tx, rx, size, IRQ_FROM_FPGA_TIMEOUT);

	EXIT_FUNC;
	return ret;
}

void CPLD_SPI_Transfer(uint8_t* tx_buffer, uint8_t* rx_buffer,
					   uint16_t num_bytes)
{
	SS_EN;
	for (int i = 0; i < num_bytes; i++)
	{
		CPLD->DR = tx_buffer[i];				   // write data to be transmitted to the SPI data register
		while ( !(CPLD->SR & SPI_I2S_FLAG_TXE) );  // wait until transmit complete
		while ( !(CPLD->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
		while ( CPLD->SR & SPI_I2S_FLAG_BSY );	   // wait until SPI is not busy anymore
		rx_buffer[i] = CPLD->DR;				   // return received data from SPI data register
	}
	SS_DIS;
}

void configure_CPLD(uint8_t control_type, uint8_t direction, CPLD_CONFIG* cpld_config, uint32_t multiplier)
{
	//log_printf("Configuring CPLD...\n\n");
	uint8_t tx_buffer[10], rx_buffer[10]; // reduced the size
	uint8_t byte_count;
	CPLD_CONFIG* new_cpld_config = NULL;

	// write a 1 to the debug_test register to switch CPLD modes
	tx_buffer[0] = 0x02;
	tx_buffer[1] = 0x06;
	tx_buffer[2] = 0x01;
	uint8_t rep_byte[2] = {0x1, 0x0};
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);

	/*
	registers to write:

	0x06        : debug_test, always 1
	0x0a - 0x15 : duty cycles, 2 bytes each, 12 bytes total
	0x20 - 0x37 : repetition counts, 4 bytes each, 24 bytes total
	0x50 - 0x57 : group repetition counts, 4 bytes each, 8 bytes total
	0x70 - 0x73 : overall repetition count, 4 bytes total
	0x80 - 0x81 : frequency, 2 bytes total

	50 bytes total, not including 0x06
	*/
	if (cpld_config != NULL)
	{
		new_cpld_config = cpld_config;
	}
	else
	{
		new_cpld_config = set_config_buffer(control_type, direction);
		if (multiplier)
		{
			rep_byte[0] = new_cpld_config->rep_ovr_cnt1.byte[0];
			rep_byte[1] = new_cpld_config->rep_ovr_cnt1.byte[1];
			new_cpld_config->rep_ovr_cnt1.byte[0] = (uint8_t) (multiplier & 0xFF);
			new_cpld_config->rep_ovr_cnt1.byte[1] = (uint8_t) ((multiplier >> 8) & 0xFF);
		}

		if (!new_cpld_config)
		{
			log_error(" No CPLD configuration found, using old settings\n");
			return ;
		}
	}

	// Remember that the CPLD can only receive up to 10 bytes
	// at a time - this means that with the byte for R/W
	// and address, you can only send 8 bytes of data at a time

	// duty cycle
	log_fpga("Duty cycles 1-3:\n");
	tx_buffer[1] = 0x0a;
	memcpy(&tx_buffer[2], &new_cpld_config->duty_cycle1, 6);
	byte_count = 6 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	log_fpga("Duty cycles 4-6:\n");
	tx_buffer[1] = 0x10;
	memcpy(&tx_buffer[2], &new_cpld_config->duty_cycle4, 6);
	byte_count = 6 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	// repetition count
	log_fpga("Repetition counts 1-2:\n");
	tx_buffer[1] = 0x20;
	memcpy(&tx_buffer[2], &new_cpld_config->rep_cnt1, 8);
	byte_count = 8 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	log_fpga("Repetition counts 3-4:\n");
	tx_buffer[1] = 0x28;
	memcpy(&tx_buffer[2], &new_cpld_config->rep_cnt3, 8);
	byte_count = 8 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	log_fpga("Repetition counts 5-6:\n");
	tx_buffer[1] = 0x30;
	memcpy(&tx_buffer[2], &new_cpld_config->rep_cnt5, 8);
	byte_count = 8 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	// group repetition count
	log_fpga("Group repetition count:\n");
	tx_buffer[1] = 0x50;
	memcpy(&tx_buffer[2], &new_cpld_config->rep_group_cnt1, 8);
	byte_count = 8 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	// overall repetition count
	log_fpga("Overall repetition count:\n");
	tx_buffer[1] = 0x70;
	memcpy(&tx_buffer[2], &new_cpld_config->rep_ovr_cnt1, 4);
	byte_count = 4 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	// frequency
	log_fpga("Frequency:\n");
	tx_buffer[1] = 0x80;
	memcpy(&tx_buffer[2], &new_cpld_config->freq, 2);
	byte_count = 2 + 2;
	for (int i = 0; i < byte_count; i++)
		log_fpga("tx_buffer[%d] = 0x%02x\n", i, tx_buffer[i]);
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, byte_count);

	if (multiplier)
	{
		new_cpld_config->rep_ovr_cnt1.byte[0] = rep_byte[0];
		new_cpld_config->rep_ovr_cnt1.byte[1] = rep_byte[1];
	}
}


CPLD_CONFIG* set_config_buffer(uint8_t control_type, uint8_t direction)
{
	// this function fills cpld_config_buffer with the appropriate
	// values, which are then written by configure_CPLD()

	/*
		Copying is an O(n) operation. If I was going about this
		in a more intelligent way, I would simply change the buffer
		that gets sent to configure_CPLD, and not waste 50 copy operations
		modifying that one buffer. Alas I'd rather not break the working
		structure for now.
	 */
	CPLD_CONFIG* cpld_config = NULL;
	switch (control_type)
	{
		case USER_DEFINED_CONTROL:
			log_printf("Using user defined controls.\n");
			break;
#ifdef BOARD_VERSION_P1
		case COARSE_CONTROL:
			log_debug("Using coarse control\n");
			if (direction == RETRACT)
			{
				cpld_config = &coarse_retract_values;
			}
			else if (direction == EXTEND)
			{
				cpld_config = &coarse_extend_values;
			}
			else
				log_printf("Invalid direction given to set_config_buffer.\n");
			break;

		case FINE_CONTROL:
			log_debug("Using fine control\n");
			if (direction == RETRACT)
			{
				cpld_config = &fine_retract_values;
			}
			else if (direction == EXTEND)
			{
				cpld_config = &fine_extend_values;
			}
			else
				log_printf("Invalid direction given to set_config_buffer.\n");

			break;
		case HARD_STOP:
			log_debug("Using hard stop\n");
			if (direction == RETRACT)
			{
				cpld_config = &hard_stop_retract_values;
			}
			else if (direction == EXTEND)
			{
				cpld_config = &hard_stop_extend_values;
			}
			else
				log_printf("Invalid direction given to set_config_buffer.\n");

			break;
#endif
#ifdef BOARD_VERSION_P1_1
		case COARSE_CONTROL:
			log_debug("Using coarse control\n");
			if (direction == RETRACT)
			{
				cpld_config = &fine_retract_nudge_values;
			}
			else if (direction == EXTEND)
			{
				cpld_config = &fine_extend_nudge_values;
			}
			else
				log_printf("Invalid direction given to set_config_buffer.\n");
			break;

		case FINE_CONTROL:
			log_debug("Using fine control\n");
			if (direction == RETRACT)
			{
				cpld_config = &fine_retract_values;
			}
			else if (direction == EXTEND)
			{
				cpld_config = &fine_extend_values;
			}
			else
				log_printf("Invalid direction given to set_config_buffer.\n");

			break;

		case HARD_STOP:
			log_debug("Using hard stop\n");
			if (direction == RETRACT)
			{
				cpld_config = &coarse_retract_values;
			}
			else if (direction == EXTEND)
			{
				cpld_config = &coarse_extend_values;
			}
			else
				log_printf("Invalid direction given to set_config_buffer.\n");

			break;
#endif
		default:
			log_printf("Invalid control type given to set_config_buffer.\n");
	}
	return cpld_config;
}

int run_parallel_lens_move(uint8_t * data, unsigned int cam_mask, uint8_t tolerance)
{
	//unsigned long time_start = 0, time_end = 0, total_time = 0;
	int ret = ETrue, i;
	//Bool result = ETrue;
	PiezoModule*   m;
	PiezoActuator* actuator;
	uint8_t * pos_data = data;
	uint16_t dst_pos;
	uint16_t final_position;
	//int failure_cnt = 0;
    int j = 0;

	ENTER_FUNC;

	// move cameras
	for (i = CAM_ID_B1; i < CAM_ID_MAX; i++)
	{
		if ((cam_mask & ( 0x1 << i)) || (cam_mask & 0x1))
		{
			if(check_cam_focus_status_actived(i - 1, MOVING_ERROR))
			{
				reset_cam_focus_status(i - 1, MOVING_ERROR);
			}
			set_cam_focus_status(i - 1, MOVING);
		}
		i += 1;
	}

	i = CAM_ID_B1;
	dst_pos = (pos_data[0] << 8) | pos_data[1];

	while (i < CAM_ID_MAX)
		{
			if ((cam_mask & ( 0x1 << i)) || (cam_mask & 0x1))
			{
				//failure_cnt = 0;
				CamDevice_TypeDef * pCam = CamDeviceTbl[i -1];
				if (!(cam_mask & 0x1))
				{
					dst_pos = (pos_data[0] << 8) | pos_data[1];
					pos_data+=2;
				}

				m = CameraIdToModule(pCam->camera_id); //do I use i
				actuator = m->Lens;

                // Measure the time individually for each lens
                //time_start = xTaskGetTickCount();

                /*
				do
				{
					log_debug("Moving cam %s\n", pCam->Name);

                    */
					/*result = */start_closed_control_lens(pCam, actuator, pCam->CPLD_select, dst_pos, FALSE, tolerance);
                    /*
					failure_cnt++;

				}
				while((result == EFalse) && (failure_cnt < 20));
                */

                /*
                time_end = xTaskGetTickCount();
                total_time = (time_end - time_start);
                log_time("Time taken to move %s: %d milliseconds\n", pCam->Name, (unsigned int)total_time);
                */

				final_position = ReadHallSensor(&(actuator->Hall));
                save_cam_m_status_field((uint8_t*)(cam_m_status->cam_m_lens_hall + j), 2,
                                        LITTLE_ENDIAN, (uint8_t*)&final_position);
                j += 2;

			}
            i += 1;
		}

	// stop the move
	for (i = CAM_ID_B1; i < CAM_ID_MAX; i++)
	{

		if ((cam_bitmask  & (0x1 << i)) != 0 || (cam_bitmask & 0x1)) //fix me
		{

			if(ret == EFalse)
			{
				reset_cam_focus_status(i - 1, MOVING);
				set_cam_focus_status(i - 1, MOVING_ERROR);
			}
			else
			{
				reset_cam_focus_status(i - 1, MOVING);
				set_cam_focus_status(i - 1, IDLE);
			}
		}
	}

	EXIT_FUNC;
	return ret;
}

int run_parallel_mirror_move(uint8_t * data, unsigned int cam_mask)
{
	int ret = ETrue, i;
	Bool result = ETrue;
	PiezoModule*   m;
	PiezoActuator* actuator;
	uint8_t * pos_data = data;
	uint16_t dst_pos;
	int failure_cnt = 0;

	ENTER_FUNC;

	// move cameras
	for (i = CAM_ID_B1; i < CAM_ID_MAX; i++)
	{
		if ((cam_mask & ( 0x1 << i)) || (cam_mask & 0x1))
		{
			if(check_cam_mirror_status_actived(i - 1, MOVING_ERROR))
			{
				reset_cam_mirror_status(i - 1, MOVING_ERROR);
			}
			set_cam_mirror_status(i - 1, MOVING);
		}
		i += 1;
	}

	i = CAM_ID_B1;
	dst_pos = (pos_data[0] << 8) | pos_data[1];

	while (i < CAM_ID_MAX)
		{
			if ((cam_mask & ( 0x1 << i)) || (cam_mask & 0x1))
			{
				failure_cnt = 0;
				CamDevice_TypeDef * pCam = CamDeviceTbl[i -1];
#ifdef BOARD_VERSION_P1_1
				if (pCam->camera_id == CAM_ID_B4 || pCam->camera_id == CAM_ID_C5
						|| pCam->camera_id == CAM_ID_C6)
				{
					log_error("Mirrors for B4/C5/C6 are glued");
					i += 1;
					continue;
				}
#endif
				if (!(cam_mask & 0x1))
				{
					dst_pos = (pos_data[0] << 8) | pos_data[1];
					pos_data+=2;
				}

				m = CameraIdToModule(pCam->camera_id); //do I use i
				actuator = m->Mirror;

				do
				{

				result = start_closed_control_mirror(pCam, actuator, pCam->CPLD_select, dst_pos, FALSE);
				failure_cnt++;

				}

				while((result == 1) && (failure_cnt < 2));
			}
			i += 1;
		}

	// stop the move
	for (i = CAM_ID_B1; i < CAM_ID_MAX; i++)
	{

		if ((cam_bitmask  & (0x1 << i)) != 0 || (cam_bitmask & 0x1)) //fix me
		{

			if(ret == EFalse)
			{
				reset_cam_mirror_status(i - 1, MOVING);
				set_cam_mirror_status(i - 1, MOVING_ERROR);
			}
			else
			{
				reset_cam_mirror_status(i - 1, MOVING);
				set_cam_mirror_status(i - 1, IDLE);
			}
		}
	}

	EXIT_FUNC;
	return ret;
}
void calibrate_freq(CamDevice_TypeDef* pCam, PiezoActuator* actuator)
{
	/*
		Frequency calibration concept from PIC code is move actuator with
		frequency in the range (freq-5Khz) < freq < (freq+5Khz) and choose
		optimal frequency which moved actuator fastest (if > 2.0mm/s).
	*/
	int8_t	 direction = -1;
	uint16_t start_pos, prev_pos, cur_pos, s, max, freq, p_freq;
	uint16_t CPLD_select = pCam->CPLD_select;
	uint16_t old_freq, new_freq;
	uint8_t* ptr_coarse_values;

	max = 0;
	freq = 0;

	// Set start-position
	start_pos = (actuator->Type == ActuatorType_Lens) ? 0x200 : 0x400;

	// Backup old frequency
	ptr_coarse_values = (uint8_t*) &coarse_extend_values;
	old_freq = *(ptr_coarse_values + 49)<<8 | *(ptr_coarse_values + 48);

	// Decrement 5unit for freq
	*(ptr_coarse_values + 48) -= 5;

	for (int i = 0; i < 10; i++)
	{
		// Move to start position

		start_closed_control_lens(pCam, actuator, CPLD_select, start_pos, TRUE,0);
		prev_pos = ReadHallSensor(&(actuator->Hall));

		// Set moving direction EXTEND for LENS and RETRACT for MIRROR, reverse if
		// actuator was reversed.
		if (pCam->lens_hall_polarity == HALL_POLARITY_NORMAL)
			direction = (actuator->Type == ActuatorType_Lens) ? EXTEND : RETRACT;
		else
			direction = (actuator->Type == ActuatorType_Lens) ? RETRACT: EXTEND;

		*(ptr_coarse_values + 48) += 1;
		p_freq = *(ptr_coarse_values + 49)<<8 | *(ptr_coarse_values + 48);

		log_printf("Start position	   : 0x%04x\n", prev_pos);
		// Configure CPLD to generate PWM frequency to be test
		for (int j = 0; j < 10; j++)
		{
			// Moving during 40ms and read position, because start_PWM_control spent 4ms
			// each time so need to call 10 times.
			configure_CPLD(COARSE_CONTROL, direction, NULL, 0);
			start_PWM_control(actuator, CPLD_select, 1, 4 * 1000);
		}

		cur_pos = ReadHallSensor(&(actuator->Hall));
		log_printf("Stop position	   : 0x%04x\r\n", cur_pos);
		// Calculate the speed
		s = (actuator->Type == ActuatorType_Lens) ? cur_pos - prev_pos : prev_pos - cur_pos;
		s /= 4;				// Convert from distance to speed with 0.1mm/s unit
		if (s > max)		// Store max speed and frequency
		{
			max = s;
			freq = p_freq;
		}
		log_printf("freq: 0x%04x speed %d.%dmm/s\r\n", p_freq, max/10, max%10);
	}

	// Update new PWM frequency whenever actuator has speed move over 2.0mm/s
	new_freq = (max >= 20) ? freq : old_freq;
	log_printf("freq: 0x%04x max speed %d.%dmm/s\r\n", new_freq, max/10, max%10);

	ptr_coarse_values = (uint8_t*) &coarse_extend_values;
	*(ptr_coarse_values + 48) = (uint8_t)new_freq;
	*(ptr_coarse_values + 49) = (uint8_t)(new_freq>>8);
	ptr_coarse_values = (uint8_t*) &coarse_retract_values;
	*(ptr_coarse_values + 48) = (uint8_t)new_freq;
	*(ptr_coarse_values + 49) = (uint8_t)(new_freq>>8);
	ptr_coarse_values = (uint8_t*) &fine_extend_values;
	*(ptr_coarse_values + 48) = (uint8_t)new_freq;
	*(ptr_coarse_values + 49) = (uint8_t)(new_freq>>8);
	ptr_coarse_values = (uint8_t*) &fine_retract_values;
	*(ptr_coarse_values + 48) = (uint8_t)new_freq;
	*(ptr_coarse_values + 49) = (uint8_t)(new_freq>>8);
	ptr_coarse_values = (uint8_t*) &hard_stop_extend_values;
	*(ptr_coarse_values + 48) = (uint8_t)new_freq;
	*(ptr_coarse_values + 49) = (uint8_t)(new_freq>>8);
	ptr_coarse_values = (uint8_t*) &hard_stop_retract_values;
	*(ptr_coarse_values + 48) = (uint8_t)new_freq;
	*(ptr_coarse_values + 49) = (uint8_t)(new_freq>>8);
}

void fine_nudge(PiezoActuator* actuator, uint16_t CPLD_select, uint8_t direction, uint16_t multiplier, uint8_t is_lens)
{
	/*
	This command should be used to nudge the lenses or mirrors in fine
	increments. For piezo actuators, the minimum nudge will be less
	than a single hall sensor value, so it will be difficult to tell
	from the miniterm logs whether or not the actuator has actually
	moved. This sub-hall sensor movement is required for finer focus.
	*/

	// hacky sort of workaround...modify the repetition byte in the fine control
	// CPLD config tables
#ifdef BOARD_VERSION_P1_1
	uint8_t tx_buffer[3], rx_buffer[3];
#endif
	log_debug("Starting FINE_NUDGE for %s\n", is_lens ? "LENS" : "MIRROR");
	if (direction == EXTEND)
	{
		log_debug("Direction = EXTEND\n");
		fine_extend_values.rep_ovr_cnt1.byte[0] = (uint8_t) (multiplier & 0xFF);
		fine_extend_values.rep_ovr_cnt1.byte[1] = (uint8_t) ((multiplier >> 8) & 0xFF);
	}
	else // direction == RETRACT
	{
		log_debug("Direction = RETRACT\n");
		fine_retract_values.rep_ovr_cnt1.byte[0] = (uint8_t) (multiplier & 0xFF);
		fine_retract_values.rep_ovr_cnt1.byte[1] = (uint8_t) ((multiplier >> 8) & 0xFF);
	}

	// configure_CPLD will switch CPLD modes
	configure_CPLD(FINE_CONTROL, direction, NULL, multiplier);
	start_PWM_control(actuator, CPLD_select, is_lens, NUDGE_FINE_DELAY_PERIOD * (uint32_t) multiplier);

	// switch CPLD modes back to normal tx_buffer[0] = 0x02;
#ifdef BOARD_VERSION_P1_1
	tx_buffer[1] = 0x06;
	tx_buffer[2] = 0x00;
	CPLD_SPI_Transfer(tx_buffer, rx_buffer, 3);
#endif
	log_debug("Finished FINE_NUDGE for %s\n", is_lens ? "LENS" : "MIRROR");
	// set the values back to default
	if (direction == EXTEND)
	{
#ifdef BOARD_VERSION_P1
		fine_extend_values.rep_ovr_cnt1.byte[0] = 0x14;
		fine_extend_values.rep_ovr_cnt1.byte[1] = 0x00;
#endif /* BOARD_VERSION_P1 */
#ifdef BOARD_VERSION_P1_1
		fine_extend_values.rep_ovr_cnt1.byte[0] = (uint8_t) (multiplier & 0xFF);
		fine_extend_values.rep_ovr_cnt1.byte[1] = (uint8_t) ((multiplier >> 8) & 0xFF);
#endif /* BOARD_VERSION_P1_1 */
	}
	else // direction == RETRACT
	{
		fine_retract_values.rep_ovr_cnt1.byte[0] = 0x14;
		fine_retract_values.rep_ovr_cnt1.byte[1] = 0x00;
	}
}
uint16_t get_i2c_channel_bit(int32_t i)
{
    /* There isn't a pattern to the i2c channel mask, so I decided to implement
    a hasty lookup table implemented as a case statement. */
    if(i > CAM_NUM_CAMERA_MAX)
    {
        log_error("Invalid input to get_i2c_channel_bit: %d\n", (int) i);
        return 0;
    }
    switch (i)
    {
        case 0: // A1
            return 0x0002;
        case 1: // A2
            return 0x0001;
        case 2: // A3
            return 0x0010;
        case 3: // A4
            return 0x0004;
        case 4: // A5
            return 0x0008;
        case 5: // B1
            return 0x0200;
        case 6: // B2
            return 0x0100;
        case 7: // B3
            return 0x0080;
        case 8: // B4
            return 0x0020;
        case 9: // B5
            return 0x0040;
        case 10: // C1
            return 0x0400;
        case 11: // C2
            return 0x0800;
        case 12: // C3
            return 0x2000;
        case 13: // C4
            return 0x1000;
        case 14: // C5
            return 0x8000;
        case 15: // C6
            return 0x4000;
        default:
            log_error("Invalid input to get_i2c_channel_bit: %d\n", (int) i);
            return 0;
    }
}

void cam_module_open_iterate(uint8_t i, uint8_t cam_ctrl,
		CamDevice_TypeDef* pCam, cam_open_data_t* cam_open_data, UInt16 tid)
{
    //TODO: Split into smaller functions
    uint8_t status;
    if(pCam == NULL)
    {
        //continue;
        log_error("NULL pointer passed to cam_module_open_iterate\n");
        save_command_log(tid, ERROR_INVALID_ARG);
        return;
    }
    if((cam_ctrl == SW_STANDBY) || (cam_ctrl == HW_STANDBY))
    {
        StatusEnable(i, MODULE_POWER_ON | MODULE_CLOCK_ON);
        if(StatusActivated(i, MODULE_STREAM_ON))
        {
            log_warning("Disable streaming on module %s before switching to HW_STANDBY or SW_STANDBY.\n", pCam->Name);
            return;
        }
        if(cam_ctrl == HW_STANDBY)
        {
            log_info("%d: Putting %s into HW_STANDBY!\n", i, pCam->Name);

            if(StatusActivated(i, MODULE_HW_STANDBY))
            {
                log_warning("[%s] is already in HW_STANDBY mode \n\r", pCam->Name);
                //continue;
                return;
            }

            if(pCam->Type == CAM_TYPE_35MM)
            {
                status = af_controller_stop(pCam);
                log_debug("[%s] Closing VCM %s!\n\r", pCam->Name, (status == 1) ? "successful" : "fail");
            }

            cam_open_data->hw_standby_bitmask |= (1 << (i + 1));
            cam_open_data->hw_standby_slave_order |= get_i2c_channel_bit(i);

            StatusEnable(i, MODULE_HW_STANDBY);
            StatusDisable(i, MODULE_OPEN | MODULE_SW_STANDBY);

            //continue;
            return;
        }
        else
        {
            /* Check if this camera is already in SW_STANDBY */
            if(StatusActivated(i, MODULE_SW_STANDBY))
            {
                log_warning("[%s] is already open \n\r", pCam->Name);
                /*
                if(cam_open_data->sw_standby_bitmask & 0x01)
                    cam_open_data->sw_standby_bitmask &= 0xFFFFFE;
                    */
                //continue;
                return;
            }
            log_info("%d: Putting %s into SW_STANDBY!\n", i, pCam->Name);
            cam_open_data->sw_standby_bitmask |= (1 << (i + 1));
            cam_open_data->sw_standby_slave_order |= get_i2c_channel_bit(i);

            StatusEnable(i, MODULE_OPEN | MODULE_SW_STANDBY);
            StatusDisable(i, MODULE_HW_STANDBY);

            if(pCam->Type == CAM_TYPE_35MM)
                af_controller_init(pCam);
            //continue;
            return;
        }
    }
    else if(cam_ctrl == CLOSE)
    {
        log_info("Closing %s!\n", pCam->Name);
        cam_open_data->close_bitmask |= (1 << (i + 1));
        cam_open_data->close_slave_order |= get_i2c_channel_bit(i);
        if(!(StatusActivated(i, MODULE_OPEN | MODULE_STREAM_ON | MODULE_SW_STANDBY | MODULE_HW_STANDBY)))
        {
           log_warning("[%s] is already closed \n\r", pCam->Name);
           //continue;
           return;
        }
        if(StatusActivated(i, MODULE_STREAM_ON))
        {
            // TODO: Gracefully turn off the preview, trigger, etc. Wait for EOF, etc
            log_error("Cannot close %s while it is streaming.\n", pCam->Name);
            return;
        }
        StatusDisable(i, MODULE_OPEN | MODULE_STREAM_ON | MODULE_SW_STANDBY | MODULE_HW_STANDBY);
        if(pCam->Type == CAM_TYPE_35MM)
        {
            status = af_controller_stop(pCam);
            log_debug("[%s] Closing VCM %s!\n\r", pCam->Name, (status == 1) ? "successful" : "fail");
        }

        log_debug("[%s] closed !\n\r", pCam->Name);
    }
    else
    {
        log_error("Invalid input data!!!");
        save_command_log(tid, ERROR_INVALID_ARG);
        *CCB_ASIC_CAM_RD = -1;
    }
    return;
}

void cam_module_open_apply(uint8_t isGlobal, cam_open_data_t* cam_open_data)
{
    //uint8_t status = 0;
    CamDevice_TypeDef* pCam = NULL;

    if(cam_open_data->hw_standby_slave_order!= 0)
        write_xshutdown_signal(HW_STANDBY, cam_open_data->hw_standby_slave_order);

    if(cam_open_data->sw_standby_slave_order!= 0)
    {
        write_xshutdown_signal(SW_STANDBY, cam_open_data->sw_standby_slave_order);
        SensorsSetting(cam_open_data->sw_standby_bitmask, CamOpenRegs, ARRAY_COUNT(CamOpenRegs));
        for (int i = 0; i < CAM_NUM_CAMERA_MAX; i++)
        {
            if (((cam_open_data->sw_standby_bitmask >> 1) & 1 << i) != 0 || isGlobal)
            {
                pCam = CamDeviceTbl[i];
                /* Set default resolution before setting sensor */
#ifdef BOARD_VERSION_P1_1
                log_debug("%s \n",pCam->Name);
                //TODO :FIXME
                Set_Resolution(pCam, 4208, 3120);
#endif
#ifdef BOARD_VERSION_P1
                Set_Resolution(pCam, 3264, 2448);
#endif
            }
        }
        if(!is_active_ucid_activated() && get_active_ucid() != UC_DISABLED)
            activate_ucid(get_active_ucid());
    }

    if(cam_open_data->close_bitmask != 0)
        write_xshutdown_signal(CLOSE, cam_open_data->close_slave_order);

    /*
    tx[0] = 0x00A1;
    status = fpga_send_command(tx, rx, (num_bytes/2) - 2, IRQ_FROM_FPGA_TIMEOUT);
    if (status != FPGA_CMD_SUCCESS)
        log_error("Failed to read XShutdown status!\n");

    // send a dummy buffer to clock out the data from FPGA
    status = fpga_send_command(dummy_tx, rx, 1, IRQ_FROM_FPGA_TIMEOUT);
    if (status != FPGA_CMD_SUCCESS)
        log_error("Failed to read XShutdown status!\n");

    xshutdown_value = rx[0];
    log_info("value = 0x%04x\n", xshutdown_value);
    */
}

void write_xshutdown_signal(uint8_t cam_ctrl, uint32_t slave_order_mask)
{
    uint16_t num_bytes = 10;
    // The tx buffer required by fpga_send_command is 16bit, but the
    // input array is 8bit
    uint16_t tx[] = {0x00A1, 0x1054, 0x0008, 0x0000, 0xFFFF};
    uint16_t dummy_tx[] = {0xFFFF};
    uint16_t rx[num_bytes/2];
    uint16_t xshutdown_value = 0;
    uint8_t  status = 0;

    // READ
    status = fpga_send_command(tx, rx, (num_bytes/2) - 2, IRQ_FROM_FPGA_TIMEOUT);
    if (status != FPGA_CMD_SUCCESS)
        log_error("Failed to read XShutdown status!\n");

    // send a dummy buffer to clock out the data from FPGA
    status = fpga_send_command(dummy_tx, rx, 1, IRQ_FROM_FPGA_TIMEOUT);
    if (status != FPGA_CMD_SUCCESS)
        log_error("Failed to read XShutdown status!\n");

    xshutdown_value = rx[0];

    if(cam_ctrl == HW_STANDBY)
    {
        log_info("hw_standby_slave_order = %x\n", (unsigned int) slave_order_mask);

        // MODIFY
        tx[num_bytes/2 - 1] = xshutdown_value & (~slave_order_mask);
    }

    else if(cam_ctrl == SW_STANDBY)
    {
        log_info("sw_standby_slave_order = %x\n", (unsigned int) slave_order_mask);

        // MODIFY
        tx[num_bytes/2 - 1] = xshutdown_value | slave_order_mask;
    }

    else if(cam_ctrl == CLOSE)
    {
        // TODO: Handle the CLOSE case
    }

    // WRITE
    tx[0] = 0x00A0;
    status = fpga_send_command(tx, rx, num_bytes/2, IRQ_FROM_FPGA_TIMEOUT);
    if (status != FPGA_CMD_SUCCESS)
        log_error("Failed to deassert XShutdown!\n");
}

/**
 * Save command status of a specific transaction id
 */

Bool save_command_log(uint16_t tid,cam_cmd_status cmd_status)
{
	int i = 0;
	uint8_t save_idx;
	for(i = 0; i < MAX_CAM_TID_LOG_NUM; i++)
	{
		if(cam_cmd_log.cmd_log[i].tid == tid)
		{
			cam_cmd_log.cmd_log[i].cmd_status = cmd_status;
			return ETrue;
		}
	}
	save_idx = cam_cmd_log.cur_idx % MAX_CAM_TID_LOG_NUM;
	cam_cmd_log.cmd_log[save_idx].tid = tid;
	cam_cmd_log.cmd_log[save_idx].cmd_status = cmd_status;
	cam_cmd_log.cur_idx ++;
	return ETrue;
}

STATIC cam_cmd_status get_command_status(uint16_t tid)
{
	int i = 0;
	cam_cmd_status cmd_status = CMD_UNKNOWN;
	for(i = 0; i < MAX_CAM_TID_LOG_NUM; i++)
	{
		if(cam_cmd_log.cmd_log[i].tid == tid)
		{
			cmd_status = cam_cmd_log.cmd_log[i].cmd_status;
			break;
		}
	}
	return cmd_status;
}

STATIC char *cmd_status_to_string(cam_cmd_status status)
{
	switch(status)
	{
		case CMD_UNKNOWN:
		{
			return "CMD_UNKNOWN";
		}
		case CMD_SUCCESS:
		{
			return "CMD_SUCCESS";
		}
		case CMD_PENDING:
		{
			return "CMD_PENDING";
		}
		case CMD_INVALID_ARG:
		{
			return "CMD_INVALID_ARG";
		}
		case ERROR_INVALID_MBITMASK:
		{
			return "ERROR_INVALID_MBITMASK";
		}
		case ERROR_ASIC_UNAVAILABLE:
		{
			return "ERROR_ASIC_UNAVAILABLE";
		}
		case ERROR_MODULE_FAULT:
		{
			return "ERROR_MODULE_FAULT";
		}
		case ERROR_CAM_MODULE_RESOLUTION_INVALID_ARG:
		{
			return "ERROR_CAM_MODULE_RESOLUTION_INVALID_ARG";
		}
		case ERROR_EEPROM_FAULT:
		{
			return "ERROR_EEPROM_FAULT";
		}
		case ERROR_UNUSED:
		{
			return "ERROR_UNUSED";
		}
		default:
		{
			return "UNKNOWN STATUS";
		}
	}
}

void save_cam_m_status_field(volatile uint8_t* status_buffer,
                             uint8_t num_bytes,
                             uint8_t endianness,
                             uint8_t* data)
{
    if ((!status_buffer) || (!data))
    {
        log_error("Invalid input to save_cam_m_status_field!\n");
        return;
    }

    /* Potentially unsafe behavior: this procedure does not check the size
    of the input buffer, and hence can overwrite memory next to the buffer
    if left unhecked */

    for(int i = 0; i < num_bytes; i++)
    {
        if (endianness == LITTLE_ENDIAN)
        {
            // our architecture is little endian by default
            //log_printf("data[%d] = 0x%x", i, data[i]);
            status_buffer[i] = data[i];
        }
        else // BIG_ENDIAN
            status_buffer[i] = data[num_bytes - i - 1];
    }
}
