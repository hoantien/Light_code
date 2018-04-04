/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    img_sensor.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-06-2016
 * @brief   This file contains expand of the camera image sensor
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __IMG_SENSOR__
#define __IMG_SENSOR__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "lcc_system.h"
#include "ar1335.h"
#include "os.h"

/* Exported define------------------------------------------------------------*/
#define CAM_SEND_TIMEOUT			10 /* ms */
#define CAM_WAIT_TIMEOUT			10 /* ms */
/* Number of registers to set resolution */
#define CAM_RES_REGISTER_SIZE		17
/** CAM_OPEN data */
#define SW_STANDBY					(0x02)
#define HW_STANDBY					(0x01)
/** CAM_STREAM data */
#define CAM_STREAM_DISABLE			(0x00)
#define CAM_STREAM_ENABLE			(0x01)
#define CAM_STREAM_CSI0				(0x10)
#define CAM_STREAM_CSI1				(0x20)
#define CAM_STREAM_CSI0_CSI1		(0x30)
/** Virtual channel identifier */
#define CAM_STREAM_VC_MASK			(0xC0)
/** Data Type (DT) */
#define CAM_STREAM_DT_MASK			(0x3F)

/** Cam setting updated */
#define CAM_UPDATED_RESOLUTION		((uint32_t)BIT0)
#define CAM_UPDATED_SENSITIVITY		((uint32_t)BIT1)
#define CAM_UPDATED_EXPOSURE		((uint32_t)BIT2)
#define CAM_UPDATED_FPS				((uint32_t)BIT3)
#define CAM_UPDATED_FOCUS_DISTANCE	((uint32_t)BIT4)
#define CAM_UPDATED_FOCAL_LEN		((uint32_t)BIT5)

/** Cam drive updated */
#define CAM_UPDATED_MIRROR_POSITION	((uint32_t)BIT6)
#define CAM_UPDATED_LENS_POSITION	((uint32_t)BIT7)
#define CAM_UPDATED_VCM_POSITION	((uint32_t)BIT8)

/** Cam Module Open updated */
#define CAM_UPDATED_M_O_STATUS		((uint32_t)BIT0)
#define CAM_UPDATED_M_STREAM		((uint32_t)BIT1)

/** Cam module EEPROM FLAGS */
#define CAM_EEPROM_MODULE_WRITE_FLAG	BIT0
#define CAM_EEPROM_MODULE_READ_FLAG		BIT1
#define CAM_EEPROM_R_W_BUSY_FLAG		BIT2
#define CAM_EEPROM_R_DONE_FLAG			BIT3
#define CAM_EEPROM_W_DONE_FLAG			BIT4
#define CAM_EEPROM_R_W_ERROR_FLAG		BIT5

/** Cam external clock */
#ifdef PLL_25MHZ
#define	EXT_CLK_FREQ_MHZ				25
#else
#define	EXT_CLK_FREQ_MHZ				24
#endif

/** Convert from MHZ to HZ */
#define MHZ_TO_HZ						1000000

#define CAM_REG_GROUP_ON_OFF	0x0104
#define CAM_REG_SENSITIVITY		0x305E
#define CAM_REG_STREAM_CTRL		0x0100
#define CAM_REG_X_ADDR_START	0x0344
#define CAM_REG_X_ADDR_END		0x0348
#define CAM_REG_Y_ADDR_START	0x0346
#define CAM_REG_Y_ADDR_END		0x034A
#define CAM_REG_X_ADDR_MAX		4239
#define CAM_REG_Y_ADDR_MAX		3151
#define MAX_ANALOG_GAIN_SNAPSHOT		7.75
#define MAX_ANALOG_GAIN_CODE_SNAPSHOT	0x203F
#define MAX_SENSOR_REG_VAL		0xFFFF
/* Exported typedef  ---------------------------------------------------------*/
/**
* @brief cam_access_t typedef
* Camera command access mode
*/
typedef enum img_sensor_access
{
	WRITE,
	READ
} img_sensor_access_t;

/**
 * @brief cam_power_mode_t
 */
typedef enum cam_power_mode
{
	PWR_ON,
	PWR_OFF
} cam_power_mode_t;
/**
 * @brief cam_sensor_mode_t
 * The camera operating mode
 */
typedef enum cam_sensor_mode
{
	SLAVE_MODE,
	I2C_MODE
} cam_sensor_mode_t;

/**
* @brief cam_data_mode_t typedef
* Indicate the size of access data
*/
typedef enum img_sensor_data_mode
{
	ONE_BYTE = 1,
	TWO_BYTES = 2
} img_sensor_data_mode_t;

/**
 * @brief pattern_mode_t
 * Image sensor pattern mode
 */
typedef enum pattern_mode
{
	PAT_NONE 		= (uint16_t)0x0000,
	PAT_SOLID_COLOR = (uint16_t)0x0001,
	PAT_COLOR_BAR	= (uint16_t)0x0002,
} pattern_mode_t;

/**
 * @brief flip_mode_t
 * Image sensor flip mode
 */
typedef enum flip_mode
{
	FLIP_NONE		= 0x0,
	FLIP_VERTICAL	= 0x2,
	FLIP_HORIZONTAL = 0x1,
	FLIP_BOTH		= 0x3
} flip_mode_t;
/**
 * @brief cam_x_resolution_type_t typedef
 */
typedef enum img_sensor_x_res
{
	X_3M		= 2104,
	X_3M_P2		= 2048,
	X_13M		= 4208,
	X_13M_P2	= 4160,
	X_720P		= 1280,
	X_1080P		= 1920,
	X_4K_UHD	= 3840,
	X_4K_CINEMA	= 4096
} img_sensor_x_res_t;

/**
 * @brief cam_y_resolution_type_t typedef
 */
typedef enum img_sensor_y_res
{
	Y_3M			= 1560,
	Y_3M_P2			= 1536,
	Y_13M			= 3120,
	Y_720P			= 720,
	Y_1080P			= 1080,
	Y_4K_UHD_CINEMA	= 2160
} img_sensor_y_res_t;

/**
 * @brief cam_resolution_type_t typedef
 */
typedef enum img_res
{
	IMG_RES_3M			= X_3M * Y_3M,
	IMG_RES_13M			= X_13M * Y_13M,
	IMG_RES_720P		= X_720P * Y_720P,
	IMG_RES_1080P		= X_1080P * Y_1080P,
	IMG_RES_4K_UHD		= X_4K_UHD * Y_4K_UHD_CINEMA,
	IMG_RES_4K_CINEMA	= X_4K_CINEMA * Y_4K_UHD_CINEMA
} img_res_t;

/**
 * @brief cam_fps_type_t typedef
 */
typedef enum img_fps_type
{
	FPS30 = 30,
	FPS60 = 60,
	FPS15 = 15,
	FPS24 = 24
} img_fps_type_t;

/**
 * @brief cam_sensor_type_t typedef
 */
typedef enum {
	AR1335_COLOR_SENSOR,
	AR1335_PAN_SENSOR,
} cam_sensor_type_t;

/**
* @brief cam_resolution_t typedef
* Camera module resolution
*/
typedef struct img_sensor_res
{
	uint32_t	x;			/* x axis */
	uint32_t	y;			/* y axis */
	img_res_t	res_type;	/* x*y axis */
} img_sensor_res_t;

/**
* @brief cam_command_t typedef
* Camera command
*/
typedef struct img_sensor_command
{
	uint16_t				cmd_id;
	img_sensor_access_t		mode;		/* Access mode */
	img_sensor_data_mode_t	d_mode;
	uint16_t		reg_addr;	/* Address Access on camera module */
	uint8_t			*data;		/* Data pointer */
	uint16_t		len;
	uint16_t		timeout;	/* Timeout milisecond */
	void			(*call_back)(void *cam); /* Callback handler */
} img_sensor_command_t;

/*
 * @brief img_data_t
 * Image sensor setting for each of stream on UCID
 */
typedef struct
{
	uint8_t exposure[8];
	uint8_t resolution[8];
	uint8_t sensitivity[4];
	uint8_t fps[2];
} img_data_t;

typedef struct
{
	i2c_t		i2c_dev;				/* I2C channel */
	gpio_t		standby;				/* [out] Camera XSHUTN control */
	img_data_t	*settings[UC_MAX];		/* Data caching for each of UCID */
	uint16_t	default_fll;			/* Default frame length line to back */
	uint16_t	default_llpclk;			/* Default line length PCLK */
	uint16_t	old_llpclk;				/* Some functions change llpclk temporary,
											need to store it to revert back */
	img_sensor_command_t	*cmd; 		/* Camera command */
	SemaphoreHandle_t semaphore;		/* Semaphore to access shared setting */
	EventGroupHandle_t event;			/* Event update from LCC command */
	flip_mode_t	flip;					/* Image sensor flip mode for capture/streaming */
	uint16_t	x_size;					/* X-output size */
	uint16_t	y_size;					/* Y-output size */
	cam_sensor_type_t	sensor_type;	/* panchromatic or color */
} img_sensor_t;

/* Exported variable ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int img_sensor_init(img_sensor_t *img_sensor);
int img_sensor_deinit(img_sensor_t *img_sensor);
int img_sensor_open(img_sensor_t *img_sensor, uint8_t mode);
int img_sensor_close(img_sensor_t *img_sensor);
int img_sensor_stream_on(img_sensor_t *img_sensor,  cam_sensor_mode_t mode);
int img_sensor_stream_off(img_sensor_t *img_sensor, cam_sensor_mode_t mode);
int img_sensor_set_sensitivity(img_sensor_t *img_sensor, uint32_t sensitivity);
int img_sensor_set_exposure_time(void *pcam, uint64_t exposure);
int img_sensor_config_mode(img_sensor_t *img_sensor,
										cam_sensor_mode_t sensor_mode);
int img_sensor_cropping(img_sensor_t *img_sensor, uint16_t x_position,
					uint16_t y_position, uint16_t x_width, uint16_t y_width, uint16_t scale);
uint16_t img_sensor_read_reg(img_sensor_t *img_sensor, uint16_t reg,
											img_sensor_data_mode_t datamode);
int img_sensor_write_reg(img_sensor_t *img_sensor, uint16_t reg, uint16_t data,
											img_sensor_data_mode_t datamode);
int img_sensor_send_config(img_sensor_t *img_sensor, cam_reg_array_t *config,
														uint32_t num_config);
float img_sensor_get_vt_pix_clk_mhz(img_sensor_t *img_sensor);
int img_is_fps_supported(uint16_t fps);
int img_sensor_pattern_mode(img_sensor_t *img_sensor, pattern_mode_t pattmode);
int img_sensor_set_fps(void *cam, uint16_t fps);
int img_sensor_flip_capture(img_sensor_t *img_sensor, flip_mode_t flip);
int img_sensor_flip_preview(img_sensor_t *img_sensor, flip_mode_t flip);
void img_sensor_ctrl_reg_group(img_sensor_t *img_sensor, uint8_t state);
cam_sensor_type_t get_image_sensor_type(img_sensor_t *image);
uint8_t img_sensor_read_pixel_order(img_sensor_t *img_sensor);
double img_sensor_get_frame_time(img_sensor_t *image, double *frame_time);
void img_sensor_reset_trigger(img_sensor_t *image);
uint16_t calculate_gain(float *a_gain, float *total_gain, float r_gain, uint8_t c_mode);
float get_total_gain(uint16_t gain_code, uint8_t c_mode);
#ifdef __cplusplus
}
#endif
#endif /* __IMG_SENSOR__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
