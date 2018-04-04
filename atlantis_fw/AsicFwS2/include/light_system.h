/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    light_system.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-16-2016
 * @brief   This file contains expand of the Light system object
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __LIGHT_SYSTEM_H__
#define __LIGHT_SYSTEM_H__

/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "tof.h"
#include "gyro.h"
#include "img_sensor.h"
#include "optical.h"
#include "pwr_ctrl.h"
#include "eeprom.h"
#include "temperature.h"
#include "task_af_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define------------------------------------------------------------*/

#define CAMERA_MAX()				ARRAY_SIZE(lookup_tbl)
#define CAMERA_NAME(id)				((uint8_t)((id & 0xF000) >> 12))
#define CAMERA_IDX(id)				((uint8_t)((id & 0x0F00) >> 8))
#define CAMERA_CH(id)				((uint8_t)((id & 0x000F) >> 0))

/* INTR queue definitions */
#define INTR_QUEUE_SIZE			20
#define INTR_DATA_SIZE			4

#define BITS_PER_PIXEL_RAW10	10
#define BITS_PER_PIXEL_RAW12	12
#define BITS_PER_PIXEL_RAW14	14
#define BITS_PER_PIXEL_RAW8		8

#define ASIC_READ_ASIC_2				BIT0
#define ASIC_READ_ASIC_3				BIT1
#define ASIC_READ_EVENT_ALLS			((ASIC_READ_ASIC_3<<1) - 1)

#define PZT1_VOLTAGE_DEFAULT_VALUE	12.0
#define PZT2_VOLTAGE_DEFAULT_VALUE	12.0
/* Exported typedef  ---------------------------------------------------------*/
/* Typedef wrappers */
/*
 * @brief cam_name_t
 * Camera group name
 */
typedef enum {
	GRP_A = 0x0A,
	GRP_B = 0x0B,
	GRP_C = 0x0C
} cam_grp_t ;
/*
 * @brief cam_focal_len_t
 * Camera focal length
 */
typedef enum {
	CAM_28MM	= 28,
	CAM_70MM	= 70,
	CAM_150MM	= 150
} cam_focal_len_t ;
/*
 * @brief cam_module_type_t
 * Camera module type
 */
typedef enum {
	CAM_TYPE_UNKNOWN	= 0,
	CAM_TYPE_28MM		= 1,
	CAM_TYPE_70MM		= 2,
	CAM_TYPE_150MM		= 3
} cam_module_type_t;
/*
 * @brief asic_id_t
 */
typedef enum {
	ASIC_1		= (BIT1),
	ASIC_2		= (BIT2),
	ASIC_3		= (BIT3),
	ASIC_ALL	= (BIT1 | BIT2 | BIT3)
} asic_id_t;
/*
 * @brief cam_id_t
 */
typedef enum {
	CAM_A1 = 0xA1,
	CAM_A2 = 0xA2,
	CAM_A3 = 0xA3,
	CAM_A4 = 0xA4,
	CAM_A5 = 0xA5,
	CAM_B1 = 0xB1,
	CAM_B2 = 0xB2,
	CAM_B3 = 0xB3,
	CAM_B4 = 0xB4,
	CAM_B5 = 0xB5,
	CAM_C1 = 0xC1,
	CAM_C2 = 0xC2,
	CAM_C3 = 0xC3,
	CAM_C4 = 0xC4,
	CAM_C5 = 0xC5,
	CAM_C6 = 0xC6
} cam_id_t;
/**
 * @brief ucid_mode_t
 */
typedef enum ucid_mode_idx {
	UCID_DISABLED =			0x00,
	UCID_UNKNOWN =			0x01,
	UCID_DEBUG =			0x02,
	UCID_PREVIEW =			0x03,
	UCID_VIDEO =			0x04,
	UCID_HIRES_CAPTURE =	0x05,
	UCID_FOCAL_STACKING =	0x06,
	UCID_HDR_CAPTURE =		0x07,
	UCID_RESERVED =			0x08,
	UCID_FTM_QUICK_CHECK =	0x09,
	UCID_FTM_CALIBRATION =	0x0A
} ucid_mode_t;

/**
 * @brief cam_return_t
 */
typedef enum
{
	CAM_OK = 0,
	CAM_INVALID_ARG,
	CAM_BUSY,
} cam_return_t;

/**
 * @brief cam_dir_t
 */
typedef enum
{
	CAM_DIR_RETRACT_NARROW = 0,
	CAM_DIR_EXTEND_WIDE,
} cam_dir_t;

/**
 * @brief pzt_pot_sel_t
 */
typedef enum
{
	PZT_POT_1 = 0,
	PZT_POT_2
} pzt_pot_sel_t;

/**
 * @brief lcc_cmd_tid_t
 */
typedef struct lcc_cmd_tid {
	uint16_t cmd_tid;
	uint32_t event;
} lcc_cmd_tid_t;

/*
 * @brief cam_t
 * Camera type format, This type applied for CAMERA_TBL
 */
typedef union
{
	struct
	{
		struct
		{
			uint8_t ch		 : 4;	/* Camera channel: 1-6 */
			uint8_t sub_asic : 4;	/* Sub ASIC manage by this ASIC */
		};
		union
		{
			struct
			{
				uint8_t idx	: 4;	/* Camera index in group: 1-6 */
				uint8_t grp	: 4;	/* Camera group name  : A/B/C */
			};
			uint8_t module;			/* Camera module : A1 - C6 */
		};
	};
	uint16_t ident;					/* Identify include grp, index, channel */
} cam_t;

typedef struct
{
    int16_t left_x;
    int16_t top_y;
    uint16_t width;
    uint16_t height;
} roi_rectangle_t;

/*
 * @brief cam_data_t
 * Camera setting for each of stream on UCID
 */
typedef struct
{
	uint8_t open;					/* Camera module open register settings */
	uint8_t stream[3];				/* Camera module streaming settings */
	uint8_t status[4];				/* Camera module status */
	uint8_t uuid[16];				/* Camera UUID unique for each of camera */
	uint8_t thumb_id[8];			/* Thumbnail ID (TID) for snapshot */
	uint8_t type;					/* Type of camera, it's same with group */
} cam_data_t;

typedef struct
{
	uint8_t burst_requested;		/* Request number of consecutive frame */
	uint8_t burst_available;		/* Number of buffers can be serve to snapshot */
	uint8_t burst_actual;			/* Number of actual frame has captured */
} burst_data_t;

/*
 * @brief cam_common_data_t
 * Camera settings/data that is common to all UCIDs
 */
typedef struct
{
    roi_rectangle_t roi_rectangle;      /* Focus ROI for the sensor (in sensor non-flipped x/y coordinates)
                                           2-byte left x, 2 byte top y,
                                           2 byte width, 2 byte height */
    float roi_center_x; /* 0-1 value, ratio in calibration coordinates (i.e. flipped and 0-4160/0-3120) */
    float roi_center_y;
} cam_common_data_t;

typedef struct
{
    uint32_t distance_mm;
    uint8_t ref_mod_idx;
    roi_rectangle_t ref_mod_roi;
} roi_transfer_t;

/*
 * @brief ccb_data_t
 */
typedef struct
{
	uint8_t fw_version[8];
	uint8_t light_protocol_version[2];
	uint8_t light_calib_version[2];
	uint8_t light_config_version[2];
	uint8_t asic_status[2];
	uint8_t module_status[2];		/* ASIC_MODULE_STATUS */
	uint8_t temperature[8];			/* CCB temperature from 4 TMP112 sensors */
	uint8_t log_ctrl;				/* LOG control */
	uint8_t mipi_cfg[4];			/* MIPI configurate */
	uint8_t snapshot_uuid[16];		/* Snapshot UUID, clear after snapshot */
	burst_data_t burst[UC_MAX];		/* burst for USECASEID */
	uint8_t roi_af[30];			/* Region of Interest buffer */
	uint8_t roi_ae[30];			/* Region of Interest buffer */
	uint8_t roi_count_af;			/* Number of af roi data */
	uint8_t roi_count_ae;			/* Number of ae roi data */
	roi_transfer_t roi_transfer;    /* ROI transfer parameters */
	float zoom_factor;			    /* Optical zoom factor wrt 28mm */
	uint32_t spim_speed;			/* Speed of spi master */
} ccb_data_t;

/*
 * @brief cam_typedef_t
 * Camera object typedef
 */
typedef struct
{
	img_sensor_t		*image;				/* Camera image sensor object */
	optical_t			*optical;			/* Optical system */
	cam_data_t			*settings;			/* Data caching for each of UCID */
    cam_common_data_t   cam_common_data;    /* Data that is not per UCID */
	cam_eeprom_t		eeprom;
	cam_t				info;				/* Camera identification */
	sync_t				sync;				/* SyncIO control */
	EventGroupHandle_t	event;				/* Event update from LCC command */
	SemaphoreHandle_t	semaphore;			/* Semaphore to access settings */
	QueueHandle_t		queue;				/* Queue data for LCC command */
} cam_typedef_t;

/*
 * @brief light_ccb_t
 * Light CCB object typedef
 */
typedef struct
{
	cam_typedef_t	*cam_tbl;
    af_ctrl_t       *af_ctrl;           /* auto-focus control */
	ccb_data_t      *settings;
	power_ctrl_t	pwr;
	uint32_t		m_bitmask_all;		/* All camera module bit-mask */
	uint32_t		m_filter;			/* My modules filter */
	uint32_t		m_filter_asic1;		/* Sub-ASIC1 modules filter */
	uint32_t		m_filter_asic2;		/* Sub-ASIC2 modules filter */
	uint16_t		cur_tid;			/* Current transaction ID */
	uint16_t		active_ucid;		/* Active Use-case ID */
	uint16_t		prev_ucid;			/* Previous Use-case ID */
	uint8_t			cam_tbl_size;		/* Number of camera */

	/* Clock */
	gpio_t ref_clk_in;			/* [in]  CAM_MCLK */
	gpio_t xtal_pypass;			/* [in]  XTAL PYPASS */
	gpio_t cam_power_en;		/* [out] Power enable for all cameras */
	gpio_t asic_ready;			/* [out] Ready state */
	gpio_t host_intr;			/* [in]  Host interrupt */
	void (*cb_host_intr)(void);
	asic_id_t asic_intr_mask;	/* Interrupt come from which ASIC */

	EventGroupHandle_t	event;	/* Event update from LCC command */

#if (ASIC_NUM == ASIC1)
	/* Gyro, Torch, Tof and Flash LED */
	lm3644_info_t	flash_dev;	/* Flash LED */
#ifdef USING_TOF_FUNC
	VL53L0X_Dev_t	ToF_dev;	/* ToF device */
#endif
#ifdef USING_GYRO_FUNC
	gyro_info_t		gyro_dev;	/* Gyro sensor device */
#endif
	/* Power measurement sensor device */
	uint8_t power_ch;				/* power channel */
	int curr[3];				/* The current value (uA)*/
	int volt[3];				/* The voltafe value (mV)*/
	int power[3];				/* The power value (mW)*/
#endif /* (ASIC_NUM == ASIC1) */

#if (ASIC_NUM == ASIC1)
	/* Sub ASIC control */
	gpio_t ref_clk_out;			/* [out] ASIC2/ASIC3 CAM_MCLK */
	gpio_t xtal_pypass_asic2;	/* [out] XTAL PYPASS to ASIC 2 */
	gpio_t xtal_pypass_asic3;	/* [out] XTAL PYPASS to ASIC 3 */

	gpio_t asic2_rst;			/* [out] ASIC2 reset */
	gpio_t asic3_rst;			/* [out] ASIC3 reset */

	gpio_t asic2_por;			/* [out] ASIC2 POR_N */
	gpio_t asic3_por;			/* [out] ASIC3 POR_N */

	gpio_t asic2_intr;			/* [in]  ASIC2 interrupt */
	gpio_t asic3_intr;			/* [in]  ASIC3 interrupt */
	void (*cb_asic2_intr)(void);
	void (*cb_asic3_intr)(void);


	/* I2C main bus */
	gpio_t i2c_alert;			/* [in]  I2C bus alert */
	void (*cb_i2c_alert)(void);

	/* HW Trigger */
	gpio_t hw_trig;				/* [out] HW TRIGGER */

	/* Event read the data from ASIC2/ASIC3 */
	EventGroupHandle_t	asic1_read_event;
	/* This variable indicate asic1 wait stream interrupt */
	uint8_t wait_intr_stream;
	lcc_cmd_interrupt_queue_t asic_intr_queue_num;
#else /* (ASIC_NUM == ASIC2) || (ASIC_NUM == ASIC3) */
	gpio_t asic_intr;			/* [out] ASIC2/ASIC3 interrupt signal to ASIC1*/
	void (*cb_asic_intr)(void);
	uint8_t check_asic_ready; /* Check the CAM status of ASIC2/ASIC3 */
	uint8_t is_first_command; /* Using to check it is first command */
#endif /* (ASIC_NUM == ASIC1) */
} light_ccb_t;


/* The struct contains cmd_tid and status of intr_src */
typedef struct
{
	uint16_t cmd_tid;
	uint16_t status;
} intr_src_t;
/* Exported variables --------------------------------------------------------*/
extern light_ccb_t * const light_system;
#if(ASIC_NUM == ASIC1)
typedef struct asic_sync
{
	uint32_t flag;
	uint32_t len;
	uint16_t m_bitmask;
	uint16_t tid;
	uint16_t ucid;
	struct
	{
		union
		{
			uint8_t r : 1; /* Read action*/
			uint8_t w : 1; /* Write action*/
			uint8_t reserved : 6;
		};
		uint8_t action;
	};
} asic_sync_t;
extern asic_sync_t *asic_sync_queue;
extern const uint8_t modules_tlb_size;
#endif
/* Exported functions --------------------------------------------------------*/
void light_main(void);
uint8_t module_to_idx(uint8_t module);
uint8_t idx_to_module(uint8_t idx);
cam_typedef_t *idx_to_object(uint8_t idx);
uint32_t object_to_cam_bitmask(cam_typedef_t *pcam);
cam_typedef_t *chan_to_object(uint8_t ch);

#if(ASIC_NUM == ASIC1)
void asic2_intr_handler(void);
void asic3_intr_handler(void);
#endif

void send_intr_signal(void);
int intr_queue_push(uint16_t cmd_tid, uint16_t status);
int intr_queue_pop(uint8_t *intr_status);
void intr_queue_reset(void);
void intr_signal(void);
uint8_t pzt_voltage_config(uint8_t pzt, float vol, uint8_t write_eeprom);
void set_intr_pin(uint8_t data);

/**
 * @brief cam_move_to_distance
 * This API will move a lens/VCM to a distance
 * @param pcam: reference to cam_typedef_t structure
 * @param distance: the target distance
 * @param cb_func: call-back function after moving done
 * @param cb_param: call-back parameter passed to the call-back function
 * @param cmd_tid: command transction id
 * @return reference to cam_return_t structure
 */
cam_return_t cam_move_to_distance(cam_typedef_t *pcam, uint32_t distance,
					void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid);

/**
 * @brief cam_move_lens_to_position
 * This API will move a lens/VCM to a position
 * @param pcam: reference to cam_typedef_t structure
 * @param position: the target position
 * @param tolerance: the target position tolerance
 * @param cb_func: call-back function after moving done
 * @param cb_param: call-back parameter passed to the call-back function
 * @param cmd_tid: command transction id
 * @return reference to cam_return_t structure
 */
cam_return_t cam_move_lens_to_position(cam_typedef_t *pcam, uint16_t position,
                    uint16_t tolerance,
					void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid);

/**
 * @brief cam_move_mirr_to_position
 * This API will move a mirror to a position
 * @param pcam: reference to cam_typedef_t structure
 * @param position: the target position
 * @param cb_func: call-back function after moving done
 * @param cb_param: call-back parameter passed to the call-back function
 * @param cmd_tid: command transction id
 * @return reference to cam_return_t structure
 */
cam_return_t cam_move_mirr_to_position(cam_typedef_t *pcam, uint16_t position,
					void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid);

/**
 * @brief cam_nudge_lens
 * This API will nudges a lens/VCM in the desired direction
 * @param pcam: reference to cam_typedef_t structure
 * @param direction: the movement direction
 * @param multiplier: the number of PWM pulses
 * @param cb_func: call-back function after moving done
 * @param cb_param: call-back parameter passed to the call-back function
 * @param cmd_tid: command transction id
 * @return reference to cam_return_t structure
 */
cam_return_t cam_nudge_lens(cam_typedef_t *pcam, cam_dir_t direction,
uint16_t multiplier, void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid);

/**
 * @brief cam_nudge_mirr
 * This API will nudges a mirror in the desired direction
 * @param pcam: reference to cam_typedef_t structure
 * @param direction: the movement direction
 * @param multiplier: the number of PWM pulses
 * @param cb_func: call-back function after moving done
 * @param cb_param: call-back parameter passed to the call-back function
 * @param cmd_tid: command transction id
 * @return reference to cam_return_t structure
 */
cam_return_t cam_nudge_mirr(cam_typedef_t *pcam, cam_dir_t direction,
uint16_t multiplier, void (*cb_func)(void *), void *cb_param, uint16_t cmd_tid);
#if(ASIC_NUM == ASIC1)
EventBits_t wait_intr_signal(asic_id_t asic_id_msk, uint32_t timeout, uint8_t waitall);
#endif
void send_hw_sync_trigger(void);

/**
 * @brief get_calib_size
 * This function used to get size of calib data on spi flash
 * @param m_bitmask: the bit mask of camera channel
 * @return size of calib data on spi flash
 */
uint32_t get_calib_size(void);

/**
 * @brief copy_calib_data
 * This function used to copy calib data on spi flash to specific address in
 * memory
 * @param offset: offset to data within calibration data in flash (i.e. default is 0)
 * @param dst: pointer to anywhere in memory
 * @param size: size of calib data
 * @note: Don't worry about memory allocation; caller is supposed to first
 * allocate memory for this.
 * @return status
 */
void copy_calib_data(uint32_t offset, uint8_t *dst, uint32_t size);

#ifdef __cplusplus
}
#endif
#endif /* __LIGHT_SYSTEM_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
