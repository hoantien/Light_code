/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    lcc_cmd.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains definitions of lcc_cmd
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCC_CMD_H__
#define __LCC_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "lcc_system.h"
#include "task_ccb_ctrl.h"

/* Exported define -----------------------------------------------------------*/
/* Total number of camera on device */
#define CAM_A1_INDEX						1
#define CAM_NUM_CAMERA_MAX					16

/* Maximum of ROI */
#define CAM_ROI_DATA_MAX_COUNT				3
#define CAM_ROI_DATA_SIZE					10

/* LCC command transaction ID index */
#define LCC_CMD_TRANSACTION_ID_IDX			0
/* LCC transaction ID size define at 0 if not use TRANSACTION ID,
if use TRANSACTIONID define as 2*/
#define LCC_CMD_TRANSACTION_ID_SIZE			(2)
/* LCC command index */
#define LCC_CMD_COMMAND_IDX					(LCC_CMD_TRANSACTION_ID_IDX + \
											LCC_CMD_TRANSACTION_ID_SIZE)
/* LCC command size */
#define LCC_CMD_COMMAND_SIZE				(2)
/* M_BitMask index */
#define LCC_CMD_M_BITMASK_IDX				(LCC_CMD_COMMAND_IDX + \
											LCC_CMD_COMMAND_SIZE)
/* M_BitMask index filter */
#define LCC_CMD_M_BITMASK_FILTER_IDX		0

/* M_BitMask size */
#define LCC_CMD_M_BITMASK_FILTER_SIZE		(LCC_CMD_M_BITMASK_FILTER_IDX + 3)
/* Command global bitmask */
#define LCC_CMD_M_BITMASK_GLOBAL_MASK		((uint32_t)(1<<0))
/* Data index in message */
#define LCC_CMD_DATA_INDEX					LCC_CMD_M_BITMASK_FILTER_SIZE
/* UCID data index */
#define LCC_CMD_UCID_INDEX					8
/* Mask of m_bitmask data now is 16 cameras module  */
#define LCC_CMD_M_BITMASK_MASK				0x0001FFFF

/* LCC command group 0 via I2C */
/*
 * CMD FORMAT (32bit)
 * BIT [7 - 0]: c			Command
 * BIT [14- 8]: b			Base address
 * BIT [15-15]: i			Interrupt, setting by host
 * BIT [16-16]: m			m_bitmask support
 * BIT [17-17]: g			Global support
 * BIT [18-18]: u			UCID support
 * BIT [19-19]: r			Read support
 * BIT [20-20]: w			Write support
 * BIT [21-21]: l			Do not predetermined the data length
 * BIT [31-24]: s			Size of data field of each camera module
 */
#define LCC_CMD_DEFINE(c, b, m, g, u, r, w, l, s)	\
			(uint32_t)((s<<24)|(l<<21)|(w<<20)|(r<<19)|(u<<18)|(g<<17)|(m<<16)|(b<<8)|c)

#define LCC_CMD_GROUP_0000					0x0000
										/*	LCC_CMD_DEFINE(c   , b   , m, g, u, r, w, l, s) */
#define LCC_CMD_CAM_MODULE_OPEN				LCC_CMD_DEFINE(0x00, 0x00, 1, 1, 0, 1, 1, 0, 1)
#define LCC_CMD_CAM_STREAMING				LCC_CMD_DEFINE(0x02, 0x00, 1, 1, 0, 1, 1, 0, 3)
#define LCC_CMD_CAM_COMMAND_STATUS			LCC_CMD_DEFINE(0x24, 0x00, 0, 0, 0, 1, 1, 0, 2)
#define LCC_CMD_CAM_MODULE_STATUS			LCC_CMD_DEFINE(0x28, 0x00, 1, 0, 0, 1, 0, 0, 4)
#define LCC_CMD_CAM_MODULE_RESOLUTION		LCC_CMD_DEFINE(0x2C, 0x00, 1, 1, 1, 1, 1, 0, 8)
#define LCC_CMD_CAM_MODULE_SENSITIVITY		LCC_CMD_DEFINE(0x30, 0x00, 1, 1, 1, 1, 1, 0, 4)
#define LCC_CMD_CAM_MODULE_EXPOSURE_TIME	LCC_CMD_DEFINE(0x32, 0x00, 1, 1, 1, 1, 1, 0, 8)
#define LCC_CMD_CAM_MODULE_FPS				LCC_CMD_DEFINE(0x50, 0x00, 1, 1, 1, 1, 1, 0, 2)
#define LCC_CMD_CAM_MODULE_FOCAL_LENGTH		LCC_CMD_DEFINE(0x3A, 0x00, 1, 1, 0, 1, 0, 0, 2)
#define LCC_CMD_CAM_MODULE_FOCUS_DISTANCE	LCC_CMD_DEFINE(0x48, 0x00, 1, 1, 0, 1, 1, 0, 4)
#define LCC_CMD_CAM_MODULE_LENS_POSITION	LCC_CMD_DEFINE(0x40, 0x00, 1, 1, 0, 1, 1, 0, 2)
#define LCC_CMD_CAM_MODULE_MIRROR_POSITION	LCC_CMD_DEFINE(0x44, 0x00, 1, 1, 0, 1, 1, 0, 2)
#define LCC_CMD_CAM_BURST_REQUESTED			LCC_CMD_DEFINE(0x10, 0x00, 0, 0, 1, 1, 1, 0, 1)
#define LCC_CMD_CAM_BURST_AVAILABLE			LCC_CMD_DEFINE(0x12, 0x00, 0, 0, 1, 1, 1, 1, 0)
#define LCC_CMD_CAM_BURST_ACTUAL			LCC_CMD_DEFINE(0x14, 0x00, 0, 0, 1, 1, 1, 0, 0)
#define LCC_CMD_CAM_SNAPSHOT_UUID			LCC_CMD_DEFINE(0x06, 0x00, 0, 0, 0, 1, 1, 0, 16)
#define LCC_CMD_CAM_SNAPSHOT_TID			LCC_CMD_DEFINE(0x08, 0x00, 1, 0, 0, 1, 1, 0, 8)
#define LCC_CMD_CAM_MODULE_UUID				LCC_CMD_DEFINE(0x4A, 0x00, 1, 0, 0, 1, 0, 0, 16)
#define LCC_CMD_CAM_MODULE_TYPE				LCC_CMD_DEFINE(0x4C, 0x00, 1, 0, 0, 1, 0, 0, 1)
#define LCC_CMD_CAM_ROI						LCC_CMD_DEFINE(0x58, 0x00, 0, 0, 0, 1, 1, 1, 62)
#define LCC_CMD_CAM_ROI_TRANSFER            LCC_CMD_DEFINE(0x59, 0x00, 1, 1, 0, 0, 1, 0, 13)
#define LCC_CMD_CAM_ROI_CALIBRATION			LCC_CMD_DEFINE(0x5A, 0x00, 1, 1, 0, 1, 1, 0, 8)
#define LCC_CMD_I2C_FORWARD					LCC_CMD_DEFINE(0x5B, 0x00, 0, 0, 0, 1, 1, 1, 0)
#define LCC_CMD_ZOOM_FACTOR					LCC_CMD_DEFINE(0x5C, 0x00, 0, 0, 0, 1, 1, 0, 4)
#define LCC_CMD_CAM_MODULE_MIRROR_CALIBRATION	\
											LCC_CMD_DEFINE(0x5E, 0x00, 1, 1, 0, 0, 1, 0, 1)
#define LCC_CMD_I2C_SPEED					LCC_CMD_DEFINE(0x60, 0x00, 0, 0, 0, 1, 1, 1, 1)
#define LCC_CMD_SPI_SPEED					LCC_CMD_DEFINE(0x62, 0x00, 0, 0, 0, 1, 1, 0, 4)

/* LCC Command group 1 via I2C */
#define LCC_CMD_GROUP_1000					0x1000
#define LCC_CMD_LIGHT_ACTIVE_UCID			LCC_CMD_DEFINE(0x00, 0x10, 0, 0, 0, 1, 1, 0, 2)

/* LCC Command group 2 via I2C*/
#define LCC_CMD_GROUP_0200					0x0200
#define LCC_CMD_ASIC_LIGHT_PROTOCOL			LCC_CMD_DEFINE(0x03, 0x02, 0, 0, 0, 1, 0, 0, 2)
#define LCC_CMD_ASIC_FW_VERSION				LCC_CMD_DEFINE(0x05, 0x02, 0, 0, 0, 1, 0, 0, 8)
#define LCC_CMD_ASIC_CALIB_VERSION			LCC_CMD_DEFINE(0x0D, 0x02, 0, 0, 0, 1, 0, 0, 2)
#define LCC_CMD_ASIC_STATUS					LCC_CMD_DEFINE(0x15, 0x02, 0, 0, 0, 1, 0, 0, 2)
#define LCC_CMD_ASIC_LOG_CTRL				LCC_CMD_DEFINE(0x19, 0x02, 0, 0, 0, 1, 1, 0, 1)
#define LCC_CMD_ASIC_MODULE_STATUS			LCC_CMD_DEFINE(0x1B, 0x02, 1, 0, 0, 1, 0, 0, 2)
#define LCC_CMD_ASIC_TMPx					LCC_CMD_DEFINE(0x1C, 0x02, 0, 0, 0, 1, 0, 0, 8)
#define LCC_CMD_ASIC_CMD_DUMP				LCC_CMD_DEFINE(0x24, 0x02, 0, 0, 0, 1, 0, 0, 48)
#define LCC_CMD_ASIC_MODULE_METADATA		LCC_CMD_DEFINE(0x72, 0x02, 1, 1, 0, 1, 1, 1, 4)
#define LCC_CMD_ASIC_DEVICE_CALIBRATION		LCC_CMD_DEFINE(0x74, 0x02, 0, 0, 0, 1, 1, 0, 0)
#define LCC_CMD_ASIC_PWR_CTRL				LCC_CMD_DEFINE(0x78, 0x02, 0, 0, 0, 1, 1, 0, 2)
#define LCC_CMD_CCB_PWR_CTRL				LCC_CMD_DEFINE(0x7A, 0x02, 0, 0, 0, 1, 1, 0, 2)
#define LCC_CMD_AISC_INTR_SRC				LCC_CMD_DEFINE(0x7C, 0x02, 0, 0, 0, 1, 0, 0, 0)
#define LCC_CMD_ASIC_FLASH_LIGHT			LCC_CMD_DEFINE(0x54, 0x02, 0, 0, 0, 0, 1, 1, 0)
#define LCC_CMD_ASIC_TOF					LCC_CMD_DEFINE(0x56, 0x02, 0, 0, 0, 1, 1, 1, 1)
#define LCC_CMD_ASIC_GYRO					LCC_CMD_DEFINE(0x52, 0x02, 0, 0, 0, 1, 1, 0, 6)
#if (ASIC_NUM == ASIC1)
#define LCC_CMD_ASIC_CTRL					LCC_CMD_DEFINE(0x7D, 0x02, 0, 0, 0, 0, 1, 1, 0)
#define LCC_CMD_ASIC_POWER_INFO				LCC_CMD_DEFINE(0x7B, 0x02, 0, 0, 0, 1, 1, 1, 12)
#endif
#define LCC_CMD_ASIC_CALIB_DATA				LCC_CMD_DEFINE(0x7E, 0x02, 0, 0, 0, 0, 1, 1, 0)
#define LCC_CMD_ASIC_PZT_PWR_CTRL			LCC_CMD_DEFINE(0x7F, 0x02, 0, 0, 0, 1, 1, 1, 0)
#define SWAP32(n)				swap32((uint32_t)(*(uint32_t *)&n))
#define SWAP16(n)				swap16((uint16_t)(*(uint16_t *)&n))

#define CMD_WRITE				0
#define CMD_READ				1

#define	LCC_CMD_CAM_COMMAND_STATUS_READ_SIZE	4
#define LCC_CMD_GYRO_DATA_SIZE					12

#define LCC_CMD_READ_EEPROM_METADATA_ID			0x72
#define LCC_CMD_READ_EEPROM_METADATA_BASE		0x02
#define LCC_CMD_STREAM_STATUS_ID				0x02
#define LCC_CMD_STREAM_STATUS_BASE				0x00

/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief lcc_cmd_t
 * LCC commands structure
 */
typedef union
{
	struct
	{
		uint32_t cmd		: 8;	/* BIT [7 - 0]: Command */
		uint32_t base		: 7;	/* BIT [14- 8]: Base address */
		uint32_t intr		: 1;	/* BIT [15-15]: Interrupt, setting by host */
		uint32_t m_bitmask	: 1;	/* BIT [16-16]: m_bitmask support */
		uint32_t global		: 1;	/* BIT [17-17]: Global support */
		uint32_t ucid		: 1;	/* BIT [18-18]: UCID support */
		uint32_t read		: 1;	/* BIT [19-19]: Read support */
		uint32_t write		: 1;	/* BIT [20-20]: Write support */
		uint32_t bypass		: 1;	/* BIT [21-21]: Bypass the length checking*/
		uint32_t reserved	: 2;	/* BIT [23-22]: Reserved */
		uint32_t size		: 8;	/* BIT [31-24]: Size of data field */
	};
	uint32_t word;
} lcc_cmd_info_t;
/**
 * @brief lcc_cmd_t
 * LCC commands structure
 */
typedef struct
{
	uint32_t m_bitmask;			/* Module status */
	uint8_t *data;				/* Data buffer pointer */
	uint16_t len;				/* Data length */
	uint16_t tid;				/* Transaction ID */
	uint16_t cmd;				/* Command value */
	uint16_t ucid;				/* Usercase ID */
	uint16_t status;			/* Command status */
	uint8_t m_number;			/* Number of module selected in m_bitmask */
	struct
	{
		uint8_t action : 1;				/* Command read/write action */
		uint8_t global : 1;				/* Command read/write action */
		uint8_t intr	: 1;				/* Command read/write action */
	};
} lcc_cmd_t;

typedef struct lcc_cmd_store_asic1
{
	uint8_t cmd_idx;
	uint16_t tid;
	uint16_t ucid;
	uint32_t m_bitmask;
	lcc_cmd_info_t cmd;
	struct lcc_cmd_store_asic1 *next;
} lcc_cmd_store_asic1_t;

typedef struct
{
	uint8_t asic2_intr_num;
	uint8_t asic3_intr_num;
} lcc_cmd_interrupt_queue_t;

/**
 * @brief lcc_command_status_t typedef
 * LCC command status
 */
typedef enum lcc_cmd_status
{
	LCC_CMD_UNSUCCESS							= 0x00,
	LCC_CMD_SUCCESS								= 0x01,
	LCC_CMD_PENDING								= 0x02,
	LCC_CMD_INVALID_ARG							= 0x04,
	LCC_CMD_ERROR_INVALID_MBITMASK				= 0x08,
	LCC_CMD_ERROR_ASIC_UNAVAILABLE				= 0x10,
	LCC_CMD_ERROR_MODULE_FAULT					= 0x20
} lcc_cmd_status_t;

/* Exported functions --------------------------------------------------------*/
/* Group 0x0000 */
extern void cmd_cam_module_open(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_streaming(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_burst_requested(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_burst_available(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_burst_actual(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_snapshot_uuid(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_snapshot_tid(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_command_status(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_status(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_resolution(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_sensitivity(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_exposure_time(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_fps(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_focal_length(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_focus_distance(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_lens_position(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_mirror_position(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_uuid(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_type(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_i2c_forward(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_roi(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_roi_transfer(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_roi_calibration(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_zoom_factor(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_module_mirror_calib(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_i2c_speed(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_spi_speed(lcc_cmd_t *in, lcc_cmd_t *out);

/* Group 0x1000 */
extern void cmd_light_active_ucid(lcc_cmd_t *in, lcc_cmd_t *out);

/* Group 0x0200 */
extern void cmd_asic_light_protocol(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_fw_version(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_calib_version(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_cfg_version(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_fw_checksum(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_cfg_checksum(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_status(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_config(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_log_ctrl(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_tmpx(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_cmd_dump(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_module_metadata(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_device_calibration(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_pwr_ctrl(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_cam_asic_intr_src(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_flash_light(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_tof(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_gyro(lcc_cmd_t *in, lcc_cmd_t *out);
#if (ASIC_NUM == ASIC1)
extern void cmd_asic_control(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_power_info(lcc_cmd_t *in, lcc_cmd_t *out);
#endif
extern void cmd_asic_calib_data(lcc_cmd_t *in, lcc_cmd_t *out);
extern void cmd_asic_pzt_pwr_ctrl(lcc_cmd_t *in, lcc_cmd_t *out);

/**
 * @brief task_lcc_cmd
 * Handle all LCC commands
 * @param vParameter: user data passing to task
 * @return None
 */
void task_lcc_cmd(void *vParameter);

/**
 * @brief lcc_cmd_init
 */
void lcc_cmd_init(void);

#if (ASIC_NUM == ASIC1)
void asic_cmd_delete_msg(lcc_cmd_store_asic1_t *in);
lcc_cmd_store_asic1_t *asic_cmd_pop_msg(uint16_t tid);
#endif

#ifdef __cplusplus
}
#endif
#endif /* __LCC_CMD_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
