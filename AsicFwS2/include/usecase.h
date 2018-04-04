/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    usecase.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-13-2016
 * @brief   This file contain declaration for use-cases in the Light CCB system.
 * Each use-case will be activated base on active ucid sent from host and the
 * CAM_STREAMING command.
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USECASE_H__
#define __USECASE_H__
/* Includes ------------------------------------------------------------------*/
#include "light_system.h"
#include "hal_axi2mipi.h"
#include "hal_mipi2axi.h"

/* Exported define------------------------------------------------------------*/
#define ATCM_SIZE		((uint32_t)0x00010000) /*!< ATCM (64KB) */
#define BTCM_SIZE		((uint32_t)0x00020000) /*!< BTCM (128KB) */

#define GET_2_BITS			0x03
#define GET_4_BITS			0x0f
#define GET_6_BITS			0x3f

#define		PLL_MULTIPLIER_CAM_REG				0x0306
#define		PRE_PLL_CLK_DIV_CAM_REG				0x0304
#define		VT_SYS_CLK_DIV_CAM_REG				0x0302
#define		VT_PIX_CLK_DIV_CAM_REG				0x0300
#define		ROW_SPEED_CAM_REG					0x3016
#define		AR1335_I2C_BUSY						0

/* line_length_pclk from register */
#define 	LINE_LENGTH_PCLK_CAM_REG			0x0342
/* Wait time (milliseconds) for MIPI to stop its operation */
#define		MIPI_STOP_WAIT_TIME					100
#ifdef EVT3_REWORK
#define I2C_WRITE_DELAY_SEC_ASIC1_1MHZ			(float)(-0.000025)
#define I2C_WRITE_DELAY_SEC_ASIC23_1MHZ			(float)(0.00010)
#define I2C_WRITE_DELAY_SEC_ASIC1_400KHZ		(float)(0.000025)
#define I2C_WRITE_DELAY_SEC_ASIC23_400KHZ		(float)(0.00025)
#define I2C_WRITE_DELAY_SEC_ASIC1_100KHZ		(float)(0.00045)
#define I2C_WRITE_DELAY_SEC_ASIC23_100KHZ		(float)(0.0006)
#else
#define I2C_WRITE_DELAY_SEC_ASIC1_1MHZ			(float)(-0.000025)
#define I2C_WRITE_DELAY_SEC_ASIC23_1MHZ			(float)(0.00010)
#define I2C_WRITE_DELAY_SEC_ASIC1_400KHZ		(float)(-0.000025)
#define I2C_WRITE_DELAY_SEC_ASIC23_400KHZ		(float)(0.00010)
#define I2C_WRITE_DELAY_SEC_ASIC1_100KHZ		(float)(0.00045)
#define I2C_WRITE_DELAY_SEC_ASIC23_100KHZ		(float)(0.0006)
#endif
/* Exported typedef  ---------------------------------------------------------*/
typedef enum streaming_csi
{
	CSI_0 = 1,
	CSI_1 = 2,
	CSI_ALL = 3
} streaming_csi_t;

typedef enum streaming_vc
{
	VCID_0,
	VCID_1,
	VCID_2,
	VCID_3,
	VCID_MAX
} streaming_vc_t;

typedef struct snapshot_info
{
	cam_typedef_t *pcam;
	uint8_t syncio_idx;
	uint8_t cam_idx;
	uint8_t inused;
	uint32_t offset;
	uint32_t syncio_lat;
	double it;
	double it_tolerance;
	double ft;
} snapshot_info_t;

typedef struct snapshot_timer_data
{
	uint8_t sinfo_cnt;
	snapshot_info_t **sinfo;
	EventGroupHandle_t evenhdl;
	uint64_t irq_cnt;
} snapshot_timer_data_t;
void mipi_rx_reset(mipi2axi_channel_t channel);
void mipi_tx_reset(axi2mipi_channel_t channel);
lcc_cmd_status_t ucid_preview_hdl(uint32_t mbitmask, uint8_t mnumber);
lcc_cmd_status_t ucid_hires_hdl(uint32_t mbitmask, uint8_t mnumber, uint8_t wait_asics);
void stop_mipi_for_cam(uint8_t cam_ch, uint8_t vc, uint8_t tx_ch);
void reconfig_spg_for_previewing_sensor(cam_typedef_t *pcam);
void tx_fix_pattern(uint32_t mbitmask, uint8_t mnumber);
void ucid_preview_hdl_snapshot(uint32_t mbitmask, uint8_t mnumber);
void light_header_test(uint32_t mbitmask);
#endif /*__USECASE_H__*/
