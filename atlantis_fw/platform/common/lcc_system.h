/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    lcc_system.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains header of __lcc_system
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCC_SYSTEM_H__
#define __LCC_SYSTEM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "board_config.h"
#include "hal_syncio.h"
#include "hal_pwm.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "hal_com.h"
#include "hal_ddr.h"
#include "hal_dma.h"
#include "hal_mipi2axi.h"
#include "hal_qspi.h"
#include "hal_spi.h"
#include "hal_timer.h"
#include "hal_wdt.h"

/* Exported define -----------------------------------------------------------*/
#if (ASIC_NUM == ASIC1)
	#define IS_ASIC1
#elif (ASIC_NUM == ASIC2)
	#define IS_ASIC2
#elif (ASIC_NUM == ASIC3)
	#define IS_ASIC3
#else
	#error "Invalid ASIC_NUM\n"
#endif
/* AsicFW version v1.0 */
#define ASIC_FW_VERSION						"1.0"
#define ASIC_FW_VERSION_MAJOR				1
#define ASIC_FW_VERSION_MINOR				0
#define ASIC_FW_VERSION_LEN					8
#if ((BUILD_ID < 0) || (BUILD_ID > 0xFFFF))
	#error "BUILD_ID is invalid\n\r";
#endif
#if ((LOG_VERBOSE_DEFAULT < 0) || (LOG_VERBOSE_DEFAULT > 0x1F))
	#error "LOG_VERBOSE_DEFAULT is invalid\n\r";
#endif

#define ASIC_LIGHT_PROTOCOL_MAJOR			0x00
#define ASIC_LIGHT_PROTOCOL_MINOR			0x00
#define ASIC_LIGHT_PROTOCOL_LEN				2

#define ASIC_CALIB_VERSION_MAJOR			0x01
#define ASIC_CALIB_VERSION_MINOR			0x00
#define ASIC_CALIB_VERSION_LEN				2

#define ASIC_CFG_VERSION_MAJOR				0x01
#define ASIC_CFG_VERSION_MINOR				0x00
#define ASIC_CFG_VERSION_LEN				2

#define ASIC_TEMPx_LEN						8

/** For development software debugging */
#ifndef LOG_VERBOSE
#define LOG_VERBOSE							STD_ON
#endif
#define LOG_I2C_MESSAGE						STD_ON
#define LOG_MSG								STD_ON
#define LOG_DEBUG							STD_ON
#define LOG_INFO							STD_ON
#define LOG_WARN							STD_ON
#define LOG_ERROR							STD_ON

#define LOG_SLOG_INFO_MAX_ENTRY				3
#define COMMAND_PROMPT						"#"
#define CONSOLE_LINE_LEN					64
#define LOG_CTRL_LEN						1

/* The SLOG_ID for driver/application definitions */
#define SLOG_ID_SLOG                        0
#define SLOG_ID_I2C_SLAVE					1
#define SLOG_ID_I2C_MASTER					2
#define SLOG_ID_SPI_SLAVE					3
#define SLOG_ID_SPI_MASTER					4
#define SLOG_ID_TIMER						5
#define SLOG_ID_PWM							6
#define SLOG_ID_MIPI_DEBUG					7
#define SLOG_ID_INA231						8
#define SLOG_ID_LPDDR3						9
#define SLOG_ID_TOF							10
#define SLOG_ID_GYRO						11
/* Application layer slog id from 30 */
#define SLOG_ID_LIGHT_SYSTEM				30
#define SLOG_ID_CAMERA_CTRL					31
#define SLOG_ID_LCC_CMD						32
#define SLOG_ID_LCC_CMD_BASE_0000			33
#define SLOG_ID_LCC_CMD_BASE_0200			34
#define SLOG_ID_LCC_CMD_BASE_1000			35
#define SLOG_ID_LCC_CMD_LOG					36
#define SLOG_ID_VCM_DRIVER					37
#define SLOG_ID_IMG_SENSOR					38
#define SLOG_ID_ACTUATOR_LOG				39
#define SLOG_ID_OPTICAL						40
#define SLOG_ID_CCB_CTRL					41
#define SLOG_ID_AF_CTRL						42
#define SLOG_ID_AF                          43

#define DEBUG_HEAP_MALLOC_FAILED			0
/* Exported typedef ----------------------------------------------------------*/
/* Typedef wrappers */
typedef hal_i2c_channel_t i2c_t;
typedef hal_pwm_channel_t pwm_t;
typedef hal_syncio_channel_t sync_t;
typedef uint8_t gpio_t;
/*
 * @brief ucid_t
 * Stream on UCID, the setting caching for each of UC
 */
typedef enum
{
	UC_DISABLED = 0x00,
	UC_UNKNOWN,
	UC_DEBUG,
	UC_PREVIEW,
	UC_VIDEO,
	UC_HIRES_CAPTURE,
	UC_FOCAL_STACKING,
	UC_HDR_CAPTURE,
	UC_RESERVE,
	UC_FTM_QUICK_CHECK,
	UC_FTM_CALIBRATION,
	UC_MAX
} ucid_t;

/* Exported Global variables--------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /* __LCC_SYSTEM_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
