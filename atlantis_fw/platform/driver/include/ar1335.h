/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    ar1335.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-6-2016
 * @brief   This file contains expand of the ar1335 driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AR1335_H__
#define __AR1335_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported define -----------------------------------------------------------*/

/* AR1335 Registers */

#define	PLL_MULTIPLIER_CAM_REG				0x0306
#define	PRE_PLL_CLK_DIV_CAM_REG				0x0304
#define	VT_SYS_CLK_DIV_CAM_REG				0x0302
#define	VT_PIX_CLK_DIV_CAM_REG				0x0300
#define	ROW_SPEED_CAM_REG					0x3016
#define LINE_LENGTH_PCLK_CAM_REG			0x0342
#define FLL_CAM_REG							0x0340
#define X_ODD_INC_CAM_REG					0x0382
#define Y_ODD_INC_CAM_REG					0x0386
#define X_OUTPUT_SIZE_CAM_REG				0x034C
#define Y_OUTPUT_SIZE_CAM_REG				0x034E
#define Y_ADDR_END_CAM_REG					0x034A
#define Y_ADDR_START_CAM_REG				0x0346
#define X_ADDR_END_CAM_REG					0x0348
#define X_ADDR_START_CAM_REG				0x0344
#define MIN_FB_LINES_CAM_REG				0x114A
#define CIT_CAM_REG							0x0202
#define CIT_MAX_MARGIN_CAM_REG				0x1006
#define PATTERN_GEN_CAM_REG					0x0600
#define IMG_ORIENTATION_CAM_REG				0x0101
#define AR1335_CUSTOMER_REV					0x31FE
#define PIXEL_ORDER_REG						0x0006
/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief msm_camera_i2c_reg_array_t
 * Send command to AR1335
 */
typedef struct msm_camera_i2c_reg_array
{
	uint16_t reg_addr;
	uint16_t reg_val;
} msm_camera_i2c_reg_array_t;
/**
 * @brief camera_reg_type_t
 * Flag to detect that the register arrangement is continuous or not
 */
typedef enum camera_reg_type
{
	CAM_REG_CONTINUOUS,
	CAM_REG_NONCONTINUOUS
} camera_reg_type_t;
/**
 * @brief msm_camera_reg_array_t
 * Array set up register AR1335
 */
typedef struct cam_reg_array
{
	msm_camera_i2c_reg_array_t *regs;
	uint16_t reg_size;
	uint16_t data_size;
	camera_reg_type_t reg_type;
} cam_reg_array_t;

/**
 * @brief msm_camera_stream_type_t
 * Set up camera stream
 */
typedef struct msm_camera_stream_type
{
	cam_reg_array_t *config;
	uint16_t width;
	uint16_t height;
	uint16_t fps;
	uint8_t sizeofconfig;
} msm_camera_stream_type_t;
typedef struct iso_gain
{
	float total_gain;
	uint16_t iso;
} iso_t;
/* Export variables ----------------------------------------------------------*/
extern msm_camera_i2c_reg_array_t enable_streaming[];
extern msm_camera_i2c_reg_array_t analog_control_on[];
extern msm_camera_i2c_reg_array_t analog_control_off[];
extern msm_camera_i2c_reg_array_t analog_control_reset[];
extern msm_camera_i2c_reg_array_t stop_reg_array[];
extern msm_camera_i2c_reg_array_t groupon_reg_array[];
extern msm_camera_i2c_reg_array_t groupoff_reg_array[];
extern msm_camera_i2c_reg_array_t slave_mode_control_reg_array[];
extern msm_camera_i2c_reg_array_t slave_mode_enable_reg_array[];
extern msm_camera_i2c_reg_array_t slave_mode_disable_reg_array[];
extern msm_camera_i2c_reg_array_t reset_reg_array[];
extern msm_camera_i2c_reg_array_t correction_recommend[];
extern msm_camera_i2c_reg_array_t pixel_timing_recommended[];
extern msm_camera_i2c_reg_array_t analog_setup_recommended_0[];
extern msm_camera_i2c_reg_array_t mipi_timing_max[];
extern msm_camera_i2c_reg_array_t mipi_timing_880M[];
extern msm_camera_i2c_reg_array_t mipi_timing_880M_DPCM8[];
extern msm_camera_i2c_reg_array_t mipi_timing_recommended[];
extern msm_camera_i2c_reg_array_t pll_setup_max[];
extern msm_camera_i2c_reg_array_t pll_setup_880M_DPCM8[];
extern msm_camera_i2c_reg_array_t pll_setup_880M[];
extern msm_camera_i2c_reg_array_t pll_setup_recommended[];
extern msm_camera_i2c_reg_array_t defect_correction[];
extern msm_camera_i2c_reg_array_t res13_4208_3120_recommended_0[];
extern msm_camera_i2c_reg_array_t res13_4208_3120_recommended_1[];
extern msm_camera_i2c_reg_array_t res13_4224_3136_border_reg_array[];
extern msm_camera_i2c_reg_array_t res13_4208_3120_DPCM8_reg_array[];
extern msm_camera_i2c_reg_array_t res13_4208_3120_reg_array[];
extern msm_camera_i2c_reg_array_t res3_4208_3120_Ybin2_Xscale2_reg_array[];
extern msm_camera_i2c_reg_array_t res1080_3840_2160_Ybin2_Xscale2_reg_array[];
extern msm_camera_i2c_reg_array_t res3_4208_3120_Ybin2_XBin2_reg_array[];
extern msm_camera_i2c_reg_array_t res1080_3840_2160_Ybin2_XBin2_reg_array[];
extern msm_camera_i2c_reg_array_t res720_3840_2160_Yskip3_XScale3_reg_array[];
extern msm_camera_i2c_reg_array_t res720_3840_2160_Ybin3_Xbin3_reg_array[];
extern msm_camera_i2c_reg_array_t res4k_3840_2160_reg_array[];
extern msm_camera_i2c_reg_array_t res4k_4096_2160_reg_array[];
extern cam_reg_array_t resolution_testing_reg_array[];
extern cam_reg_array_t common_stream[];
extern cam_reg_array_t stream_13m_30fps_border[];
extern cam_reg_array_t stream_13m_30fps_DPCM8[];
extern msm_camera_i2c_reg_array_t stream_13m_30fps_custom[];
extern cam_reg_array_t stream_13m_30fps[];
extern msm_camera_i2c_reg_array_t stream_13m_24fps_custom[];
extern cam_reg_array_t stream_13m_24fps[];
extern msm_camera_i2c_reg_array_t stream_13m_15fps_custom[];
extern cam_reg_array_t stream_13m_15fps[];
extern cam_reg_array_t stream_3M_30fps_HQ[];
extern cam_reg_array_t stream_1080p_30fps_HQ[];
extern msm_camera_i2c_reg_array_t stream_1080p_60fps_HQ_custom[];
extern cam_reg_array_t stream_1080p_60fps_HQ[];
extern cam_reg_array_t stream_3M_30fps_LP[];
extern cam_reg_array_t stream_1080p_30fps_LP[];
extern msm_camera_i2c_reg_array_t stream_1080p_60fps_LP_custom[];
extern cam_reg_array_t stream_1080p_60fps_LP[];
extern cam_reg_array_t stream_720p_30fps_HQ[];
extern msm_camera_i2c_reg_array_t stream_720p_60fps_HQ_custom[];
extern cam_reg_array_t stream_720p_60fps_HQ[];
extern cam_reg_array_t stream_720p_30fps_LP[];
extern msm_camera_i2c_reg_array_t stream_720p_60fps_LP_custom[];
extern cam_reg_array_t stream_720p_60fps_LP[];
extern cam_reg_array_t stream_4k_uhd[];
extern cam_reg_array_t stream_4k_cinema[];
extern msm_camera_stream_type_t stream_types[];
extern cam_reg_array_t cam_open_default[];
extern uint32_t cam_open_default_size;
extern uint32_t stream_types_size;
extern iso_t iso_table[];
extern uint32_t iso_table_size;
extern uint32_t config_13MP_1200Mbs_size;
extern cam_reg_array_t config_4160x3120_30fps_mipi_1200Mbs[];
extern cam_reg_array_t config_4208x3120_30fps_mipi_1200Mbs[];
extern uint32_t config_13MP_400Mbs_size;
extern cam_reg_array_t config_4160x3120_10fps_mipi_400Mbs[];
extern cam_reg_array_t config_4208x3120_10fps_mipi_400Mbs[];
extern uint32_t defect_correction_size;
extern uint32_t	pan_defect_correction_size;
extern msm_camera_i2c_reg_array_t defect_correction_panchromatic[];
#ifdef __cplusplus
}
#endif
#endif /* __AR1335_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
