/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    img_sensor.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-07-2016
 * @brief   This file contains expand for camera image sensor driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "assert.h"
#include "log.h"
#include "os.h"
#include "ar1335.h"
#include "img_sensor.h"
#include "light_system.h"
#include "usecase.h"
#include <math.h>
/* Private define-------------------------------------------------------------*/
#define SLOGF_ID				SLOG_ID_IMG_SENSOR
#define CONVERT_TO_PLL(exposure, llpclk, vtpixclk)\
					(uint32_t)((exposure / llpclk)  * 2 * vtpixclk * 1000000)

/* Private typedef -----------------------------------------------------------*/
/*
 *
 *@gain_t struct
 * This is struct to calculate gain code
 */

typedef struct
	{
		union
		{
			struct
			{
				uint16_t fine_gain    : 4;
				uint16_t coarse_gain  : 3;
				uint16_t digital_gain : 9;
			};
			uint16_t gain_code;
		};
	}gain_t;
/* Private macro -------------------------------------------------------------*/
/* Private variables----------------------------------------------------------*/
/* Private function-----------------------------------------------------------*/
static uint8_t set_reg_continuous(img_sensor_t *img_sensor, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr,  img_sensor_data_mode_t reg_size);
static uint8_t set_reg_noncontinuous(img_sensor_t *img_sensor, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr,  img_sensor_data_mode_t reg_size);
void img_sensor_ctrl_reg_group(img_sensor_t *img_sensor, uint8_t state);
static void config_camera_power(cam_power_mode_t is_enable);
int get_min_fll(img_sensor_t *img_sensor);

int get_min_fll(img_sensor_t *img_sensor)
{
	uint16_t min_fll = 0;
	uint16_t y_addr_end = img_sensor_read_reg(img_sensor,
											Y_ADDR_END_CAM_REG, TWO_BYTES);
	uint16_t y_addr_start = img_sensor_read_reg(img_sensor,
											Y_ADDR_START_CAM_REG, TWO_BYTES);
	uint16_t y_odd_inc = img_sensor_read_reg(img_sensor,
											Y_ODD_INC_CAM_REG, TWO_BYTES);
	uint16_t min_fb_lines = img_sensor_read_reg(img_sensor,
											MIN_FB_LINES_CAM_REG, TWO_BYTES);
	uint8_t yskip = 1;
	switch(y_odd_inc)
	{
		case 1:
			yskip = 1;
			break;
		case 3:
			yskip = 2;
			break;
		case 5:
			yskip = 3;
			break;
		case 7:
			yskip = 4;
			break;
		default:
			yskip = 1;
			y_odd_inc = 1;
			break;
	}
	min_fll = (((y_addr_end - y_addr_start + y_odd_inc) / yskip) + min_fb_lines);
	return min_fll;
}
static void img_sensor_y_addr_add_one(img_sensor_t *img_sensor)
{
    uint32_t y_addr_s = img_sensor_read_reg(img_sensor, Y_ADDR_START_CAM_REG, TWO_BYTES);

    uint32_t y_addr_e = img_sensor_read_reg(img_sensor, Y_ADDR_END_CAM_REG, TWO_BYTES);

    /* increment the y start and y end if not already incremented
     * By default y start is even and y end is odd. We do this to fix the
     * Bayer shift introduced due to vertical flip */
    if ((y_addr_s & 0x1) == 0  || (y_addr_e & 0x1) == 1)
    {
    	y_addr_s += 1;
    	y_addr_e += 1;
    	img_sensor_write_reg(img_sensor, Y_ADDR_START_CAM_REG, y_addr_s, TWO_BYTES);
    	img_sensor_write_reg(img_sensor, Y_ADDR_END_CAM_REG, y_addr_e, TWO_BYTES);
    }

}
/* Exported functions --------------------------------------------------------*/
int img_sensor_flip_capture(img_sensor_t *img_sensor, flip_mode_t flip)
{
	i2cm_error_t ret = img_sensor_write_reg(img_sensor, IMG_ORIENTATION_CAM_REG, flip, ONE_BYTE);
	uint32_t y_addr_s = img_sensor_read_reg(img_sensor, Y_ADDR_START_CAM_REG, TWO_BYTES);
	uint32_t y_addr_e = img_sensor_read_reg(img_sensor, Y_ADDR_END_CAM_REG, TWO_BYTES);
	if ((y_addr_s & 0x1) != 0  || (y_addr_e & 0x1) != 1)
	{
		y_addr_s -= 1;
		y_addr_e -= 1;
		img_sensor_write_reg(img_sensor, Y_ADDR_START_CAM_REG, y_addr_s, TWO_BYTES);
		img_sensor_write_reg(img_sensor, Y_ADDR_END_CAM_REG, y_addr_e, TWO_BYTES);
	}
	return ret;
}

int img_sensor_flip_preview(img_sensor_t *img_sensor, flip_mode_t flip)
{
    if ((flip & FLIP_HORIZONTAL) != 0)
    {
        SLOGF(SLOG_WARN, "Horizontal flip not supported");
        flip &= ~FLIP_HORIZONTAL;
    }

    img_sensor_write_reg(img_sensor, IMG_ORIENTATION_CAM_REG, flip, ONE_BYTE);

    if ((flip & FLIP_VERTICAL) != 0)
        img_sensor_y_addr_add_one(img_sensor);

	return 1;
	/*
	i2cm_error_t ret = img_sensor_write_reg(img_sensor, IMG_ORIENTATION_CAM_REG, flip, ONE_BYTE);
	return ret;
	*/
}

int img_sensor_pattern_mode(img_sensor_t *img_sensor, pattern_mode_t pattmode)
{
	img_sensor_write_reg(img_sensor, PATTERN_GEN_CAM_REG, pattmode, TWO_BYTES);
	return 1;
}

uint8_t img_sensor_read_pixel_order(img_sensor_t *img_sensor)
{
	uint8_t reg = img_sensor_read_reg(img_sensor, PIXEL_ORDER_REG, ONE_BYTE);
	return reg;
}
int img_sensor_send_config(img_sensor_t *img_sensor, cam_reg_array_t *config,
														uint32_t num_config)
{
	int i = 0;
	int ret = 0;
	for(i = 0; i < num_config; i++)
	{
		if(config[i].reg_type == CAM_REG_CONTINUOUS)
		{
			ret = set_reg_continuous(img_sensor,
									config[i].reg_size,
									config[i].regs,
									config[i].data_size);
		}
		else
		{
			ret = set_reg_noncontinuous(img_sensor,
									config[i].reg_size,
									config[i].regs,
									config[i].data_size);
		}
		if(!ret)
			return 0;
	}
	return 1;

}
int img_sensor_init(img_sensor_t *img_sensor)
{
	img_sensor->cmd = assert_malloc(img_sensor->cmd, sizeof(img_sensor_command_t));
	img_sensor->cmd->data = assert_malloc(img_sensor->cmd->data, 8 * sizeof(uint8_t));
	img_sensor->cmd->len = 0;
	return 1;
}

int img_sensor_deinit(img_sensor_t *img_sensor)
{
	return 1;
}

int img_sensor_open(img_sensor_t *img_sensor, uint8_t mode)
{
	int ret = 0;

	config_camera_power(PWR_ON);

	hal_gpio_t gpio;
	gpio.port = GPIO_PORT(img_sensor->standby);
	gpio.pin = GPIO_PIN(img_sensor->standby);
	gpio.direction = GPIO_DIR_OUT;
	hal_gpio_init(&gpio);

	if(mode == SW_STANDBY)
	{
		hal_gpio_set_high(&gpio);
		/* Wait 4 ticks for the power to stable */
		vTaskDelay(4);
		ret = img_sensor_send_config(img_sensor,
									config_4208x3120_30fps_mipi_1200Mbs,
									config_13MP_1200Mbs_size);
		if(!ret)
			return 0;
		/* Disable clock gating LSC */
		uint16_t val = img_sensor_read_reg(img_sensor, 0x30FE, TWO_BYTES);
		val &= ~((uint32_t)(BIT4));
		img_sensor_write_reg(img_sensor, 0x30FE, val, TWO_BYTES);

		img_sensor->sensor_type = get_image_sensor_type(img_sensor);

		if (img_sensor->sensor_type == AR1335_COLOR_SENSOR)
		{
			set_reg_noncontinuous(img_sensor, defect_correction_size,
					defect_correction, TWO_BYTES);

		}
		else
		{
			set_reg_noncontinuous(img_sensor,
					pan_defect_correction_size,
					defect_correction_panchromatic, TWO_BYTES);
		}
		/* Store the default FLL for later use */
		img_sensor->default_fll = img_sensor_read_reg(img_sensor,
													FLL_CAM_REG,
													TWO_BYTES);
		img_sensor->default_llpclk = img_sensor_read_reg(img_sensor,
													LINE_LENGTH_PCLK_CAM_REG,
													TWO_BYTES);
		img_sensor->x_size = img_sensor_read_reg(img_sensor,
													X_OUTPUT_SIZE_CAM_REG,
													TWO_BYTES);
		img_sensor->y_size = img_sensor_read_reg(img_sensor,
													Y_OUTPUT_SIZE_CAM_REG,
													TWO_BYTES);
	}
	else if(mode == HW_STANDBY)
	{
		/* TODO: investigate how to set sensor in HW standby mode
		 * Current gpio pull low not really work
		 * It will make the next preview wrong
		 */
		hal_gpio_set_low(&gpio);
		return 1;
	}
	else
	{
		log_error("Mode 0x%x is not supported \r\n");
		return 0;
	}

	return 1;
}

int img_sensor_close(img_sensor_t *img_sensor)
{
	/* Due to the ToF_xShut pin using common pin with CAM_PWR_EN,
	 * so for temporarily, the CAM_PWR_EN will be not pulled down.
	 * FIXME: Need to re-implement later */
	// config_camera_power(PWR_OFF);
	return 1;
}

int img_sensor_stream_on(img_sensor_t *img_sensor, cam_sensor_mode_t mode)
{
	if(I2C_MODE == mode)
	{
		img_sensor_write_reg(img_sensor, 0x3F3C, 0x0003, TWO_BYTES);
		img_sensor_write_reg(img_sensor, 0x0100, 0x01, ONE_BYTE);
	}
	else
	{
		img_sensor_write_reg(img_sensor, 0x301A, 0x031C, TWO_BYTES);
	}
	return 1;
}

int img_sensor_stream_off(img_sensor_t *img_sensor, cam_sensor_mode_t mode)
{
	if(I2C_MODE == mode)
	{
		img_sensor_write_reg(img_sensor, 0x3F3C, 0x0002, TWO_BYTES);
		img_sensor_write_reg(img_sensor, 0x3FE0, 0x0001, TWO_BYTES);
		img_sensor_write_reg(img_sensor, 0x0100, 0x00, ONE_BYTE);
		img_sensor_write_reg(img_sensor, 0x3FE0, 0x0000, TWO_BYTES);
	}
	else
	{
		img_sensor_write_reg(img_sensor, 0x301A, 0x0318, TWO_BYTES);
	}
	return 1;
}

/**
 * @brief img_sensor_set_sensitivity
 * The function set sensitivity to img sensor
 * @param	img_sensor - pointer to img_sensor
 * 			sensitivity - sensitivity value will be set
 * @return int
 */
int img_sensor_set_sensitivity(img_sensor_t *img_sensor, uint32_t sensitivity)
{
	/*TODO: need a mapping from ISO value to register value */
	img_sensor_ctrl_reg_group(img_sensor, ON);
	img_sensor_write_reg(img_sensor, CAM_REG_SENSITIVITY, sensitivity, TWO_BYTES);
	/* Close Control Group in camera module */
	img_sensor_ctrl_reg_group(img_sensor, OFF);

	return 1;
}

/**
 * @brief img_sensor_set_exposure_time
 * The function set sensitivity to img sensor
 * @param	img_sensor - pointer to img_sensor
 * 			exposure - exposure value will be set
 * @return
 */
int img_sensor_set_exposure_time(void *cam, uint64_t exposure)
{
	cam_typedef_t *pcam = (cam_typedef_t *)cam;
	uint32_t vt_pix_clk = img_sensor_get_vt_pix_clk_mhz(pcam->image);
	SLOGF(SLOG_DEBUG, "vt_pixel_clock_freq_mhz: %d", vt_pix_clk);

	double exposure_in_second = (double)(exposure * 1.0) / (double)(1000*1000*1000);
	uint32_t llpclk = (uint32_t)img_sensor_read_reg(pcam->image,
										LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	uint32_t cit = CONVERT_TO_PLL(exposure_in_second, pcam->image->default_llpclk, vt_pix_clk);
#if(LOG_VERBOSE == STD_ON)
	uint32_t cur_fll = img_sensor_read_reg(pcam->image, FLL_CAM_REG, TWO_BYTES);
#endif
	if(cit >= MAX_SENSOR_REG_VAL)
	{
		llpclk = 0x1FFE;
		cit = CONVERT_TO_PLL(exposure_in_second, llpclk, vt_pix_clk);
		while(cit >= MAX_SENSOR_REG_VAL && llpclk < MAX_SENSOR_REG_VAL)
		{
			llpclk *= 2;
			cit = CONVERT_TO_PLL(exposure_in_second, llpclk, vt_pix_clk);
		}
		if(llpclk >= MAX_SENSOR_REG_VAL)
		{
			SLOGF(SLOG_ERROR, "Unsupported exposure time 0x%x%x (ns)",
								(unsigned int)(exposure >> 32),
								(unsigned int)(exposure));
			return 0;
		}
		/*img_sensor_ctrl_reg_group(pcam->image, ON);*/
		img_sensor_write_reg(pcam->image, LINE_LENGTH_PCLK_CAM_REG, llpclk, TWO_BYTES);
		img_sensor_write_reg(pcam->image, FLL_CAM_REG, cit + 1, TWO_BYTES);
		if((pcam->settings->stream[0] & CAM_STREAM_ENABLE) &&
			light_system->active_ucid == UCID_PREVIEW)
		{
			reconfig_spg_for_previewing_sensor(pcam);
		}
		img_sensor_write_reg(pcam->image, CIT_CAM_REG, cit, TWO_BYTES);
		/*img_sensor_ctrl_reg_group(pcam->image, OFF);*/
	}
	else
	{
		/*img_sensor_ctrl_reg_group(pcam->image, ON);*/
		img_sensor_write_reg(pcam->image, LINE_LENGTH_PCLK_CAM_REG,
										pcam->image->default_llpclk, TWO_BYTES);
		if(cit >= pcam->image->default_fll)
			img_sensor_write_reg(pcam->image, FLL_CAM_REG, cit + 1, TWO_BYTES);
		else
			img_sensor_write_reg(pcam->image, FLL_CAM_REG,
										pcam->image->default_fll, TWO_BYTES);
		if((pcam->settings->stream[0] & CAM_STREAM_ENABLE) &&
									light_system->active_ucid == UCID_PREVIEW)
		{
			reconfig_spg_for_previewing_sensor(pcam);
		}
		img_sensor_write_reg(pcam->image, CIT_CAM_REG, cit, TWO_BYTES);
		/*img_sensor_ctrl_reg_group(pcam->image, OFF);*/
	}
	/*if((pcam->settings->stream[0] & CAM_STREAM_ENABLE) &&
				light_system->active_ucid == UCID_PREVIEW)
	{
		hal_syncio_trigger();
	}*/
#if(LOG_VERBOSE == STD_ON)
	llpclk = (uint32_t)img_sensor_read_reg(pcam->image,
										LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	cur_fll = img_sensor_read_reg(pcam->image, FLL_CAM_REG, TWO_BYTES);
	SLOGF(SLOG_DEBUG, "line_length_pck: 0x%x", llpclk);
	SLOGF(SLOG_DEBUG, "frame_length_lines: 0x%x", cur_fll);
	SLOGF(SLOG_DEBUG,"Exposure: 0x%x%x (ns) -> 0x%x (CIT)",
								(unsigned int)(exposure >> 32),
								(unsigned int)(exposure),
								cit);
#endif

	return 1;
}

int img_sensor_set_fps(void *cam, uint16_t fps)
{
	cam_typedef_t *pcam = (cam_typedef_t *)cam;
	uint32_t vt_pix_clk = img_sensor_get_vt_pix_clk_mhz(pcam->image);
	SLOGF(SLOG_DEBUG, "vt_pixel_clock_freq_mhz: %d", vt_pix_clk);
	uint32_t llpclk = (uint32_t)img_sensor_read_reg(pcam->image,
											LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	uint32_t fll = img_sensor_read_reg(pcam->image, FLL_CAM_REG, TWO_BYTES);

	uint32_t new_fll = fll;
	/* Current frame rate calculation */
	double frame_rate = (2 * vt_pix_clk * MHZ_TO_HZ) / (llpclk * fll * 1.0);

	if(llpclk == pcam->image->default_llpclk)
	{
		/* User was not change exposure */
		/* Frame rate should use the default */
		frame_rate = (2 * vt_pix_clk * MHZ_TO_HZ) /
			(pcam->image->default_llpclk * pcam->image->default_fll * 1.0);
	}
	SLOGF(SLOG_DEBUG, "Current frame rate (x100): %d", (int)(frame_rate * 100));
	/* Tolerance of 1 frame */
	if(fps > (frame_rate + 1))
	{
		SLOGF(SLOG_ERROR, "Unsupported FPS %d with current setting", fps);
		return 0;
	}

	new_fll = (2 * vt_pix_clk * MHZ_TO_HZ) / (llpclk * fps * 1.0);
	if(new_fll <= MAX_SENSOR_REG_VAL)
	{
		/*img_sensor_ctrl_reg_group(pcam->image, ON);*/
		img_sensor_write_reg(pcam->image, FLL_CAM_REG, new_fll, TWO_BYTES);
		if((pcam->settings->stream[0] & CAM_STREAM_ENABLE) &&
					light_system->active_ucid == UCID_PREVIEW)
		{
			reconfig_spg_for_previewing_sensor(pcam);
		}
		/*img_sensor_ctrl_reg_group(pcam->image, OFF);*/
		SLOGF(SLOG_DEBUG, "%s:%d FLL = 0x%x", __FUNCTION__, __LINE__, (int)new_fll);
	}
#if 1
	else
	{
		SLOGF(SLOG_ERROR, "Unsupported FPS %d with current setting", fps);
		return 0;
	}
#else
	else if(new_fll > MAX_SENSOR_REG_VAL)
	{
		new_fll = MAX_SENSOR_REG_VAL;
		/* FLL can't be increased */
		/* Increase LLPCLK instead */
		llpclk = (2 * vt_pix_clk * MHZ_TO_HZ) / (new_fll * fps * 1.0);
		if(llpclk >= MAX_SENSOR_REG_VAL)
		{
			SLOGF(SLOG_ERROR, "Unsupported FPS %d", fps);
			return 0;
		}
		/*img_sensor_ctrl_reg_group(pcam->image, ON);*/
		img_sensor_write_reg(pcam->image, FLL_CAM_REG, new_fll, TWO_BYTES);
		img_sensor_write_reg(pcam->image, LINE_LENGTH_PCLK_CAM_REG,
														llpclk, TWO_BYTES);
		SLOGF(SLOG_DEBUG, "%s:%d FLL = 0x%x", __FUNCTION__, __LINE__, (int)new_fll);
		SLOGF(SLOG_DEBUG, "%s:%d LLPCLK = 0x%x", __FUNCTION__, __LINE__, (int)llpclk);
		if((pcam->settings->stream[0] & CAM_STREAM_ENABLE) &&
					light_system->active_ucid == UCID_PREVIEW)
		{
			reconfig_spg_for_previewing_sensor(pcam);
		}
		/*img_sensor_ctrl_reg_group(pcam->image, OFF);*/
	}
	else if(new_fll < get_min_fll(pcam->image))
	{
		new_fll = pcam->image->default_fll;
		llpclk = (2 * vt_pix_clk * MHZ_TO_HZ) / (new_fll * fps * 1.0);
		if(llpclk >= MAX_SENSOR_REG_VAL)
		{
			SLOGF(SLOG_ERROR, "Unsupported FPS %d", fps);
			return 0;
		}
		/*img_sensor_ctrl_reg_group(pcam->image, ON);*/
		img_sensor_write_reg(pcam->image, FLL_CAM_REG, new_fll, TWO_BYTES);
		img_sensor_write_reg(pcam->image, LINE_LENGTH_PCLK_CAM_REG,
														llpclk, TWO_BYTES);
		if((pcam->settings->stream[0] & CAM_STREAM_ENABLE) &&
					light_system->active_ucid == UCID_PREVIEW)
		{
			reconfig_spg_for_previewing_sensor(pcam);
		}
		/*img_sensor_ctrl_reg_group(pcam->image, OFF);*/
		SLOGF(SLOG_DEBUG, "%s:%d FLL = 0x%x", __FUNCTION__, __LINE__, (int)new_fll);
		SLOGF(SLOG_DEBUG, "%s:%d LLPCLK = 0x%x", __FUNCTION__, __LINE__, (int)llpclk);
	}
#endif
	/*if((pcam->settings->stream[0] & CAM_STREAM_ENABLE) &&
					light_system->active_ucid == UCID_PREVIEW)
	{
		hal_syncio_trigger();
	}*/
	return 1;
}

int img_sensor_config_mode(img_sensor_t *img_sensor,
												cam_sensor_mode_t sensor_mode)
{
	if(SLAVE_MODE == sensor_mode)
	{
		/* SW Standby */
		img_sensor_write_reg(img_sensor, 0x301A, 0x0318, TWO_BYTES);
		/* Trigger pin select */
		img_sensor_write_reg(img_sensor, 0x3026, 0xFD7B, TWO_BYTES);
		/* Slave mode control */
		img_sensor_write_reg(img_sensor, 0x3158, 0xA000, TWO_BYTES);
	}
	else
	{
		/* SW Standby */
		img_sensor_write_reg(img_sensor, 0x301A, 0x0318, TWO_BYTES);
		/* Clear trigger pin */
		img_sensor_write_reg(img_sensor, 0x3026, 0xFFFB, TWO_BYTES);
		/* Clear slave mode */
		img_sensor_write_reg(img_sensor, 0x3158, 0x0000, TWO_BYTES);
	}
	return 0;
}

/**
 * @brief img_sensor_cropping
 * The function set img sensor cropping
 * @param	img_sensor - pointer to img_sensor
 * 			x_position - x co-ordinate
 * 			y_position - y co-ordinate
 * 			x_width - rectangle width
 * 			y_width - rectangle height
 * @return int
 */
int img_sensor_cropping(img_sensor_t *img_sensor, uint16_t x_position,
					uint16_t y_position, uint16_t x_width, uint16_t y_width, uint16_t scale)
{
	/* x_output_size and y_output_size must be even number – this is enforced in hardware */
	/* x_width: x_width if even or x_width - 1 if odd */
	x_width = x_width % 2 ? x_width - 1 : x_width;
	/* y_width: y_width if even or y_width - 1 if odd */
	y_width = y_width % 2 ? y_width - 1 : y_width;
	/* X_ADDR_START: x_position if even or x_position + 1 if odd */
	x_position = x_position % 2 ? x_position + 1 : x_position;
	/* Y_ADDR_START: y_position if even or y_position + 1 if odd */
	y_position = y_position % 2 ? y_position + 1 : y_position;
	/* X_ADDR_END: x_position + x_width if odd and plus 1 if even */
	uint16_t x_end = x_position + x_width;
	x_end = x_end % 2 ? x_end : x_end - 1;
	/* Y_ADDR_END: y_position + y_width if odd and plus 1 if even */
	uint16_t y_end = y_position + y_width;
	y_end = y_end % 2 ? y_end : y_end - 1;
	SLOGF(SLOG_DEBUG, "x_end %d, y_end %d", x_end, y_end);

	if(x_position > CAM_REG_X_ADDR_MAX || x_end > CAM_REG_X_ADDR_MAX
		|| y_position > CAM_REG_Y_ADDR_MAX || y_end > CAM_REG_Y_ADDR_MAX)
	{
		SLOGF(SLOG_ERROR, "Address is not valid");
		return 0;
	}

	img_sensor_write_reg(img_sensor, CAM_REG_X_ADDR_START, x_position,
						TWO_BYTES);
	img_sensor_write_reg(img_sensor, CAM_REG_Y_ADDR_START, y_position,
						TWO_BYTES);
	img_sensor_write_reg(img_sensor, CAM_REG_X_ADDR_END, x_end, TWO_BYTES);
	img_sensor_write_reg(img_sensor, CAM_REG_Y_ADDR_END, y_end, TWO_BYTES);
	/* Set image width, height */
	x_width = (x_width + scale - 1) / scale;
	x_width = (x_width + 15) / 16 * 16;  // needs to be multiple of 16 as per sensor spec
	y_width = (y_width + scale - 1) / scale;
	img_sensor_write_reg(img_sensor, X_OUTPUT_SIZE_CAM_REG, x_width,
						TWO_BYTES);
	img_sensor_write_reg(img_sensor, Y_OUTPUT_SIZE_CAM_REG, y_width,
						TWO_BYTES);
	img_sensor->x_size = x_width;
	img_sensor->y_size = y_width;
	return 1;
}

uint16_t img_sensor_read_reg(img_sensor_t *img_sensor, uint16_t reg,
											img_sensor_data_mode_t datamode)
{
	uint8_t len = BYTE_ADDR16;
	uint8_t buf[2];
	uint16_t data = 0;
	i2cm_error_t ret = I2CM_ERROR_NONE;
	if(datamode == TWO_BYTES)
		len = WORD_ADDR16;
	ret = i2cm.read(img_sensor->i2c_dev, len, IMAGE_SENSOR_ADDRESS, reg, buf);
	if(I2CM_ERROR_TRANSCEIVED == ret)
	{

		data = buf[0];
		if(datamode == TWO_BYTES)
			data = (data << 8) | buf[1];
		return data;
	}
	SLOGF(SLOG_ERROR, "Can't read register: %x", reg);
	return 0;
}

int img_sensor_write_reg(img_sensor_t *img_sensor, uint16_t reg, uint16_t data,
											img_sensor_data_mode_t datamode)
{
	uint8_t len = BYTE_ADDR16;
	uint8_t reg_val[2];
	reg_val[0] = (uint8_t)data;
	if(datamode == TWO_BYTES)
	{
		len = WORD_ADDR16;
		reg_val[0] = (data >> 8) & 0xFF;
		reg_val[1] = data & 0xFF;
	}
	return i2cm.write(img_sensor->i2c_dev, len, IMAGE_SENSOR_ADDRESS,
														reg, (uint8_t *)reg_val);
}
float img_sensor_get_vt_pix_clk_mhz(img_sensor_t *img_sensor)
{
	uint32_t vt_pix_clk = 0;
	uint16_t pll_multiplier1 = 0;
	uint16_t pre_pll_clk_div1 = 0;
	uint16_t vt_sys_clk_div = 0;
	uint16_t vt_pix_clk_div = 0;
	uint16_t row_speed = 0;

	pll_multiplier1 = img_sensor_read_reg(img_sensor,
										PLL_MULTIPLIER_CAM_REG,
										TWO_BYTES);
	pll_multiplier1 &= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);

	pre_pll_clk_div1 = img_sensor_read_reg(img_sensor,
										PRE_PLL_CLK_DIV_CAM_REG,
										TWO_BYTES);
	pre_pll_clk_div1 &= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);

	vt_sys_clk_div = img_sensor_read_reg(img_sensor,
										VT_SYS_CLK_DIV_CAM_REG,
										TWO_BYTES);
	vt_pix_clk_div = img_sensor_read_reg(img_sensor,
										VT_PIX_CLK_DIV_CAM_REG,
										TWO_BYTES);
	row_speed = img_sensor_read_reg(img_sensor,
										ROW_SPEED_CAM_REG,
										TWO_BYTES);
	row_speed &= (BIT0 | BIT1 | BIT2);

	/* Refer EQ1 in datasheet - page 31 */
	vt_pix_clk = (EXT_CLK_FREQ_MHZ * pll_multiplier1)
			/ (pre_pll_clk_div1 * vt_sys_clk_div * vt_pix_clk_div * row_speed);

	return vt_pix_clk;
}
static uint8_t set_reg_continuous(img_sensor_t *img_sensor, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr, img_sensor_data_mode_t reg_size)
{
	i2cm_error_t status;
	uint16_t len = 0;
	uint32_t byte_size = (reg_size == TWO_BYTES) ? reg_arr_size * 2 : reg_arr_size;
	uint8_t *bytes = NULL;
	uint32_t i = 0;
	if(reg_size != ONE_BYTE && reg_size != TWO_BYTES)
	{
		SLOGF(SLOG_ERROR, "Data size is not supported");
		return 0;
	}
	bytes = (uint8_t *)pvPortMalloc(byte_size + 2);
	if(!bytes)
	{
		SLOGF(SLOG_ERROR, "Can't allocate I2C block memory");
		return 0;
	}
	bytes[len++] = (reg_arr->reg_addr >> 8) & 0xFF;
	bytes[len++] = (reg_arr->reg_addr) & 0xFF;

	for(i = 0; i < reg_arr_size; i++)
	{
		if(reg_size == ONE_BYTE)
		{
			bytes[len++] =  reg_arr->reg_val;
		}
		else
		{
			bytes[len++] =  (reg_arr->reg_val >> 8) & 0xFF;
			bytes[len++] =  (reg_arr->reg_val) & 0xFF;
		}
		reg_arr++;
	}
	status = i2cm.transceiver(img_sensor->i2c_dev, IMAGE_SENSOR_ADDRESS,
				bytes, len, NULL, 0);
	vPortFree(bytes);

	if(status != I2CM_ERROR_TRANSMITTED)
	{
		SLOGF(SLOG_ERROR, "Write block I2C [size: %d] timeout", len);
		return 0;
	}
	else
		return 1;

}

static uint8_t set_reg_noncontinuous(img_sensor_t *img_sensor, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr,  img_sensor_data_mode_t reg_size)
{
	uint8_t reg_val[2];
	uint32_t i = 0;
	i2cm_mode_t wmode;
	int ret = I2CM_ERROR_TRANSMITTED;
	if(reg_size != ONE_BYTE && reg_size != TWO_BYTES)
	{
		SLOGF(SLOG_ERROR, "Data size is not supported");
		return 0;
	}
	for(i = 0; i < reg_arr_size; i++)
	{
		if(reg_size == ONE_BYTE)
		{
			reg_val[0] = reg_arr->reg_val;
			wmode = BYTE_ADDR16;
		}
		else
		{
			reg_val[0] = (reg_arr->reg_val >> 8) & 0xFF;
			reg_val[1] = (reg_arr->reg_val) & 0xFF;
			wmode = WORD_ADDR16;
		}
		ret = i2cm.write(img_sensor->i2c_dev,
					wmode,
					IMAGE_SENSOR_ADDRESS,
					reg_arr->reg_addr,
					reg_val);
		if(I2CM_ERROR_TRANSMITTED != ret)
		{
			SLOGF(SLOG_ERROR, "Write register %x timeout", reg_arr->reg_addr);
			return 0;
		}
#ifdef DEBUG_CAM
		uint16_t readval = img_sensor_read_reg(img_sensor, reg_arr->reg_addr, reg_size);
		SLOGF(SLOG_DEBUG, "0x%04x = 0x%04x",reg_arr->reg_addr, readval);
		if(readval != reg_arr->reg_val)
			SLOGF(SLOG_ERROR, " Register %x. Write %x Read %x", reg_arr->reg_addr, reg_arr->reg_val, readval);
#endif
		reg_arr++;
	}
	return 1;
}

/*
 * cam_ctrl_reg_group
 */
void img_sensor_ctrl_reg_group(img_sensor_t *img_sensor, uint8_t state)
{
	uint8_t mode = (state == ON ? 0x01 : 0x00);
	img_sensor_write_reg(img_sensor, CAM_REG_GROUP_ON_OFF, mode, ONE_BYTE);
}

static void config_camera_power(cam_power_mode_t pwr_mode)
{
	hal_gpio_t gpio;
	gpio.pin = GPIO_PIN(light_system->cam_power_en);
	gpio.port = GPIO_PORT(light_system->cam_power_en);
	if(PWR_ON == pwr_mode)
	{
		gpio.direction = GPIO_DIR_OUT;
		hal_gpio_init(&gpio);
		hal_gpio_set_high(&gpio);
	}
	else
	{
		gpio.direction = GPIO_DIR_OUT;
		hal_gpio_init(&gpio);
		hal_gpio_set_low(&gpio);
	}
}
int img_is_fps_supported(uint16_t fps)
{
	if(fps != FPS30 && fps != FPS60 && fps != FPS24 && fps != FPS15)
		return FALSE;
	return TRUE;
}
cam_sensor_type_t get_image_sensor_type(img_sensor_t *image)
{
	uint16_t reg_val = img_sensor_read_reg(image,
			AR1335_CUSTOMER_REV,
			TWO_BYTES);
	cam_sensor_type_t sensor_color = AR1335_COLOR_SENSOR;

	switch (0x0070 & reg_val)
	{
	case 0x01:
		sensor_color = AR1335_COLOR_SENSOR;
		break;
	case 0x02:
		sensor_color = AR1335_PAN_SENSOR;
		break;
	default:
		sensor_color = AR1335_COLOR_SENSOR;
		break;
	}
	return sensor_color;
}
double img_sensor_get_frame_time(img_sensor_t *image, double *frame_time)
{
	uint32_t llpclk = img_sensor_read_reg(image,
										LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	uint32_t vtpixclk = img_sensor_get_vt_pix_clk_mhz(image);

	uint32_t cit = img_sensor_read_reg(image, CIT_CAM_REG, TWO_BYTES);
	double it_time = (double)(cit * llpclk * 1.0) /
									(double)(2 * vtpixclk * MHZ_TO_HZ * 1.0);
	if(frame_time)
	{
		uint32_t fll = img_sensor_read_reg(image, FLL_CAM_REG, TWO_BYTES);
		*frame_time = (double)(llpclk * fll * 1.0) /
									(double)(2 * vtpixclk * MHZ_TO_HZ * 1.0);
	}
	return it_time;
}
void img_sensor_reset_trigger(img_sensor_t *image)
{
	img_sensor_write_reg(image, 0x0100, 0x00 , ONE_BYTE);
	img_sensor_write_reg(image, 0x3F3C, 0x0003 , TWO_BYTES);
	img_sensor_write_reg(image, 0x301A, 0x001C , TWO_BYTES);
	img_sensor_write_reg(image, 0x301A, 0x021C , TWO_BYTES);
	img_sensor_write_reg(image, 0x301A, 0x0318 , TWO_BYTES);
	img_sensor_write_reg(image, 0x3026, 0xFD7F , TWO_BYTES);
	img_sensor_write_reg(image, 0x3158, 0x8800 , TWO_BYTES);

	/*On-Semi sequence for blk_offset fix due to bug in analog unit*/
	img_sensor_write_reg(image, 0x301A, 0x031C , TWO_BYTES);
	img_sensor_write_reg(image, 0x3F3C, 0x0002 , TWO_BYTES);
	img_sensor_write_reg(image, 0x3FE0, 0x0001 , TWO_BYTES);
	img_sensor_write_reg(image, 0x301A, 0x0318 , TWO_BYTES);
	img_sensor_write_reg(image, 0x3FE0, 0x0000 , TWO_BYTES);
	img_sensor_write_reg(image, 0x3F3C, 0x0003 , TWO_BYTES);
}
/*
 * calculate_gain
 * @brief This function to calculate gain.
 * @Param a_gain analog gain
 * @param d_gain digital gain
 * @param r_gain requested gain
 * @param c_mode refer to #ucid_mode_t
 * @return gain code to apply into the image sensor
*/
uint16_t calculate_gain(float *a_gain, float *total_gain, float r_gain, uint8_t c_mode)
{

	/* R0x305E
	 * Fine gain    [3:0]
	 * Coarse gain  [6:4]
	 * Digital gain [15:7]
	 * Total gain 7.75x max for snapshot, 61x max for preview
	 */
	gain_t gain;
	float f_gain, c_gain, d_gain, calculated_gain;
	/* Initial set d_gain to unity i.e. 64 for just analog gain calculation*/
	d_gain = 64;
	/* Calculate coarse gain from requested gain*/
	c_gain = floor(r_gain);
	/* Range 0 - 3*/
	c_gain = c_gain > 3 ? 3 : c_gain;
	/* Calculate fine gain*/
	f_gain = (r_gain * 64/d_gain * pow(0.5, c_gain - 1) * 16) - 16;
	f_gain = floor(f_gain);
	f_gain = f_gain > 15 ? 15 : f_gain;
	/* Calculate analog gain*/
	calculated_gain = pow(2, c_gain - 1) * ((16 + f_gain)/16) * (d_gain/64);
	*a_gain = calculated_gain;
	/* For preview mode*/
	if(c_mode == UCID_PREVIEW)
	{
		/* Calculate digital gain. Only use for preview*/
		d_gain = r_gain * pow(0.5, c_gain - 1) * (16/(16 + f_gain)*64);
		d_gain = floor(d_gain);
		d_gain = d_gain > 511 ? 511 : d_gain;
		calculated_gain = pow(2, c_gain - 1) * ((16 + f_gain)/16) * (d_gain/64);
	}
	*total_gain = calculated_gain;
	gain.digital_gain = (uint16_t)d_gain;
	gain.fine_gain = (uint16_t)f_gain;
	gain.coarse_gain = (uint16_t)c_gain;
	return gain.gain_code;
}
float get_total_gain(uint16_t gain_code, uint8_t c_mode)
{
	gain_t gain;
	gain.gain_code = gain_code;
	float total_gain, d_gain;
	d_gain = 64;
	/* Calculate total gain from gain code*/
	total_gain = pow(2, (float)gain.coarse_gain - 1) *
			((16 + (float)gain.fine_gain)/16) * (d_gain/64);
	if(c_mode == UCID_PREVIEW)
	{
		total_gain = pow(2, (float)gain.coarse_gain - 1) *
			((16 + (float)gain.fine_gain)/16) * ((float)gain.digital_gain/64);
	}
	return total_gain;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
