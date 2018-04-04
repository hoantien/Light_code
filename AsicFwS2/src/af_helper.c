/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 * @file    af_helper.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jul-13-2016
 * @brief   This file contain the real implementation for each UCID handler.
 *
 ******************************************************************************/
#include "af_helper.h"

#define READ_BACK(ADDR)		(*(uint32_t *)((uint32_t)(ADDR)))
#define SLOGF_ID						SLOG_ID_AF_CTRL

SemaphoreHandle_t move_frame_done_sem_;

static int mipi_rx_convert_in_halfword(uint16_t width, mipi2axi_data_type_t data_type,
								mipi2axi_stream_mode_t mode)
{
	int num_of_halfword;
	int bits_per_pixel;
	switch(data_type)
	{
		case MIPI2AXI_RAW10:
		{
			bits_per_pixel = 10;
			break;
		}
		case MIPI2AXI_RAW12:
		{
			bits_per_pixel = 12;
			break;
		}
		case MIPI2AXI_RAW14:
		{
			bits_per_pixel = 14;
			break;
		}
		case MIPI2AXI_RAW8:
		default:
		{
			bits_per_pixel = 8;
			break;
		}
	}
	if(mode == MIPI2AXI_SNAPSHOT_MODE)
	{
		/* 10bits per pixel would be packed in 16 bytes */
		num_of_halfword = (width * bits_per_pixel) / 8;
		/* If it is not exactly multiple of 16 then do +1 to it */
		if((num_of_halfword % 16) != 0)
			num_of_halfword = (num_of_halfword / 16) + 1;
		else
			num_of_halfword = (num_of_halfword / 16);
	}
	else
	{
		/* num_of_halfword should be num_of_pixels*2/16
		 * because 1 pixel occupies 2 bytes */
		num_of_halfword = (width * 2) / 16;
	}
	return num_of_halfword;
}

static void frame_captured_cb(void)
{
	BaseType_t higher_pri_task_woken = pdFALSE;
	xSemaphoreGiveFromISR(move_frame_done_sem_, &higher_pri_task_woken);
	portEND_SWITCHING_ISR(higher_pri_task_woken);
}

static void rx_cb(uint8_t iidx, uint32_t signal_irq,
										uint32_t error, void *user_data)
{
	if(signal_irq & R1_VC0_FRAME_END)
	{
		unsigned int error_val = READ_MIPI2AXI_REG(iidx, INT2_ST_REG);
		if(error_val)
			printf("\r\nMIPI Error: %x \r\n", error_val);

		frame_captured_cb();
	}
}

void setup_mipi_rx_roi_snapshot(uint8_t port_idx, uint16_t width,
								uint16_t height,
								uint32_t address, mipi2axi_callback af_cb,
								void* cb_user_data)

{
	/* Initialize MIPI RX DPHY */
	hal_mipi2axi_init(port_idx);

	/* Configure stream mode */
	mipi2axi_data_type_t data_type 			= MIPI2AXI_RAW10;
	mipi2axi_property_t mipi2axi_property;
	mipi2axi_property.vc_option				= MIPI2AXI_VC0_CH;
	mipi2axi_property.addr.dst_addr			= address;
	mipi2axi_property.addr.alu_src_addr 	= 0; /* don't care */
	mipi2axi_property.img_type.vc_will_captured = MIPI2AXI_VC0_CH;
	mipi2axi_property.img_type.data_type	= data_type;
	mipi2axi_property.img_size.height		= height;
	mipi2axi_property.img_size.width		= width;
	mipi2axi_property.stream_mode 			= MIPI2AXI_SNAPSHOT_MODE;
	hal_mipi2axi_set_stream_property(port_idx, &mipi2axi_property);

	/* Setup the DMA wrap around function as the data received
	 * will be more than the space available in TCM. */
	/* Calculate image pixel */
	int img_px = mipi_rx_convert_in_halfword(width, data_type,
			MIPI2AXI_SNAPSHOT_MODE) * 16;
	/* Calculate max height base on address: SRAM ATCM BTCM DDR READ IN reset_handler.s*/
	uint32_t size = 0;
	uint32_t height_max = 0;
	if(address < ATCM_BASE)
	{
		/* SRAM region */
		SLOGF(SLOG_DEBUG, "SRAM region: %X", address);
	}
	else if(address >= ATCM_BASE && address < (ATCM_BASE + ATCM_SIZE))
	{
		/* ATCM region */
		SLOGF(SLOG_DEBUG, "ATCM region: %X", address);
		size = ATCM_BASE + ATCM_SIZE - address;
		height_max = size / img_px;
		height = height < height_max ? height : height_max;
	}
	else if(address >= BTCM_BASE && address < (BTCM_BASE + BTCM_SIZE))
	{
		/* BTCM region */
		SLOGF(SLOG_DEBUG, "BTCM region: %X", address);
		size = BTCM_BASE + BTCM_SIZE - address;
		height_max = size / img_px;
		height = height < height_max ? height : height_max;
	}
	else
	{
		/* DDR region */
		SLOGF(SLOG_DEBUG, "DDR region: %X", address);
	}
	SLOGF(SLOG_DEBUG,
			"height: %d max: %d, width: %d, image pixel: %d, size: %d", height,
			height_max, width, img_px, size);
	/* Configure wrap */
	hal_mipi2axi_wrap_cfg(port_idx, MIPI2AXI_VC0_CH, height, ENABLE);

	/* Configure interrupt */
	hal_mipi2axi_irq_mask(
			port_idx,
			MIPI2AXI_SIGNAL_MASK_INTR,
			R1_VC0_FRAME_END | R1_VC0_FRAME_START,
			ENABLE);
	hal_mipi2axi_irq_enable(port_idx, af_cb, cb_user_data);

	/* Start MIPI 2 AXI */
	hal_mipi2axi_start(port_idx);
}

int setup_roi_mipi(cam_typedef_t *pcam, uint16_t x_position,
		uint16_t y_position, uint16_t x_width, uint16_t y_width, uint16_t scale,
		uint32_t destination, uint8_t mipi_rx_chid,  mipi2axi_callback af_cb, void* cb_user_data)
{
	/* Get CAM port index number on EVB board */
	uint8_t preview_ins = pcam->info.ch - 1;

	if(preview_ins == mipi_rx_chid)
	{
		uint8_t ret = 0;
		/* Duplication is not required */
		SLOGF(SLOG_DEBUG, "Duplication is not required - RX_ID: %d ", preview_ins);
		/* Configure the sensor in ROI mode */
		ret = img_sensor_cropping(pcam->image, x_position, y_position, x_width, y_width, scale);

		if(!ret)
		{
			SLOGF(SLOG_ERROR, "Configure the sensor in ROI mode unsuccessfully");
			return 1;
		}
	}
	else
	{
		/* TODO: Use the duplicate */
		SLOGF(SLOG_DEBUG, "Duplication is required: %d %d", preview_ins, mipi_rx_chid);
		/* Write SCU to map this cam with specify mipi_rx_chid parameter */
		uint32_t cur = READ_BACK(SCU_BASE + 0x1C);
		SLOGF(SLOG_DEBUG, "SCU current: %x ", cur);
		/* Clear 3 bit at mipi_rx_chid position */
		cur &= ~(0x07 << (3*mipi_rx_chid));
		/* Set 3 bit at mipi_rx_chid position */
		cur |= (preview_ins << (3*mipi_rx_chid));
		/* Write to SCU reg */
		*((uint32_t *)(SCU_BASE + 0x1C)) = cur;

		/* DEBUG-Read back SCU value*/
		cur = READ_BACK(SCU_BASE + 0x1C);
		SLOGF(SLOG_DEBUG, "SCU mapped: %x ", cur);
	}

    /* Configure the RX in snapshot mode for the given ROI */
    /* Setup snapshot on mipi_rx_chid
     * and set the destination address provided in the API */
    setup_mipi_rx_roi_snapshot(mipi_rx_chid, pcam->image->x_size, pcam->image->y_size,
            destination, af_cb, cb_user_data);

	return 0;
}

static int sensor_sw_standby(af_setting_t* setting)
{
	int 			ret 		= 0;
	cam_typedef_t * cam			= setting->cam;
	img_sensor_t * 	sensor		= setting->cam->image;
	uint32_t 		cam_status	= 0;

	memcpy((uint8_t *)&cam_status, cam->settings->status, sizeof(uint32_t));

	if(cam_status & S_MODULE_SW_STANDBY)
	{
		SLOGF(SLOG_INFO, "CAM-%X already in SW standby",
				cam->info.module);
		ret = LCC_CMD_SUCCESS;
		goto error;
	}

	if (!(cam_status & S_MODULE_POWER_ON))
	{
		if(img_sensor_open(sensor, HW_STANDBY))
		{
			cam_status = S_MODULE_POWER_ON | S_MODULE_HW_STANDBY;
			if(cam->info.grp == GRP_A)
				vcm_init((vcm_t *)cam->optical->lens);
		}
		else
		{
			cam->settings->open = CAM_MODULE_CLOSE;
			SLOGF(SLOG_ERROR, "CAM-%X changed to SW standby failed",
					cam->info.module);
			/* Update camera status */
			cam_status = S_ERROR_SENSOR_I2C_DETECT_FAILURE;
			ret = 0;
		}
	}

	/* Check if switching CAM to HW_STANDBY in previous is
	 * successfully or failed */
	if(cam_status & S_MODULE_POWER_ON)
	{
		if(img_sensor_open(sensor, SW_STANDBY))
		{
			/* Recover the status to un-error state */
			cam_status = S_MODULE_POWER_ON |
					S_MODULE_SW_STANDBY | S_MODULE_CLOCK_ON;
			cam->settings->open = CAM_MODULE_SW_STANDBY;

			SLOGF(SLOG_INFO, "%s[%d]: Open CAM-%X was successfully",
					__FUNCTION__, __LINE__, cam->info.module);
			ret = LCC_CMD_SUCCESS;
		}
		else
		{
			/* Update cache of cam open */
			cam->settings->open = CAM_MODULE_HW_STANDBY;
			/* We use write in open to send configuration
			 * to sensor */
			cam_status = S_MODULE_POWER_ON |
					S_MODULE_HW_STANDBY |
					S_ERROR_SENSOR_I2C_DETECT_FAILURE;
			ret = 0;
			SLOGF(SLOG_ERROR, "%s:[%d] Cannot switch CAM-%X to SW STANDBY",
					__FUNCTION__, __LINE__, cam->info.module);
		}
	}
	else
	{
		cam->settings->open = CAM_MODULE_CLOSE;
		cam_status = S_ERROR_SENSOR_I2C_DETECT_FAILURE;
		ret = 0;
		SLOGF(SLOG_ERROR, "%s:[%d] Cannot switch CAM-%X to HW STANDBY",
				__FUNCTION__, __LINE__, cam->info.module);
	}
	memcpy(cam->settings->status, &cam_status, sizeof(uint32_t));

error:
	return ret;
}

flip_mode_t get_af_flip_mode(flip_mode_af_t flip_mode_af, const img_sensor_t* img_sensor)
{
    flip_mode_t flip_mode;
    switch (flip_mode_af)
    {
    case FLIP_MODE_CAPTURE:
        flip_mode = img_sensor->flip;
        break;
    default:
        flip_mode = FLIP_NONE;
        break;
    }
    return flip_mode;
}

static int sensor_set_resolution(af_setting_t * setting)
{
	int ret = 0;
	cam_typedef_t * pcam			= setting->cam;

	img_sensor_t *img = setting->cam->image;
	cam_reg_array_t *reg = NULL;
	if(pcam->settings->open == CAM_MODULE_SW_STANDBY)
	{
		uint32_t x_res, y_res;
		memcpy((uint8_t *) &x_res, setting->cam_settings.resolution,
				sizeof(uint32_t));
		memcpy((uint8_t *) &y_res,
				setting->cam_settings.resolution + sizeof(uint32_t),
				sizeof(uint32_t));
#ifndef P2
		if(x_res == X_13M && y_res == Y_13M)
			reg = &resolution_testing_reg_array[0];
		else if(x_res == X_1080P && y_res == Y_1080P)
			reg = &resolution_testing_reg_array[1];
		else if(x_res == X_720P && y_res  == Y_720P)
			reg = &resolution_testing_reg_array[2];
		else if(x_res == X_3M && y_res == Y_3M)
			reg = &resolution_testing_reg_array[3];
		else if(x_res == X_4K_UHD && y_res == Y_4K_UHD_CINEMA)
			reg = &resolution_testing_reg_array[4];
		else if(x_res == X_4K_CINEMA && y_res == Y_4K_UHD_CINEMA)
			reg = &resolution_testing_reg_array[5];
#else /* For P2 */
#ifdef MIPI_SPEED_1500MHZ
		if(x_res == X_13M && y_res == Y_13M)
			reg = &resolution_testing_reg_array[4];
		else if(x_res == X_13M_P2 && y_res == Y_13M)
			reg = &resolution_testing_reg_array[0];
		else if(x_res == X_3M_P2 && y_res == Y_3M_P2)
			reg = &resolution_testing_reg_array[1];
		else if(x_res == X_4K_UHD && y_res == Y_4K_UHD_CINEMA)
			reg = &resolution_testing_reg_array[2];
		else if(x_res == X_1080P && y_res == Y_1080P)
			reg = &resolution_testing_reg_array[3];
#else /* MIPI_SPEED_400MHZ */
		if(x_res == X_13M && y_res == Y_13M)
			reg = &resolution_testing_reg_array[2];
		else if(x_res == X_13M_P2 && y_res == Y_13M)
			reg = &resolution_testing_reg_array[0];
		else if(x_res == X_3M_P2 && y_res == Y_3M_P2)
			reg = &resolution_testing_reg_array[1];
#endif /* MIPI_SPEED */
#endif /* P2 */
		if(reg != NULL)
			ret = img_sensor_send_config(img, reg, 1);
		else
			SLOGF(SLOG_ERROR, "%s:[%d] Invalid resolution params",
					__FUNCTION__, __LINE__);
		if(ret)
		{
			img->default_fll = img_sensor_read_reg(img, FLL_CAM_REG, TWO_BYTES);
			img->default_llpclk = img_sensor_read_reg(img,
					LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	        img_sensor_flip_capture(img, get_af_flip_mode(setting->flip_mode, img));
		}
		if(light_system->active_ucid == UCID_PREVIEW)
		{
			reconfig_spg_for_previewing_sensor(pcam);
			hal_syncio_trigger();
		}

		if(ret)
		{
			SLOGF(SLOG_INFO,
					"CAM-%X : Resolution is updated to %d x %d",
					pcam->info.module, x_res, y_res);
		}
		else
		{
			SLOGF(SLOG_WARN,
					"CAM-%X : Resolution hasn't been updated to %d x %d",
					pcam->info.module, x_res, y_res);
		}
	}
	else
	{
		SLOGF(SLOG_ERROR, "%s:[%d]: CAM-%X was not in SW STANDBY mode",
				__FUNCTION__, __LINE__, pcam->info.module);
	}

	return ret;
}

static int sensor_set_sensitivity(af_setting_t * setting)
{
	int ret = 0;
	cam_typedef_t * cam			= setting->cam;
	img_sensor_t * 	sensor		= setting->cam->image;
	uint32_t 		cfg,sensitivity;

	float *requested_gain = NULL;
	float total_gain, analog_gain;
	memcpy((uint8_t *)&sensitivity, setting->cam_settings.sensitivity, sizeof(uint32_t));
	requested_gain = (float *)&(sensitivity);
	if(requested_gain)
	{
		cfg = calculate_gain(&analog_gain, &total_gain, *requested_gain,
											light_system->active_ucid);
		/* Apply the gain configuration*/
		img_sensor_set_sensitivity(sensor, cfg);
		SLOGF(SLOG_INFO, "%s:[%d]: Sensitivity of CAM-%X was updated"
		" total gain to [%.02f]", __FUNCTION__, __LINE__,
		cam->info.module, total_gain);
		ret = 1;
	}
	else
	{
		SLOGF(SLOG_WARN, "%s:[%d]: Sensitivity of CAM-%X"
		" was not updated", __FUNCTION__, __LINE__,
		cam->info.module);
		ret = 0;
	}
	return ret;
}

static int sensor_set_fps(af_setting_t * setting)
{
	int ret = LCC_CMD_UNSUCCESS;
	cam_typedef_t * pcam			= setting->cam;

	if(pcam->settings->open == CAM_MODULE_SW_STANDBY)
	{
		uint16_t cam_fps;
		memcpy((uint8_t *) &cam_fps, setting->cam_settings.fps,
				sizeof(uint16_t));
		/* Set fps value for camera*/
		img_sensor_set_fps(pcam, cam_fps);
		SLOGF(SLOG_INFO, "%s:[%d]: FPS of CAM-%X updated to 0x%02X ",
			__FUNCTION__, __LINE__, pcam->info.module, cam_fps);
		ret = LCC_CMD_SUCCESS;
	}
	else
	{
		SLOGF(SLOG_ERROR, "%s:[%d]: CAM-%X was not in SW STANDBY mode",
			__FUNCTION__, __LINE__, pcam->info.module);
		ret = LCC_CMD_UNSUCCESS;
	}

	return ret;
}

static int sensor_set_exposure(af_setting_t * setting)
{
	int ret = LCC_CMD_UNSUCCESS;
	cam_typedef_t * pcam			= setting->cam;

	if(pcam->settings->open == CAM_MODULE_SW_STANDBY)
	{
		uint64_t exposure;
		memcpy((uint8_t *)&exposure, setting->cam_settings.exposure,
				sizeof(uint64_t));
		SLOGF(SLOG_DEBUG, "CAM-%X applying exposure setting: %d",
							pcam->info.module, exposure);
		img_sensor_set_exposure_time(pcam, exposure);
		ret = LCC_CMD_SUCCESS;
	}
	else
	{
		SLOGF(SLOG_ERROR, "%s:[%d]: CAM-%X was not in SW STANDBY mode",
			__FUNCTION__, __LINE__, pcam->info.module);
		ret = LCC_CMD_UNSUCCESS;
	}

	return ret;
}

static int img_sensor_reset(img_sensor_t *img_sensor)
{
    img_sensor_write_reg(img_sensor, 0x0100, 0x00, ONE_BYTE);
    img_sensor_write_reg(img_sensor, 0x3F3C, 0x0003, TWO_BYTES);
    img_sensor_write_reg(img_sensor, 0x301A, 0x001C, TWO_BYTES);
    img_sensor_write_reg(img_sensor, 0x301A, 0x021C, TWO_BYTES);

    return 1;
}

static int img_sensor_capture_stream_off(img_sensor_t *img_sensor, cam_sensor_mode_t mode)
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
        img_sensor_write_reg(img_sensor, 0x3026, 0xFD7F, TWO_BYTES);
        img_sensor_write_reg(img_sensor, 0x3158, 0x8800, TWO_BYTES);
    }
    return 1;
}

/* TODO: implement sync IO base on the sensor settings */
int trigger_sensor(af_setting_t *af_setting)
{
	int 					ret 	= 0;
	cam_typedef_t * 		cam 	= af_setting->cam;
	uint16_t cit = img_sensor_read_reg(cam->image, CIT_CAM_REG, TWO_BYTES);
	uint16_t vtpixclk = img_sensor_get_vt_pix_clk_mhz(cam->image);
	uint16_t llpclk = img_sensor_read_reg(cam->image,
				LINE_LENGTH_PCLK_CAM_REG, TWO_BYTES);
	double it_time = (cit * llpclk * 0.000001) / (2 * vtpixclk);
	cam_capture_syncio_config(cam,1, it_time, SG_TRIG_SW );

	/* Stream off cam */
	img_sensor_reset(cam->image);
	img_sensor_capture_stream_off(cam->image,SLAVE_MODE);

	/* Start syncio in HW trigger mode */
	SLOGF(SLOG_DEBUG, "Send trigger");
	img_sensor_stream_on(cam->image, SLAVE_MODE);
	hal_syncio_trigger();

	return ret;
}

int af_configure_sensor_stream(af_setting_t *af_setting, uint8_t is_preview_cam)
{
	int 			ret 		= 0;
	cam_typedef_t * cam			= af_setting->cam;

	SLOGF(SLOG_DEBUG, "Entering %s", __FUNCTION__);

	/*
	 * if preview camera just setup the rx and don't do any thing else
	 * TODO: set up the dma wrap if requested??
	 */
	if (is_preview_cam)
	{
		SLOGF(SLOG_WARN, "Cannot change the sensor settings");
	}
	/* not preview */
	else
	{
		SLOGF(SLOG_DEBUG, "AF setting request for CAM-%X",
					cam->info.module);

		/* Put sensor in sw standby */
		sensor_sw_standby(af_setting);

		/* Setup resolution */
		sensor_set_resolution(af_setting);

		/* Setup sensitivity */
		sensor_set_sensitivity(af_setting);

		/* Setup fps */
		sensor_set_fps(af_setting);

		/* Set up exposure */
		sensor_set_exposure(af_setting);

		img_sensor_config_mode(cam->image, SLAVE_MODE);
		/* Set up the sensor roi and the RX receiver */
		ret = setup_roi_mipi(cam, af_setting->x_position,
				af_setting->y_position, af_setting->x_width, af_setting->y_width,
				af_setting->scale, af_setting->destination, af_setting->mipi_rx_chid, af_setting->af_cb,
				af_setting->cb_user_data);
	}

	SLOGF(SLOG_DEBUG, "Exiting %s", __FUNCTION__);

	return ret;
}

void af_test(cam_typedef_t *pcam)
{
	if(!pcam) return;

	SLOGF(SLOG_INFO, "Entering %s", __FUNCTION__);

	uint16_t left_x = pcam->cam_common_data.roi_rectangle.left_x;
	uint16_t top_y = pcam->cam_common_data.roi_rectangle.top_y;
	uint16_t width = pcam->cam_common_data.roi_rectangle.width;
	uint16_t height = pcam->cam_common_data.roi_rectangle.height;
	SLOGF(SLOG_DEBUG, "ROI: (%d, %d) - w: (%d, %d)", (int)left_x, (int)top_y, (int)width, (int)height);

	/* Config af_setting_t */
	af_setting_t af_setting;
	af_setting.cam = pcam;
	af_setting.cam_settings =
			*(pcam->image->settings[light_system->active_ucid]);
	af_setting.destination = BTCM_BASE;
	af_setting.mipi_rx_chid = (pcam->info.ch - 1) % 6; /*Set mipi_rx_chid=cam_rx_id*/

	af_setting.x_position = left_x;
	af_setting.y_position = top_y;
	af_setting.x_width = width;
	af_setting.y_width = height;

	af_setting.af_cb = rx_cb;
	af_setting.cb_user_data = NULL;
	af_setting.flip_mode = FLIP_MODE_CAPTURE;

	/* Set exposure */
	uint64_t exposure = 30000000;
	memcpy(af_setting.cam_settings.exposure, (uint8_t *) &exposure,
			sizeof(uint64_t));
	/* Set FPS */
	uint16_t cam_fps = 30;
	memcpy(af_setting.cam_settings.fps, (uint8_t *) &cam_fps, sizeof(uint16_t));
	/* Set resolution */
	uint32_t x_res = X_13M;
	uint32_t y_res = Y_13M;
	memcpy(af_setting.cam_settings.resolution, (uint8_t *) &x_res,
			sizeof(uint32_t));
	memcpy(af_setting.cam_settings.resolution + sizeof(uint32_t),
			(uint8_t *) &y_res, sizeof(uint32_t));
	/* Set sensitivity */
	float total_gain = 1.5;
	memcpy(af_setting.cam_settings.sensitivity, (uint8_t *) &total_gain,
			sizeof(uint32_t));

	move_frame_done_sem_ = xSemaphoreCreateBinary();
	/* Config the exposure, sensitivity and cam standby for non preview cams */
	af_configure_sensor_stream(&af_setting, 0);

	/* Read the sensor register to check if exposure, sensitivity and FLL are correct. */
	/* Read exposure */
#if(LOG_VERBOSE == STD_ON)
	uint16_t cit = img_sensor_read_reg(pcam->image, CIT_CAM_REG, TWO_BYTES);
	SLOGF(SLOG_DEBUG, "CIT: 0x%x", cit);
	/* Read sensitivity */
	uint16_t sensitivity = img_sensor_read_reg(pcam->image,
	CAM_REG_SENSITIVITY, TWO_BYTES);
	SLOGF(SLOG_DEBUG, "Sensitivity: 0x%x", sensitivity);
	/* Read FLL */
	uint32_t cur_fll = img_sensor_read_reg(pcam->image, FLL_CAM_REG, TWO_BYTES);
	SLOGF(SLOG_DEBUG, "frame line length: 0x%x", cur_fll);
#endif

	for(int j = 0; j < 5; j++)
	{
		/* Trigger sensor */
		trigger_sensor(&af_setting);
		// Wait for capture done
		xSemaphoreTake(move_frame_done_sem_, portMAX_DELAY); //frame_capture_timeout_ms() * portTICK_PERIOD_MS);
		SLOGF(SLOG_INFO, "Frame capture done");
	}
	vSemaphoreDelete(move_frame_done_sem_);
	SLOGF(SLOG_INFO, "Exiting %s", __FUNCTION__);
}
