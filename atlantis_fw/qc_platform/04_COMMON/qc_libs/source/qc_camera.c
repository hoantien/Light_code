/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    qc_camera.c
 * @author  The LightCo
 * @version V1.0.1
 * @date    26-July-2016
 * @brief   This file contains expand of the QC camera driver
 *
 ******************************************************************************/

/******************************************************************************/
/**								Revision history
 * * 1.0.0	26-July-2016	Initial revision:
 *                      - Infrastructure.
 */
/******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "qc_camera.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
cam_module_t cam_module_list_t[CAM_CH_MAX_NUM] =
{
	CAM_PRE_BUILD_CREATE(CAM_A1, CAM_CH_A1, CAM_TYPE_35MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_A2, CAM_CH_A2, CAM_TYPE_35MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_A3, CAM_CH_A3, CAM_TYPE_35MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_A4, CAM_CH_A4, CAM_TYPE_35MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_A5, CAM_CH_A5, CAM_TYPE_35MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_B1, CAM_CH_B1, CAM_TYPE_70MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_B2, CAM_CH_B2, CAM_TYPE_70MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_B3, CAM_CH_B3, CAM_TYPE_70MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_B4, CAM_CH_B4, CAM_TYPE_70MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_B5, CAM_CH_B5, CAM_TYPE_70MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_C1, CAM_CH_C1, CAM_TYPE_150MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_C2, CAM_CH_C2, CAM_TYPE_150MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_C3, CAM_CH_C3, CAM_TYPE_150MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_C4, CAM_CH_C4, CAM_TYPE_150MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_C5, CAM_CH_C5, CAM_TYPE_150MM, 0x6C >> 1),
	CAM_PRE_BUILD_CREATE(CAM_C6, CAM_CH_C6, CAM_TYPE_150MM, 0x6C >> 1),
};

static uint8_t first_time = 1;

/* Exported functions --------------------------------------------------------*/
static void it_i2c_callback(hal_i2c_channel_t chid, hal_i2c_status_t status)
{}

i2cm_error_t i2cm_transceiver_t(i2c_t chid, uint8_t addr,
								uint8_t *tx, size_t txlen,
								void *cb_func, void *cb_param)
{
	static uint32_t count;
	uint32_t time_out = 1;
	time_out *= 5000;
	hal_i2c_buffer_t buf;
	hal_i2c_t hal_i2c;

	hal_i2c.address_mode = I2C_7BIT;
	hal_i2c.chid = chid;
	hal_i2c.clock_speed = I2C_SPEED_100KHz;
	hal_i2c.operation_mode = I2C_MASTER;
	hal_i2c.owner_addr = addr;
	hal_i2c.irq_handler = it_i2c_callback;
	hal_i2c_init(&hal_i2c);
	hal_i2c_enable_irq(&hal_i2c);

	buf.bytes = tx;
	buf.length = txlen;
#ifdef DEBUG
	log_printf("Slave addr = %x\n\r",hal_i2c.owner_addr);
	for(int i =0;i<txlen;i++)
	{
		log_printf("data: %x \t",*(buf.bytes + i));
	}
	log_printf("\n\r");
#endif
	hal_i2c_master_tx(chid, addr, &buf);
	hal_i2c_status_t tmp = I2C_IDLE;
	while ((I2C_TX_COMPLETED != tmp) && (--time_out))
	{
		tmp = hal_i2c_get_status(chid);
	}
	count ++ ;
	if(0==time_out)
	{
		log_printf("i2cm_transceiver_t time out\t");
		log_printf("I2C sent count = %d\n\r",count);
		//log_printf("hal_i2c_get_status = %d\n\r",tmp);
		return I2CM_ERROR_TRANSMITTED;
	}
	hal_i2c_deinit(hal_i2c.chid);
	return I2CM_ERROR_NONE;
}

int open_cam(uint8_t cam_scan_idx)
{
	hal_gpio_t gpio;
	gpio.port = GPIO_PORTB;
	gpio.direction = GPIO_DIR_OUT;
	gpio.pin = GPIO_PIN_0;
	hal_gpio_init(&gpio);
	hal_gpio_set_high(&gpio);
	for(uint8_t i = 1 ; i <= 6; i++)
	{
		gpio.pin = i;
		hal_gpio_init(&gpio);
		hal_gpio_set_high(&gpio);
	}
	return cam_open_t(&cam_module_list_t[cam_scan_idx]);
}

int cam_write_reg_t(
		volatile cam_module_t *cam, uint16_t reg, uint16_t data, uint8_t d_mode)
{
	uint16_t len = 0;
	uint8_t bytes[4] = {0};
	if(d_mode != DATA_16BIT && d_mode != DATA_8BIT)
	{
		log_printf("Data size is invalid");
		return FALSE;
	}
	bytes[0] = (uint8_t)(reg >> 8);
	bytes[1] = (uint8_t)reg;
	if(d_mode == DATA_8BIT)
	{
		bytes[2] = (uint8_t)data;
		len = 3;
	}
	else
	{
		bytes[2] = (uint8_t)(data >> 8);
		bytes[3] = (uint8_t)data;
		len = 4;
	}
	i2cm_error_t ret;
	ret = i2cm_transceiver_t(cam->chid, cam->slave_addr,
					bytes, len, NULL, (void *)cam);
	if (ret==I2CM_ERROR_TRANSMITTED) return -1;
	return TRUE;
}

int cam_open_t(volatile cam_module_t *cam)
{
	uint32_t i = 0;
	int ret = 0;
	cam_module_t *pcam = (cam_module_t *)cam;
	ret = cam_write_reg_t(cam, 0x301A, 0x031D, DATA_16BIT);
	if(-1==ret) return -1;
	msm_camera_i2c_reg_array_t *preg = NULL;
	for(i = 0; i < cam_open_default_size; i++)
	{
		preg = cam_open_default[i].regs;
		if(cam_open_default[i].reg_type == CAM_REG_CONTINUOUS)
		{
			ret = cam_set_reg_continuous_t(pcam,
					cam_open_default[i].reg_size, preg,
					cam_open_default[i].data_size);
		}
		else
		{
			ret = cam_set_reg_noncontinuous_t(pcam,
					cam_open_default[i].reg_size, preg,
					cam_open_default[i].data_size);
		}
		if(!ret)
		{
			/* Open camera module is failed. */
			log_printf("Fail to open camera");
			return -1;
		}
	}
	return 1;
}

uint8_t cam_set_reg_noncontinuous_t(cam_module_t *cam, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr,  cam_data_mode_t reg_size)
{
	/* Maximum: 2 bytes address + 2 bytes data */
	uint8_t bytes[4] = {0};
	uint16_t len = 0;
	uint32_t i = 0;
	for(i = 0; i < reg_arr_size; i++)
	{
		bytes[0] = (reg_arr->reg_addr >> 8) & 0xFF;
		bytes[1] = (reg_arr->reg_addr) & 0xFF;
		if(reg_size == DATA_8BIT)
		{
			len = 3;
			bytes[2] = reg_arr->reg_val;

		}
		else
		{
			len = 4;
			bytes[2] = (reg_arr->reg_val >> 8) & 0xFF;
			bytes[3] = (reg_arr->reg_val) & 0xFF;
		}
		i2cm_transceiver_t(cam->chid,  cam->slave_addr,
					bytes, len,
					NULL, (void *)cam);
		reg_arr++;
	}
	return 1;
}

uint8_t cam_set_reg_continuous_t(cam_module_t *cam, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr,  cam_data_mode_t reg_size)
{
	uint16_t len = 0;
	uint32_t byte_size = (reg_arr_size >= 2) ? reg_arr_size * 2 : reg_arr_size;
	uint8_t *bytes = NULL;
	uint32_t i = 0;

	bytes = (uint8_t *)malloc(byte_size + 2);
	memcpy(bytes, &reg_arr->reg_addr, sizeof(uint16_t));
	len = 2;
	for(i = 0; i < reg_arr_size; i++)
	{
		if(reg_size == DATA_8BIT)
		{
			memcpy(bytes + len, &reg_arr->reg_val, sizeof(uint8_t));
			len += 1;
		}
		else
		{
			memcpy(bytes + len, &reg_arr->reg_val, sizeof(uint16_t));
			len += 2;
		}
		reg_arr++;
	}
	i2cm_transceiver_t(cam->chid,  cam->slave_addr,
				bytes, len,
				NULL, (void *)cam);

	free(bytes);
	return 1;
}

void cam_configure_t(volatile cam_module_t *cam)
{
	//log_printf("%s %d",__FUNCTION__,__LINE__);
	/* configure PLL */
	cam_write_reg_t(cam, 0x0300, 0x0005, DATA_16BIT);
	cam_write_reg_t(cam, 0x0300, 0x0005, DATA_16BIT);
	cam_write_reg_t(cam, 0x0302, 0x0001, DATA_16BIT);
	cam_write_reg_t(cam, 0x0304, 0x0202, DATA_16BIT);
	cam_write_reg_t(cam, 0x0306, 0x2020, DATA_16BIT);
	cam_write_reg_t(cam, 0x0308, 0x000A, DATA_16BIT);
	cam_write_reg_t(cam, 0x030A, 0x0001, DATA_16BIT);

	/* configure cam */
	cam_write_reg_t(cam, 0x301A, 0x0318, DATA_16BIT); /* SW Standby */
	cam_write_reg_t(cam, 0x3026, 0xFD7B, DATA_16BIT); /* Trigger pin select */
	cam_write_reg_t(cam, 0x3158, 0xA000, DATA_16BIT); /* Slave mode control */
	cam_write_reg_t(cam, 0x301A, 0x031C, DATA_16BIT); /* Stream on */
}

void cam_syncio_init(cam_module_t *cam, cam_syncio_cfg_t *sync_init)
{
	uint8_t channel = 0;
	hal_syncio_cfg_t syncio_cfg;

	/* Configures sync io */
	syncio_cfg.inf_mode  = sync_init->inf_mode;
	syncio_cfg.trig_mode = sync_init->trig_mode;
	syncio_cfg.repeat    = sync_init->num_pulse;

	syncio_cfg.lat1   = sync_init->lat_pulse;
	syncio_cfg.lat2   = sync_init->lat_pulse;
	syncio_cfg.width1 = sync_init->width_pulse;
	syncio_cfg.width2 = sync_init->width_pulse;


	/* Configures sync IO channel */
	if(sync_init->channel != SG_CHANNEL_MAX)
	{
		hal_syncio_init(sync_init->channel);
		/* Just configures 1 channel */
		hal_syncio_config(sync_init->channel, &syncio_cfg);
	}
	else
	{
		/* Configures for all channel */
		while(channel < SG_CHANNEL_MAX)
		{
			hal_syncio_init(channel);
			hal_syncio_config(channel, &syncio_cfg);
			channel++;
		}
	}

	cam_configure_t(cam);
}

void init_syncio(uint8_t infmode, uint8_t numpulse)
{
	cam_syncio_cfg_t cam_syncio;
	cam_syncio.channel = SG_CHANNEL_MAX;
	cam_syncio.inf_mode = infmode;
	cam_syncio.trig_mode = SG_TRIG_SW;
	cam_syncio.lat_pulse = 199000 * MICROSECOND_TIME;
	cam_syncio.width_pulse = 1000 * MICROSECOND_TIME;
	cam_syncio.num_pulse = numpulse;
	cam_syncio_init(&cam_module_list_t[CAM_CH_A1], &cam_syncio);
	if(first_time)
	{
		cam_configure_t(&cam_module_list_t[CAM_CH_A1]);
		first_time = 0;
	}
}

void cam_syncio_stream_on(hal_syncio_channel_t syncio_channel)
{
	uint8_t channel = 0;

	if(syncio_channel != SG_CHANNEL_MAX)
	{
		/* Just enable one channel */
		hal_syncio_enable(syncio_channel);
	}
	else
	{
		/* Enable for all channel */
		while(channel < SG_CHANNEL_MAX)
		{
			hal_syncio_enable(channel);
			channel++;
		}
	}
	/* Start generates sync IO pulse */
	hal_syncio_trigger();
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
