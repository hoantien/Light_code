/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    camera.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-07-2016
 * @brief   This file contains expand for camera driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "log.h"
#include "os.h"
#include "ar1335.h"
#include "std_type.h"
#include "board_config.h"
#include "assert.h"
#include "camera.h"
#include "lcc_system.h"
#include "i2cm.h"
#include "lcc_cmd.h"
#include "hal_gpio.h"
//TODO: for testing purpose only
#include "register_dump.h"

/* Private define-------------------------------------------------------------*/
#define SLOGF_ID					SLOG_ID_CAMERA

/* Private typedef------------------------------------------------------------*/
/* Private define-------------------------------------------------------------*/
/* Private macro--------------------------------------------------------------*/
/* Private variables----------------------------------------------------------*/
/* Private function-----------------------------------------------------------*/
static void cam_cache_init(volatile cam_module_t *cam);
/* The camera lookup table with CAMERA_TBL was predefined in lookup_tbl.mk.
 * This table can edit depend on platform or design
 */
void cam_eeprom_callback(i2cm_error_t status, void *param);
static void cam_command_callback(i2cm_error_t status, void *param);
/* Exported functions --------------------------------------------------------*/
/*
 * cam_init( cam_module_t *cam )
 * Initialize camera module, note that, i2c expander must started
 */
void cam_init(volatile cam_module_t *cam)
{
	cam->cmd = assert_malloc(cam->cmd, sizeof(cam_command_t));
	cam->cmd->data = assert_malloc(cam->cmd->data, 8 * sizeof(uint8_t));
	cam->cmd->len = 0;
	cam->cmd->status = CAM_CMD_IDLE;
	cam->eeprom.buf = NULL;
	cam->eeprom.len = 0;
	cam_cache_init(cam);
}

/**
 * cam_open
 * Open camera module in software standby mode
 */
void cam_open(volatile cam_module_t *cam)
{
	uint32_t i = 0;
	uint8_t ret = 0;
	cam_module_t *pcam = (cam_module_t *)cam;
	msm_camera_i2c_reg_array_t *preg = NULL;
	for(i = 0; i < cam_open_default_size; i++)
	{
		preg = cam_open_default[i].regs;
		if(cam_open_default[i].reg_type == CAM_REG_CONTINUOUS)
		{
			ret = cam_set_reg_continuous(pcam, cam_open_default[i].reg_size, preg,
					cam_open_default[i].data_size);
		}
		else
		{
			ret = cam_set_reg_noncontinuous(pcam, cam_open_default[i].reg_size, preg,
					cam_open_default[i].data_size);
		}
		if(!ret)
		{
			/* Open camera module is failed. */
			cam->m_o_status = CAM_MODULE_CLOSE;
			return;
		}
		vTaskDelay(1);
	}
}

/**
 * cam_close
 * Close opened camera module
 */
void cam_close(volatile cam_module_t *cam)
{

}

/**
 * cam_ctrl_read_reg
 */
uint16_t cam_read_reg(volatile cam_module_t *cam, uint16_t reg, uint8_t d_mode)
{
	uint16_t len = 0;
	uint8_t bytes[2] = {0};
	uint16_t timeout = 10;
	cam->cmd->status = CAM_CMD_IDLE;
	if(d_mode != DATA_16BIT && d_mode != DATA_8BIT)
	{
		SLOGF(SLOG_ERROR, "Data size is invalid");
		return FALSE;
	}
	bytes[0] = (uint8_t)(reg >> 8);
	bytes[1] = (uint8_t)reg;
	len = 2;
	i2cm_transceiver(cam->chid, cam->slave_addr,
					bytes, len,
					NULL, 0,
					cam_command_callback, (void *)cam);
	/* Wait for timeout in 10 ticks. */
	while(timeout && cam->cmd->status != CAM_CMD_WRITE_DONE)
	{
		timeout--;
		vTaskDelay(1);
	}
	if(!timeout)
	{
		SLOGF(SLOG_DEBUG, "%s Wrote the register was timeout!", cam->name);
		return FALSE;
	}

	if(d_mode == DATA_8BIT)
	{
		len = 1;
		i2cm_transceiver(cam->chid, cam->slave_addr,
						NULL, 0,
						bytes, len,
						cam_command_callback, (void *)cam);
		/* Wait for timeout in 10 ticks. */
		while(timeout && cam->cmd->status != CAM_CMD_READ_DONE)
		{
			timeout--;
			vTaskDelay(1);
		}
		if(!timeout)
		{
			SLOGF(SLOG_DEBUG, "%s Wrote the register was timeout!", cam->name);
			return FALSE;
		}
		return bytes[0];
	}
	i2cm_transceiver(cam->chid, cam->slave_addr,
				NULL, 0,
				bytes, len,
				cam_command_callback, (void *)cam);
	/* Wait for timeout in 10 ticks. */
	while(timeout && cam->cmd->status != CAM_CMD_READ_DONE)
	{
		timeout--;
		vTaskDelay(1);
	}
	if(!timeout)
	{
		SLOGF(SLOG_DEBUG, "%s Wrote reg was timeout!", cam->name);
		return FALSE;
	}
	return ((bytes[0] << 8) | bytes[1]);
}

/**
 * cam_ctrl_write_reg
 */
uint8_t cam_write_reg(
	volatile cam_module_t *cam, uint16_t reg, uint16_t data, uint8_t d_mode)
{
	uint16_t len = 0;
	uint8_t bytes[4] = {0};
	uint16_t timeout = 10;
	cam->cmd->status = CAM_CMD_IDLE;
	if(d_mode != DATA_16BIT && d_mode != DATA_8BIT)
	{
		SLOGF(SLOG_ERROR, "Data size is invalid");
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
	i2cm_transceiver(cam->chid, cam->slave_addr,
					bytes, len,
					NULL, 0,
					cam_command_callback, (void *)cam);
	/* Wait for timeout in 10 ticks. */
	while(timeout && cam->cmd->status != CAM_CMD_WRITE_DONE)
	{
		timeout--;
		vTaskDelay(1);
	}
	if(!timeout)
	{
		SLOGF(SLOG_DEBUG, "%s Wrote reg was timeout!", cam->name);
		return FALSE;
	}

	//! TODO: for testing push info to dump queue
	cam_regs_dump_t cam_reg;
	cam_reg.child = cam->chid;
	cam_reg.id    = cam->slave_addr;
	cam_reg.addr  = reg;
	cam_reg.size  = len;
	cam_reg.data  = pvPortMalloc(len);
	memcpy(cam_reg.data, bytes, len);
	memset(cam_reg.name, 0x00, sizeof(cam_reg.name));
	sprintf(cam_reg.name, "[CAM][REG] 0x%04x", reg);
	cam_reg.check = FALSE;
	xQueueSend(queue_cam_registers, (void *)&cam_reg, 0);

	return TRUE;
}

/*
 * cam_eeprom_read
 * The function reads data eeprom from camera
 */
void cam_eeprom_read(volatile cam_module_t *cam)
{
	if (NULL == cam)
	{
		SLOGF(SLOG_ERROR, "%s: NULL pointer ", __FUNCTION__);
		return;
	}
	else if (CAM_CH_MAX_NUM <= cam->chid || CAM_CH_A1 > cam->chid)
	{
		SLOGF(SLOG_ERROR, "%s: Invalid camera channel", __FUNCTION__);
		return;
	}
	if (cam->eeprom.len_counter < cam->eeprom.len)
	{
		/* check BUSY FLAG */
		if (!(cam->eeprom.flag & CAM_EEPROM_R_W_BUSY_FLAG))
		{
			if (I2CM_ERROR_TRANSCEIVED == i2cm_read(cam->chid, BYTE_ADDR16,
				CAM_EEPROM_SLAVE_ADD, cam->eeprom.offset,
				(uint8_t *)&cam->eeprom.tmp))
			{
				cam->eeprom.flag |= CAM_EEPROM_R_DONE_FLAG;
			}
			else
			{
				cam->eeprom.flag |= CAM_EEPROM_R_W_ERROR_FLAG;
			}
			cam->eeprom.flag |= CAM_EEPROM_R_W_BUSY_FLAG;
		}
		/* Read done. */
		if ((cam->eeprom.flag & CAM_EEPROM_R_DONE_FLAG) ||
			(cam->eeprom.flag & CAM_EEPROM_R_W_ERROR_FLAG))
		{
			if (cam->eeprom.flag & CAM_EEPROM_R_DONE_FLAG)
			{
				/* clear READ_DONE */
				cam->eeprom.flag &= ~CAM_EEPROM_R_DONE_FLAG;
			}
			else
			{
				SLOGF(SLOG_ERROR, "%s: READ EEPROM ERROR %s",\
							__FUNCTION__, cam->name);
				/* clear ERROR */
				cam->eeprom.flag &= ~CAM_EEPROM_R_W_ERROR_FLAG;
			}
			cam->eeprom.buf[cam->eeprom.len_counter++] = cam->eeprom.tmp;
			/* Increase counter. */
			cam->eeprom.offset++;
			/* clear BUSY_FLAG */
			cam->eeprom.flag &= ~CAM_EEPROM_R_W_BUSY_FLAG;
		}
	}
	else
	{
		/* READ is completed */
		cam->eeprom.flag &= ~CAM_EEPROM_MODULE_READ_FLAG;
		SLOGF(SLOG_DEBUG, "Read %d of %s success!", cam->eeprom.len, cam->name);
	#if (LOG_VERBOSE == STD_OFF)
		uint16_t scan_idx = 0;
		uint8_t blk_count = 7;
		uint8_t   *str      = assert_malloc(str, cam->eeprom.len * sizeof(uint8_t));
		uint8_t   *str_byte = assert_malloc(str_byte, 16 * sizeof(uint8_t));
		memset(str, 0, 32);
		memset(str_byte, 0, cam->eeprom.len);
		/* dump data message */
		while (scan_idx < cam->eeprom.len)
		{
			if (blk_count == 7)
			{
				blk_count = 0;
				sprintf((char *)str_byte, "\r\n\t\t\t\t\t%04X\t%02X ",
				scan_idx, cam->eeprom.buf[scan_idx]);
			}
			else
			{
				blk_count++;
				sprintf((char *)str_byte, "%02X ", cam->eeprom.buf[scan_idx]);
			}
			strcat((char *)str, (char *)str_byte);
			memset((char *)str_byte, 0, 4);
			scan_idx++;
		}
		SLOGF(SLOG_INFO, "EEPROM %s Read %d bytes data:  %s ",
						cam->name, cam->eeprom.len, str);
		/* free memory */
		vPortFree(str);
		vPortFree(str_byte);
	#endif
	}
}

/**
 * @brief cam_eeprom_write
 * The function writes data eeprom to camera module
 * @param cam - pointer to cam module
 * @return None
 */
void cam_eeprom_write(volatile cam_module_t *cam)
{
	if (cam->eeprom.len_counter < cam->eeprom.len)
	{
		/* check BUSY FLAG */
		if (!(cam->eeprom.flag & CAM_EEPROM_R_W_BUSY_FLAG))
		{
			if (I2CM_ERROR_TRANSMITTED == i2cm_write(cam->chid,
				BYTE_ADDR16, CAM_EEPROM_SLAVE_ADD, cam->eeprom.offset,
				(uint8_t *)&cam->eeprom.buf[cam->eeprom.len_counter]))
			{
				cam->eeprom.flag |= CAM_EEPROM_W_DONE_FLAG;
				//! TODO: for testing push info to dump queue
				cam_regs_dump_t cam_reg;
				cam_reg.child = cam->chid;
				cam_reg.id    = CAM_EEPROM_SLAVE_ADD;
				cam_reg.addr  = cam->eeprom.offset;
				cam_reg.size  = 1;
				cam_reg.data  = pvPortMalloc(cam_reg.size);
				memcpy(cam_reg.data,(void*)&cam->eeprom.buf[cam->eeprom.len_counter-1],
								                                          cam_reg.size);
				memset(cam_reg.name, 0x00, sizeof(cam_reg.name));
				sprintf(cam_reg.name, "[CAM][EEPROM] 0x%04x", cam->eeprom.offset);
				cam_reg.check = FALSE;
				xQueueSend(queue_cam_registers, (void *)&cam_reg, 0);
			}
			else
			{
				cam->eeprom.flag |= CAM_EEPROM_R_W_ERROR_FLAG;
			}
			/* set BUSY_FLAG */
			cam->eeprom.flag |= CAM_EEPROM_R_W_BUSY_FLAG;
		}

		if ((cam->eeprom.flag & CAM_EEPROM_W_DONE_FLAG) ||
			(cam->eeprom.flag & CAM_EEPROM_R_W_ERROR_FLAG))
		{
			if (cam->eeprom.flag & CAM_EEPROM_W_DONE_FLAG)
			{
				/* clear READ_DONE */
				cam->eeprom.flag &= ~CAM_EEPROM_W_DONE_FLAG;
			}
			else
			{
				slogf(SLOGF_ID, SLOG_ERROR, "%s: WRITE EEPROM ERROR %s",
				      __FUNCTION__, cam->name);
				/* clear ERROR */
				cam->eeprom.flag &= ~CAM_EEPROM_R_W_ERROR_FLAG;
			}
			cam->eeprom.len_counter++;
			cam->eeprom.offset++;
			/* clear BUSY_FLAG */
			cam->eeprom.flag &= ~CAM_EEPROM_R_W_BUSY_FLAG;
		}
	}
	else
	{
		/* WRITE done */
		cam->eeprom.flag &= ~CAM_EEPROM_MODULE_WRITE_FLAG;
		if(cam->eeprom.buf != NULL)
		{
			vPortFree(cam->eeprom.buf);
			cam->eeprom.buf = NULL;
		}
	}
}
/*
 * cam_ctrl_reg_group
 */
void cam_ctrl_reg_group(volatile cam_module_t *cam, uint8_t state)
{
	uint8_t mode = (state == ON ? 0x01 : 0x00);
	cam_write_reg(cam, CAM_REG_GROUP_ON_OFF, mode, DATA_8BIT);
}

/*
 * cam_eeprom_callback
 */
void cam_eeprom_callback(i2cm_error_t status, void *param)
{
	register cam_module_t *cam = (cam_module_t *)param;
	if(status == I2CM_ERROR_TRANSMITTED)
	{
		/* set READ/WRITE_DONE */
		cam->eeprom.flag |= CAM_EEPROM_W_DONE_FLAG;
		return;
	}
	else if(status == I2CM_ERROR_RECEIVED)
	{
		/* set READ/WRITE_DONE */
		cam->eeprom.flag |= CAM_EEPROM_R_DONE_FLAG;
		return;
	}
	else
	{
		cam->eeprom.flag |= CAM_EEPROM_R_W_ERROR_FLAG;
		return;
	}
}
static void cam_command_callback(i2cm_error_t status, void *param)
{
	cam_module_t *cam = (cam_module_t *)param;
	if(status == I2CM_ERROR_TRANSMITTED)
	{
		cam->cmd->status = (uint16_t)CAM_CMD_WRITE_DONE;
	}
	else if(status == I2CM_ERROR_RECEIVED)
	{
		cam->cmd->status = (uint16_t)CAM_CMD_READ_DONE;
	}
	else
	{
		printf("%s:%d: Unknown error status %04X\r\n",
						__FUNCTION__,
						__LINE__, status);
		cam->cmd->status = CAM_CMD_IDLE;
	}
}
/*
 * cam_cache_init
 */
static void cam_cache_init(volatile cam_module_t *cam)
{
	/* Read camera UUID and store on cache */
	/* Preparing to read eeprom */
	cam->eeprom.offset			= (uint16_t)CAM_EEPROM_UUID_OFFSET;
	cam->eeprom.len				= (uint16_t)CAM_EEPROM_UUID_LEN;
	cam->eeprom.flag			= 0;
	cam->eeprom.len_counter		= 0;
	cam->eeprom.buf				= (uint8_t *)cam->uuid;

	for(cam->eeprom.len_counter = 0; cam->eeprom.len_counter < cam->eeprom.len;
		cam->eeprom.len_counter++)
	{
		if(I2CM_ERROR_TRANSCEIVED == i2cm_read(cam->chid, BYTE_ADDR16,
			CAM_EEPROM_SLAVE_ADD, cam->eeprom.offset,
			(uint8_t *)&cam->eeprom.tmp))
		{
			cam->uuid[cam->eeprom.len_counter] = cam->eeprom.tmp;
			cam->eeprom.offset++;
		}
		else
		{
			SLOGF(SLOG_WARN, "%s:%d Read uuid of cam %s was failed",
				__FUNCTION__, __LINE__, cam->name);
			/* Break the loop when detect reading uuid from eeprom is failed */
			break;
		}
	}
	cam->eeprom.buf = NULL;

	/** TODO: Add other cache init here */
}

/**
 * @brief cam_update_resolution
 * The function update flag resolution
 * @param cam - pointer to cam,x-with,y-hight,ucid
 * @return int
 */
int cam_update_resolution(cam_x_resolution_type_t x, cam_y_resolution_type_t y,
											cam_module_t *cam, uint16_t ucid)
{
	/* Do not set flag if the resolution is not different from last time */
	if(cam->ucid[ucid].res.x == x && cam->ucid[ucid].res.y == y)
	{
		return 1;
	}
	switch (x)
	{
		case X_RES_3M:
		{
			if (y == Y_RES_3M)
			{
				cam->ucid[ucid].res.x = x;
				cam->ucid[ucid].res.y = y;
				cam->ucid[ucid].res.res_type = RES_3M;
				/* this value is updated from host */
				cam->ucid[ucid].host_updated |= CAM_UPDATED_RESOLUTION;
				return 1;
			}
			break;
		}
		case X_RES_13M:
		{
			if (y == Y_RES_13M)
			{
				cam->ucid[ucid].res.x = x;
				cam->ucid[ucid].res.y = y;
				cam->ucid[ucid].res.res_type = RES_13M;
				/* this value is updated from host */
				cam->ucid[ucid].host_updated |= CAM_UPDATED_RESOLUTION;
				return 1;
			}
			break;
		}
		case X_RES_720P:
		{
			if (y == Y_RES_720P)
			{
				cam->ucid[ucid].res.x = x;
				cam->ucid[ucid].res.y = y;
				cam->ucid[ucid].res.res_type = RES_720P;
				/* this value is updated from host */
				cam->ucid[ucid].host_updated |= CAM_UPDATED_RESOLUTION;
				return 1;
			}
			break;
		}
		case X_RES_1080P:
		{
			if (y == Y_RES_1080P)
			{
				cam->ucid[ucid].res.x = x;
				cam->ucid[ucid].res.y = y;
				cam->ucid[ucid].res.res_type = RES_1080P;
				/* this value is updated from host */
				cam->ucid[ucid].host_updated |= CAM_UPDATED_RESOLUTION;
				return 1;
			}
			break;
		}
		case X_RES_4K_UHD:
		{
			if (y == Y_RES_4K_UHD_CINEMA)
			{
				cam->ucid[ucid].res.x = x;
				cam->ucid[ucid].res.y = y;
				cam->ucid[ucid].res.res_type = RES_4K_UHD;
				/* this value is updated from host */
				cam->ucid[ucid].host_updated |= CAM_UPDATED_RESOLUTION;
				return 1;
			}
			break;
		}
		case X_RES_4K_CINEMA:
		{
			if (y == Y_RES_4K_UHD_CINEMA)
			{
				cam->ucid[ucid].res.x = x;
				cam->ucid[ucid].res.y = y;
				cam->ucid[ucid].res.res_type = RES_4K_CINEMA;
				/* this value is updated from host */
				cam->ucid[ucid].host_updated |= CAM_UPDATED_RESOLUTION;
				return 1;
			}
			break;
		}
		default:
			break;
		}

	return 0;
}
uint8_t cam_set_reg_continuous(cam_module_t *cam, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr,  cam_data_mode_t reg_size)
{
	//TODO: for register testing
	msm_camera_i2c_reg_array_t* array = reg_arr;
	uint16_t timeout = 20;
	uint8_t ret;
	uint16_t len = 0;
	uint32_t byte_size = (reg_arr_size >= 2) ? reg_arr_size * 2 : reg_arr_size;
	uint8_t *bytes = NULL;
	uint32_t i = 0;

	if(reg_size != DATA_8BIT && reg_size != DATA_16BIT)
	{
		SLOGF(SLOG_ERROR, "Data size is not supported");
		return 0;
	}

	bytes = (uint8_t *)assert_malloc(bytes, byte_size + 2);
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
	i2cm_transceiver(cam->chid,  cam->slave_addr,
				bytes, len,
				NULL, 0,
				cam_command_callback, (void *)cam);
	/* Wait for timeout in 10 ticks. */
	while(timeout && cam->cmd->status != CAM_CMD_WRITE_DONE)
	{
		timeout--;
		vTaskDelay(1);
	}
	if(!timeout)
	{
		SLOGF(SLOG_DEBUG, "%s Wrote the register was timeout!", cam->name);
		ret = 0;
	}
	else
	{
		ret = 1;
	}
	if(bytes)
		vPortFree(bytes);

	//TODO: for register testing
	cam_regs_dump_t cam_reg;
	for (int i = 0; i < reg_arr_size; i++)
	{
		cam_reg.child = cam->chid;
		cam_reg.id    = cam->slave_addr;
		cam_reg.addr  = array[i].reg_addr;
		cam_reg.size  = reg_size;
		cam_reg.data  = pvPortMalloc(cam_reg.size);
		memcpy(cam_reg.data,(void*)&array[i].reg_val, reg_size);
		memset(cam_reg.name, 0x00, sizeof(cam_reg.name));
		sprintf(cam_reg.name, "[CAM][REG] 0x%04x", cam_reg.addr);
		cam_reg.check = FALSE;
		xQueueSend(queue_cam_registers, (void *)&cam_reg, 0);
	}

	return ret;
}
uint8_t cam_set_reg_noncontinuous(cam_module_t *cam, uint16_t reg_arr_size,
		msm_camera_i2c_reg_array_t *reg_arr,  cam_data_mode_t reg_size)
{
	//TODO: for register testing
	msm_camera_i2c_reg_array_t* array = reg_arr;
	/* Maximum: 2 bytes address + 2 bytes data */
	uint8_t bytes[4] = {0};
	uint16_t len = 0;
	uint16_t timeout = 20;
	uint32_t i = 0;
	cam->cmd->status = CAM_CMD_IDLE;
	if(reg_size != DATA_8BIT && reg_size != DATA_16BIT)
	{
		SLOGF(SLOG_ERROR, "Data size is not supported");
		return 0;
	}
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
		i2cm_transceiver(cam->chid,  cam->slave_addr,
					bytes, len,
					NULL, 0,
					cam_command_callback, (void *)cam);
		/* Wait for timeout in 10 ticks. */
		while(timeout && cam->cmd->status != CAM_CMD_WRITE_DONE)
		{
			timeout--;
			vTaskDelay(1);
		}
		if(!timeout)
		{
			SLOGF(SLOG_DEBUG, "%s Wrote the register was timeout!", cam->name);
			return 0;
		}
		timeout = 20;
#ifdef DEBUG_CAM
		uint16_t readval = cam_read_reg(cam, reg_arr->reg_addr, reg_size);
		if(readval != reg_arr->reg_val)
			SLOGF(SLOG_ERROR, "%s Register %x. Write %x Read %x", cam->name, reg_arr->reg_addr, reg_arr->reg_val, readval);
#endif
		reg_arr++;
	}

	//TODO: for register testing
	cam_regs_dump_t cam_reg;
	for (int i = 0; i < reg_arr_size; i++)
	{
		cam_reg.child = cam->chid;
		cam_reg.id    = cam->slave_addr;
		cam_reg.addr  = array[i].reg_addr;
		cam_reg.size  = reg_size;
		cam_reg.data  = pvPortMalloc(cam_reg.size);
		memcpy(cam_reg.data,(void*)&array[i].reg_val, reg_size);
		memset(cam_reg.name, 0x00, sizeof(cam_reg.name));
		sprintf(cam_reg.name, "[CAM][REG] 0x%04x", cam_reg.addr);
		cam_reg.check = FALSE;
		xQueueSend(queue_cam_registers, (void *)&cam_reg, 0);
	}
	return 1;
}
int cam_is_fps_supported(uint16_t fps)
{
	if(fps != FPS30 && fps != FPS60 && fps != FPS24 && fps != FPS15)
		return 0;
	return 1;
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
