/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    i2c_master.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    April-07-2016
 * @brief   This file contains expand for I2C master driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "i2c_master.h"
#include "log.h"
#include "os.h"
#include "assert.h"
#include <string.h>
#include "hal_i2c.h"
#include "board_config.h"
#include "hal_vic.h"
#include "camera.h"
/* Private define------------------------------------------------------------*/
#define SLOGF_ID			11
#define I2C_CTRL_MSG_SIZE	32
#define MSG_QUEUE_WAIT		10
/* Private typedef------------------------------------------------------------*/
/**
 *
 */
typedef enum i2c_ctrl_sm
{
	SM_IDLE,
	SM_EXE,
	SM_WAIT,
} i2c_ctrl_sm_t;
/**
 *
 */
typedef struct i2c_ctrl_msg
{
	uint8_t				chid;			/* I2C channel. */
	uint8_t				slave_addr;		/* I2C slave address. */
	hal_i2c_buffer_t	buf;
	i2c_ctrl_access_t	rw;	/* Read/Write access. */
	void (*is_completed_clbk)(hal_i2c_status_t status, void *param);
	void *param; /* passing parameter. */
} i2c_ctrl_msg_t;
/* Private define-------------------------------------------------------------*/
/* Private macro--------------------------------------------------------------*/
/* Private function-----------------------------------------------------------*/
static void i2c_forwards_callback(hal_i2c_status_t status);
static void i2c_master_forwards_init(void);
/* Private variables----------------------------------------------------------*/
static QueueHandle_t msg_queue;
static xSemaphoreHandle msg_event;
static QueueHandle_t clbk_queue;
static volatile i2c_ctrl_sm_t i2c_ctrl_sm;
static i2c_ctrl_msg_t msg_received;
static hal_i2c_t i2c_forwards_asic =
{
	.clock_speed	=	I2C_SPEED_400KHz,
	.address_mode	=	I2C_7BIT,
	.operation_mode	=	I2C_MASTER,
	.owner_addr		=	0x00,
	.irq_handler	=	i2c_forwards_callback,
};
/* Mock variable -------------------------------------------------------------*/
uint8_t *i2c_ctrl_push_msg_data;
uint8_t i2c_ctrl_push_msg_cam_status;
uint8_t i2c_ctrl_push_msg_count;

/* Exported functions --------------------------------------------------------*/
i2c_ctrl_status_t i2c_ctrl_push_msg(
	uint8_t chid,
	uint8_t slave_addr,
	uint8_t *data,
	uint16_t length,
	i2c_ctrl_access_t rw,
	void *is_completed_clbk,
	void *param)
{
	i2c_ctrl_push_msg_count++;
	i2c_ctrl_push_msg_data = malloc(length*sizeof(uint8_t));

	for (int i=0;i<length;i++)
		*(i2c_ctrl_push_msg_data+i) = *(data+i);
	if((i2c_ctrl_push_msg_cam_status & 0x01) ==1)
	{
		if(i2c_ctrl_push_msg_count >= 2 && \
				(i2c_ctrl_push_msg_cam_status & 0x02) ==2)
			((cam_module_t *)param)->cmd->status = CAM_CMD_READ_DONE ;
		else
			((cam_module_t *)param)->cmd->status = CAM_CMD_WRITE_DONE ;
	}
}
void i2c_ctrl_task(void *param)
{}
static void i2c_forwards_callback(hal_i2c_status_t status)
{}
static void i2c_master_forwards_init(void)
{}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
