/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    sys_ctrl.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June 20, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June 20, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "lcc_system.h"
#include "sys_ctrl.h"
#include "log.h"
#include "os.h"
#include "message.h"
#include "mailbox.h"
#include "sys_cfg.h"
#include "bash.h"
/**
 * Include module headers
 */
//#include "it_drv_camera.h"
//#include "it_drv_ar1335.h"
//#include "it_drv_flash.h"
#include "it_drv_hall_sensor.h"
#include "it_drv_i2cm.h"
#include "it_drv_ina231.h"
#include "it_drv_temp_sensor.h"
//#include "it_drv_timer.h"
//#include "it_drv_vcm.h"

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/
/**
 * @brief Handle message from commander
 * @param[in] 	msg	: coming message
 * @param[out] 	NA
 * @return 		NA
 * @details system control task
 */
LOCAL int message_handler(message_t* msg);
/**
 * @brief Parsing system control parameters from commander
 * @param[in] 	cmd		: coming command
 * @param[out] 	argc	: argument counter
 * @param[out] 	argv	: argument parameter
 * @return		E_OK 	: success
 *              Others	: failed
 * @details Parsing system control parameters from commander
 */
LOCAL std_return_t sys_command_parser(uint8_t* cmd, \
		                                       uint8_t* argv[], uint16_t* argc);

/* Private variables ---------------------------------------------------------*/
LOCAL it_map_t sys_comps[] =
{
//		{"AR1335", it_ar1335_handler           },
//		{"CAMERA", it_drv_camera_handler       },
//		{"FLASH" , it_drv_flash_handler        },
		{"HALL"  , it_drv_hall_sensor_handler  },
		{"I2C"   , it_drv_i2cm_handler         },
//		{"SPI"   , it_drv_ina231_handler       },
		{"INA231", it_drv_ina231_handler       },
		{"TEMP"  , it_drv_temp_sensor_handler  },
//		{"VCM"   , it_drv_vcm_handler          },
//		{"TIMER" , it_drv_timer_handler        },
		{""      , NULL                        }
};
/* Exported global variables --------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Initializing system
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Initializing system
 */
void sys_init(void)
{
	BaseType_t xReturn = pdPASS;
	int i;
	//! Initialize LCC system configuration
	lcc_system_init();
	//! Configure system
	sys_config();
	//! Initialize platform
	init_platform();
	//! Initialize bash
	bash_init();
	//! Initialize all task
	for(i = 0; i < SYS_TASK_NUM; i++)
	{
		sys_task_handler[i].idx =  i;
		sys_task_handler[i].state = TASK_INITIALIZE;
		sys_task_handler[i].time_sleep = sys_task_list[i].time_sleep;
		//! Create task
		if(pdPASS != xTaskCreate(sys_task_list[i].task,
				                 sys_task_list[i].name,
								 sys_task_list[i].stacksize,
								 &sys_task_handler[i],
								 sys_task_list[i].prio,
								 &sys_task_handler[i].handle))
		{
			printf("Could not create task %s. Exit\r\n", sys_task_list[i].name);
			xReturn = pdFAIL;
			break;
		}
	}
	/* Start the scheduler so our tasks start executing. */
	if (pdPASS == xReturn)
	{
		vTaskStartScheduler();
	}
	else
	{
		while(1);
	}
}
/**
 * @brief system control task
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details system control task
 */
void task_sysctrl(void* pv)
{
	//! Start bash/shell task
	task_handle_t *handler = (task_handle_t *)(pv);
	//! Waiting for log task ready
	while (TASK_READY != xGetTaskState("slog"))
	{
		//! Call task delay to let the others run
		vTaskDelay(1);
	}
	//! Indicate task is ready
	handler->state = TASK_READY;
	taskENTER_CRITICAL();
	log_msg("[Task] %s() is ready.\r\n", __FUNCTION__);
	taskEXIT_CRITICAL();
	while(1)
	{
		message_t* msg = mail_receive(&sys_ctrl_mailbox);
		if (NULL != msg)
		{
			//! Call message handler
			if (-1 == message_handler(msg))
			{
				//! Test FAILED
				slogf(SLOG_ID_LCC_SYSTEM, SLOG_INFO,   \
					      "* Test result: %s: %s\r\n", \
					 msg->content.data.body, "FAILED");
			}
			else
			{
				//! Test PASSED
				slogf(SLOG_ID_LCC_SYSTEM, SLOG_INFO,   \
					      "* Test result: %s: %s\r\n", \
					 msg->content.data.body, "PASSED");
			}
			//! Free message
			delete_msg(msg);
		}
		//! Call task delay to let the others run
		vTaskDelay(1);
	}
}

/**
 * @brief Handle message from commander
 * @param[in] 	msg	: coming message
 * @param[out] 	NA
 * @return 		NA
 * @details system control task
 */
LOCAL int message_handler(message_t* msg)
{
	uint8_t* argv[128];
	uint16_t argc = 0;
	char cmd[128];
	//! clear cmd buffer
	memset(cmd, 0x00, 128);
	//! Save message
	strcpy(cmd, (char*)msg->content.data.body);
	//! Call command parser
	sys_command_parser((uint8_t*)cmd, argv, &argc);
	//! Call handler_parser
	int i = handler_parser((char*)argv[0], sys_comps);
	//! Check parsing result
	if (i == -1)
	{
		slogf(SLOG_ID_LCC_SYSTEM, SLOG_ERROR,"Invalid module: %s\r\n", argv[0]);
		return -1;
	}
	//! Call system handler
	return sys_comps[i].handler((char**)&argv[1], argc-1);
}

/**
 * @brief Parsing system control parameters from commander
 * @param[in] 	cmd		: coming command
 * @param[out] 	argc	: argument counter
 * @param[out] 	argv	: argument parameters
 * @return		E_OK 	: success
 *              Others	: failed
 * @details Parsing system control parameters from commander
 */
LOCAL std_return_t sys_command_parser(uint8_t* cmd, uint8_t* argv[], uint16_t* argc)
{
	if (NULL == cmd) return E_PARAM;
	if (NULL == argv) return E_PARAM;
	if (NULL == argv) return E_PARAM;
	uint8_t* ptr = cmd;
	uint16_t params = 0;
	//! Get first parameter
	argv[params++] = cmd;
	//! Get parameter table
	while (*ptr != 0)
	{
		if (*ptr == ',')
		{
			argv[params++] = ptr + 1;
			*ptr = 0;
		}
		ptr++;
	}
	*argc = params;
	return E_OK;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
