/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_H
#define __PLATFORM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "CCBConfig.h"
#include "stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "stm32f4xx_conf.h"
#include "FreeRTOS.h"
#include "task.h"

/* Exported typedef ----------------------------------------------------------*/
typedef enum
{
	NO_INTERRUPT						= (uint16_t)0x0000,
	TEMPERATURE_EXCEED					= (uint16_t)0x0001,
	VCM_FINISH_MOVING					= (uint16_t)0x0002,
	LENS_FINISH_MOVING					= (uint16_t)0x0003,
	MIRROR_FINISH_MOVING				= (uint16_t)0x0004,
	PREVIEW_SWITCH_A_TO_B				= (uint16_t)0x0010,
	PREVIEW_SWITCH_B_TO_C				= (uint16_t)0x0011,
	CCB_CMD_LACK_PARAMETER				= (uint16_t)0x0020,
	CCB_CMD_UNKNOWN						= (uint16_t)0x0021,
	CCB_CMD_HANDLING_ERR				= (uint16_t)0x0022,
	CCB_CMD_CAM_MODULE_OPEN 			= (uint16_t)0x0040,
	CCB_CMD_CAM_STREAMING 				= (uint16_t)0x0080,
	CCB_CMD_RDI_CAPTURE 				= (uint16_t)0x0100,
	CCB_CMD_CAM_MODULE_FOCUS_DISTANCE 	= (uint16_t)0x0200,
	CCB_CMD_LIGHT_ACTIVE_UCID 			= (uint16_t)0x0400,
	CCB_POWER_NOT_GOOD					= (uint16_t)0x1000
} interrupt_event;

typedef struct
{
	unsigned int current_status;
	unsigned int previous_status;
} interrupt_status;
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
 *  \brief Function to initialize the platform
 *
 *  \par Header File:
 *  platform.h
 *
 *  \par Description:
 *  This function initialize asic, memory, devices, peripherals, libraries, etc
 */
void PlatformInit(void);

/**
 *  \brief Function to initialize the platform
 *
 *  \par Header File:
 *  platform.h
 *
 *  \par Description:
 *  This function triggers an interrupt signal to host
 *  para: event of interrupt indicates the cause of interrupt
 */
void Trigger_STM_IRQ(uint32_t event);

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_H */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE****/
