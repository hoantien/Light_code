/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "platform.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Devices includes. */
#include "log.h"
/* Exported typedef -----------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/

/**
 * USER_TIMEOUT value for waiting loops. This timeout is just a guarantee that the
 * application will not remain stuck if the I2C communication is corrupted.
 * You may modify this timeout value depending on CPU frequency and application
 * conditions (interrupts routines, number of data to transfer, speed, CPU
 * frequency...).
 */
#define USER_TIMEOUT                  ((uint32_t)0x64) /* Waiting 1s */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************ Portions COPYRIGHT 2015 Light.Co., Ltd.******END OF FILE******/
