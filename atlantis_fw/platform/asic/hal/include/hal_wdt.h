/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    hal_wdt.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Feb-18-2016
 * @brief   This file contains definitions of the Watchdog driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_WDT_H__
#define __HAL_WDT_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported Define------------------------------------------------------------*/
/* Exported typedef  ---------------------------------------------------------*/

/**
 * @brief wdt_status_t
 * Watchdog status
 */
typedef enum wdt_status
{
	HAL_WDT_OK,
	HAL_INVALID_PERIOD
} wdt_status_t;
/* Exported functions---------------------------------------------------------*/

/**
 * @brief  hal_wdt_init
 * The function initializes Watchdog
 * @param  clb_func call back function to upper layer.
 * @param  priority priority of the interrupt Watchdog.
 * @retval None
 */
void hal_wdt_init(uint32_t timeout, void(*clb_func)(void), uint8_t priority);

/**
 * @brief  hal_wdt_start
 * The function starts Watchdog
 * @param  None
 * @retval None
 */
void hal_wdt_start(void);

/**
 * @brief  hal_wdt_stop
 * The function stops Watchdog
 * @param  None
 * @retval None
 */
void hal_wdt_stop(void);

/**
 * @brief	wdt_set_timeout
 * The function sets timeout for Watchdog
 * @param  period period of Watchdog
 * @retval  refer to wdt_status_t
 */
wdt_status_t wdt_set_timeout(uint32_t period);

/**
 * @brief hal_wdt_kick_dog
 * The function is used to restart the Watchdog counter.
 */
void hal_wdt_kick_dog(void);
#ifdef __cplusplus
}
#endif
#endif /* __HAL_WDT_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
