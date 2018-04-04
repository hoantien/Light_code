/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_watchdog_kick.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains header of task_watchdog_kick
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_WATCHDOG_KICK_H__
#define __TASK_WATCHDOG_KICK_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported defines  ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
 * @brief task_watchdog_kick
 * This task is used to control WDT and reset CPU if needed
 * @param vParameter: point to parameter if used
 * @return None
 */
void task_watchdog_kick(void *vParameter);

#ifdef __cplusplus
}
#endif
#endif /* __TASK_WATCHDOG_KICK_H_ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
