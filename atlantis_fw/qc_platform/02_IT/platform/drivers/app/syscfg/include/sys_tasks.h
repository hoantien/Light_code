/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    sys_tasks.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June 17, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 *
 * * 1.0.0	June 17, 2016	Initial revision:
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYS_TASKS_H_
#define SYS_TASKS_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
#include "sys_cfg.h"
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* task handler */
IMPORT task_handle_t  sys_task_handler[SYS_TASK_NUM];
/* task list configure */
IMPORT const task_configure_t sys_task_list[SYS_TASK_NUM];
/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

#endif /* SYS_TASKS_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
