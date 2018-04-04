/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    sys_ctrl.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    June 15, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 *
 * * 1.0.0	June 15, 2016	Initial revision:
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SYS_CTRL_H_
#define SYS_CTRL_H_
/* Includes ------------------------------------------------------------------*/
#include "qc_common.h"
#include "board_config.h"
#include "sys_call.h"
#include "os.h"
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief Initializing system
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Initializing system
 */
IMPORT void sys_init(void);
/**
 * @brief system control task
 * @param[in] 	pv	: own task handler
 * @param[out] 	NA
 * @return 		NA
 * @details system control task
 */
IMPORT void task_sysctrl(void* pv);
#endif /* SYS_CTRL_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
