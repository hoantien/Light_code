/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_ccb_ctrl.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains header of task_ccb_ctrl
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_CCB_CTRL_H__
#define __TASK_CCB_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "lcc_cmd_log.h"
/* Exported define -----------------------------------------------------------*/
#define CCB_EVENT_PREVIEW				BIT0
#define CCB_EVENT_SNAPSHOT_HDR			BIT1
#define CCB_EVENT_STREAM_UCID_DEBUG		BIT2
#define CCB_EVENT_FLASH					BIT3
#define CCB_EVENT_TOF					BIT4
#define CCB_EVENT_GYRO					BIT5
#define CCB_EVENT_POWER					BIT6
#define CCB_EVENT_ALLS					((CCB_EVENT_POWER<<1) - 1)

/* Exported typedef  ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief task_cam_ctrl
 * This task will process all updated data of cameras
 * @param vParameter: point to parameter if used
 * @return None
 */
void task_ccb_ctrl(void *vParameter);

#ifdef __cplusplus
}
#endif
#endif /* __TASK_CCB_CTRL_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
