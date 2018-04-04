/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_af_ctrl.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Aug-15-2016
 * @brief   Task for auto-focus/auto-exposure command execution
 *
 ******************************************************************************/

#ifndef __TASK_AF_CTRL_H__
#define __TASK_AF_CTRL_H__

#include "os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AF_EVENT_FULL_SWEEP_CH0         (1 << 0)
#define AF_EVENT_FULL_SWEEP_CH5         (1 << 5)
#define AF_EVENT_FULL_SWEEP_ALL         (((AF_EVENT_FULL_SWEEP_CH5 << 1) - 1) ^ (AF_EVENT_FULL_SWEEP_CH0 - 1))
#define AF_EVENT_ROI_TRANSFER_CH0_BIT   6
#define AF_EVENT_ROI_TRANSFER_CH0       (1 << AF_EVENT_ROI_TRANSFER_CH0_BIT)
#define AF_EVENT_ROI_TRANSFER_CH5       (1 << 11)
#define AF_EVENT_ROI_TRANSFER_ALL       (((AF_EVENT_ROI_TRANSFER_CH5 << 1) - 1) ^ (AF_EVENT_ROI_TRANSFER_CH0 - 1))
#define AF_EVENT_AFAE                   (1 << 12)

#define AF_EVENT_ALL                    ((AF_EVENT_AFAE << 1) - 1)

/*
 * @brief af_ctrl_t
 * AF control
 */
typedef struct
{
    QueueHandle_t queue;
} af_ctrl_t;


/**
 * @brief task_af_create
 * Creates auto-focus task
 * @param   None
 * @return  None
 */
void task_af_create(void);

/**
 * @brief task_af_ctrl
 * This task will process all auto-focus/auto-exposure commands
 * @param vParameter: point to parameter if used
 * @return None
 */
void task_af_ctrl(void *vParameter);

#ifdef __cplusplus
}
#endif
#endif /* __TASK_AF_CTRL_H__ */
