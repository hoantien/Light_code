/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    task_af_ctrl.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Aug-15-2016
 * @brief   Task for auto-focus/auto-exposure command execution
 *
 ******************************************************************************/

#include "assert.h"
#include "log.h"
#include "task_af_ctrl.h"
#include "contrast_sweep_manager.h"
#include "af_roi_transfer.h"
#include "calib_data_manager.h"
#include "optical_zoom_manager.h"
#include "af_ae.h"

#define SLOGF_ID                SLOG_ID_AF

void task_af_create(void)
{
    /* Attempt to create the command queue. */
    light_system->af_ctrl->queue = xQueueCreate(10, sizeof(lcc_cmd_tid_t));
    assert_param(light_system->af_ctrl->queue);

    /* AF task creation */
    BaseType_t ret = xTaskCreate(
                        task_af_ctrl,          /* task function */
                        "af_ctrl",              /* task name */
                        __TASK_STACK_SIZE_2048, /* stack size */
                        NULL,                   /* passing parameters */
                        __TASK_PRIO_HIGHEST - 6,/* task priority */
                        NULL                    /* handle */
                    );
    /* Assert the creation task return to make sure all of
        camera task was created */
    assert_param(pdPASS == ret);
}

void task_af_ctrl(void *vParameter)
{
    /* Task start */
    taskENTER_CRITICAL();
    log_msg("Start %s\r\n", __FUNCTION__);
    taskEXIT_CRITICAL();

    initialize_optical_zoom_manager();
    initialize_calib_data_manager();

    while (1)
    {
        lcc_cmd_tid_t pcmd;
        xQueueReceive(light_system->af_ctrl->queue, &pcmd, portMAX_DELAY);

        SLOGF(SLOG_DEBUG, "Before AF cmd exec main heap free bytes: %d", (int)xPortGetFreeHeapSize());

        uint32_t cam_channel_bitmask = pcmd.event & AF_EVENT_FULL_SWEEP_ALL;
#if (ASIC_NUM == ASIC1)
        cam_typedef_t *pcam = NULL;
        int cam;
        uint32_t cs;
        for (cam = 1; cam < modules_tlb_size; cam++)
        {
        	pcam = idx_to_object(cam);
        	if (pcam != NULL)
        	{
        		memcpy(&cs, pcam->settings->status, sizeof(uint32_t));
        		SLOGF(SLOG_DEBUG, "pcam %x", pcam->info.module);
        		if (cs & S_MODULE_STREAM_ON)
        		{
                    cam_ctrl_stream(pcam);
                    vTaskDelay(1);
        			break;
        		}
        	}
        	// No streaming camera found
        	pcam = NULL;
        }
#endif
        uint32_t bitmask = 0;
        if (cam_channel_bitmask != 0)
            bitmask = contrast_full_sweep(cam_channel_bitmask);
        else if (pcmd.event & AF_EVENT_AFAE) {
            execute_af_ae();
        } else {
            cam_channel_bitmask = pcmd.event & AF_EVENT_ROI_TRANSFER_ALL;
            cam_channel_bitmask >>= AF_EVENT_ROI_TRANSFER_CH0_BIT;
            if (cam_channel_bitmask != 0)
                bitmask = af_roi_transfer_execute(cam_channel_bitmask);
        }
#if (ASIC_NUM == ASIC1)

        // Set resolution
        if (pcam != NULL)
        {
        	// Stream off the camera
        	cam_ctrl_stream(pcam);
            // Stream on camera
            if (pcam->info.module == 0xA1)
                ucid_preview_hdl(0x02, 1);
            if (pcam->info.module == 0xB4)
                ucid_preview_hdl(0x200, 1);
            if (pcam->info.module == 0xC5)
                ucid_preview_hdl(0x8000, 1);
            pcam->settings->stream[0] |= CAM_STREAM_ENABLE;
        }
        else
        {
        	 SLOGF(SLOG_DEBUG, "pcam NULL");
        }
#endif
        lcc_cmd_log_update_status(pcmd.cmd_tid, LCC_CMD_SUCCESS, bitmask);
        SLOGF(SLOG_DEBUG, "Command status updated");

        SLOGF(SLOG_DEBUG, "After AF cmd exec main heap free bytes: %d", (int)xPortGetFreeHeapSize());
    }
}
