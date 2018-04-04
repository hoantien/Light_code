/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    lcc_cmd_log.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains expand of lcc_cmd_log
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "os.h"
#include "log.h"
#include "assert.h"
#include "lcc_cmd_log.h"
#include "lcc_system.h"
#include "lcc_cmd.h"
#include "light_system.h"

/* Private define-------------------------------------------------------------*/
#define SLOGF_ID					SLOG_ID_LCC_CMD_LOG

/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
lcc_cmd_log_queue_t *lcc_cmd_log;		/* Command log */

/* Exported functions --------------------------------------------------------*/
/*
 * lcc_cmd_log_init
 */
void lcc_cmd_log_init(void)
{
	lcc_cmd_log = (lcc_cmd_log_queue_t *)assert_malloc(lcc_cmd_log,
											sizeof(lcc_cmd_log_queue_t));
	lcc_cmd_log->head = NULL;
	lcc_cmd_log->tail = NULL;
	lcc_cmd_log->total = 0;
	lcc_cmd_log->semaphore = xSemaphoreCreateMutex();
	assert(lcc_cmd_log->semaphore != NULL);
}

/*
 * lcc_cmd_log_push
 * Push log message
 */
int lcc_cmd_log_push(uint16_t cmd_tid, uint16_t status, uint32_t cmd_bitmask,
						uint8_t intr_en)
{
	if(status != LCC_CMD_PENDING && cmd_bitmask == 0 && intr_en == TRUE)
	{
		intr_queue_push(cmd_tid, status);
		return 1;
	}
	uint8_t ret = -1;
	struct lcc_cmd_log_t *buf;

	if (lcc_cmd_log->total > LCC_CMD_LOG_SIZE)
	{
		SLOGF(SLOG_ERROR, "%s [%d] Too many pending commands in queue",
				__FUNCTION__, __LINE__);
		xSemaphoreTake(lcc_cmd_log->semaphore, portMAX_DELAY);
		/* Remove the oldest transaction */
		buf = lcc_cmd_log->head;
		lcc_cmd_log->head = lcc_cmd_log->head->next;
		SLOGF(SLOG_ERROR, "Removing TID 0x%x from queue",buf->cmd_tid);
		vPortFree(buf);
		lcc_cmd_log->total--;
		xSemaphoreGive(lcc_cmd_log->semaphore);
	}

	buf = (struct lcc_cmd_log_t *) assert_malloc(buf, sizeof(*buf));
	/* COPY data to here */
	/* transaction ID */
	buf->cmd_tid = cmd_tid;
	buf->status  = status;
	buf->cmd_bitmask = cmd_bitmask;
	buf->intr_en = intr_en;
	xSemaphoreTake(lcc_cmd_log->semaphore, portMAX_DELAY);
	if (lcc_cmd_log->head == NULL)
	{
		lcc_cmd_log->head = buf;
		lcc_cmd_log->tail = lcc_cmd_log->head;
	}
	else
	{
		lcc_cmd_log->tail->next = buf;
		lcc_cmd_log->tail = buf;
	}
	buf->next = NULL;
	/* increment total msg is pushed */
	lcc_cmd_log->total++;
	xSemaphoreGive(lcc_cmd_log->semaphore);
	ret = 0;
	return ret;
}

/*
 * lcc_cmd_log_pull
 * The function pulls log message from queue
 */
struct lcc_cmd_log_t *lcc_cmd_log_pull(uint16_t cmd_tid)
{
	struct lcc_cmd_log_t *buf = NULL;
	xSemaphoreTake(lcc_cmd_log->semaphore, portMAX_DELAY);
	/* scan list from head to tail */
	buf = lcc_cmd_log->head;

	while (buf != NULL)
	{
		if (buf->cmd_tid == cmd_tid)
		{
			break;
		}
		else
		{
			buf = buf->next;
		}
	}
	xSemaphoreGive(lcc_cmd_log->semaphore);
	return buf;
}

/*
 * lcc_cmd_log_update_status
 * The function update command status
 */
void lcc_cmd_log_update_status(uint16_t cmd_tid, uint16_t status,
									uint32_t cam_bitmask)
{
	struct lcc_cmd_log_t *cmd_log_ptr = lcc_cmd_log_pull(cmd_tid);
	/* check command available in cmd log */
	if (cmd_log_ptr)
	{
		uint32_t old_status = cmd_log_ptr->status;
		uint32_t old_cmd_bitmask = cmd_log_ptr->cmd_bitmask;

		old_cmd_bitmask &= ~cam_bitmask;
		if(old_status & ~(LCC_CMD_SUCCESS | LCC_CMD_PENDING))
		{
			/* Error occured */
			if(status != LCC_CMD_SUCCESS)
			{
				old_status |= status;
				old_status &= ~(LCC_CMD_SUCCESS | LCC_CMD_PENDING);
			}
			else
			{
				/* remain the current status*/
			}

		}
		else
		{
			/* there was no error before */
			if(old_status == LCC_CMD_PENDING)
			{
				old_status = status;
			}
			else
			{
				if(status != LCC_CMD_SUCCESS)
				{
					old_status = status;
				}
				else
				{
					if(old_cmd_bitmask)
					{
						old_status = LCC_CMD_PENDING;
					}
					else
					{
						old_status = LCC_CMD_SUCCESS;
					}
				}
			}
		}

		if(old_cmd_bitmask == 0)
		{
			if (cmd_log_ptr->intr_en == TRUE)
				intr_queue_push(cmd_tid, old_status);
			lcc_cmd_log_delete_tid(cmd_log_ptr);
		}
		else
		{
			cmd_log_ptr->status = old_status;
			cmd_log_ptr->cmd_bitmask = old_cmd_bitmask;
		}
	}
}

/*
 * lcc_cmd_log_delete_tid
 * The function deletes log has specyfic transaction id
 */
void lcc_cmd_log_delete_tid(struct lcc_cmd_log_t *n)
{
	if(lcc_cmd_log->head == NULL || n == NULL)
	{
		SLOGF(SLOG_ERROR, "%s [%d] Invalid head or node",
				__FUNCTION__, __LINE__);
		return;
	}
	xSemaphoreTake(lcc_cmd_log->semaphore, portMAX_DELAY);
	if (lcc_cmd_log->head == n)
	{
		if (lcc_cmd_log->head == lcc_cmd_log->tail)
		{
			lcc_cmd_log->tail = NULL;
		}
		lcc_cmd_log->head = lcc_cmd_log->head->next;
	}
	else
	{
		/* find the previous node */
		struct lcc_cmd_log_t *prev = lcc_cmd_log->head;

		while (prev != NULL && prev->next != n)
		{
			prev = prev->next;
		}

		/* Check if node really exists in Linked List */
		if (prev == NULL)
		{
			SLOGF(SLOG_ERROR, "%s:%d: Given node is not present in Linked List",
					__FUNCTION__, __LINE__);
			xSemaphoreGive(lcc_cmd_log->semaphore);
			return;
		}
		/* Remove node from Linked List */
		prev->next = n->next;
		/* Update the tail if we delete the last element */
		if (n == lcc_cmd_log->tail)
		{
			lcc_cmd_log->tail = prev;
		}
	}
	/* Free memory */
	vPortFree(n);
	lcc_cmd_log->total--;
	xSemaphoreGive(lcc_cmd_log->semaphore);

}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
