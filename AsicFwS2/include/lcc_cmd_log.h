/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    lcc_cmd_log.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-10-2016
 * @brief   This file contains header of __lcc_cmd_log
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCC_CMD_LOG_H__
#define __LCC_CMD_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief lcc_cmd_log_t
 */
struct lcc_cmd_log_t {
	uint16_t cmd_tid;		/* Command Transaction ID */
	uint32_t cmd_bitmask;	/* Command Bitmask */
	uint32_t status;		/* Command Status */
	uint8_t intr_en;	/* Interrupt flag */
	struct lcc_cmd_log_t *next;
};

/**
 * @brief lcc_cmd_log_queue_t
 */
typedef struct __lcc_cmd_log_queue {
	struct lcc_cmd_log_t *head;
	struct lcc_cmd_log_t *tail;
	uint32_t total;
	SemaphoreHandle_t	semaphore;
} lcc_cmd_log_queue_t;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief lcc_cmd_log_init
 * Light LCC system log init
 * @param None
 * @return None
 */
void lcc_cmd_log_init(void);

/**
 * @brief lcc_cmd_log_push
 * Push log message
 * @param cmd_tid - command transaction ID
 * @param status - Command status
 * @return 0 Success, -1 Full log or Error
 */
int lcc_cmd_log_push(uint16_t cmd_tid, uint16_t status, uint32_t cmd_bitmask,
						uint8_t intr_en);

/**
 * @brief lcc_cmd_log_pull
 * The function pulls log message from queue
 * @param cmd_tid - Command transaction id
 * @return log_msg - Command log message, ref at #lcc_cmd_log_t
 */
struct lcc_cmd_log_t *lcc_cmd_log_pull(uint16_t cmd_tid);

/**
 * @brief lcc_cmd_log_update_status
 * The function update command status
 * @param cmd_tid - command transaction ID
 * @param status - command status log
 * @return None
 */
void lcc_cmd_log_update_status(uint16_t cmd_tid, uint16_t status,
								uint32_t cam_bitmask);

/**
 * @brief lcc_cmd_log_delete_tid
 * The function deletes log has specyfic transaction id
 * @param cmd_tid - Command transaction id
 * @return 0 Success, -1 Full log or Error
 */
void lcc_cmd_log_delete_tid(struct lcc_cmd_log_t *n);
/**
 * @brief lcc_cmd_log_delete_multiple
 * The function deletes all command has status is the same
 * @param status - command status log
 * @return 0 Success, -1 Full log or Error
 */
int lcc_cmd_log_delete_multiple(uint16_t status);

#ifdef __cplusplus
}
#endif
#endif /* __LCC_CMD_LOG_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
