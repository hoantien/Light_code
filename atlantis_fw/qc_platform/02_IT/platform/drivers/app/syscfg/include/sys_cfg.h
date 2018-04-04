/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    sys_cfg.h
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
#ifndef SYS_CFG_H_
#define SYS_CFG_H_
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "qc_common.h"
#include "mailbox.h"
#include "os.h"
/* Exported define -----------------------------------------------------------*/
#define SYS_TASK_NUM         9

#define SYS_MAILBOX_SIZE      256
#define STORAGE_MAILBOX_SIZE  128
#define COM_SRV_MAILBOX_SIZE  512

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/**
 * Task IDs
 */
IMPORT uint16_t log_task_id      ;
IMPORT uint16_t bash_task_id     ;
IMPORT uint16_t com_srv_task_id  ;
IMPORT uint16_t storage_task_id  ;
IMPORT uint16_t sys_ctrl_task_id ;
IMPORT uint16_t spi_slave_task_id;
IMPORT uint16_t i2c_slave_task_id;
/**
 * Mailboxes
 */
IMPORT mailbox_t   sys_ctrl_mailbox;
IMPORT mailbox_t   com_srv_mailbox ;
IMPORT mailbox_t   storage_mailbox ;

/**
 * Tasks
 */
IMPORT task_handle_t  sys_task_handler[SYS_TASK_NUM];
IMPORT const task_configure_t sys_task_list[SYS_TASK_NUM];
/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief Configure none-volatile system information
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Configure none-volatile system information
 */
IMPORT void sys_config(void);
/**
 * @brief task_query_tid
 * The function will query task id from task name
 * @param task_name - name of task to get task id
 * @return None
 */
IMPORT int xGetTaskID(const char * const name, \
		              const task_configure_t* list, uint16_t size);
/**
 * @brief Get task state
 * @param[in] 	NA
 * @param[out] 	NA
 * @return 		NA
 * @details Get task state
 */
IMPORT task_state_t xGetTaskState(const char * const name);
#endif /* SYS_CFG_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
