/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    mailbox.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jun 15, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**							Revision history
 *
 * * 1.0.0	Jun 15, 2016	Initial revision:
 */
/******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAILBOX_H_
#define MAILBOX_H_
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "message.h"
#include "qc_common.h"
#include "FreeRTOS.h"
#include "semphr.h"
/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
		uint16_t          size;
		volatile uint16_t top;
		volatile uint16_t bottom;
		volatile bool     repeat;
		message_t**       inbox;
		xSemaphoreHandle  sempr;
}mailbox_t;
/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief To receive a message from mailbox
 * @param[in] 	mailbox	: indicated mailbox
 * @param[out] 	NA
 * @return 		NULL	: mailbox is empty
 *         		Others	: received message
 * @details To receive a message from mailbox
 */
IMPORT message_t* mail_receive(mailbox_t* mailbox);
/**
 * @brief To send a message to indicated mailbox
 * @param[in] 	mailbox	: indicated mailbox
 * @param[out] 	NA
 * @return 		NULL	: mailbox is empty
 *         		Others	: received message
 * @details To receive a message from mailbox
 */
IMPORT std_return_t mail_send(message_t* msg, mailbox_t* mailbox);

#endif /* MAILBOX_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
