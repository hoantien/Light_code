/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    mailbox.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    June 15, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	June 15, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "mailbox.h"
#include "sys_cfg.h"
#include "semphr.h"
#include "FreeRTOS.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported global variables --------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief To receive a message from mailbox
 * @param[in] 	mailbox	: indicated mailbox
 * @param[out] 	NA
 * @return 		NULL	: mailbox is empty
 *         		Others	: received message
 * @details To receive a message from mailbox
 */
message_t* mail_receive(mailbox_t* mailbox)
{
	//! Take semaphore for target mailbox
	xSemaphoreTake(mailbox->sempr, 5);

	bool err = FALSE;
	message_t* msg = NULL;

	//! Check mailbox status
	if (mailbox->inbox == NULL)
		err = TRUE;

	if ((!mailbox->repeat) \
	  && (mailbox->top == mailbox->bottom))
		err = TRUE;

	//! Get message
	if (FALSE == err)
	{
		msg = mailbox->inbox[mailbox->bottom];

		//! Free slot for message in target mailbox
		mailbox->inbox[mailbox->bottom++] = NULL;

		if (mailbox->bottom == mailbox->size)
			mailbox->repeat = FALSE;
	}

	//! Release semaphore
	xSemaphoreGive(mailbox->sempr);

	return msg;
}
/**
 * @brief To send a message to indicated mailbox
 * @param[in] 	mailbox	: indicated mailbox
 * @param[out] 	NA
 * @return 		NULL	: mailbox is empty
 *         		Others	: received message
 * @details To receive a message from mailbox
 */
std_return_t mail_send(message_t* msg, mailbox_t* mailbox)
{
	//! Take semaphore for target mailbox
	xSemaphoreTake(mailbox->sempr, 5);

	volatile uint32_t timeout = 50;

	if ((mailbox->inbox == NULL) && (--timeout))
	{
		//! Release semaphore
		xSemaphoreGive(mailbox->sempr);
		//! Take semaphore
		xSemaphoreTake(mailbox->sempr, 5);
	}

	//! Check timeout
	if (0 == timeout)
	{
		//! Release semaphore
		xSemaphoreGive(mailbox->sempr);
		return E_SYS;
	}

	timeout = 50;
	//! Waiting for mailbox is could receive new message
	if (((mailbox->repeat) && (mailbox->top == mailbox->bottom)) && (--timeout))
	{
		//! Release semaphore
		xSemaphoreGive(mailbox->sempr);
		//! Take semaphore
		xSemaphoreTake(mailbox->sempr, 5);
	}

	if (0 == timeout)
	{
		//! Release semaphore
		xSemaphoreGive(mailbox->sempr);
		return E_SYS;
	}

	//! Giving message to target mailbox
	mailbox->inbox[mailbox->top++] = msg;

	//! Change mailbox status if any
	if (mailbox->top == mailbox->size)
		mailbox->repeat = TRUE;

	//! Release semaphore
	xSemaphoreGive(mailbox->sempr);

	//! Exit success
	return E_OK;
}
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
