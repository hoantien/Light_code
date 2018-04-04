/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    message.c
 * @author  The LightCo.
 * @version 1.0.0
 * @date    Jun 15, 2016
 * @brief   
 *
 ******************************************************************************/

/******************************************************************************/
/**  						Revision history
 *
 * * 1.0.0	Jun 15, 2016	Initial revision
 */
/******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "message.h"
#include "time.h"
#include "os.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported global variables -------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief To create a message
 * @param[in] 	from	: Sending mailbox ID
 * @param[in] 	to		: Receiving mailbox ID
 * @param[in] 	subject	: Message subject as message ID
 * @param[out] 	content	: Message data
 * @return 		NULL	: Fail to create message
 *              Others	: Address of message
 * @details To create a message
 */
message_t* create_msg(uint16_t from,     \
		              uint16_t to,       \
					  uint16_t subject,  \
					  uint16_t size,     \
					  uint8_t* data)
{
	if (NULL == data)
	{
		return NULL;
	}
	//! Provide message memory
	message_t* msg = pvPortMalloc(sizeof(message_t));
	//! Verify memory allocation
	if (NULL == msg) return msg;
	//! Compose info
	msg->from = from;
	msg->to   = to;
	msg->subject = subject;
	msg->content.size = size + sizeof(message_header_t);
	//! Get system timing
	msg->content.data.header.TimeL = time((time_t*)&msg->content.data.header.TimeH);
	//! plug-in data
	msg->content.data.body = data;
	return msg;
}
/**
 * @brief To delete a message
 * @param[in] 	msg	: Indicated message
 * @param[out] 	NA
 * @return 		NA
 * @details To delete a message
 */
IMPORT std_return_t delete_msg(message_t* msg)
{
	if (NULL == msg) return E_PARAM;
	//! Free message body
	if (NULL != msg->content.data.body)
	{
		vPortFree(msg->content.data.body);
		msg->content.data.body = NULL;
	}
	//! Free message
	vPortFree(msg);
	msg = NULL;
	return E_OK;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
