/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    message.h
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
#ifndef MESSAGE_H_
#define MESSAGE_H_
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"
#include "qc_common.h"
/* Exported define -----------------------------------------------------------*/
/**
 * Mailbox ID
 */
#define SYS_MAILBOX           0x00
#define STORAGE_MAILBOX       0x01
#define CAMERA_MAILBOX        0x02
#define SPI_MASTER_MAILBOX    0x03
#define I2C_MASTER_MAILBOX    0x04
#define I2C_SLAVE_MAILBOX     0x05
#define VCM_MAILBOX           0x06
#define SENSOR_CTRL_MAILBOX   0x07

/**
 * Low level task ID
 */
#define MASTER_CTRL_TASK      0x00
#define BASH_CTRL_TASK        0x01
#define COM_SRV_TASK          0x02
#define CAM_CTRL_TASK
/**
 * Message ID
 */
#define RECEIVED_BASH_COMMAND 0x00
#define RECEIVED_I2C_COMMAND  0x01
#define FORWARD_I2C_COMMAND   0x02
/* Exported types ------------------------------------------------------------*/
/**
 * Message's content type
 */
typedef struct
{
	uint32_t TimeH; //! 4 high bytes of time
	uint32_t TimeL; //! 4 low bytes of time
}message_header_t;

typedef struct
{
	message_header_t header; //! Message header
	uint8_t*         body;   //! Message body
}message_data_t;

typedef struct
{
	uint16_t        size; //! Size in bytes of message
	message_data_t  data; //! Data address of message data
}msg_content_t;
/**
 * Message type
 */
typedef struct
{
	uint16_t      from; 	//! Sending  mailbox ID
	uint16_t      to;   	//! Received mailbox ID
	uint16_t      subject;  //! Subject as message ID
	msg_content_t content;  //! Message content
}message_t;
/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
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
IMPORT message_t* create_msg(uint16_t from,    \
		              	  	 uint16_t to,      \
							 uint16_t subject, \
							 uint16_t size,    \
							 uint8_t* data);
/**
 * @brief To delete a message
 * @param[in] 	msg	: Indicated message
 * @param[out] 	NA
 * @return 		NA
 * @details To delete a message
 */
IMPORT std_return_t delete_msg(message_t* msg);
#endif /* MESSAGE_H_ */

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
