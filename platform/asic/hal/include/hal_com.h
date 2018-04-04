/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_com.h
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Jan-28-2016
 * @brief	This file contains definitions of the Serial driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_COM_H__
#define __HAL_COM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported define -----------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/*
 * @brief hal_com_return_t
 *
 * COM return type
 */
typedef enum hal_com_return
{
	COM_OK,
	COM_TIMEOUT,
	COM_UNKNOWN_ERROR
} hal_com_return_t;

/*
 * @brief hal_com_name
 *
 * COM port name define
 */
typedef enum hal_com_name
{
	COM1,
	COM_MAX_IDX
} hal_com_name_t;

/*
 * Macro used for asserting port name
 */
#define IS_COM_PORT(PORT)		(PORT == COM1)

/*
 * @brief hal_com_baudrate_t
 *
 * COM baud rate define
 */
typedef enum hal_com_baudrate
{
	COM_BAUD_9600	= 9600,
	COM_BAUD_19200	= 19200,
	COM_BAUD_38400	= 38400,
	COM_BAUD_57600	= 57600,
	COM_BAUD_115200	= 115200
} hal_com_baud_t;

/*
 * Macro used for asserting baud rate
 */
#define IS_COM_BAUDRATE(BAUDRATE)		((BAUDRATE == COM_BAUD_9600) && \
										(BAUDRATE == COM_BAUD_19200) && \
										(BAUDRATE == COM_BAUD_38400) && \
										(BAUDRATE == COM_BAUD_57600) && \
										(BAUDRATE == COM_BAUD_115200))

/*
 * @brief hal_com_t
 *
 * COM driver structure
 */
typedef struct hal_com
{
	const hal_com_name_t	port_name;
	const hal_com_baud_t	baudrate;
	/* Pointer to function handled in irq when reception interrupt occurs */
	void					(*irq_handler)(uint8_t c);
	uint8_t					*data;
} hal_com_t;

/* Exported functions ------------------------------------------------------- */

/*
 * @brief hal_com_init
 * Initialize independent COM channel
 * @param com: pointer to hal_com_t structure
 * @return: none
 */
void hal_com_init(hal_com_t *com);

/*
 * @brief hal_com_sendbyte
 * Send one byte to hardware
 * @param com: pointer to hal_com_t structure
 * @return: reference to hal_com_return_t
 */
hal_com_return_t hal_com_sendbyte(hal_com_t *com);

/*
 * @brief hal_com_readbyte
 * Read one byte from hardware
 * @param com: pointer to hal_com_t structure
 * @return: reference to hal_com_return_t
 */
hal_com_return_t hal_com_readbyte(hal_com_t *com);

/*
 * @brief hal_com_enable_irq
 * Enable UART interrupt
 * @param com: pointer to hal_com_t structure
 * @return: none
 */
void hal_com_enable_irq(hal_com_t *com);

/*
 * @brief hal_com_disable_irq
 * Disable UART interrupt
 * @param com: pointer to hal_com_t structure
 * @return: none
 */
void hal_com_disable_irq(hal_com_t *com);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_COM_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
