/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    fifo.c
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
#include "fifo.h"
#include "qc_common.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Static functions prototype-------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported global variables --------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Push a byte to fifo
 * @param[in] byte		: input byte
 * @param[out] fifo		: destination fifo
 * @return  E_OK	: success
 *          Other	: failed
 * @details Push a byte to fifo
 */
std_return_t fifo_push(fifo_t* fifo, uint8_t byte)
{
	//! Check overflow
	if ((fifo->overflow) && (fifo->top = fifo->bottom))
		return E_SYS;
	fifo->data[fifo->top++] = byte;
	if (fifo->top == fifo->size)
	{
		fifo->overflow = TRUE;
		fifo->top      = ZERO;
	}
	return E_OK;
}
/**
 * @brief Pop a byte from FIFO
 * @param[int] 	fifo: destination FIFO
  * @param[out] byte: input byte
 * @return  E_OK	: success
 *          Other	: failed
 * @details Pop a byte from FIFO
 */
std_return_t fifo_pop(fifo_t* fifo, uint8_t* byte)
{
	if ((!fifo->overflow) && (fifo->top = fifo->bottom))
		return E_SYS;
	*byte = fifo->data[fifo->bottom++];
	if (fifo->bottom == fifo->size)
	{
		fifo->overflow = FALSE;
		fifo->bottom   = ZERO;
	}
	return E_OK;
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
