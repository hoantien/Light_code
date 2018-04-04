/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    fifo.h
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
#ifndef FIFO_H_
#define FIFO_H_
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported define -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
		uint16_t size;
		uint16_t top;
		uint16_t bottom;
		volatile bool overflow;
		uint8_t* data;
}fifo_t;
/* Exported variables --------------------------------------------------------*/

/* Inline functions  ---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/**
 * @brief Push a byte to FIFO
 * @param[in] byte	: input byte
 * @param[out] fifo	: destination FIFO
 * @return  E_OK	: success
 *          Other	: failed
 * @details Push a byte to FIFO
 */
std_return_t fifo_push(fifo_t* fifo, uint8_t byte);

/**
 * @brief Pop a byte from FIFO
 * @param[int] 	fifo: destination FIFO
  * @param[out] byte: input byte
 * @return  E_OK	: success
 *          Other	: failed
 * @detail Pop a byte from FIFO
 */
std_return_t fifo_pop(fifo_t* fifo, uint8_t* byte);

#endif /* FIFO_H_ */


/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.***** END OF FILE *********/
