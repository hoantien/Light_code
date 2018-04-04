/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_syncio.h
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Mar-02-2016
 * @brief	This file contains definitions of the SyncIO driver IP
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_SYNCIO_H__
#define __HAL_SYNCIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported typedef ----------------------------------------------------------*/
/**
* @brief  SyncIO Pulse generator channel enumeration
*/
typedef enum
{
	SG_CHANNEL_0 = 0,	/*!< Pulse generator channel 0 */
	SG_CHANNEL_1,		/*!< Pulse generator channel 1 */
	SG_CHANNEL_2,		/*!< Pulse generator channel 2 */
	SG_CHANNEL_3,		/*!< Pulse generator channel 3 */
	SG_CHANNEL_4,		/*!< Pulse generator channel 4 */
	SG_CHANNEL_5,		/*!< Pulse generator channel 5 */
	SG_CHANNEL_MAX		/*!< Pulse generator channel size */
} hal_syncio_channel_t;

/**
  * @brief  Infinite type enumeration
  */
typedef enum
{
	SG_INF_DIS	= 0x00,	/*!< Infinite mode disable */
	SG_INF_EN	= 0x02,	/*!< Infinite mode enable */
} syncio_infinite_mode_t;

/**
  * @brief  Pulse generator trigger mode enumeration
  */
typedef enum
{
	SG_TRIG_SW = 0x00,	/*!< Software mode, triggered by SGn_EN immediately */
	SG_TRIG_HW = 0x01,	/*!< Hardware mode, triggered by input signal trigger */
} syncio_trig_mode_t;

/**
  * @brief  Pulse generator interrupt mode enumeration
  */
typedef enum
{
	SG_INTR_PULSE_DONE		= 0x00000001,		/*!< Pulse done interrupt */
	SG_INTR_REPEAT_DONE		= 0x00000040,		/*!< Repeat done interrupt */
	SG_INTR_FALLING_EDGE	= 0x00001000		/*!< Falling edge interrupt */
} hal_syncio_irq_mode_t;

/**
  * @brief  SyncIO Pulse generator channel configuration structure
  */
typedef struct
{
	syncio_infinite_mode_t	inf_mode;		/*!< Infinite mode */
	syncio_trig_mode_t		trig_mode;		/*!< Trigger mode */
	uint32_t				lat1;			/*!< Latency 1 */
	uint32_t				lat2;			/*!< Latency 2 */
	uint32_t				width1;			/*!< Pulse width 1 */
	uint32_t				width2;			/*!< Pulse width 2 */
	uint8_t					repeat;			/*!< Pulse generator repeat cycle */
} hal_syncio_cfg_t;

/**
  * @brief  SyncIO Pulse generator channel interrupt configuration structure
  */
typedef struct
{
	hal_syncio_irq_mode_t	irq_modes;		/*!< Interrupt modes */

	/*!< Interrupt callback handler */
	void					(*callback_handler)(hal_syncio_channel_t chid,
											hal_syncio_irq_mode_t irq_status);
} hal_syncio_irq_t;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief hal_syncio_init
 * The function initializes SyncIO global features
 * @param None
 * @return None
 */
void hal_syncio_init(hal_syncio_channel_t chid);

/**
 * @brief hal_syncio_config
 * The function configures pulse generator channel parameters
 * @param chid Pulse generator channel
 * @param cfg_val pointer to syncio_cfg_t structure
 * @return None
 */
void hal_syncio_config(hal_syncio_channel_t chid, hal_syncio_cfg_t *cfg_val);

/**
 * @brief hal_syncio_enable
 * The function enables generating the SyncIO pulse of independent channel.
 * @param chid Pulse generator channel
 * @return None
 */
void hal_syncio_enable(hal_syncio_channel_t chid);

/**
 * hal_syncio_disable
 * The function disables generating the SyncIO pulse of independent channel.
 * @param chid Pulse generator channel
 * @return None
 */
void hal_syncio_disable(hal_syncio_channel_t chid);

/**
 * hal_syncio_trigger
 * The function triggers generating the SyncIO pulse
 * 		of the channels configured as SW trigger mode.
 * @param None
 * @return None
 */
void hal_syncio_trigger(void);

/**
 * @brief hal_syncio_enable_irq
 * The function enables interrupt of independent SyncIO channel
 * @param chid Pulse generator channel
 * @param irq_cfg Pointer to syncio_irq_t structure
 * @return None
 */
void hal_syncio_enable_irq(hal_syncio_channel_t chid,
												hal_syncio_irq_t *irq_cfg);

/**
 * @brief hal_syncio_disable_irq
 * The function disables interrupt of independent SyncIO channel
 * @param chid Pulse generator channel
 * @return None
 */
void hal_syncio_disable_irq(hal_syncio_channel_t chid);

/**
 * @brief hal_syncio_get_counter
 * The function gets the counter register of independent SyncIO channel,
 * used for debug purpose.
 * @param chid Pulse generator channel
 * @return The value of counter register
 */
uint32_t hal_syncio_get_counter(hal_syncio_channel_t chid);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_SYNCIO_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
