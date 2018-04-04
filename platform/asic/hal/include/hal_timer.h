/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file    hal_timer.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Jan-28-2016
 * @brief   This file contains definitions of the TIMER driver IP
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "std_type.h"

/* Exported define------------------------------------------------------------*/
/* Exported typedef  ---------------------------------------------------------*/

/**
 * @brief hal_tim_chanel_t
 *
 * Timer channels
 */
typedef enum hal_tim_channel
{
	HAL_TIM_CH1 = 0,
	HAL_TIM_CH2,
	HAL_TIM_CH3,
	HAL_TIM_CH4,
	HAL_TIM_CH5,
	HAL_TIM_CH6,
	HAL_TIM_CH7,
	HAL_TIM_CH8,
	HAL_TIM_MAX
} hal_timer_channel_t;

/**
 * @brief hal_timer_return_t
 *
 * Timer return type
 */
typedef enum hal_tim_return
{
	HAL_TIM_OK,
	HAL_TIM_NULL_PTR,
	HAL_TIM_INVALID_CHANNEL,
	HAL_TIM_UNKNOWN_ERROR
} hal_timer_return_t;

/**
 * @brief hal_timer_t
 *
 * TIM driver structure
 */
typedef struct hal_timer
{
	hal_timer_channel_t	chid;			/* hal timer channel id */
	uint32_t			period;			/* hal timer period (microsecond) */
	/* pointer to function handled in irq every when TIMER expires */
	void				(*callback_handler)(void *params);
	void				*params;
} hal_timer_t;

typedef void (*handler_t)(void);
/* Exported function  -------------------------------------------------------*/

/*
 * @brief hal_timer_init
 * Initializes timer module
 * @param tim: point to hal_timer_t structure
 * @return reference to hal_timer_return_t
 */
hal_timer_return_t hal_timer_init(hal_timer_t *tim);

/*
 * @brief hal_timer_update_period
 * Updates timer period
 * @param tim: point to hal_timer_t structure
 * @return reference to hal_timer_return_t
 */
hal_timer_return_t hal_timer_update_period(hal_timer_t *tim);

/*
 * @brief hal_timer_start
 * Enables timer
 * @param chid: TIM channel id
 * @return reference to hal_timer_return_t
 */
hal_timer_return_t hal_timer_start(hal_timer_channel_t chid);

/*
 * @brief hal_timer_stop
 * Disables timer
 * @param chid: TIM channel id
 * @return reference to hal_timer_return_t
 */
hal_timer_return_t hal_timer_stop(hal_timer_channel_t chid);

/*
 * @brief hal_timer_read_counter_reg
 * Read value from counter register of Timer
 * @param chid: TIM channel id
 * @return value in timer counter register
 */
unsigned int hal_timer_read_counter_reg(hal_timer_channel_t chid);

/*
 * @brief hal_timer_set_period
 * Configure period for timer
 * @param chid: TIM channel id
 * @param value: raw timer value
 * @return none
 */
void hal_timer_set_period(hal_timer_channel_t chid, unsigned int value);

#ifdef __cplusplus
}
#endif
#endif /* __HAL_TIMER_H__ */
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
