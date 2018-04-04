/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    timestamp.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-21-2016
 * @brief   This file contains expand of the timestamp driver
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "hal_timer.h"

#define TIMESTAMP_TIMER_ID				HAL_TIM_CH1
#define TIMESTAMP_BOARD_CLOCK			50000000

/* Local variable ------------------------------------------------------------*/
static hal_timer_t timer_stamp =
{
	.chid             = TIMESTAMP_TIMER_ID,
	.period           = 1000,
	.callback_handler = NULL
};

/* Exported functions --------------------------------------------------------*/
/*
 * timestamp_start(void)
 * The function starts time stamp driver
 */
void timestamp_start(void)
{
	hal_timer_stop(timer_stamp.chid);
	hal_timer_init(&timer_stamp);
	hal_timer_set_period(timer_stamp.chid, 0xFFFFFFFF);
	hal_timer_start(timer_stamp.chid);
}

/*
 * timestamp_ms(void)
 * The function reads time stamp value from start point
 */
unsigned int timestamp_ms(void)
{
	unsigned int start_timestamp = 0xFFFFFFFF;
	unsigned int stop_timestamp = hal_timer_read_counter_reg(timer_stamp.chid);
	stop_timestamp = start_timestamp - stop_timestamp;
	return (unsigned int)(stop_timestamp / (TIMESTAMP_BOARD_CLOCK / 1000));
}

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
