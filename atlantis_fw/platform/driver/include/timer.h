/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file    timer.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    Mar-07-2016
 * @brief   This file contains expand of the timer driver
 *
 ******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
/* Exported define------------------------------------------------------------*/
#define TIMER_MAX_NBR		20

/* Exported typedef  ---------------------------------------------------------*/
/**
 * timer_state_t typedef
 *
 */
typedef enum timer_state
{
	TIMER_STOPPED = 0,
	TIMER_STARTED
} timer_state_t;

/**
 * ltimer_t structure
 *
 */
struct ltimer_t
{
	uint8_t			id;				/* Timer ID */
	uint8_t			autoreload;		/* Timer auto reload timer ON/OFF */
	uint32_t		interval;		/* Timer interval in milisecond */
	uint32_t		time_expired;	/* Timer expire */
	timer_state_t	state;			/* Timer state START/STOP */
	void (*handler)(void *param);	/* Timer handler when time expired */
	void			*param;			/* Passing parameters in expired handler */
	struct ltimer_t	*next;
};

/* Exported functions --------------------------------------------------------*/
/**
 * @brief timer_init(void)
 * The function init system timer
 * @param None
 * @return None
 */
void timer_init(void);

/**
 * @brief timer_create(void)
 * @param None
 * @return struct ltimer_t
 * NULL: timer create unsuccessful, valid pointer: create successful
 */
struct ltimer_t *timer_create(void);

/**
 * @brief timer_start(void)
 * The function starts timer
 * @param *timer - point to timer valid
 * @return 0: Start successful, Start failed
 */
int timer_start(struct ltimer_t *timer);

/**
 * @brief timer_stop(void)
 * The function stops timer
 * @param *timer - point to timer valid
 * @return 0: stop successful, stop failed
 */
int timer_stop(struct ltimer_t *timer);

/**
 * @brief timer_delete(void)
 * The function deletes timer
 * @param *timer - point to timer valid
 * @return 0: delete successful, delete failed
 */
int timer_delete(struct ltimer_t **timer);

#ifdef __cplusplus
}
#endif
#endif /* __TIMER_H__ */

/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
